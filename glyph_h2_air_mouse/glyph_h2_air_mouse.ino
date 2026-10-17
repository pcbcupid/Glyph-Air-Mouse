/*
 * Glyph H2 - Bluetooth air mouse (absolute pointer, Wii-remote style) - latest version
 *
 * Point the board like a remote: where it points is where the cursor goes.
 *   left/right swing (yaw)  -> cursor left/right, ±YAW_RANGE_DEG = screen edges
 *   up/down tip (pitch)     -> cursor up/down,    ±PITCH_RANGE_DEG = screen edges
 * Pointing straight ahead (level) = screen centre. Swinging past an edge keeps
 * the cursor at the edge, and coming back lines up again.
 *
 * The board only controls the cursor while it is being moved: lying still it
 * sends nothing, so the touchpad works normally. Moving it takes over again.
 *
 * Buttons (1x3 module):
 *   K3 = left click (hold = drag), K2 (middle) = right click,
 *   K1 tap = re-centre when released (whatever direction you point now = screen centre),
 *   K1 hold = scroll mode: cursor freezes, tilt up/down to scroll up/down
 *             (further tilt = faster); release to carry on pointing.
 *
 * Filters:
 *   up/down    - Kalman filter (angle + gyro bias) fusing gyro and accelerometer,
 *                so it never drifts and level is always the centre.
 *   left/right - Kalman-filtered gyro rate, integrated to an angle. There is no
 *                gravity reference for this direction, so it can drift slowly;
 *                press K1 to re-centre.
 *
 * Either way up works (buttons up or buttons down): the orientation is
 * detected from gravity at power-up.
 *
 * Keep the board still for ~1 s after power-up while it calibrates.
 *
 * Bluetooth name: type "NAME My Mouse" in the Serial Monitor (115200 baud,
 * newline), then press RESET. The name is saved in flash. "NAME" alone
 * shows the current name.
 *
 * Sleep: never while the laptop is connected. After SLEEP_UNCONNECTED_MIN
 * minutes without a connection the board goes into deep sleep with the
 * MPU6050 asleep too. Press any button to wake it: it
 * restarts, calibrates (keep it still ~1 s) and reconnects by itself.
 * "SLEEP" in the Serial Monitor sends it to sleep straight away.
 *
 * Wiring: MPU6050 SDA=IO23, SCL=IO24, AD0=IO1 (driven LOW -> 0x68)
 *         Button module VCC=3.3V, GND=IO14 (driven LOW), D1=IO13, D2=IO12, D3=IO10
 */

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLESecurity.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

#define DEFAULT_NAME "Glyph H2 Mouse"
const int BT_NAME_MAX = 20;  // longer names don't fit in the advertising packet

#define SDA_PIN 23
#define SCL_PIN 24
#define AD0_PIN 1
#define XDA_PIN 25
#define XCL_PIN 22
#define BTN_GND_PIN 14  // button module GND, driven LOW to act as ground
#define K1_PIN 13       // re-centre (short press) / scroll (hold)
#define K2_PIN 12       // right click (middle button)
#define K3_PIN 10       // left click

#define MPU_ADDR 0x68

#define POS_MAX 32767   // absolute coordinate range 0..POS_MAX covers the whole screen

// Tuning
const float YAW_RANGE_DEG = 15.0;    // swing this far left/right to reach the screen edge
const float PITCH_RANGE_DEG = 12.0;  // tip this far up/down to reach the screen edge
const float BACKLASH_DEG = 0.15;     // movement smaller than this is ignored (hand jitter)
const float YAW_DEADZONE_DPS = 0.5;  // slower yaw than this is not integrated (limits drift)
const float STILL_DPS = 1.5;         // board counts as "still" below this, for drift correction
const float ACTIVE_DPS = 2.5;        // moving faster than this = board takes control of the cursor
const uint32_t IDLE_MS = 1000;       // this long without movement = hand control back to touchpad
const uint32_t RECENTER_CONTROL_MS = 5000;  // after K1 the board keeps the cursor this long, even moving slowly
const uint32_t DEBOUNCE_MS = 20;
const uint32_t K1_DEBOUNCE_MS = 60;  // longer for K1: a release bounce would re-centre mid-movement
// Click stabilization: pressing a button jolts the board slightly. The click
// lands where the cursor was CLICK_LOOKBACK samples (x10 ms) before the press,
// and the cursor freezes for CLICK_FREEZE_MS after each press and release.
// While a button is held the cursor stays locked until the board turns more
// than DRAG_START_DEG (then it drags).
const int CLICK_LOOKBACK = 6;          // 60 ms
const uint32_t CLICK_FREEZE_MS = 150;
const uint32_t DOUBLE_CLICK_MS = 400;  // a press this soon after the last click reuses its exact spot
const float DRAG_START_DEG = 3.0;      // turn this far with a button held to start dragging
// Scroll mode (hold K1)
const uint32_t SCROLL_HOLD_MS = 400;   // hold K1 this long to enter scroll mode (shorter = a tap)
const float SCROLL_DEADZONE_DEG = 3.0; // tilt less than this doesn't scroll
const float SCROLL_SPEED = 0.6;        // wheel clicks per second per degree of tilt
const float SCROLL_MAX = 30.0;         // wheel clicks per second at most
const uint32_t REPORT_MS = 10;       // 100 Hz
// Sleep (any button wakes it)
const uint32_t SLEEP_UNCONNECTED_MIN = 2;  // no laptop connected for this long

// Axes and directions (board flat). Flip a sign if that direction is reversed.
const int YAW_AXIS = 2, YAW_SIGN = -1;      // gyro Z: swing left/right
const int PITCH_AXIS = 0, PITCH_SIGN = -1;  // gyro X: tip up/down

// Kalman filter on the yaw rate:
//   KF_Q = how fast the real rotation can change (higher = more responsive)
//   KF_R = how noisy the gyro reading is (higher = smoother, more lag)
const float KF_Q = 2.0;           // (deg/s)^2 per sample
const float KF_R = 4.0;           // (deg/s)^2

// One Euro filter on the left/right angle (adaptive smoothing):
//   slow movement -> cutoff near EURO_MIN_CUTOFF (strong smoothing, no jitter)
//   fast movement -> cutoff rises by EURO_BETA per deg/s (little smoothing, no lag)
const float EURO_MIN_CUTOFF = 2.5;  // Hz - lower = less jitter when moving slowly
const float EURO_BETA = 0.2;        // higher = less lag when moving fast
const float EURO_D_CUTOFF = 1.0;    // Hz - smoothing of the speed estimate

// HID absolute pointer: 3 buttons, X, Y (0..32767), scroll wheel (relative)
const uint8_t hidReportDescriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x02,        // Usage (Mouse)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x01,        //   Report ID (1)
  0x09, 0x01,        //   Usage (Pointer)
  0xA1, 0x00,        //   Collection (Physical)
  0x05, 0x09,        //     Usage Page (Button)
  0x19, 0x01,        //     Usage Minimum (1)
  0x29, 0x03,        //     Usage Maximum (3)
  0x15, 0x00,        //     Logical Minimum (0)
  0x25, 0x01,        //     Logical Maximum (1)
  0x95, 0x03,        //     Report Count (3)
  0x75, 0x01,        //     Report Size (1)
  0x81, 0x02,        //     Input (Data, Var, Abs)
  0x95, 0x01,        //     Report Count (1)
  0x75, 0x05,        //     Report Size (5)
  0x81, 0x03,        //     Input (Const) - padding
  0x05, 0x01,        //     Usage Page (Generic Desktop)
  0x09, 0x30,        //     Usage (X)
  0x09, 0x31,        //     Usage (Y)
  0x16, 0x00, 0x00,  //     Logical Minimum (0)
  0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
  0x75, 0x10,        //     Report Size (16)
  0x95, 0x02,        //     Report Count (2)
  0x81, 0x02,        //     Input (Data, Var, Abs)
  0x09, 0x38,        //     Usage (Wheel)
  0x15, 0x81,        //     Logical Minimum (-127)
  0x25, 0x7F,        //     Logical Maximum (127)
  0x75, 0x08,        //     Report Size (8)
  0x95, 0x01,        //     Report Count (1)
  0x81, 0x06,        //     Input (Data, Var, Rel)
  0xC0,              //   End Collection
  0xC0               // End Collection
};

// Report ID is carried by the characteristic's Report Reference descriptor,
// so it is not part of the payload.
struct MouseReport {
  uint8_t buttons;
  uint16_t x;
  uint16_t y;
  int8_t wheel;
} __attribute__((packed));

// 1-D Kalman filter: estimates the true rotation rate from noisy readings.
// Each step predicts (rate stays the same, uncertainty grows by Q), then
// corrects towards the measurement weighted by the Kalman gain K.
struct Kalman1D {
  float x = 0;  // estimated rate, deg/s
  float p = 1;  // estimate uncertainty (variance)

  float update(float z) {
    p += KF_Q;                // predict
    float k = p / (p + KF_R); // Kalman gain
    x += k * (z - x);         // correct with measurement z
    p *= (1 - k);
    return x;
  }
};

// 2-state Kalman filter (angle + gyro bias). The gyro rate predicts the
// angle; the accelerometer angle corrects it, and the filter also learns
// the gyro's bias, so the angle never drifts.
// accTrust (0..1) scales how much the accelerometer is believed: when the
// hand pushes the board around, the accelerometer feels the push as well as
// gravity, so its angle is wrong and the filter should lean on the gyro.
struct KalmanAngle {
  const float Q_ANGLE = 0.003;  // process noise of the angle (higher = settles faster)
  const float Q_BIAS = 0.003;   // process noise of the bias
  const float R_MEAS = 0.03;    // accelerometer angle noise
  float angle = 0, bias = 0;
  float P[2][2] = {{0, 0}, {0, 0}};

  void reset(float a) {
    angle = a;
    bias = 0;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
  }

  float update(float accAngle, float gyroRate, float dt, float accTrust) {
    // Predict
    angle += dt * (gyroRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_ANGLE);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_BIAS * dt;
    // Correct with the accelerometer angle (skipped while it can't be trusted)
    if (accTrust <= 0) return angle;
    float s = P[0][0] + R_MEAS / accTrust;
    float k0 = P[0][0] / s, k1 = P[1][0] / s;
    float y = accAngle - angle;
    angle += k0 * y;
    bias += k1 * y;
    float p00 = P[0][0], p01 = P[0][1];
    P[0][0] -= k0 * p00;
    P[0][1] -= k0 * p01;
    P[1][0] -= k1 * p00;
    P[1][1] -= k1 * p01;
    return angle;
  }
};

BLEHIDDevice *hid;
BLECharacteristic *inputMouse;
bool connected = false;
BLEServer *bleServer;
uint16_t connHandle = 0;
float connIntervalMs = 0;   // Bluetooth connection interval the laptop chose
uint32_t connectedAt = 0;
int paramRetries = 0;
uint32_t lastSend = 0;      // when the last position report was sent

float gyroBias[3] = {0, 0, 0};
uint16_t stillCount = 0;    // consecutive samples with the board still
Kalman1D yawKf;

// One Euro filter (Casiez et al.): a low-pass filter whose cutoff rises with
// speed, so it removes jitter when slow without adding lag when fast.
struct OneEuro {
  float x = 0, dx = 0;
  bool ready = false;

  static float alpha(float cutoff, float dt) {
    float tau = 1.0 / (2 * PI * cutoff);
    return 1.0 / (1.0 + tau / dt);
  }
  void reset(float v) {
    x = v;
    dx = 0;
    ready = true;
  }
  float filter(float v, float dt) {
    if (!ready) reset(v);
    float rawDx = (v - x) / dt;
    dx += alpha(EURO_D_CUTOFF, dt) * (rawDx - dx);
    float cutoff = EURO_MIN_CUTOFF + EURO_BETA * fabs(dx);
    x += alpha(cutoff, dt) * (v - x);
    return x;
  }
};

OneEuro yawEuro;
float up[3] = {0, 0, 1};    // "up" direction in board coordinates (unit vector)
KalmanAngle pitchKf;
float yawAngle = 0;         // integrated swing angle, deg (0 = centre)
float yawAnchor = 0, pitchAnchor = 0;  // jitter-free angles used for the cursor
float pitchOffset = 0;      // up/down angle that maps to the screen centre (0 = level)
float orient = 1;           // +1 = MPU6050 facing up (buttons down), -1 = flipped (buttons up)
uint16_t lastX = 0xFFFF, lastY = 0xFFFF;
uint8_t buttons = 0;
uint32_t lastActive = 0;    // last time the board was being moved
uint32_t controlUntil = 0;  // board keeps control of the cursor until this time (after K1)
// A thumb pushing a button shakes the board and fools the accelerometer into
// a false tilt; until this time up/down runs on the gyro alone
uint32_t ignoreAccUntil = 0;
const uint32_t BUTTON_SHAKE_MS = 300;
uint32_t freezeUntil = 0;   // cursor ignores movement until this time (click stabilization)
uint32_t lastRelease = 0;   // when a button was last released (double-click detection)
bool holdLock = false;      // button held, cursor locked until the board turns DRAG_START_DEG
float pressYaw = 0, pressPitch = 0;  // angles when the button was pressed
uint16_t clickX = 0, clickY = 0;  // where the last click landed
const int HIST_LEN = 16;
uint16_t histX[HIST_LEN], histY[HIST_LEN];  // recent cursor positions, one per sample
int histIdx = 0, histCount = 0;
bool scrollMode = false;    // K1 held: tilt up/down scrolls instead of moving the cursor
uint32_t k1Start = 0;
float scrollStartPitch = 0, scrollSavedYaw = 0, scrollAcc = 0;
uint32_t lastUse = 0;       // last connect or disconnect (sleep timer)
// Kept through deep sleep: button released levels, since the wake-up button
// is still held at startup and would be mistaken for "released"
RTC_DATA_ATTR bool savedIdleValid = false;
RTC_DATA_ATTR bool savedIdle[3];

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    connected = true;
    lastUse = connectedAt = millis();
    paramRetries = 0;
    lastX = lastY = 0xFFFF;  // force a report so the cursor jumps into place
    Serial.println("Laptop connected");
  }
#if defined(CONFIG_NIMBLE_ENABLED)
  // Ask the laptop for a fast connection interval (7.5-15 ms) so cursor
  // updates are delivered quickly. Units: interval 1.25 ms, timeout 10 ms.
  void onConnect(BLEServer *s, ble_gap_conn_desc *desc) {
    connHandle = desc->conn_handle;
    connIntervalMs = desc->conn_itvl * 1.25;
    Serial.printf("Connection interval: %.2f ms\n", connIntervalMs);
    s->requestConnParams(desc->conn_handle, 6, 12, 0, 400);
  }
  void onConnParamsUpdate(uint16_t conn_handle, uint16_t interval, uint16_t latency, uint16_t timeout, uint8_t status) {
    connIntervalMs = interval * 1.25;
    Serial.printf("Connection interval: %.2f ms\n", connIntervalMs);
  }
#endif
  void onDisconnect(BLEServer *s) {
    connected = false;
    lastUse = millis();
    Serial.println("Disconnected, advertising again");
    BLEDevice::startAdvertising();
  }
};

// ---------- MPU6050 ----------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Accelerometer (±2 g -> 16384 LSB per g) and gyro (raw, ±500 dps -> 65.5 LSB per deg/s)
bool readAll(float acc[3], float g[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 3; i++) {
    int16_t v = (Wire.read() << 8) | Wire.read();
    acc[i] = v / 16384.0;
  }
  Wire.read();  // temperature, unused
  Wire.read();
  for (int i = 0; i < 3; i++) {
    int16_t v = (Wire.read() << 8) | Wire.read();
    g[i] = v;
  }
  return true;
}

// Up/down angle from gravity, matching the gyro X direction:
// rotation about +X moves gravity from Z into +Y
// When the board is flipped, Z points down, so it is mirrored with orient.
float accelPitch(const float acc[3]) {
  return atan2(acc[1], orient * acc[2]) * 180.0 / PI;
}

void sendReport(uint16_t x, uint16_t y, int8_t wheel = 0);

// Pointing direction now = screen centre, and move the cursor there.
void recenter() {
  lastActive = millis();  // follow the board straight away after re-centring
  controlUntil = lastActive + RECENTER_CONTROL_MS;  // and follow even slow aiming movements
  yawAngle = yawAnchor = 0;
  yawEuro.reset(0);
  pitchOffset = pitchAnchor;
  if (connected) {
    // The laptop drops an absolute position equal to the last one this
    // device sent (e.g. centre again after the touchpad moved the cursor),
    // so step 1 unit off-centre first (invisible) to force the jump
    sendReport(POS_MAX / 2 + 1, POS_MAX / 2 + 1);
    sendReport(POS_MAX / 2, POS_MAX / 2);
  } else {
    lastX = lastY = 0xFFFF;
  }
  Serial.println("Re-centred");
}

// Gyro bias calibration. Repeats until the board was held still for the
// whole window, so moving it at power-up can't spoil the bias.
void calibrate() {
  Serial.println("Calibrating, keep it still...");
  const int N = 300;
  const float MAX_SPREAD = 5.0 * 65.5;  // allow 5 deg/s of wobble
  for (int attempt = 0; attempt < 20; attempt++) {
    float sum[3] = {0, 0, 0}, accSum[3] = {0, 0, 0}, lo[3], hi[3], acc[3], g[3];
    for (int a = 0; a < 3; a++) {
      lo[a] = 1e9;
      hi[a] = -1e9;
    }
    int n = 0;
    for (int i = 0; i < N; i++) {
      if (readAll(acc, g)) {
        for (int a = 0; a < 3; a++) {
          sum[a] += g[a];
          lo[a] = min(lo[a], g[a]);
          hi[a] = max(hi[a], g[a]);
        }
        for (int a = 0; a < 3; a++) accSum[a] += acc[a];
        n++;
      }
      delay(3);
    }
    if (n == 0) continue;
    bool still = true;
    for (int a = 0; a < 3; a++)
      if (hi[a] - lo[a] > MAX_SPREAD) still = false;
    for (int a = 0; a < 3; a++) gyroBias[a] = sum[a] / n;
    // Which way up: gravity on Z is +1 g with the MPU6050 facing up, -1 g flipped
    orient = accSum[2] < 0 ? -1 : 1;
    float accMean[3] = {accSum[0] / n, accSum[1] / n, accSum[2] / n};
    float m = sqrt(accMean[0] * accMean[0] + accMean[1] * accMean[1] + accMean[2] * accMean[2]);
    for (int a = 0; a < 3; a++) up[a] = accMean[a] / m;
    float p = accelPitch(accMean);
    pitchKf.reset(p);
    pitchAnchor = p;
    if (still) break;
    Serial.println("Moved during calibration, retrying...");
  }
  recenter();
  pitchOffset = 0;  // at power-up, level = centre for up/down
  Serial.println(orient > 0 ? "Calibration done (buttons down)" : "Calibration done (buttons up)");
}

// Backlash: the anchor only follows once the angle moves past it by
// BACKLASH_DEG, so jitter is ignored but no movement is lost.
void follow(float angle, float &anchor) {
  if (angle > anchor + BACKLASH_DEG) anchor = angle - BACKLASH_DEG;
  else if (angle < anchor - BACKLASH_DEG) anchor = angle + BACKLASH_DEG;
}

// Screen position: angle 0 = centre, ±range = edges
uint16_t toPos(float angle, int sign, float rangeDeg) {
  float frac = 0.5 + sign * angle / (2 * rangeDeg);
  return constrain(frac, 0.0f, 1.0f) * POS_MAX;
}

void sendReport(uint16_t x, uint16_t y, int8_t wheel) {
  MouseReport r = {buttons, x, y, wheel};
  inputMouse->setValue((uint8_t *)&r, sizeof(r));
  inputMouse->notify();
  lastSend = millis();
  lastX = x;
  lastY = y;
}

// ---------- Buttons ----------
// Order: left click, right click, re-centre
const uint8_t BTN_PINS[3] = {K3_PIN, K2_PIN, K1_PIN};
bool btnIdle[3];       // pin level when released (detected at startup)
bool btnState[3];      // debounced: true = pressed
uint32_t btnChanged[3];

// The module's wiring is unknown (buttons to VCC or to GND), so read each
// pin at startup while nobody presses it: that level means "released".
// After waking from sleep the levels saved before sleeping are used instead,
// and the wake-up press is not sent as a click.
void setupButtons() {
  gpio_hold_dis((gpio_num_t)BTN_GND_PIN);  // was held LOW during sleep
  pinMode(BTN_GND_PIN, OUTPUT);
  digitalWrite(BTN_GND_PIN, LOW);
  for (int i = 0; i < 3; i++) pinMode(BTN_PINS[i], INPUT_PULLDOWN);
  delay(20);
  bool woke = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1 && savedIdleValid;
  for (int i = 0; i < 3; i++) {
    btnIdle[i] = woke ? savedIdle[i] : digitalRead(BTN_PINS[i]);
    btnState[i] = digitalRead(BTN_PINS[i]) != btnIdle[i];
    btnChanged[i] = 0;
  }
  if (woke) Serial.println("Woke up from sleep");
}

// Deep sleep: MPU6050 asleep, button module ground held LOW, any button wakes
void goToSleep() {
  Serial.println("Going to sleep - press any button to wake");
  Serial.flush();
  writeReg(0x6B, 0x40);  // MPU6050 sleep
  for (int i = 0; i < 3; i++) {
    savedIdle[i] = btnIdle[i];
    // wake when the pin leaves its released level
    esp_sleep_enable_ext1_wakeup_io(1ULL << BTN_PINS[i],
                                    btnIdle[i] ? ESP_EXT1_WAKEUP_ANY_LOW : ESP_EXT1_WAKEUP_ANY_HIGH);
  }
  savedIdleValid = true;
  gpio_hold_en((gpio_num_t)BTN_GND_PIN);
  esp_deep_sleep_start();
}

// Returns true if button i changed state (debounced); state is in btnState[i]
bool readButton(int i, uint32_t now) {
  bool pressed = digitalRead(BTN_PINS[i]) != btnIdle[i];
  if (pressed != btnState[i] && now - btnChanged[i] >= (i == 2 ? K1_DEBOUNCE_MS : DEBOUNCE_MS)) {
    btnState[i] = pressed;
    btnChanged[i] = now;
    return true;
  }
  return false;
}

// Send the current button state at the current cursor position
void sendButtons() {
  if (lastX > POS_MAX || lastY > POS_MAX) lastX = lastY = POS_MAX / 2;  // never sent a position yet
  sendReport(lastX, lastY);
}

void setup() {
  Serial.begin(115200);
  // Never wait on the USB serial: plugged into a computer with no program
  // reading the port, each print would block up to 2 s (20 x 100 ms) once the
  // buffer is full, freezing the cursor (e.g. after K1's "Re-centred")
  Serial.setTxTimeoutMs(0);

  setupButtons();
  pinMode(XDA_PIN, INPUT);
  pinMode(XCL_PIN, INPUT);
  pinMode(AD0_PIN, OUTPUT);
  digitalWrite(AD0_PIN, LOW);
  delay(50);

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  writeReg(0x6B, 0x01);  // wake up, PLL clock
  delay(100);
  writeReg(0x1A, 0x03);  // low-pass filter ~44 Hz (less delay than 20 Hz)
  writeReg(0x1B, 0x08);  // gyro ±500 deg/s
  writeReg(0x1C, 0x00);  // accel ±2 g
  delay(100);
  calibrate();

  BLEDevice::init(loadName().c_str());
  // Maximum transmit power (+20 dBm): a stronger signal survives the
  // laptop's Wi-Fi sharing the 2.4 GHz band and fingers over the antenna
  BLEDevice::setPower(ESP_PWR_LVL_P20, ESP_BLE_PWR_TYPE_DEFAULT);
  BLEDevice::setPower(ESP_PWR_LVL_P20, ESP_BLE_PWR_TYPE_ADV);

  BLESecurity *sec = new BLESecurity();
  sec->setCapability(ESP_IO_CAP_NONE);       // "Just Works" pairing
  sec->setAuthenticationMode(true, false, true);

  BLEServer *server = bleServer = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  hid = new BLEHIDDevice(server);
  hid->manufacturer()->setValue("PCBCupid");
  hid->pnp(0x02, 0xe502, 0xa115, 0x0210);
  hid->hidInfo(0x00, 0x02);
  hid->reportMap((uint8_t *)hidReportDescriptor, sizeof(hidReportDescriptor));
  inputMouse = hid->inputReport(1);
  hid->setBatteryLevel(100);
  hid->startServices();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->setAppearance(0x03C2);  // HID Mouse
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("%s ready - pair it from laptop Bluetooth settings\n", loadName().c_str());
  Serial.printf("Last restart: %s\n", resetReasonText());
  Serial.println("Type NAME <new name> to rename it");
}

// Why the board last restarted, for diagnosing unexpected restarts
const char *resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_EXT: return "reset pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "CRASH (panic)";
    case ESP_RST_INT_WDT: return "CRASH (interrupt watchdog)";
    case ESP_RST_TASK_WDT: return "CRASH (task watchdog)";
    case ESP_RST_WDT: return "CRASH (watchdog)";
    case ESP_RST_DEEPSLEEP: return "wake from sleep";
    case ESP_RST_BROWNOUT: return "POWER DIP (brownout)";
    case ESP_RST_USB: return "USB reset";
    default: return "other";
  }
}

// ---------- Bluetooth name (saved in flash) ----------
String loadName() {
  Preferences prefs;
  prefs.begin("mouse", true);
  String name = prefs.getString("name", DEFAULT_NAME);
  prefs.end();
  return name;
}

// Serial command "NAME <new name>": save the name, used after RESET
void checkSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (line.length() < 64) line += c;
      continue;
    }
    line.trim();
    if (line.equalsIgnoreCase("NAME")) {
      Serial.printf("Name: %s\n", loadName().c_str());
    } else if (line.length() > 5 && line.substring(0, 5).equalsIgnoreCase("NAME ")) {
      String name = line.substring(5);
      name.trim();
      if (name.length() == 0 || name.length() > BT_NAME_MAX) {
        Serial.printf("Name must be 1-%d characters\n", BT_NAME_MAX);
      } else {
        Preferences prefs;
        prefs.begin("mouse", false);
        prefs.putString("name", name);
        prefs.end();
        Serial.printf("Name saved: %s - press RESET to use it\n", name.c_str());
      }
    } else if (line.equalsIgnoreCase("SLEEP")) {
      goToSleep();
    } else if (line.length() > 0) {
      Serial.println("Unknown command. Use: NAME <new name> or SLEEP");
    }
    line = "";
  }
}

void loop() {
  static uint32_t lastReport = 0;
  static uint32_t lastMicros = micros();

  checkSerial();

  if (!connected) {
    if (millis() - lastUse > SLEEP_UNCONNECTED_MIN * 60000UL) goToSleep();
    delay(50);
    lastMicros = micros();
    return;
  }

#if defined(CONFIG_NIMBLE_ENABLED)
  // The laptop may ignore the fast-interval request made right at connect:
  // ask again a few times, 2 s apart, until it is 15 ms or less
  if (connIntervalMs > 15 && paramRetries < 3 && millis() - connectedAt > 2000UL * (paramRetries + 1)) {
    paramRetries++;
    bool ok = bleServer->requestConnParams(connHandle, 6, 12, 0, 400);
    Serial.printf("Asked for a faster connection interval (now %.2f ms): %s\n", connIntervalMs, ok ? "sent" : "failed");
  }
#endif

  // Buttons: K3 = left (hold = drag), K2 = right, K1 = re-centre / scroll
  uint32_t now = millis();
  bool changed = false;
  for (int i = 0; i < 2; i++) changed |= readButton(i, now);
  if (changed) {
    uint8_t newButtons = (btnState[0] ? 0x01 : 0) | (btnState[1] ? 0x02 : 0);
    // A new press while the board is in use: click where the cursor was just
    // before the thumb started pushing, not where the jolt moved it
    bool newPress = (newButtons & ~buttons) != 0;
    bool released = (buttons & ~newButtons) != 0;
    if (newPress && lastRelease != 0 && now - lastRelease < DOUBLE_CLICK_MS) {
      // Second click of a double-click: land on exactly the same spot as the
      // first, so the laptop sees a double-click even if the board jolted
      lastX = clickX;
      lastY = clickY;
    } else if (newPress && lastActive != 0 && now - lastActive < 300 && histCount > CLICK_LOOKBACK) {
      int i = (histIdx - 1 - CLICK_LOOKBACK + HIST_LEN) % HIST_LEN;
      lastX = histX[i];
      lastY = histY[i];
    }
    if (newPress) {
      clickX = lastX;
      clickY = lastY;
    }
    if (newPress) {
      holdLock = true;
      pressYaw = yawAnchor;
      pressPitch = pitchAnchor;
    }
    if (newButtons == 0) holdLock = false;
    if (released) lastRelease = now;
    buttons = newButtons;
    sendButtons();
    freezeUntil = now + CLICK_FREEZE_MS;  // hold still through the press/release jolt
    ignoreAccUntil = now + BUTTON_SHAKE_MS;
  }
  // K1: tap = re-centre on release, hold = scroll mode (cursor stays put)
  if (readButton(2, now)) {
    ignoreAccUntil = now + BUTTON_SHAKE_MS;
    if (btnState[2]) {
      k1Start = now;
    } else if (scrollMode) {
      // Leave scroll mode: put the pointing reference back so the cursor
      // carries on from where it was frozen instead of jumping
      scrollMode = false;
      pitchOffset += pitchAnchor - scrollStartPitch;
      yawAngle = yawAnchor = scrollSavedYaw;
      yawEuro.reset(scrollSavedYaw);
      freezeUntil = now + CLICK_FREEZE_MS;
    } else {
      recenter();  // released before SCROLL_HOLD_MS: a tap
    }
  }
  if (btnState[2] && !scrollMode && now - k1Start >= SCROLL_HOLD_MS) {
    scrollMode = true;
    Serial.println("Scroll mode on");
    scrollStartPitch = pitchAnchor;
    scrollSavedYaw = yawAnchor;
    scrollAcc = 0;
  }

  // Sensor -> cursor position
  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    float acc[3], g[3];
    if (readAll(acc, g)) {
      uint32_t nowMicros = micros();
      float dt = constrain((nowMicros - lastMicros) / 1e6, 0.001, 0.05);
      lastMicros = nowMicros;

      float dps[3];
      bool still = true;
      for (int a = 0; a < 3; a++) {
        dps[a] = (g[a] - gyroBias[a]) / 65.5;
        if (fabs(dps[a]) > STILL_DPS) still = false;
      }
      // Drift correction only after 1 s of real stillness, so slow
      // movements are never mistaken for sensor bias
      stillCount = still ? min(stillCount + 1, 1000) : 0;
      if (stillCount > 100)
        for (int a = 0; a < 3; a++) gyroBias[a] += 0.002 * (g[a] - gyroBias[a]);

      // How much to trust the accelerometer: at rest it reads exactly 1 g.
      // Pushing the board (whole-hand movement) adds acceleration, so the
      // further from 1 g, the less it is believed; past 0.25 g it is ignored.
      float accMag = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
      float accDev = fabs(accMag - 1.0);
      float accTrust = accDev > 0.25 ? 0 : 1.0 / (1.0 + 400.0 * accDev * accDev);
      if (now < ignoreAccUntil) accTrust = 0;  // button push shaking the board

      // Track "up" in board coordinates: the gyro turns it as the board
      // rotates (du/dt = -w x u), the accelerometer gently corrects it.
      float w[3];
      for (int a = 0; a < 3; a++) w[a] = dps[a] * PI / 180.0;
      float du0 = -(w[1] * up[2] - w[2] * up[1]);
      float du1 = -(w[2] * up[0] - w[0] * up[2]);
      float du2 = -(w[0] * up[1] - w[1] * up[0]);
      up[0] += du0 * dt;
      up[1] += du1 * dt;
      up[2] += du2 * dt;
      float k = 0.02 * accTrust;
      for (int a = 0; a < 3; a++) up[a] += k * (acc[a] / accMag - up[a]);
      float um = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
      for (int a = 0; a < 3; a++) up[a] /= um;

      // Left/right: rotation around TRUE vertical (gyro projected onto "up"),
      // so tilting or twisting the board during arm movement doesn't leak in.
      // (This also handles a flipped board automatically.)
      float vertRate = dps[0] * up[0] + dps[1] * up[1] + dps[2] * up[2];
      float yawRate = yawKf.update(vertRate);
      if (fabs(yawRate) > YAW_DEADZONE_DPS) yawAngle += yawRate * dt;
      yawAngle = constrain(yawAngle, -2 * YAW_RANGE_DEG, 2 * YAW_RANGE_DEG);
      float yawSmooth = yawEuro.filter(yawAngle, dt);  // adaptive anti-jitter
      follow(yawSmooth, yawAnchor);

      // Up/down: Kalman filter fusing gyro and accelerometer (no drift)
      float pitch = pitchKf.update(accelPitch(acc), orient * dps[PITCH_AXIS], dt, accTrust);
      follow(pitch, pitchAnchor);

      // Button held: stay locked until the board clearly turns (drag)
      if (holdLock && max(fabs(yawAnchor - pressYaw), fabs(pitchAnchor - pressPitch)) > DRAG_START_DEG)
        holdLock = false;

      // Only control the cursor while the board is really being moved, so
      // noise and slow drift never pull the cursor away from the touchpad
      if (fabs(dps[YAW_AXIS]) > ACTIVE_DPS || fabs(dps[PITCH_AXIS]) > ACTIVE_DPS) lastActive = now;
      bool active = (lastActive != 0 && now - lastActive < IDLE_MS) || now < controlUntil;

      uint16_t x = toPos(yawAnchor, YAW_SIGN, YAW_RANGE_DEG);
      uint16_t y = toPos(pitchAnchor - pitchOffset, PITCH_SIGN, PITCH_RANGE_DEG);
      histX[histIdx] = x;
      histY[histIdx] = y;
      histIdx = (histIdx + 1) % HIST_LEN;
      if (histCount < HIST_LEN) histCount++;
      if (scrollMode) {
        // Scroll: tilt from where scrolling started sets the scroll speed
        // (tilt up = scroll up); the cursor stays frozen
        float tilt = pitchAnchor - scrollStartPitch;
        float over = fabs(tilt) - SCROLL_DEADZONE_DEG;
        if (over > 0) {
          float rate = min(over * SCROLL_SPEED, SCROLL_MAX);
          scrollAcc += (tilt > 0 ? rate : -rate) * dt;
          int ticks = (int)scrollAcc;
          if (ticks) {
            scrollAcc -= ticks;
            if (lastX > POS_MAX || lastY > POS_MAX) lastX = lastY = POS_MAX / 2;
            sendReport(lastX, lastY, constrain(ticks, -127, 127));
          }
        } else {
          scrollAcc = 0;
        }
        return;
      }
      bool frozen = holdLock || now < freezeUntil;  // button held or just pressed/released
      // At most one position per connection interval: positions are absolute,
      // so the newest one is all that matters, and sending faster than the
      // link delivers would only queue up old positions (cursor lag)
      bool linkReady = now - lastSend >= max((float)REPORT_MS, connIntervalMs);
      if (active && !frozen && linkReady && (x != lastX || y != lastY)) sendReport(x, y);
    }
  }
}
