# glyph_h2_air_mouse

Board: PCB Cupid Glyph H2 (ESP32-H2) — https://learn.pcbcupid.com/

## Sketches
- `glyph_h2_air_mouse/` — **LATEST** Wii-remote style pointer (absolute): point the flat board, swing left/right (±15°) / tip up/down (±12°); straight ahead = screen centre. Up/down uses a 2-state Kalman filter (gyro + accel, no drift); left/right is the gyro rate around true vertical (tilt-compensated), Kalman-filtered, integrated to an angle and smoothed by a One Euro filter (may drift slowly). The accelerometer is trusted less while the board is being pushed (whole-arm movement). Buttons: K3 = left click (hold = drag), K2 (middle) = right click, K1 tap = re-centre when released (the direction you point now = screen centre, cursor jumps there; for 5 s afterwards even slow aiming is followed; 60 ms debounce), K1 hold (0.4 s) = scroll mode (cursor stays where it is; tilt up/down to scroll, further = faster). Clicks are stabilized (click lands where the cursor was just before the press; cursor stays locked while a button is held until the board turns more than 3°, then it drags; cursor freezes 150 ms around press/release; a second click within 0.4 s lands on the same spot, so double-click works). Lying still it sends nothing, so the touchpad works. For 0.3 s around every button press/release, up/down runs on the gyro alone, so the thumb push can't tilt the cursor. Works buttons-up or buttons-down (detected from gravity at RESET). Keep it still ~1 s after power-up while it calibrates.

## MPU6050 wiring
| MPU6050 | Glyph H2 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | IO23 |
| SCL | IO24 |
| AD0 | IO1 (driven LOW → address 0x68) |

## Button module (1x3) wiring
| Module | Glyph H2 |
|---|---|
| VCC | 3.3V |
| GND | IO14 (driven LOW in code, acts as ground) |
| D1 (K1) | IO13 |
| D2 (K2) | IO12 |
| D3 (K3) | IO10 |

Pressed level is auto-detected at startup, so don't hold a button while pressing RESET.

## Build & flash
Core: `esp32:esp32` 3.3.12. "USB CDC On Boot" must be enabled because IO23/24 (UART0) are used for I2C.

```
cd glyph_h2_air_mouse
arduino-cli compile --upload -p /dev/ttyACM0 -b esp32:esp32:esp32h2:CDCOnBoot=cdc .
```

After flashing, **press RESET once**: on this board, software resets over USB leave it in download mode
("waiting for download"). Close any Serial Monitor before uploading, or the port is busy.

## Serial commands (115200 baud, newline)
| Command | Action |
|---|---|
| `NAME` | Show the Bluetooth name |
| `NAME My Mouse` | Set the Bluetooth name (1–20 characters, saved in flash), then press RESET |
| `SLEEP` | Go to sleep now |

Serial output never blocks: with the board on USB and no program reading the port, messages are dropped instead of making the board wait (otherwise each message could freeze it for up to 2 s, e.g. the cursor hanging in the middle after K1).

At every start the board also prints `Last restart: …` (power on, reset pin, crash, watchdog, power dip). Use it if the board ever restarts by itself.

## Sleep
The board never sleeps while the laptop is connected. After 2 minutes without a connection it goes into deep sleep, with the MPU6050 asleep too. Press any button to wake it: it restarts, calibrates (keep it still ~1 s) and reconnects by itself. The time is `SLEEP_UNCONNECTED_MIN` in the sketch.

## Bluetooth
Default name "Glyph H2 Mouse" (change it with `NAME`). Pair it from the laptop's Bluetooth settings ("Just Works", no PIN); once trusted it reconnects automatically. After changing the name, remove the device on the laptop and pair again.

On connect the board asks the laptop for a fast connection interval (7.5–15 ms) and asks again up to 3 times if the laptop doesn't apply it; the interval it gets is printed on the serial port ("Connection interval: … ms"). The board sends at most one cursor position per connection interval, always the newest, so positions never queue up and the cursor doesn't lag behind the board. Transmit power is the maximum (+20 dBm).

K1 sends a position 1 unit off-centre and then the exact centre: Linux drops an absolute position equal to the last one a device sent, so without the extra step a K1 press could be ignored until the board moved.

If the cursor freezes or lags now and then, the cause is usually the computer's Bluetooth sharing radio time with 2.4 GHz Wi-Fi; Wi-Fi off, Wi-Fi on 5 GHz or a USB Bluetooth 5 adapter fixes it.
