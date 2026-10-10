# glyph_h2_air_mouse

Board: PCB Cupid Glyph H2 (ESP32-H2) — https://learn.pcbcupid.com/

## Sketches
- `glyph_h2_air_mouse/` — **LATEST** Wii-remote style Bluetooth air mouse: point the flat board, swing left/right (30° = screen width) / tip up/down (24° = screen height). It sends relative movement like a normal mouse, so the cursor always carries on from where it is, even after the touchpad moved it. Up/down uses a 2-state Kalman filter (gyro + accel, no drift); left/right is the gyro rate around true vertical (tilt-compensated), Kalman-filtered, integrated to an angle and smoothed by a One Euro filter. The accelerometer is trusted less while the board is being pushed (whole-arm movement). Buttons: K3 = left click (hold = drag), K2 (middle) = right click, K1 tap = re-centre the swing range, K1 hold = scroll mode (tilt up/down to scroll, further = faster). Clicks are stabilized (click lands where the cursor was just before the press; cursor stays locked while a button is held until the board turns more than 3°, then it drags; cursor freezes 250 ms around press/release; a second click within 0.7 s lands on the same spot, so double-click works). Lying still it sends nothing, so the touchpad works. Works buttons-up or buttons-down (detected from gravity at RESET).

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

## Sleep
After 5 minutes without movement or buttons (or 2 minutes without a connection) the board goes into deep sleep, with the MPU6050 asleep too. Press any button to wake it: it restarts, calibrates (keep it still ~1 s) and reconnects by itself. Times are `SLEEP_IDLE_MIN` / `SLEEP_UNCONNECTED_MIN` in the sketch.

## Bluetooth
Default name "Glyph H2 Mouse" (change it with `NAME`). Pair it from the laptop's Bluetooth settings ("Just Works", no PIN); once trusted it reconnects automatically. After changing the name or the HID report format, remove the device on the laptop and pair again.

Cursor speed is set by `SCREEN_W_PX` / `SCREEN_H_PX` (default 1366 x 768). If the cursor speed feels uneven, turn off mouse acceleration on the laptop.
