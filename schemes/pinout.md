# Pinout and bus map

Everything here was read straight out of the firmware we actually flash
(`open-challenge/firmware/esp32/esp32_carro.ino`), so it matches the car on the
table, not a plan we never built.

## ESP32 (hardware controller)

| Signal | ESP32 pin | Goes to | Notes |
|---|---|---|---|
| `RPWM` | GPIO 25 | IBT-2 (BTS7960) RPWM | Motor PWM, 20 kHz, 8-bit. 20 kHz keeps it out of the audible range |
| `LPWM` | GPIO 26 | IBT-2 (BTS7960) LPWM | The other direction |
| `R_EN` | GPIO 27 | IBT-2 R_EN | Held HIGH while the car is armed |
| `L_EN` | GPIO 33 | IBT-2 L_EN | Held HIGH while the car is armed |
| `SERVO` | GPIO 32 | MG996R signal | 50 Hz, 16-bit resolution, 500–2400 µs |
| `SDA` | GPIO 21 | MPU-6050 + TCS34725 | Shared I2C at 400 kHz |
| `SCL` | GPIO 22 | MPU-6050 + TCS34725 | Same bus |
| `INT_TCS` | GPIO 19 | TCS34725 INT | Open-drain, active LOW, internal pull-up. Optional |
| `INT_MPU` | GPIO 18 | MPU-6050 INT | Push-pull, active HIGH. Optional |
| `RX2` | GPIO 16 | Raspberry Pi GPIO 14 (TX) | Alternative link if we do not use USB |
| `TX2` | GPIO 17 | Raspberry Pi GPIO 15 (RX) | Crossed, 115200 baud |

I2C addresses: MPU-6050 at `0x68` (or `0x69`), TCS34725 at `0x29`.

The two INT lines are optional on purpose. If we do not wire them the firmware
notices by itself and falls back to polling — slower and with more jitter, but
the car still runs. That was deliberate: during a competition weekend a broken
jumper should cost us precision, not the round.

## Raspberry Pi 5

| Port | Connected to |
|---|---|
| USB 2.0 | WN-L1812.K56R camera (IMX179 sensor, UVC, MJPG) |
| USB 2.0 | ESP32 (this is the link we normally use: data + power for the ESP32) |
| GPIO 14 / 15 (UART) | ESP32 GPIO 16 / 17, only if the USB link is not used |
| USB-C | 5 V input |

The firmware listens on **both** mouths at the same time and answers through the
one the last valid frame came from, so the same binary works over USB or over
the GPIO UART with no recompile.

## Link protocol

Binary frames with CRC, firmware version 3. Commands go Pi → ESP32 at ~100 Hz,
telemetry comes back at 20 Hz (motor/servo state) and 40 Hz (yaw + colour).
Line crossings travel as **counters**, not as events, so a dropped frame never
loses a crossing.

If the Pi goes quiet for 300 ms the ESP32 stops the car on its own. That
failsafe is the whole reason the ESP32 sits in the middle instead of hanging the
driver straight off the Pi.

## Still to confirm

> **TODO (team):** the power side of the table — battery chemistry and capacity,
> the regulator feeding the Pi, the rail feeding the servo, and the fuse (if
> any). Fill in `power-budget.md` and the power block of `wiring-diagram.svg`.

> **TODO (team):** the physical **start button**. The rules allow one switch and
> one start button, and no wireless while the vehicle is running, so the ARM
> button in the web panel cannot be what starts the round in competition. Decide
> the pin, wire it, and document it here.
