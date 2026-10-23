# ESP32 motor / IMU board

The same board firmware as [`firmware/teensy41`](../teensy41/README.md) for a classic **ESP32 dev
board** (ESP32-WROOM-32 DevKit). Protocol, wheel controller, safety behaviour, `rover_cli.py`, the
ROS plugin and `real_hardware.yaml` are shared and unchanged; only the pins, the peripherals and the
serial link differ. Read the Teensy README for the background (protocol table, motor driver
behaviour, bring-up steps); this page only lists what is different.

## What differs from the Teensy

| | Teensy 4.1 | ESP32 |
|---|---|---|
| host link | native USB, rate ignored | UART0 through the CP210x / CH340 chip at **921600 baud** (`baud_rate` in `real_hardware.yaml`) |
| opening the port | nothing happens | **resets the board** (auto-reset circuit); tools wait for the reboot (handshake up to 3 s) |
| encoders | `Encoder` library, interrupts | PCNT hardware counters with a 12.8 us glitch filter (`ESP32Encoder`) |
| PWM | 12 bit, 20 kHz | LEDC, 10 bit, 20 kHz (20 kHz x 2^bits must stay below 80 MHz) |
| watchdog | WDOG1, 0.5 s | task watchdog, 1 s, plus a 10 ms timer task that switches the driver off after 150 ms |
| `rover_cli.py bootloader` | reboots into the Teensy bootloader | only restarts; `pio run -t upload` resets into the bootloader by itself |
| "no host" detection | USB DTR | none: telemetry always streams; the LED flashes when no byte arrived for 3 s |
| logic level | 3.3 V, not 5 V tolerant | 3.3 V, not 5 V tolerant |

After the move, **run `rover_cli.py check` again**: the encoder count direction of `ESP32Encoder` may
be the opposite of the Teensy library's, which only means flipping the `invert_*_encoder` flags in
`real_hardware.yaml`.

## Wiring (ESP32-WROOM-32 DevKit)

| ESP32 GPIO | goes to | notes |
|---|---|---|
| 32, 33 | left encoder A, B | |
| 25, 26 | right encoder A, B | |
| 3V3, GND | both encoders VCC, GND | **3.3 V, never 5 V** |
| 21 (SDA), 22 (SCL) | GY-521 SDA, SCL | GY-521 VCC → 3V3, AD0 → GND, own pull-ups |
| 16, 17 | left module RPWM, LPWM | not on WROVER modules (PSRAM uses GPIO16/17) |
| 18, 19 | right module RPWM, LPWM | |
| 23 | R_EN + L_EN of **both** modules (4 pins tied) | **10 kΩ pull-down to GND** keeps the bridges off while booting |
| 3V3 | VCC of both modules | logic supply 3.3 V, as with the Teensy |
| GND | module GND, battery −, GY-521 GND | common ground |
| 2 | on-board LED | status |
| 34 | battery divider (optional) | input only, ADC1; set `BATTERY_DIVIDER_RATIO` |

Avoided on purpose: GPIO 0, 5, 12, 15 (strapping pins), 6-11 (flash), 1 and 3 (UART0 = host link),
34-39 as encoder inputs (no pull-ups). Pins and driver settings live in
[`src/config.h`](src/config.h).

The ESP32's 3V3 output (the board's regulator, a few hundred mA) is enough for two encoders, the
GY-521 and the modules' logic. Do not feed it from the 5 V pin through anything but a regulator.

## Switching the robot to the ESP32

1. Udev rule (stable port name `/dev/jgb_rover_mcu`, one such board at a time):
   ```bash
   sudo cp firmware/esp32/99-jgb-rover-esp32.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   (CH340 clones on Ubuntu are sometimes grabbed by `brltty`; if no `/dev/ttyUSB0` appears, remove it.)
2. `ros2_ws/src/jgb_rover_description/config/real_hardware.yaml`: set `mcu.board`, `mcu.baud_rate: 921600`
   and, if you skip the udev rule, `mcu.serial_port: /dev/ttyUSB0`. Rebuild the workspace.
3. Build and flash (no button needed on most boards, hold BOOT if the upload waits):
   ```bash
   cd firmware/esp32
   pio run -t upload
   ```
4. Bench test with the wheels off the ground, exactly as in the Teensy README:
   `rover_cli.py info`, `imu`, `check`, `vel 3 3`, `step 6 --plot`, `monitor`. `--baud` overrides the
   yaml value for one run.

To go back to the Teensy: `baud_rate: 115200` (ignored there) and the Teensy udev rule.

## Notes

- UART0 also carries the ROM boot messages (115200 baud, garbage at 921600) for a moment after every
  reset. The COBS framing resynchronises; `rx_err` may count a frame or two after boot.
- `main.cpp` is a copy of the Teensy `main.cpp` with the link and loop timing adapted; the
  controller, protocol and safety logic come from the shared `firmware/common` and `firmware/protocol`.
  A fix in one `main.cpp` has to be made in the other.
- ESP32-S3 / C3 boards (native USB) need a different pin table and `Serial` handling; not done here.
- Not tested on hardware yet: the firmware builds without warnings, the host side (baud rate, ROS
  plugin, `rover_cli.py`) is tested against the fake board.

## Files

```
firmware/esp32/
  platformio.ini                   build settings (esp32dev, Arduino core), shared code as libraries
  src/config.h                     pins, baud rate, PWM, IMU axes, defaults
  src/main.cpp                     100 Hz loop, link handling, command timeout
  src/hw.{h,cpp}                   encoders (PCNT), PWM motors (LEDC), MPU6050, battery ADC, LED, watchdogs
  99-jgb-rover-esp32.rules         udev rule -> /dev/jgb_rover_mcu for CP210x / CH340
```
