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
| "no host" detection | USB DTR | none: telemetry always streams |
| status LED, battery ADC | pin 13 LED, A0 | not wired (no free pin on the one-header layout) |
| logic level | 3.3 V, not 5 V tolerant | 3.3 V, not 5 V tolerant |

After the move, **run `rover_cli.py check` again**: the encoder count direction of `ESP32Encoder` may
be the opposite of the Teensy library's, which only means flipping the `invert_*_encoder` flags in
`real_hardware.yaml`.

## Wiring (HW-394)

Target board: **HW-394** (ESP32-WROOM-32 module, CH340 USB-UART chip, USB-C). Encoders and motors
are on the header that carries **3V3 and GND** (D23, D22, TX0, RX0, D21, D19, D18, D5, TX2, RX2, D4,
D2, D15, GND, 3V3); the GY-521 is on D32 / D33 of the other header (D21 / D22 stay free). The board shows up as `/dev/ttyUSB0`. Some cheap clones have
no auto-reset circuit: if an upload waits at "Connecting...", hold the BOOT button until it starts.

| board label (GPIO) | goes to | notes |
|---|---|---|
| RX2 (16), TX2 (17) | left encoder A, B | |
| D18 (18), D19 (19) | right encoder A, B | |
| 3V3, GND | both encoders VCC, GND | **3.3 V, never 5 V** |
| D32 (32), D33 (33) | GY-521 **SDA, SCL** (SDA on 32) | GY-521 VCC → 3V3, AD0 → GND, own pull-ups; checked with a wiring test: 0x68 answers on SDA 32 / SCL 33 |
| D4 (4), D5 (5) | left module RPWM, LPWM | |
| D15 (15), D2 (2) | right module RPWM, LPWM | D2 also drives the on-board LED, which flickers with the PWM |
| D23 (23) | R_EN + L_EN of **both** modules (4 pins tied) | **10 kΩ pull-down to GND** keeps the bridges off while booting |
| 3V3 | VCC of both modules | logic supply 3.3 V, as with the Teensy |
| GND | module GND, battery −, GY-521 GND | common ground |
| TX0, RX0 | nothing | the link to the PC (through the CH340), do not use |

There is **no status LED and no battery ADC** (both are switched off with `-1` in `src/config.h`;
the status flags are in `rover_cli.py monitor`). D21 / D22 are unused: they are the usual I2C pins
and can be used for the GY-521 instead (`PIN_I2C_SDA` / `PIN_I2C_SCL`).

- GPIO 2, 5 and 15 are strapping pins and are used as **outputs** on purpose: while the chip boots,
  the driver is disabled (D23 held low by its pull-down), so whatever those pins do at boot reaches
  no motor. Never put an encoder or other input that idles high on GPIO2: it breaks uploads.
- If an upload fails with a wrong-boot-mode error, unplug the wire on D2 (and D5) and upload again.
- GPIO16/17 are PSRAM pins on WROVER modules; the HW-394 uses a WROOM module, where they are free.

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
  src/config.h                     pins (one header), baud rate, PWM, IMU axes, defaults
  src/main.cpp                     100 Hz loop, link handling, command timeout
  src/hw.{h,cpp}                   encoders (PCNT), PWM motors (LEDC), MPU6050, battery ADC, LED, watchdogs
  99-jgb-rover-esp32.rules         udev rule -> /dev/jgb_rover_mcu for CP210x / CH340
```
