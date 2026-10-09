# RP2040 motor / IMU board

Firmware for a **Waveshare RP2040-Zero** that drives the two JGB37-520 motors, reads their Hall
encoders and the MPU6050, and talks to the ROS 2 computer over USB. It is connected to the
development PC now and later to the Raspberry Pi 4 with the same USB cable; nothing changes on
either side. The temperature sensor is not on this board (it goes to the Pi's I2C, see the main
README).

```
 ROS 2 computer (PC now, Pi later)                         RP2040-Zero
 ┌───────────────────────────────────────────┐   USB    ┌──────────────────────────────────┐
 │ diff_drive_controller   imu_sensor_bcast. │   CDC    │ 100 Hz loop:                     │
 │        │ velocity cmd        ▲ imu_sensor │◀────────▶│  encoders (PIO) ─▶ velocity est. │
 │        ▼                     │            │ rover_   │  PI + feed-forward ─▶ PWM / DIR  │
 │  jgb_rover_hardware/JgbRoverSystem        │ link     │  MPU6050 (I2C) ─▶ gyro, accel    │
 │  (ros2_control plugin, McuLink)           │          │  telemetry frame ─▶ host         │
 └───────────────────────────────────────────┘          └──────────────────────────────────┘
```

The board closes the wheel-speed loop itself (100 Hz, PI + feed-forward). The host sends wheel
velocity targets in rad/s and gets back encoder counts, estimated wheel velocities and the IMU
sample. Gains, motor/encoder directions and the command timeout live in
[`real_hardware.yaml`](../../ros2_ws/src/jgb_rover_description/config/real_hardware.yaml) and are
sent to the board on every connect, so tuning needs no reflash.

## Safety behaviour

- **Command timeout** (`cmd_timeout_ms`, 250 ms): no velocity command → motors off (driver
  disabled), integrators cleared. Closing the serial port stops the motors immediately.
- **Hardware watchdog** (200 ms): if the loop hangs the chip resets; all motor pins go back to
  inputs. Put a **10 kΩ pull-down from GP9 to GND**: it holds R_EN / L_EN of both BTS7960 modules
  low, so both bridges stay off while the RP2040 boots, is being flashed or is unplugged.
- No commands are sent by the host until the board has confirmed the config from
  `real_hardware.yaml`, so a mirrored motor never runs with the wrong default direction.
- A board reset while running is detected (telemetry sequence restarts); the host re-sends the
  config and keeps the wheel angles continuous.

## Wiring

All signals are 3.3 V. **The RP2040 is not 5 V tolerant.**

| RP2040-Zero | to | notes |
|-------------|----|-------|
| GP2, GP3 | left encoder A, B | A and B must be on consecutive pins (PIO decoder) |
| GP4, GP5 | right encoder A, B | |
| 3V3, GND | encoder VCC, GND (both motors) | **power the encoders from 3V3**: their outputs are pulled up to their own supply |
| GP6 (SDA), GP7 (SCL) | GY-521 SDA, SCL | I2C1, 400 kHz. GY-521 VCC → 3V3, AD0 → GND (address 0x68); the GY-521 has its own pull-ups |
| GP8 | GY-521 INT | optional, not used yet |
| GP9 | R_EN **and** L_EN of **both** BTS7960 modules (4 pins tied together) | high = bridges enabled; 10 kΩ pull-down to GND |
| GP12, GP13 | left module RPWM, LPWM | |
| GP14, GP15 | right module RPWM, LPWM | |
| 3V3 | VCC of both modules | logic supply: use 3.3 V, not 5 V (see below) |
| (none) | R_IS, L_IS | leave unconnected for now (see below) |
| GP10, GP11 | free | only used with `MOTOR_DRIVER_PWM_DIR` drivers |
| GP26 | battery + through 100 kΩ / 22 kΩ divider | optional; set `BATTERY_DIVIDER_RATIO` |
| GP0, GP1 | free | reserved for a UART link / debug |
| GP16 | on-board RGB LED | status, see below |
| GND | module GND pins, battery − | common ground |
| USB-C | PC / Pi | power + data |

**Motor driver: 2 × BTS7960 module (IBT-2, 43 A).** One module per motor: B+ / B− to the battery
(6–27 V, so the 12 V AGM is fine), M+ / M− to the motor. Firmware setting
`MOTOR_DRIVER_PWM_PWM` in [`src/config.h`](src/config.h) (the default):

| wheel command | RPWM | LPWM | R_EN = L_EN (GP9) |
|---------------|------|------|-------------------|
| forward (positive duty) | PWM 20 kHz | low | high |
| reverse | low | PWM | high |
| zero, `idle_brake: true` | high | high | high (both motor terminals at B+: brake) |
| zero, `idle_brake: false` | low | low | high |
| stopped by timeout / no host / boot | low | low | **low** (both half-bridges off: coast) |

- Despite some shop descriptions, **R_EN / L_EN do not set the direction**: they enable the two
  half-bridges. Tie all four together to GP9. Direction comes from which of RPWM / LPWM carries the PWM.
- **VCC = 3.3 V.** The module buffers its inputs with a 74HC244 powered from VCC; at VCC = 5 V a
  3.3 V "high" is below its guaranteed input-high level.
- **R_IS / L_IS** are current-sense / fault outputs of the BTS7960. In a fault they can rise above
  3.3 V, so do not wire them straight to an RP2040 pin. Over-current and over-temperature
  protection works inside the chip anyway. Reading them later needs a divider to an ADC pin
  (GP27–GP29 are free).
- If the "forward" wheel turns backwards, either swap M+ / M− or set `invert_*_motor` in
  `real_hardware.yaml` (`rover_cli.py check` tells you which).

Other drivers: `MOTOR_DRIVER_PWM_DIR` for one PWM plus IN1/IN2 per motor (TB6612FNG, L298N;
PWM on GP10/GP11, IN1/IN2 on GP12–GP15). For an L298N lower `MOTOR_PWM_FREQ_HZ` to a few kHz.

**IMU orientation.** `robot_spec.yaml` places the GY-521 with its X arrow forward and Z up. If it is
mounted differently, set `IMU_AXIS_MAP` / `IMU_AXIS_SIGN` in `config.h` (board axes → `imu_link`);
`rover_cli.py imu` shows whether gravity comes out on +Z.

**Status LED.** Blue blink: waiting for commands. Green: driving. Yellow: IMU not found (host
connected). Red blink: IMU not found, no host. White: booting. If red and green look swapped, set
`STATUS_LED_GRB 0`.

## Build

Needs the Pico SDK 2.x and an `arm-none-eabi` GCC:

```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
git clone -b 2.1.1 https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
git -C ~/pico-sdk submodule update --init lib/tinyusb
export PICO_SDK_PATH=~/pico-sdk

cd firmware/rp2040
cmake -B build && cmake --build build -j
# -> build/jgb_rover_mcu.uf2
```

The first configure downloads and builds `picotool` (the SDK needs it to make the `.uf2`).
Installing picotool system-wide avoids that.

## Flash

1. Hold **BOOT**, plug in USB (or press RESET while holding BOOT). A drive `RPI-RP2` appears.
2. Copy `build/jgb_rover_mcu.uf2` onto it. The board reboots into the firmware.

Later updates without pressing buttons: `firmware/tools/rover_cli.py bootsel`, then copy the
`.uf2` (or `picotool load -x build/jgb_rover_mcu.uf2`).

The board then appears as `/dev/ttyACM0` (USB product `jgb_rover_mcu`). Install the udev rule for
a stable name and permissions:

```bash
sudo cp firmware/rp2040/99-jgb-rover-mcu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER      # log out and in once
ls -l /dev/jgb_rover_mcu
```

Without the rule the driver and the CLI fall back to `/dev/ttyACM0`.

## Bring-up, step by step (no ROS needed)

`firmware/tools/rover_cli.py` talks to the board directly (`sudo apt install python3-serial`). It
sends the config from `real_hardware.yaml` first, like the ROS driver does. **Lift the wheels off
the ground** for steps 3 to 5.

1. `rover_cli.py info`: firmware version, board ID, IMU `WHO_AM_I` (0x68 for a genuine MPU6050;
   clones answer 0x70 / 0x72 / 0x98 and usually work too; 0x00 = not found).
2. `rover_cli.py imu`: robot still on the floor. Gravity must come out as **accel z ≈ +9.81**,
   x and y near 0. The gyro std values are the real noise; put them into
   `robot_spec.yaml` `imu.sim_noise.gyro_stddev` / `accel_stddev` (used by `imu_bias_calibration`).
3. `rover_cli.py check`: pulses each wheel at 30 % duty and asks whether it turned forward. It
   prints which `invert_*` flags in `real_hardware.yaml` to change.
4. `rover_cli.py vel 3 3`: both wheels at 3 rad/s, closed loop. Velocities should settle at 3.00.
5. `rover_cli.py step 6 --plot`: speed step. Tune `wheel_controller` in `real_hardware.yaml`:
   - `kff` ≈ steady-state duty / target speed (printed). With a good `kff` the PI terms only correct.
   - `deadband_duty`: raise until a slow command (`vel 0.5 0.5`) turns the wheel smoothly
     on the ground.
   - `kp`: raise for a faster rise, lower on oscillation. `ki`: raise if a steady error lingers.
   - Try one-off values without editing the file: `rover_cli.py --kp 0.08 step 6`.
   - Repeat on the ground (robot loaded); that is what matters.
6. `rover_cli.py monitor`: live counts, velocities, IMU, error counters. `rx_err` / `drop` should
   stay constant.

Then the ROS side ([main README, Sim to real](../../README.md#sim-to-real)):

```bash
ros2 launch jgb_rover_bringup bringup_real.launch.py camera:=false   # wheels + IMU + EKF
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

`ros2 run jgb_rover_hardware mcu_link_probe /dev/jgb_rover_mcu 2 1.0 1.0` exercises the C++ link
layer alone (handshake, 2 s at 1 rad/s, prints the wheel angles).

## Without hardware

`firmware/tools/fake_board.py` creates a pseudo-terminal that behaves like the board (ideal
wheels). It prints the device path; use it as `--port` for `rover_cli.py` or as `serial_port:=` for
the launch file.

Tests (no board needed):

```bash
python3 -m pytest firmware/test     # C vs Python framing, wheel controller vs a motor model
colcon test --packages-select jgb_rover_hardware   # C++ link layer vs fake_board.py
```

## Protocol

[`firmware/protocol/rover_link.h`](../protocol/rover_link.h) is the reference; it is compiled into
both the firmware and the ROS plugin, and `firmware/tools/rover_link.py` mirrors it. Frames are
COBS-encoded with a CRC-16 and end with `0x00`, so either side resynchronises after garbage or a
partial frame. All values are little-endian.

| type | direction | payload | when |
|------|-----------|---------|------|
| `0x01` CMD_VEL | host → board | 2 × f32 wheel velocity [rad/s] | every control cycle |
| `0x02` CMD_PWM | host → board | 2 × f32 duty (−1..1), open loop | bench tests |
| `0x03` SET_CONFIG | host → board | `rl_config_t` (28 B) | connect; answered with CONFIG_ACK |
| `0x04` PING | host → board | none | connect; answered with INFO + CONFIG |
| `0x05` STOP | host → board | none | motors off now |
| `0x06` BOOTSEL | host → board | none | reboot into the USB bootloader |
| `0x81` TELEMETRY | board → host | `rl_telemetry_t` (76 B) | 100 Hz |
| `0x82` INFO | board → host | `rl_info_t` (20 B) | reply to PING, on USB connect |
| `0x83` CONFIG | board → host | `rl_config_t` in use | reply to PING |
| `0x84` LOG | board → host | text | IMU lost / recovered, command timeout |
| `0x85` CONFIG_ACK | board → host | `rl_config_t` as applied (clamped) | reply to SET_CONFIG |

Positive wheel values always mean "this wheel drives the robot forward" (the URDF wheel joints both
rotate about +Y). Directions are fixed on the board with the `invert_*` flags.

A UART link to the Pi (GP0/GP1) would only need a different byte transport in `main.c`; the
framing already copes with a lossy link.

## Files

```
firmware/
  protocol/rover_link.{h,c}    wire protocol + COBS/CRC framing (firmware and ROS plugin)
  rp2040/
    CMakeLists.txt             Pico SDK build, USB product name
    src/config.h               pins, motor driver type, IMU axes, defaults
    src/main.c                 100 Hz loop, link handling, watchdogs
    src/control.{h,c}          velocity estimate + PI/feed-forward (portable, unit tested)
    src/hw.{h,c}               PIO encoders, PWM motors, MPU6050, battery ADC, status LED
    pio/                       quadrature_encoder.pio, ws2812.pio (from pico-examples, BSD-3)
    99-jgb-rover-mcu.rules     udev rule -> /dev/jgb_rover_mcu
  tools/
    rover_cli.py               bench tool (info, imu, check, vel, pwm, step, monitor, bootsel)
    rover_link.py              Python protocol
    fake_board.py              board simulator on a pty
  test/                        host tests (pytest + small C programs)
```
