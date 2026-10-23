# Real robot quick start

From a bare Teensy 4.1 to driving the robot with the keyboard. Details and background are in
[`firmware/teensy41/README.md`](firmware/teensy41/README.md) and the main [README](README.md#sim-to-real).
Using an ESP32 instead of the Teensy: see [`firmware/esp32/README.md`](firmware/esp32/README.md).

**Safety: lift the robot so the wheels are off the ground until step 6.**

---

## 1. Wire it

| Teensy 4.1 pin | goes to |
|----------------|---------|
| 20, 3 | left encoder A, B (pin 2 is damaged: don't use it) |
| 4, 5 | right encoder A, B |
| 3.3V, GND | both encoders VCC, GND (**3.3 V, never 5 V**) |
| 25, 24 | GY-521 SDA, SCL (GY-521 VCC → 3.3V, AD0 → GND) |
| 6, 7 | left module RPWM, LPWM |
| 8, 9 | right module RPWM, LPWM |
| 10 | R_EN + L_EN of **both** BTS7960 modules, plus a 10 kΩ resistor to GND |
| 3.3V | VCC of both modules |
| GND | GND of both modules and battery − |

BTS7960 modules: B+ / B− to the 12 V battery, M+ / M− to the motor. Leave R_IS / L_IS unconnected.
Mount the GY-521 flat, X arrow pointing forward.

## 2. Install (once)

```bash
# firmware build tool (downloads the Teensy toolchain itself) + bench tool dependency
python3 -m pip install --user platformio
sudo apt install python3-serial
curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules | sudo tee /etc/udev/rules.d/00-teensy.rules

# ROS packages for the real robot
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-v4l2-camera

# stable device name /dev/jgb_rover_mcu + serial port permission
sudo cp firmware/teensy41/99-jgb-rover-mcu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER     # then log out and back in
```

## 3. Build and flash the firmware

Plug the Teensy in over USB, then:

```bash
cd firmware/teensy41
pio run -t upload       # builds and flashes; press the button on the Teensy if it asks
```

The board restarts and its orange LED gives a **short flash every 2 s** (no program has the port
open yet). Later uploads work the same way, no button needed.

## 4. Test the board (no ROS)

Run from the repository root. Every command reads `ros2_ws/src/jgb_rover_description/config/real_hardware.yaml`.

```bash
firmware/tools/rover_cli.py info     # firmware version, IMU found? (WHO_AM_I not 0x00)
firmware/tools/rover_cli.py imu      # robot still: accel z must be about +9.81
firmware/tools/rover_cli.py check    # spins each wheel, asks if it went forward, says what to flip
firmware/tools/rover_cli.py vel 3 3  # both wheels at 3 rad/s: "vel" should settle at 3.00
```

If `check` says to flip something, change `invert_*` under `wiring:` in `real_hardware.yaml` and run
`check` again until it says **directions OK**.

## 5. Build the ROS workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
colcon build --symlink-install
source install/setup.bash
```

Rebuild after editing `real_hardware.yaml` or `robot_spec.yaml`.

## 6. First drive test: 1 m out and back

Put the robot on the floor with about 1.5 m of free space in front. Put a piece of tape on the
floor at the middle between the two drive wheels, and one in front of the robot, so you can see
where it started. Terminal 1 (drive stack only, no camera):

```bash
ros2 launch jgb_rover_bringup bringup_real.launch.py camera:=false
```

**Do not touch the robot for 3 s** (the gyro calibrates). You should see `MCU board configured`.

Terminal 2:

```bash
source ros2_ws/install/setup.bash
ros2 run jgb_rover_bringup out_and_back.py
```

It waits for the gyro calibration and asks before moving. Then it drives 1 m out, turns 180°,
drives 1 m back and turns 180° again, at 0.15 m/s. **Ctrl-C stops it at any moment.** It also
stops on its own if a move takes too long (wheel stuck, link lost).

What to look at:

- **Your tape marks.** It should end within a few cm and a few degrees of where it started.
  That gap is the real error of the whole chain: encoders, gyro, EKF and controller.
- **TURNS line.** It compares the wheel-odometry angle with the gyro. Above 2 % it suggests a
  corrected `drive.wheel_separation` for `robot_spec.yaml`.
- **Wheel radius.** Run `out_and_back.py --straight-only`, measure the distance driven with a tape
  and type it in. It prints the effective `drive.wheel_radius` (the spec expects about 0.042 m
  instead of the nominal 0.0435 m).

After editing `robot_spec.yaml`, rebuild (step 5) and run the test again. Options: `--distance`,
`--speed`, `--turn-rate`, `--yes` (no question before driving).

## 7. Drive by keyboard

With the launch from step 6 still running:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Keys `i` forward, `,` back, `j` / `l` turn, `k` stop. Hold the key: the robot stops 0.5 s after the
last command.

Check that everything runs:

```bash
ros2 topic hz /imu/data_raw          # ~100 Hz
ros2 topic hz /wheel/odom            # ~50 Hz
ros2 topic echo /odometry/filtered --field pose.pose.position   # changes as you drive
```

With the camera (calibrate it first, see the main README): drop `camera:=false` to also get the
floor scan and the SLAM map.

---

## When something does not work

| symptom | fix |
|---------|-----|
| no `/dev/ttyACM0` or `/dev/jgb_rover_mcu` | charge-only USB cable, or no firmware yet / still in the bootloader (red LED next to the button): run `pio run -t upload` |
| upload hangs at "waiting for Teensy" | press the button on the Teensy; check `00-teensy.rules` is installed |
| `Permission denied` on the port | `dialout` group: log out and in after `usermod` |
| `board did not answer` / `no handshake` | another program has the port open (close `rover_cli.py monitor`), or firmware not flashed |
| LED **blinks fast** or IMU `0x00` | GY-521 wiring: SDA pin 25, SCL pin 24, VCC 3.3V, AD0 to GND |
| encoder `did not count` | encoder power (3.3V + GND) and A/B wires (left 20, 3 / right 4, 5) |
| wheel never turns | pin 10 to all four EN pins, battery on B+ / B−, module VCC at 3.3V |
| wheel turns the wrong way | `rover_cli.py check`, then fix `invert_*` in `real_hardware.yaml` |
| robot stops after a moment | keep the teleop key pressed (commands time out after 0.5 s) |
| `out_and_back.py` says `STOPPED: motion did not finish in time` | wheel blocked or not turning: check with `rover_cli.py vel 3 3`; if the robot crawls, raise `deadband_duty` |
| speed wobbles or is slow to react | tune `wheel_controller` with `rover_cli.py step 6 --plot` (see the firmware README) |
| `/imu/data` silent | the robot moved during the first 3 s: keep it still, it retries |

Try without hardware: `firmware/tools/fake_board.py` prints a port like `/dev/pts/5`; use it as
`--port /dev/pts/5` for `rover_cli.py` or `serial_port:=/dev/pts/5` for the launch file.
