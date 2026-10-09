# Real robot quick start

From a bare RP2040-Zero to driving the robot with the keyboard. Details and background are in
[`firmware/rp2040/README.md`](firmware/rp2040/README.md) and the main [README](README.md#sim-to-real).

**Safety: lift the robot so the wheels are off the ground until step 6.**

---

## 1. Wire it

| RP2040-Zero | goes to |
|-------------|---------|
| GP2, GP3 | left encoder A, B |
| GP4, GP5 | right encoder A, B |
| 3V3, GND | both encoders VCC, GND (**3.3 V, never 5 V**) |
| GP6, GP7 | GY-521 SDA, SCL (GY-521 VCC → 3V3, AD0 → GND) |
| GP9 | R_EN + L_EN of **both** BTS7960 modules, plus a 10 kΩ resistor to GND |
| GP12, GP13 | left module RPWM, LPWM |
| GP14, GP15 | right module RPWM, LPWM |
| 3V3 | VCC of both modules |
| GND | GND of both modules and battery − |

BTS7960 modules: B+ / B− to the 12 V battery, M+ / M− to the motor. Leave R_IS / L_IS unconnected.
Mount the GY-521 flat, X arrow pointing forward.

## 2. Install (once)

```bash
# firmware toolchain
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential python3-serial
git clone -b 2.1.1 https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
git -C ~/pico-sdk submodule update --init lib/tinyusb
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc && source ~/.bashrc

# ROS packages for the real robot
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-v4l2-camera

# stable device name /dev/jgb_rover_mcu + serial port permission
sudo cp firmware/rp2040/99-jgb-rover-mcu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER     # then log out and back in
```

## 3. Build and flash the firmware

```bash
cd firmware/rp2040
cmake -B build && cmake --build build -j      # -> build/jgb_rover_mcu.uf2
```

Hold **BOOT** on the RP2040-Zero, plug in USB, release. A drive `RPI-RP2` appears; copy
`build/jgb_rover_mcu.uf2` onto it. The board restarts and its LED blinks **blue**.

Next time, without buttons: `firmware/tools/rover_cli.py bootsel`, then copy the file again.

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

## 6. Drive

Put the robot on the floor. Terminal 1:

```bash
ros2 launch jgb_rover_bringup bringup_real.launch.py camera:=false
```

**Do not touch the robot for 3 s** (the gyro calibrates). You should see `RP2040 board configured`
and the LED turns green while driving.

Terminal 2:

```bash
source ros2_ws/install/setup.bash
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
| no `/dev/ttyACM0` or `/dev/jgb_rover_mcu` | charge-only USB cable, or the board is still in BOOT mode (`RPI-RP2` drive visible: flash it) |
| `Permission denied` on the port | `dialout` group: log out and in after `usermod` |
| `board did not answer` / `no handshake` | another program has the port open (close `rover_cli.py monitor`), or firmware not flashed |
| LED **yellow** or IMU `0x00` | GY-521 wiring: SDA GP6, SCL GP7, VCC 3V3, AD0 to GND |
| encoder `did not count` | encoder power (3V3 + GND) and A/B wires |
| wheel never turns | GP9 to all four EN pins, battery on B+ / B−, module VCC at 3V3 |
| wheel turns the wrong way | `rover_cli.py check`, then fix `invert_*` in `real_hardware.yaml` |
| robot stops after a moment | keep the teleop key pressed (commands time out after 0.5 s) |
| speed wobbles or is slow to react | tune `wheel_controller` with `rover_cli.py step 6 --plot` (see the firmware README) |
| `/imu/data` silent | the robot moved during the first 3 s: keep it still, it retries |

Try without hardware: `firmware/tools/fake_board.py` prints a port like `/dev/pts/5`; use it as
`--port /dev/pts/5` for `rover_cli.py` or `serial_port:=/dev/pts/5` for the launch file.
