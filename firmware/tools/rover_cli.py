#!/usr/bin/env python3
"""Bench tool for the jgb_rover RP2040 board, without ROS. Needs pyserial (sudo apt install python3-serial).

Lift the wheels off the ground for check / pwm / vel / step.

  rover_cli.py info                     board, firmware, IMU and the config in use
  rover_cli.py monitor                  live telemetry (Ctrl-C to stop)
  rover_cli.py imu [--time 5]           IMU statistics with the robot standing still
  rover_cli.py check                    which motors / encoders need inverting
  rover_cli.py pwm 0.3 0.3 [--time 2]   open-loop duty
  rover_cli.py vel 5 5 [--time 3]       closed-loop wheel speed [rad/s]
  rover_cli.py step 6 [--csv f.csv]     speed step response: rise time, overshoot, steady-state error
  rover_cli.py bootsel                  reboot the board into its USB bootloader (to flash)

Controller gains, directions and timeouts are read from
ros2_ws/src/jgb_rover_description/config/real_hardware.yaml and sent to the board first; override
single values with --kp, --ki, --kff, --deadband, --max-duty, --flip-left-motor etc.
"""
import argparse
import math
import os
import statistics
import sys
import time

import rover_link as rl

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
HW_YAML = os.path.join(REPO, 'ros2_ws', 'src', 'jgb_rover_description', 'config', 'real_hardware.yaml')
SPEC_YAML = os.path.join(REPO, 'robot_spec.yaml')


def load_config(args) -> rl.Config:
    cfg = rl.Config()
    try:
        import yaml
        with open(HW_YAML) as f:
            hw = yaml.safe_load(f)
        with open(SPEC_YAML) as f:
            spec = yaml.safe_load(f)
        wc, w = hw['wheel_controller'], hw['wiring']
        cfg = rl.Config(
            counts_per_rev=float(spec['encoders']['counts_per_output_rev_x4']),
            kp=wc['kp'], ki=wc['ki'], kff=wc['kff'], deadband_duty=wc['deadband_duty'],
            max_duty=wc['max_duty'], cmd_timeout_ms=int(hw['mcu']['cmd_timeout_ms']),
            vel_window=int(wc['vel_window']),
            flags=(rl.CFG_INVERT_LEFT_MOTOR * bool(w['invert_left_motor'])
                   | rl.CFG_INVERT_RIGHT_MOTOR * bool(w['invert_right_motor'])
                   | rl.CFG_INVERT_LEFT_ENCODER * bool(w['invert_left_encoder'])
                   | rl.CFG_INVERT_RIGHT_ENCODER * bool(w['invert_right_encoder'])
                   | rl.CFG_IDLE_BRAKE * bool(w['idle_brake'])))
        if args.port is None:
            args.port = hw['mcu']['serial_port']
    except (ImportError, OSError, KeyError) as e:
        print(f'note: using built-in defaults ({e})', file=sys.stderr)
    for name, attr in (('kp', 'kp'), ('ki', 'ki'), ('kff', 'kff'), ('deadband', 'deadband_duty'),
                       ('max_duty', 'max_duty'), ('vel_window', 'vel_window')):
        if getattr(args, name) is not None:
            setattr(cfg, attr, getattr(args, name))
    for name, bit in (('flip_left_motor', rl.CFG_INVERT_LEFT_MOTOR), ('flip_right_motor', rl.CFG_INVERT_RIGHT_MOTOR),
                      ('flip_left_encoder', rl.CFG_INVERT_LEFT_ENCODER),
                      ('flip_right_encoder', rl.CFG_INVERT_RIGHT_ENCODER)):
        if getattr(args, name):
            cfg.flags ^= bit
    return cfg


class Link:
    def __init__(self, port):
        import serial
        if not os.path.exists(port) and port == '/dev/jgb_rover_mcu' and os.path.exists('/dev/ttyACM0'):
            print('note: /dev/jgb_rover_mcu missing (udev rule not installed?), using /dev/ttyACM0',
                  file=sys.stderr)
            port = '/dev/ttyACM0'
        # the baud rate is ignored by USB CDC, but must never be 1200 (= reboot into the bootloader)
        self.ser = serial.Serial(port, 115200, timeout=0.01)
        self.dec = rl.Decoder()
        self.info = None
        self.config = None
        self.config_ack = None
        self.latest = None

    def send(self, frame: bytes):
        self.ser.write(frame)

    def poll(self, timeout=0.0):
        """Read what is there (waiting up to timeout for the first byte); returns telemetry list."""
        end = time.monotonic() + timeout
        out = []
        while True:
            data = self.ser.read(self.ser.in_waiting or 1)
            for t, p in self.dec.feed(data):
                if t == rl.TELEMETRY and len(p) == 76:
                    self.latest = rl.Telemetry.unpack(p)
                    out.append(self.latest)
                elif t == rl.INFO and len(p) == 20:
                    self.info = rl.Info.unpack(p)
                elif t == rl.CONFIG and len(p) == 28:
                    self.config = rl.Config.unpack(p)
                elif t == rl.CONFIG_ACK and len(p) == 28:
                    self.config = self.config_ack = rl.Config.unpack(p)
                elif t == rl.LOG:
                    print(f'[board] {p.decode(errors="replace")}')
            if out or time.monotonic() >= end:
                return out

    def handshake(self, cfg: rl.Config = None, timeout=3.0):
        self.ser.reset_input_buffer()
        self.info = self.config = None
        end = time.monotonic() + timeout
        while self.info is None and time.monotonic() < end:
            self.send(rl.encode(rl.PING))
            self.poll(0.2)
        if self.info is None:
            raise SystemExit('no answer from the board (wrong port? firmware flashed?)')
        if self.info.protocol_version != rl.PROTOCOL_VERSION:
            raise SystemExit(f'board speaks protocol {self.info.protocol_version}, '
                             f'this tool {rl.PROTOCOL_VERSION}: reflash or update the tool')
        if cfg is not None:
            self.config_ack = None
            self.send(rl.encode(rl.SET_CONFIG, cfg.pack()))
            while self.config_ack is None and time.monotonic() < end + 1.0:
                self.poll(0.1)
            if self.config_ack is None:
                raise SystemExit('board did not confirm the config')

    def run(self, frame_fn, seconds, rate=50.0):
        """Send frame_fn(t) at rate for seconds; collect telemetry. Always ends with STOP."""
        samples = []
        t0 = time.monotonic()
        next_send = t0
        try:
            while (now := time.monotonic()) - t0 < seconds:
                if now >= next_send:
                    self.send(frame_fn(now - t0))
                    next_send += 1.0 / rate
                samples += [(time.monotonic() - t0, s) for s in self.poll(0.005)]
        finally:
            self.send(rl.encode(rl.STOP))
        return samples


def fmt_flags(flags):
    names = ['invert_left_motor', 'invert_right_motor', 'invert_left_encoder', 'invert_right_encoder', 'idle_brake']
    return ', '.join(n for i, n in enumerate(names) if flags & (1 << i)) or 'none'


def cmd_info(link, args, cfg):
    link.handshake()
    i = link.info
    who = {0x68: 'MPU6050', 0x70: 'MPU6500 / clone', 0x72: 'clone', 0x98: 'clone', 0: 'NOT FOUND'}
    print(f'firmware      {i.firmware_version >> 8}.{i.firmware_version & 0xFF}, protocol {i.protocol_version}, '
          f'{i.control_hz} Hz')
    print(f'board id      {i.board_id.hex()}')
    print(f'uptime        {i.uptime_ms / 1000:.1f} s, last reset: {"WATCHDOG" if i.reset_reason else "power-on / reset"}')
    print(f'IMU WHO_AM_I  0x{i.imu_whoami:02x} ({who.get(i.imu_whoami, "unknown, may still work")})')
    c = link.config
    print(f'config        cpr {c.counts_per_rev:.0f}, kp {c.kp:.3f}, ki {c.ki:.3f}, kff {c.kff:.4f}, '
          f'deadband {c.deadband_duty:.3f}, max_duty {c.max_duty:.2f}, window {c.vel_window}, '
          f'timeout {c.cmd_timeout_ms} ms')
    print(f'flags         {fmt_flags(c.flags)}')
    print('(this is what the board uses now; ROS / the other commands send real_hardware.yaml first)')


def cmd_monitor(link, args, cfg):
    link.handshake(cfg)
    last_print, n, first = 0.0, 0, None
    try:
        while True:
            for s in link.poll(0.1):
                n += 1
                first = first or (time.monotonic(), s.seq)
            now = time.monotonic()
            if link.latest and now - last_print > 0.2:
                s = link.latest
                rate = (s.seq - first[1]) / (now - first[0]) if first and now > first[0] + 0.5 else 0
                print(f'seq {s.seq:7d} {rate:5.1f} Hz | enc {s.enc[0]:8d} {s.enc[1]:8d} | '
                      f'vel {s.vel[0]:6.2f} {s.vel[1]:6.2f} rad/s | '
                      f'gyro {s.gyro[0]:6.3f} {s.gyro[1]:6.3f} {s.gyro[2]:6.3f} | '
                      f'acc {s.accel[0]:5.2f} {s.accel[1]:5.2f} {s.accel[2]:5.2f} | '
                      f'{s.battery_mv / 1000:.2f} V | rx_err {s.rx_errors} drop {s.tx_dropped} | '
                      f'{" ".join(s.status_names())}')
                last_print = now
    except KeyboardInterrupt:
        pass


def cmd_imu(link, args, cfg):
    link.handshake(cfg)
    print(f'keep the robot still for {args.time:.0f} s ...')
    samples = []
    end = time.monotonic() + args.time
    while time.monotonic() < end:
        samples += link.poll(0.1)
    ok = [s for s in samples if s.status & rl.STATUS_IMU_OK]
    if not ok:
        raise SystemExit('no IMU data (status IMU_OK never set): check wiring, address 0x68')
    print(f'{len(ok)} samples ({len(ok) / args.time:.1f} Hz), {len(samples) - len(ok)} without IMU')
    for name, idx, unit in (('gyro', 'gyro', 'rad/s'), ('accel', 'accel', 'm/s^2')):
        for axis in range(3):
            v = [getattr(s, idx)[axis] for s in ok]
            print(f'  {name} {"xyz"[axis]}: mean {statistics.fmean(v):+8.4f}  std {statistics.pstdev(v):.4f} {unit}')
    norm = statistics.fmean(math.sqrt(sum(a * a for a in s.accel)) for s in ok)
    print(f'  |accel| {norm:.3f} m/s^2 (expect ~9.81), temperature {ok[-1].imu_temp_c:.1f} C')
    az = statistics.fmean(s.accel[2] for s in ok)
    if az < 8.0:
        print('  WARNING: accel z is not ~+9.81: the board is not mounted Z up, fix IMU_AXIS_MAP / '
              'IMU_AXIS_SIGN in firmware/rp2040/src/config.h')
    print('  robot_spec.yaml imu.sim_noise.gyro_stddev / accel_stddev can be set from the std values above')


def cmd_check(link, args, cfg):
    """Open-loop pulse on each wheel; the encoder must count up and the wheel must turn forward."""
    link.handshake(cfg)
    print('wheels OFF the ground. Each wheel turns for 1 s at +%.0f %% duty; watch which way it turns.'
          % (args.duty * 100))
    advice = []
    for side, name in ((0, 'left'), (1, 'right')):
        input(f'press Enter to pulse the {name} wheel ...')
        duty = [0.0, 0.0]
        duty[side] = args.duty
        link.ser.reset_input_buffer()     # drop telemetry queued while waiting for Enter
        samples = link.run(lambda t: rl.cmd_pwm(*duty), 1.0)
        if not samples:
            raise SystemExit('no telemetry')
        start = samples[0][1].enc[side]
        delta = samples[-1][1].enc[side] - start
        speed = max(abs(s.vel[side]) for _, s in samples)
        print(f'  {name}: encoder moved {delta:+d} counts, peak {speed:.2f} rad/s')
        if abs(delta) < 20:
            print('  -> encoder did not count: check its power (3V3), A/B wiring and the motor driver')
            continue
        fwd = input('  did the wheel turn so that the robot would drive FORWARD? [y/n] ').strip().lower() == 'y'
        if not fwd:
            advice.append(f'invert_{name}_motor (flip it)')
        if (delta > 0) != fwd:
            advice.append(f'invert_{name}_encoder (flip it)')
        time.sleep(0.3)
    if advice:
        print('change in real_hardware.yaml (wiring):', '; '.join(advice))
        print('(currently: %s)' % fmt_flags(cfg.flags))
    else:
        print('directions OK: positive duty drives forward and counts up on both wheels')


def cmd_pwm(link, args, cfg):
    link.handshake(cfg)
    show(link.run(lambda t: rl.cmd_pwm(args.left, args.right), args.time))


def cmd_vel(link, args, cfg):
    link.handshake(cfg)
    show(link.run(lambda t: rl.cmd_vel(args.left, args.right), args.time))


def show(samples):
    for t, s in samples[::10]:
        print(f't {t:5.2f} | target {s.target[0]:6.2f} {s.target[1]:6.2f} | vel {s.vel[0]:6.2f} {s.vel[1]:6.2f} | '
              f'duty {s.duty[0]:+.2f} {s.duty[1]:+.2f} | enc {s.enc[0]} {s.enc[1]}')
    if samples:
        tail = [s for t, s in samples if t > samples[-1][0] - 0.5]
        for side, name in ((0, 'left'), (1, 'right')):
            print(f'{name}: last 0.5 s mean {statistics.fmean(s.vel[side] for s in tail):.3f} rad/s, '
                  f'duty {statistics.fmean(s.duty[side] for s in tail):+.3f}')


def cmd_step(link, args, cfg):
    link.handshake(cfg)
    target = args.target
    samples = link.run(lambda t: rl.cmd_vel(target, target) if t >= 0.3 else rl.cmd_vel(0, 0), args.time + 0.3)
    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('t,target_l,target_r,vel_l,vel_r,duty_l,duty_r\n')
            for t, s in samples:
                f.write(f'{t:.4f},{s.target[0]},{s.target[1]},{s.vel[0]},{s.vel[1]},{s.duty[0]},{s.duty[1]}\n')
        print(f'wrote {args.csv}')
    for side, name in ((0, 'left'), (1, 'right')):
        step = [(t - 0.3, s.vel[side], s.duty[side]) for t, s in samples if abs(s.target[side] - target) < 1e-4]
        if len(step) < 20:
            print(f'{name}: not enough samples')
            continue
        t_rise = next((t for t, v, _ in step if abs(v) >= 0.9 * abs(target)), None)
        peak = max(step, key=lambda x: abs(x[1]))[1]
        tail = [v for t, v, _ in step if t > step[-1][0] - 0.5]
        ss = statistics.fmean(tail)
        duty = statistics.fmean(d for t, _, d in step if t > step[-1][0] - 0.5)
        print(f'{name}: rise (90 %) {t_rise * 1000 if t_rise is not None else float("nan"):.0f} ms, '
              f'overshoot {max(0.0, (abs(peak) - abs(target)) / abs(target) * 100):.0f} %, '
              f'steady {ss:.3f} rad/s (error {ss - target:+.3f}, ripple {statistics.pstdev(tail):.3f}), '
              f'duty {duty:+.3f}')
    print('kff ~ steady duty / target speed. Rise slow -> raise kp; overshoot / oscillation -> lower kp or ki; '
          'steady error that lingers -> raise ki.')
    if args.plot:
        import matplotlib.pyplot as plt
        t = [x[0] for x in samples]
        fig, ax = plt.subplots(2, 1, sharex=True)
        for side, name in ((0, 'left'), (1, 'right')):
            ax[0].plot(t, [s.vel[side] for _, s in samples], label=f'{name} vel')
            ax[1].plot(t, [s.duty[side] for _, s in samples], label=f'{name} duty')
        ax[0].plot(t, [s.target[0] for _, s in samples], 'k--', label='target')
        ax[0].set_ylabel('rad/s')
        ax[1].set_ylabel('duty')
        ax[1].set_xlabel('s')
        for a in ax:
            a.legend()
            a.grid(True)
        plt.show()


def cmd_bootsel(link, args, cfg):
    link.send(rl.encode(rl.BOOTSEL))
    print('board rebooting into BOOTSEL: copy the .uf2 to the RPI-RP2 drive (or picotool load -x)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial port (default: real_hardware.yaml mcu.serial_port)')
    for name in ('kp', 'ki', 'kff', 'deadband', 'max-duty'):
        ap.add_argument(f'--{name}', type=float)
    ap.add_argument('--vel-window', type=int)
    for name in ('left-motor', 'right-motor', 'left-encoder', 'right-encoder'):
        ap.add_argument(f'--flip-{name}', action='store_true', help='invert relative to real_hardware.yaml')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('info')
    sub.add_parser('monitor')
    p = sub.add_parser('imu')
    p.add_argument('--time', type=float, default=5.0)
    p = sub.add_parser('check')
    p.add_argument('--duty', type=float, default=0.3)
    for name in ('pwm', 'vel'):
        p = sub.add_parser(name)
        p.add_argument('left', type=float)
        p.add_argument('right', type=float)
        p.add_argument('--time', type=float, default=2.0)
    p = sub.add_parser('step')
    p.add_argument('target', type=float, help='rad/s, both wheels')
    p.add_argument('--time', type=float, default=2.0)
    p.add_argument('--csv')
    p.add_argument('--plot', action='store_true')
    sub.add_parser('bootsel')
    args = ap.parse_args()

    cfg = load_config(args)
    link = Link(args.port or '/dev/ttyACM0')
    {'info': cmd_info, 'monitor': cmd_monitor, 'imu': cmd_imu, 'check': cmd_check, 'pwm': cmd_pwm,
     'vel': cmd_vel, 'step': cmd_step, 'bootsel': cmd_bootsel}[args.cmd](link, args, cfg)


if __name__ == '__main__':
    main()
