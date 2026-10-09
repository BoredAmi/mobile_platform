#!/usr/bin/env python3
"""Fake RP2040 board on a pseudo-terminal: speaks rover_link like the firmware, with ideal wheels.

For testing the host side without hardware:
    ./fake_board.py                        # prints the device path, e.g. /dev/pts/7
    ./rover_cli.py --port /dev/pts/7 vel 3 3
The ROS hardware interface can use it too (serial_port:=/dev/pts/7).

As a library (tests): FakeBoard(...).start(); .path; .reset() simulates a board reset.
"""
import math
import os
import pty
import struct
import sys
import threading
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rover_link as rl  # noqa: E402


class FakeBoard:
    def __init__(self, start_counts=(0, 0), gyro_z=0.01, imu=True, rate_hz=100.0):
        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        self.path = os.ttyname(slave)
        self._slave = slave            # kept open so the pty stays alive between host opens
        self.rate = rate_hz
        self.gyro_z = gyro_z
        self.imu = imu
        self.lock = threading.Lock()
        self.received = []             # (type, payload) from the host
        self._boot(start_counts)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _boot(self, start_counts=(0, 0)):
        self.seq = 0
        self.angle = [c * 2 * math.pi / 3960.0 for c in start_counts]
        self.config = rl.Config()
        self.configured = False
        self.target = [0.0, 0.0]
        self.last_cmd = 0.0
        self.boot_time = time.monotonic()

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        os.close(self.master)
        os.close(self._slave)

    def reset(self):
        """Simulate a board reset: seq and encoder counts restart at 0, config back to defaults."""
        with self.lock:
            self._boot()

    def counts(self):
        with self.lock:
            return [int(math.floor(a / (2 * math.pi) * self.config.counts_per_rev)) for a in self.angle]

    def _send(self, msg_type, payload=b''):
        try:
            os.write(self.master, rl.encode(msg_type, payload))
        except OSError:
            pass

    def _handle(self, t, p):
        self.received.append((t, p))
        if t == rl.PING:
            info = struct.pack(rl.Info.FORMAT, rl.PROTOCOL_VERSION, 0x0100, int(self.rate), 0x68 if self.imu else 0,
                               0, int((time.monotonic() - self.boot_time) * 1000), b'FAKEBRD!')
            self._send(rl.INFO, info)
            self._send(rl.CONFIG, self.config.pack())
        elif t == rl.SET_CONFIG and len(p) == 28:
            self.config = rl.Config.unpack(p)
            self.configured = True
            self._send(rl.CONFIG_ACK, p)
        elif t in (rl.CMD_VEL, rl.CMD_PWM) and len(p) == 8:
            v = struct.unpack('<2f', p)
            self.target = list(v) if t == rl.CMD_VEL else [d * 11.52 for d in v]
            self.last_cmd = time.monotonic()
        elif t == rl.STOP:
            self.target = [0.0, 0.0]

    def _run(self):
        import select
        dec = rl.Decoder()
        period = 1.0 / self.rate
        next_t = time.monotonic()
        while not self._stop.is_set():
            r, _, _ = select.select([self.master], [], [], max(0.0, next_t - time.monotonic()))
            if r:
                try:
                    data = os.read(self.master, 4096)
                except OSError:
                    data = b''
                with self.lock:
                    for t, p in dec.feed(data):
                        self._handle(t, p)
            if time.monotonic() < next_t:
                continue
            next_t += period
            with self.lock:
                if time.monotonic() - self.last_cmd > self.config.cmd_timeout_ms / 1000.0:
                    self.target = [0.0, 0.0]
                for i in range(2):
                    self.angle[i] += self.target[i] * period
                cpr = self.config.counts_per_rev
                enc = [((int(math.floor(a / (2 * math.pi) * cpr)) + 2**31) % 2**32) - 2**31 for a in self.angle]
                status = (rl.STATUS_IMU_OK if self.imu else 0) | (1 << 5 if self.configured else 0)
                if any(self.target):
                    status |= 0b110
                payload = struct.pack(
                    rl.Telemetry.FORMAT, self.seq, int(time.monotonic() * 1e6) & 0xFFFFFFFF, *enc,
                    *self.target, *self.target, *(x / 11.52 for x in self.target),
                    0.0, 0.0, self.gyro_z if self.imu else 0.0, 0.0, 0.0, 9.81 if self.imu else 0.0,
                    25.0, 0, status, dec.errors & 0xFFFF, 0)
                self.seq += 1
            self._send(rl.TELEMETRY, payload)


if __name__ == '__main__':
    board = FakeBoard().start()
    print(board.path, flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        board.stop()
