"""McuLink (C++, via mcu_link_probe) against the Python fake board (firmware/tools/fake_board.py).

Run by colcon test, or directly:
    MCU_LINK_PROBE=build/jgb_rover_hardware/mcu_link_probe python3 -m pytest test/test_mcu_link.py
"""
import math
import os
import subprocess
import sys
import threading
import time

import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'firmware', 'tools'))
import rover_link as rl  # noqa: E402
from fake_board import FakeBoard  # noqa: E402

PROBE = os.environ.get('MCU_LINK_PROBE', '')
pytestmark = pytest.mark.skipif(not os.path.exists(PROBE), reason='MCU_LINK_PROBE not built')
RAD_PER_COUNT = 2 * math.pi / 3960.0


def run_probe(port, seconds, left=0.0, right=0.0, during=None):
    proc = subprocess.Popen([PROBE, port, str(seconds), str(left), str(right)],
                            stdout=subprocess.PIPE, text=True)
    if during:
        threading.Timer(during[0], during[1]).start()
    out, _ = proc.communicate(timeout=seconds + 10)
    result = {}
    for line in out.splitlines():
        key, _, rest = line.partition(' ')
        result.setdefault(key, []).append(rest)
    return proc.returncode, result, out


def floats(result, key):
    return [float(x) for x in result[key][-1].split()]


@pytest.fixture
def board():
    b = FakeBoard().start()
    yield b
    b.stop()


def test_handshake_config_and_drive(board):
    code, r, out = run_probe(board.path, 1.0, 2.0, -1.0)
    assert code == 0, out
    assert 'ready' in r
    # the config reached the board before any command
    types = [t for t, _ in board.received]
    assert types.index(rl.SET_CONFIG) < types.index(rl.CMD_VEL)
    assert board.config.flags == rl.CFG_INVERT_RIGHT_MOTOR | rl.CFG_INVERT_RIGHT_ENCODER | rl.CFG_IDLE_BRAKE
    left, right = floats(r, 'position')
    assert left == pytest.approx(2.0, abs=0.15)      # ~1 s at 2 rad/s
    assert right == pytest.approx(-1.0, abs=0.08)
    assert floats(r, 'gyro')[2] == pytest.approx(0.01, abs=1e-6)
    assert floats(r, 'accel')[2] == pytest.approx(9.81, abs=1e-4)
    assert int(r['resets'][0]) == 0
    assert int(r['gaps'][0]) == 0
    # the probe's last action is STOP
    assert board.received[-1][0] == rl.STOP


def test_encoder_wrap(tmp_path):
    board = FakeBoard(start_counts=(2**31 - 2000, -2**31 + 2000)).start()
    try:
        # 1.5 s at +-5 rad/s = ~4700 counts: both counters wrap
        code, r, out = run_probe(board.path, 1.5, 5.0, -5.0)
    finally:
        board.stop()
    assert code == 0, out
    start = floats(r, 'start_position')
    end = floats(r, 'position')
    assert end[0] - start[0] == pytest.approx(7.5, abs=0.3)
    assert end[1] - start[1] == pytest.approx(-7.5, abs=0.3)


def test_board_reset_keeps_position_and_reconfigures(board):
    code, r, out = run_probe(board.path, 2.0, 3.0, 3.0, during=(1.0, board.reset))
    assert code == 0, out
    assert int(r['resets'][0]) == 1
    assert int(r['ready_at_end'][0]) == 1
    assert board.configured, 'config not re-sent after the reset'
    left, right = floats(r, 'position')
    # no jump back to 0 at the reset; a few cycles without commands during the re-handshake
    assert 5.0 < left < 6.3 and 5.0 < right < 6.3, out


def test_no_imu(tmp_path):
    board = FakeBoard(imu=False).start()
    try:
        code, r, out = run_probe(board.path, 0.5)
    finally:
        board.stop()
    assert code == 0
    assert 'IMU not responding' in out
    assert floats(r, 'accel') == [0.0, 0.0, 0.0]


def test_no_board():
    code, r, out = run_probe('/dev/does_not_exist', 0.1)
    assert code == 1
    assert r['result'] == ['connect_failed']


def test_silent_device(tmp_path):
    """A port that never answers (wrong firmware): handshake fails with a clear message."""
    import pty
    master, slave = pty.openpty()
    try:
        t0 = time.monotonic()
        code, r, out = run_probe(os.ttyname(slave), 0.1)
        assert code == 1 and time.monotonic() - t0 < 6
        assert 'did not answer' in out
    finally:
        os.close(master)
        os.close(slave)
