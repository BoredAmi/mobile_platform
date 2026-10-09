"""Host tests for the firmware's portable parts. Needs a C compiler (cc).

    python3 -m pytest firmware/test

- the C framing (firmware/protocol) and the Python one (firmware/tools/rover_link.py) produce
  identical frames and decode each other's output;
- the wheel controller and velocity estimator (firmware/rp2040/src/control.c) against a motor model.
"""
import os
import random
import struct
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(FW, 'tools'))
import rover_link as rl  # noqa: E402


def compile_c(tmp_path_factory, name, sources, includes):
    out = tmp_path_factory.mktemp('bin') / name
    cmd = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-D_DEFAULT_SOURCE', '-o', str(out)]
    cmd += [f'-I{i}' for i in includes] + sources + ['-lm']
    subprocess.run(cmd, check=True)
    return str(out)


@pytest.fixture(scope='module')
def frame_tool(tmp_path_factory):
    return compile_c(tmp_path_factory, 'frame_tool',
                     [os.path.join(HERE, 'frame_tool.c'), os.path.join(FW, 'protocol', 'rover_link.c')],
                     [os.path.join(FW, 'protocol')])


def c_encode(tool, msg_type, payload):
    out = subprocess.run([tool, 'enc', str(msg_type), payload.hex()], check=True,
                         capture_output=True, text=True).stdout.strip()
    return bytes.fromhex(out)


def c_decode(tool, stream):
    lines = subprocess.run([tool, 'dec', stream.hex()], check=True, capture_output=True,
                           text=True).stdout.splitlines()
    frames = []
    for line in lines[:-1]:
        t, _, p = line.partition(' ')
        frames.append((int(t, 16), bytes.fromhex(p)))
    return frames, int(lines[-1].split()[1])


def payloads():
    rng = random.Random(1)
    yield b''
    yield b'\x00' * 20
    yield bytes(range(1, 161))          # longest payload, no zeros
    yield struct.pack('<2f', 1.5, -2.25)
    for _ in range(40):
        n = rng.randint(0, 160)
        yield bytes(rng.choice([0, 0, rng.randint(1, 255)]) for _ in range(n))


def test_c_and_python_frames_identical(frame_tool):
    for i, payload in enumerate(payloads()):
        py = rl.encode(0x80 + i % 4, payload)
        assert c_encode(frame_tool, 0x80 + i % 4, payload) == py
        assert py[-1] == 0 and 0 not in py[:-1]


def test_c_decodes_python_stream_with_garbage(frame_tool):
    frames = [(0x81, p) for p in payloads()]
    stream = b'\x13\x37garbage\x00'       # tail of a frame: receiver started mid-frame
    for t, p in frames:
        stream += rl.encode(t, p)
    corrupted = bytearray(rl.encode(0x01, b'\x01\x02\x03\x04'))
    corrupted[2] ^= 0xFF
    stream += bytes(corrupted) + b'\x00\x00' + rl.encode(0x02, b'end')
    decoded, errors = c_decode(frame_tool, stream)
    assert decoded == frames + [(0x02, b'end')]
    assert errors == 2                    # the leading garbage and the corrupted frame


def test_python_decoder():
    dec = rl.Decoder()
    data = b'\xff\xfe\x00' + rl.encode(rl.TELEMETRY, b'\x00' * 76) + rl.cmd_vel(1.0, -1.0)
    out = []
    for i in range(0, len(data), 7):      # arbitrary chunking
        out += dec.feed(data[i:i + 7])
    assert out == [(rl.TELEMETRY, b'\x00' * 76), (rl.CMD_VEL, struct.pack('<2f', 1.0, -1.0))]
    assert dec.errors == 1


def test_cobs_long_runs():
    for n in (253, 254, 255, 508, 600):
        data = bytes((i % 255) + 1 for i in range(n))
        assert rl.cobs_decode(rl.cobs_encode(data)) == data
        assert rl.cobs_decode(rl.cobs_encode(data + b'\x00' + data)) == data + b'\x00' + data


def test_struct_roundtrip():
    cfg = rl.Config(kp=0.1, flags=rl.CFG_IDLE_BRAKE)
    back = rl.Config.unpack(cfg.pack())
    for name, value in vars(cfg).items():
        assert getattr(back, name) == pytest.approx(value, rel=1e-6), name
    tm = rl.Telemetry.unpack(bytes(76))
    assert tm.seq == 0 and tm.enc == (0, 0) and tm.status_names() == []


def test_control(tmp_path_factory):
    src = os.path.join(FW, 'rp2040', 'src')
    binary = compile_c(tmp_path_factory, 'test_control',
                       [os.path.join(HERE, 'test_control.c'), os.path.join(src, 'control.c')],
                       [src, os.path.join(FW, 'protocol')])
    result = subprocess.run([binary], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
