"""rover_link protocol in Python (no serial dependency). Mirrors firmware/protocol/rover_link.h."""
import struct
from dataclasses import dataclass, fields

PROTOCOL_VERSION = 1

CMD_VEL = 0x01
CMD_PWM = 0x02
SET_CONFIG = 0x03
PING = 0x04
STOP = 0x05
BOOTSEL = 0x06
TELEMETRY = 0x81
INFO = 0x82
CONFIG = 0x83
LOG = 0x84
CONFIG_ACK = 0x85

STATUS_BITS = {
    0: 'IMU_OK', 1: 'MOTORS_ENABLED', 2: 'MODE_VEL', 3: 'MODE_PWM', 4: 'CMD_TIMEOUT',
    5: 'CONFIGURED', 6: 'WATCHDOG_REBOOT', 7: 'BATTERY_VALID', 8: 'SATURATED_L', 9: 'SATURATED_R',
}
STATUS_IMU_OK = 1 << 0

CFG_INVERT_LEFT_MOTOR = 1 << 0
CFG_INVERT_RIGHT_MOTOR = 1 << 1
CFG_INVERT_LEFT_ENCODER = 1 << 2
CFG_INVERT_RIGHT_ENCODER = 1 << 3
CFG_IDLE_BRAKE = 1 << 4


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_pos, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_pos] = code
            code_pos, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_pos] = code
                code_pos, code = len(out), 1
                out.append(0)
    out[code_pos] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            raise ValueError('bad COBS block')
        out += data[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode(msg_type: int, payload: bytes = b'') -> bytes:
    raw = bytes([msg_type]) + payload
    raw += struct.pack('<H', crc16(raw))
    return cobs_encode(raw) + b'\x00'


class Decoder:
    """Feed bytes, get (type, payload) tuples for every valid frame."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        frames = []
        for byte in data:
            if byte != 0:
                self.buf.append(byte)
                continue
            if not self.buf:
                continue
            try:
                raw = cobs_decode(bytes(self.buf))
            except ValueError:
                raw = b''
            self.buf.clear()
            if len(raw) < 3 or struct.unpack('<H', raw[-2:])[0] != crc16(raw[:-2]):
                self.errors += 1
                continue
            frames.append((raw[0], raw[1:-2]))
        return frames


@dataclass
class Config:
    counts_per_rev: float = 3960.0
    kp: float = 0.05
    ki: float = 0.3
    kff: float = 0.085
    deadband_duty: float = 0.04
    max_duty: float = 0.95
    cmd_timeout_ms: int = 250
    vel_window: int = 4
    flags: int = CFG_INVERT_RIGHT_MOTOR | CFG_INVERT_RIGHT_ENCODER | CFG_IDLE_BRAKE
    FORMAT = '<6fHBB'

    def pack(self) -> bytes:
        return struct.pack(self.FORMAT, *(getattr(self, f.name) for f in fields(self)))

    @classmethod
    def unpack(cls, payload: bytes) -> 'Config':
        return cls(*struct.unpack(cls.FORMAT, payload))


@dataclass
class Telemetry:
    seq: int
    t_us: int
    enc: tuple
    vel: tuple
    target: tuple
    duty: tuple
    gyro: tuple
    accel: tuple
    imu_temp_c: float
    battery_mv: int
    status: int
    rx_errors: int
    tx_dropped: int
    FORMAT = '<II2i2f2f2f3f3ffHHHH'

    @classmethod
    def unpack(cls, payload: bytes) -> 'Telemetry':
        v = struct.unpack(cls.FORMAT, payload)
        return cls(v[0], v[1], v[2:4], v[4:6], v[6:8], v[8:10], v[10:13], v[13:16], *v[16:])

    def status_names(self):
        return [name for bit, name in STATUS_BITS.items() if self.status & (1 << bit)]


@dataclass
class Info:
    protocol_version: int
    firmware_version: int
    control_hz: int
    imu_whoami: int
    reset_reason: int
    uptime_ms: int
    board_id: bytes
    FORMAT = '<HHHBBI8s'

    @classmethod
    def unpack(cls, payload: bytes) -> 'Info':
        return cls(*struct.unpack(cls.FORMAT, payload))


assert struct.calcsize(Config.FORMAT) == 28
assert struct.calcsize(Telemetry.FORMAT) == 76
assert struct.calcsize(Info.FORMAT) == 20


def cmd_vel(left: float, right: float) -> bytes:
    return encode(CMD_VEL, struct.pack('<2f', left, right))


def cmd_pwm(left: float, right: float) -> bytes:
    return encode(CMD_PWM, struct.pack('<2f', left, right))
