"""MCP9808 register access (Microchip DS25095). No ROS imports.

The bus object only needs read_i2c_block_data / write_byte_data / write_i2c_block_data
(smbus2.SMBus or the older smbus.SMBus).
"""
REG_CONFIG = 0x01
REG_AMBIENT = 0x05
REG_MANUFACTURER_ID = 0x06
REG_DEVICE_ID = 0x07
REG_RESOLUTION = 0x08

MANUFACTURER_ID = 0x0054
DEVICE_ID = 0x04                      # upper byte of REG_DEVICE_ID (lower byte = revision)

# resolution register value per resolution in C (conversion time 30 / 65 / 130 / 250 ms)
RESOLUTION_CODES = {0.5: 0x00, 0.25: 0x01, 0.125: 0x02, 0.0625: 0x03}


def raw_to_celsius(raw: int) -> float:
    """Ambient temperature register (16 bit, big endian) to C: 13-bit two's complement, 1/16 C."""
    upper, lower = (raw >> 8) & 0x1F, raw & 0xFF
    t = (upper & 0x0F) * 16.0 + lower / 16.0
    return t - 256.0 if upper & 0x10 else t


class MCP9808:
    def __init__(self, bus, address: int):
        self.bus = bus
        self.address = address

    def read_u16(self, reg: int) -> int:
        msb, lsb = self.bus.read_i2c_block_data(self.address, reg, 2)
        return (msb << 8) | lsb

    def check_ids(self):
        manufacturer, device = self.read_u16(REG_MANUFACTURER_ID), self.read_u16(REG_DEVICE_ID) >> 8
        if manufacturer != MANUFACTURER_ID or device != DEVICE_ID:
            raise IOError(f'no MCP9808 at 0x{self.address:02x} '
                          f'(manufacturer 0x{manufacturer:04x}, device 0x{device:02x})')

    def configure(self, resolution_c: float):
        if resolution_c not in RESOLUTION_CODES:
            raise ValueError(f'resolution must be one of {sorted(RESOLUTION_CODES)}')
        self.bus.write_i2c_block_data(self.address, REG_CONFIG, [0x00, 0x00])   # continuous conversion
        self.bus.write_byte_data(self.address, REG_RESOLUTION, RESOLUTION_CODES[resolution_c])

    def read_celsius(self) -> float:
        return raw_to_celsius(self.read_u16(REG_AMBIENT))
