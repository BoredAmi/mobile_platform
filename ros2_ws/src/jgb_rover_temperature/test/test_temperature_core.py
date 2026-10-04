import math

import numpy as np

from jgb_rover_temperature.field import TemperatureField
from jgb_rover_temperature.heatmap_core import LagCompensator, OccMap, build_heatmap, mask_with_map, to_grid_values
from jgb_rover_temperature.mcp9808 import MCP9808, raw_to_celsius


def test_raw_to_celsius():
    assert raw_to_celsius(0x0190) == 25.0          # datasheet example: 0x0190 = +25.0 C
    assert raw_to_celsius(0x0194) == 25.25
    assert raw_to_celsius(0x1FFF) == -0.0625       # sign bit set
    assert raw_to_celsius(0xE190) == 25.0          # alert flag bits 13-15 ignored


class FakeBus:
    def __init__(self):
        self.regs = {0x05: [0x01, 0x94], 0x06: [0x00, 0x54], 0x07: [0x04, 0x00]}
        self.writes = []

    def read_i2c_block_data(self, addr, reg, n):
        return self.regs[reg]

    def write_byte_data(self, addr, reg, value):
        self.writes.append((reg, value))

    def write_i2c_block_data(self, addr, reg, data):
        self.writes.append((reg, data))


def test_driver_registers():
    bus = FakeBus()
    s = MCP9808(bus, 0x18)
    s.check_ids()
    s.configure(0.0625)
    assert (0x08, 0x03) in bus.writes
    assert s.read_celsius() == 25.25


def test_lag_compensator_recovers_ramp():
    tau, dt, rate = 6.0, 0.02, 0.3                  # air warms at 0.3 C/s
    lag = LagCompensator(tau, window_s=4.0)
    sensor, out = 20.0, None
    for i in range(int(40 / dt)):
        t = i * dt
        air = 20.0 + rate * t
        sensor += (1 - math.exp(-dt / tau)) * (air - sensor)
        if i % 25 == 0:                             # 2 Hz readings
            out = lag.add(t, sensor) or out
    t_est, est = out
    assert abs(est - (20.0 + rate * t_est)) < 0.05
    assert abs(sensor - (20.0 + rate * t)) > 1.5    # without compensation it would be far off


def test_heatmap_interpolates_field():
    field = TemperatureField({'ambient_c': 20.0, 'sources': [{'xy': [1.0, 1.0], 'delta_c': 4.0, 'sigma': 0.8}]})
    # back-and-forth lanes 0.4 m apart, like a lawnmower sweep
    xy = np.array([(x, y) for y in np.arange(0.0, 2.01, 0.4) for x in np.arange(0.0, 2.01, 0.05)])
    hm = build_heatmap(xy, field(xy[:, 0], xy[:, 1]), resolution=0.1, sigma=0.2, max_distance=0.4)
    xs, ys = hm.cell_centres()
    inside = (xs > 0.1) & (xs < 1.9) & (ys > 0.1) & (ys < 1.9)
    err = hm.values[inside] - field(xs[inside], ys[inside])
    assert np.all(np.isfinite(hm.values[inside]))
    assert np.sqrt(np.mean(err ** 2)) < 0.25        # cell snapping on a ~3 C/m slope
    assert np.isnan(hm.lookup(np.array([3.0]), np.array([3.0])))[0]   # far from samples: unknown
    g = to_grid_values(hm.values, 20.0, 24.0)
    assert g.min() == -1 and g[np.isfinite(hm.values)].min() >= 1 and g.max() <= 98


def test_mask_with_map():
    xy = np.array([[0.0, 0.0], [1.0, 0.0]])
    hm = build_heatmap(xy, np.array([20.0, 22.0]), 0.1, 0.2, 0.5)
    data = np.zeros((40, 40), np.int8)               # free, 2 x 2 m from (-1, -1)
    data[:, 20:] = -1                                # x >= 0 unknown
    masked = mask_with_map(hm, OccMap(data, np.array([-1.0, -1.0]), 0.05))
    xs, _ = masked.cell_centres()
    assert np.all(np.isnan(masked.values[xs > 0.0]))
    assert np.any(np.isfinite(masked.values[xs < 0.0]))
