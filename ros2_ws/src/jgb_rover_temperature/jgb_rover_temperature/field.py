"""Air temperature field of a simulated world (the 'temperature' section of the layout yaml).

T(x, y) = ambient_c + gradient . (x, y) + sum_i delta_c_i * exp(-|p - p_i|^2 / (2 sigma_i^2)),
in the world frame. Used by the simulated sensor and by the heatmap evaluation.
"""
import numpy as np
import yaml


class TemperatureField:
    def __init__(self, cfg: dict = None, default_ambient_c: float = 21.0):
        cfg = cfg or {}
        self.ambient = float(cfg.get('ambient_c', default_ambient_c))
        self.gradient = np.asarray(cfg.get('gradient_c_per_m', [0.0, 0.0]), float)
        self.sources = [(np.asarray(s['xy'], float), float(s['delta_c']), float(s['sigma']))
                        for s in cfg.get('sources', [])]

    @staticmethod
    def from_layout(path: str, default_ambient_c: float = 21.0) -> 'TemperatureField':
        with open(path) as f:
            layout = yaml.safe_load(f) or {}
        return TemperatureField(layout.get('temperature'), default_ambient_c)

    def __call__(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        t = self.ambient + self.gradient[0] * x + self.gradient[1] * y
        for (sx, sy), delta, sigma in self.sources:
            t = t + delta * np.exp(-((x - sx) ** 2 + (y - sy) ** 2) / (2.0 * sigma ** 2))
        return t
