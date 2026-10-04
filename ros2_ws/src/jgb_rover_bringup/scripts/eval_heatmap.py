#!/usr/bin/env python3
"""Compare a saved temperature heatmap (temperature_mapper ~/save) with the simulated true field.

The true field is the 'temperature' section of the world layout yaml (world frame); the map frame
starts at the robot spawn pose (--spawn). Metrics over the heatmap's known cells:
  - error RMS / median |e| / p90 |e| / mean (bias) in C; target RMS < --target
  - warmest / coldest spot: distance between the heatmap's and the true field's extreme cell
Writes an image (measured | true | error) and a json next to the heatmap.
"""
import argparse
import json
import math
import os

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

from jgb_rover_temperature.field import TemperatureField
from jgb_rover_temperature.heatmap_core import Heatmap, colour_range, colourize


def panel(values, lo, hi, scale, title, cmap=True):
    known = np.isfinite(values)
    if cmap:
        img = colourize(values, lo, hi)
    else:                                                   # error: blue negative, red positive
        u = np.clip(np.nan_to_num(values) / hi, -1, 1)
        img = np.full(values.shape + (3,), 255, np.uint8)
        img[..., 0] = (255 * (1 - np.clip(u, 0, 1))).astype(np.uint8)
        img[..., 2] = (255 * (1 - np.clip(-u, 0, 1))).astype(np.uint8)
        img[..., 1] = (255 * (1 - np.abs(u))).astype(np.uint8)
    img[~known] = 205
    img = cv2.resize(img[::-1], None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    out = np.full((img.shape[0] + 28, img.shape[1] + 10, 3), 255, np.uint8)
    out[28:, 5:-5] = img
    cv2.putText(out, title, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('heatmap', help='temperature_heatmap.npz saved by temperature_mapper')
    ap.add_argument('--layout', default=os.path.join(get_package_share_directory('jgb_rover_gazebo'),
                                                     'config', 'apartment_layout.yaml'))
    ap.add_argument('--spawn', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'YAW'))
    ap.add_argument('--target', type=float, default=1.0, help='C, RMS error target')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    z = np.load(args.heatmap)
    hm = Heatmap(z['values'], z['origin'], float(z['resolution']))
    field = TemperatureField.from_layout(args.layout)
    xs, ys = hm.cell_centres()
    sx, sy, syaw = args.spawn
    wx = sx + math.cos(syaw) * xs - math.sin(syaw) * ys
    wy = sy + math.sin(syaw) * xs + math.cos(syaw) * ys
    truth = np.where(np.isfinite(hm.values), field(wx, wy), np.nan)
    known = np.isfinite(hm.values)
    e = (hm.values - truth)[known]

    def extreme(values, fn):
        i = fn(np.where(known, values, np.nan))
        return np.unravel_index(i, values.shape)

    hot_m, hot_t = extreme(hm.values, np.nanargmax), extreme(truth, np.nanargmax)
    cold_m, cold_t = extreme(hm.values, np.nanargmin), extreme(truth, np.nanargmin)
    cell_dist = lambda a, b: float(hm.resolution * math.hypot(a[0] - b[0], a[1] - b[1]))   # noqa: E731
    res = {
        'known_cells': int(known.sum()),
        'area_m2': float(known.sum() * hm.resolution ** 2),
        'samples': int(len(z['samples'])),
        'error_rms_c': float(np.sqrt(np.mean(e ** 2))),
        'error_median_abs_c': float(np.median(np.abs(e))),
        'error_p90_abs_c': float(np.percentile(np.abs(e), 90)),
        'error_mean_c': float(e.mean()),
        'true_range_c': [float(np.nanmin(truth)), float(np.nanmax(truth))],
        'measured_range_c': [float(np.nanmin(hm.values)), float(np.nanmax(hm.values))],
        'warmest_spot_offset_m': cell_dist(hot_m, hot_t),
        'coldest_spot_offset_m': cell_dist(cold_m, cold_t),
    }
    print(json.dumps(res, indent=2))
    ok = res['error_rms_c'] < args.target
    print(f'heatmap error (RMS over {res["area_m2"]:.1f} m2): {res["error_rms_c"]:.2f} C '
          f'(target < {args.target:.1f} C) -> {"PASS" if ok else "FAIL"}')

    lo, hi = colour_range(np.concatenate([hm.values[known], truth[known]]))
    scale = max(1, int(round(0.04 / hm.resolution * 10)))
    emax = max(1.0, float(np.abs(e).max()))
    img = np.hstack([panel(hm.values, lo, hi, scale, f'measured ({lo:.1f}..{hi:.1f} C)'),
                     panel(truth, lo, hi, scale, 'true field'),
                     panel(hm.values - truth, -emax, emax, scale,
                           f'error, RMS {res["error_rms_c"]:.2f} C (red +, blue -, +-{emax:.1f})', cmap=False)])
    out = args.out or os.path.splitext(args.heatmap)[0] + '_eval.png'
    cv2.imwrite(out, img)
    with open(os.path.splitext(out)[0] + '.json', 'w') as f:
        json.dump(res, f, indent=2)
    print(f'image: {out}')
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
