#!/usr/bin/env python3
"""Phase 4 evaluation: compare a saved slam_toolbox map with the ground truth from the world SDF.

1. Ground truth: floor-standing collision geometry of the world (walls, legs, boxes; no table
   tops), rasterised at the map resolution.
2. Alignment: the map frame starts at the robot spawn pose (--spawn), refined by a small rigid
   search (dx, dy, dyaw) that minimises the truncated distance of map walls to true surfaces.
3. Metrics, on the occupied map cells:
   - wall-alignment error: RMS / median distance to the nearest true surface for cells that
     belong to a wall (within --match-dist of one); target < 0.10 m
   - false walls: occupied cells farther than --match-dist from any true surface
   - coverage: fraction of true surface cells (inside the mapped area) with a map wall within
     --coverage-dist
4. Overlay PNG: true geometry in grey, map walls coloured by their error.
"""
import argparse
import json
import math
import os

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from jgb_rover_bringup.world_geometry import load_world, rasterize


def load_map(path):
    meta = yaml.safe_load(open(path))
    img_path = meta['image'] if os.path.isabs(meta['image']) else os.path.join(os.path.dirname(path), meta['image'])
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    img = img[::-1, :]                                    # row 0 = lowest y (map convention)
    p = img.astype(float) / 255.0
    if not meta.get('negate', 0):
        p = 1.0 - p
    occupied = p > meta['occupied_thresh']
    free = p < meta['free_thresh']
    return occupied, free, meta['resolution'], np.array(meta['origin'][:2], float), float(meta['origin'][2])


def cells_to_xy(mask, res, origin):
    r, c = np.nonzero(mask)
    return np.stack([origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res], axis=1)


def transform(pts, dx, dy, dyaw):
    c, s = math.cos(dyaw), math.sin(dyaw)
    return pts @ np.array([[c, s], [-s, c]]) + [dx, dy]


class Truth:
    """Distance field to the true surfaces on a fine grid around the world."""

    def __init__(self, boxes, circles, res, margin=1.0):
        pts = [b.corners() for b in boxes] + [np.array([[c.x - c.r, c.y - c.r], [c.x + c.r, c.y + c.r]])
                                             for c in circles]
        allp = np.vstack(pts)
        self.res = res
        self.origin = allp.min(axis=0) - margin
        size = allp.max(axis=0) + margin - self.origin
        self.shape = (int(math.ceil(size[1] / res)), int(math.ceil(size[0] / res)))
        self.occ = rasterize(boxes, circles, res, self.origin, self.shape)
        k = np.ones((3, 3), np.uint8)
        self.surface = self.occ & ~cv2.erode(self.occ.astype(np.uint8), k).astype(bool)
        self.dist = cv2.distanceTransform((~self.surface).astype(np.uint8), cv2.DIST_L2, 5) * res

    def lookup(self, pts):
        c = np.clip(((pts[:, 0] - self.origin[0]) / self.res).astype(int), 0, self.shape[1] - 1)
        r = np.clip(((pts[:, 1] - self.origin[1]) / self.res).astype(int), 0, self.shape[0] - 1)
        return self.dist[r, c]


def align(truth, pts, search_xy, search_yaw, trunc):
    """Grid search, then a finer one around the best, minimising mean truncated distance."""
    best = (0.0, 0.0, 0.0)
    cost = lambda p: np.minimum(truth.lookup(transform(pts, *p)), trunc).mean()   # noqa: E731
    best_cost = cost(best)
    for step_xy, step_yaw, span_xy, span_yaw in ((0.03, math.radians(0.5), search_xy, search_yaw),
                                                 (0.006, math.radians(0.1), 0.03, math.radians(0.5))):
        cx, cy, cyaw = best
        for dyaw in np.arange(cyaw - span_yaw, cyaw + span_yaw + 1e-9, step_yaw):
            for dx in np.arange(cx - span_xy, cx + span_xy + 1e-9, step_xy):
                for dy in np.arange(cy - span_xy, cy + span_xy + 1e-9, step_xy):
                    c = cost((dx, dy, dyaw))
                    if c < best_cost:
                        best, best_cost = (dx, dy, dyaw), c
    return best, best_cost


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('map', help='map yaml saved by map_saver_cli')
    ap.add_argument('--world', default='apartment')
    ap.add_argument('--spawn', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'YAW'),
                    help='robot spawn pose in the world = map frame origin')
    ap.add_argument('--search-xy', type=float, default=0.15, help='m, alignment search half-width')
    ap.add_argument('--search-yaw', type=float, default=3.0, help='deg, alignment search half-width')
    ap.add_argument('--match-dist', type=float, default=0.25, help='m, map wall cell belongs to a true wall')
    ap.add_argument('--coverage-dist', type=float, default=0.10, help='m')
    ap.add_argument('--target', type=float, default=0.10, help='m, wall-alignment error target')
    ap.add_argument('--out', default=None, help='overlay PNG (default: next to the map)')
    args = ap.parse_args()

    occ, free, res, origin, oyaw = load_map(args.map)
    if abs(oyaw) > 1e-6:
        raise SystemExit('map origin with a yaw is not supported')
    world = os.path.join(get_package_share_directory('jgb_rover_gazebo'), 'worlds', f'{args.world}.sdf')
    boxes, circles = load_world(world)
    truth = Truth(boxes, circles, res / 3.0)

    pts_map = cells_to_xy(occ, res, origin)
    pts0 = transform(pts_map, args.spawn[0], args.spawn[1], args.spawn[2])     # map -> world (spawn)
    raw = truth.lookup(pts0)
    (dx, dy, dyaw), _ = align(truth, pts0, args.search_xy, math.radians(args.search_yaw), args.match_dist)
    pts = transform(pts0, dx, dy, dyaw)
    d = truth.lookup(pts)
    wall = d <= args.match_dist

    # coverage of true surfaces inside the explored area (free or occupied map cells nearby)
    explored = cv2.dilate((occ | free).astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    surf_xy = cells_to_xy(truth.surface, truth.res, truth.origin)
    back = transform(surf_xy - [dx, dy], 0, 0, 0)
    back = transform(back, 0, 0, -dyaw)
    back = transform(back - args.spawn[:2], 0, 0, -args.spawn[2])
    ci = ((back[:, 0] - origin[0]) / res).astype(int)
    ri = ((back[:, 1] - origin[1]) / res).astype(int)
    inside = (ci >= 0) & (ci < occ.shape[1]) & (ri >= 0) & (ri < occ.shape[0])
    seen = np.zeros(len(surf_xy), bool)
    seen[inside] = explored[ri[inside], ci[inside]]
    occ_dist = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * res
    covered = np.zeros(len(surf_xy), bool)
    covered[inside] = occ_dist[ri[inside], ci[inside]] <= args.coverage_dist

    res_ = {
        'occupied_cells': int(len(d)),
        'alignment_dx_m': dx, 'alignment_dy_m': dy, 'alignment_dyaw_deg': math.degrees(dyaw),
        'wall_error_rms_m': float(np.sqrt(np.mean(d[wall] ** 2))),
        'wall_error_median_m': float(np.median(d[wall])),
        'wall_error_p90_m': float(np.percentile(d[wall], 90)),
        'wall_error_rms_unaligned_m': float(np.sqrt(np.mean(raw[raw <= args.match_dist] ** 2))),
        'false_wall_fraction': float(np.mean(~wall)),
        'coverage_of_explored_surfaces': float(covered[seen].mean()) if seen.any() else 0.0,
    }
    print(json.dumps(res_, indent=2))
    ok = res_['wall_error_rms_m'] < args.target
    print(f'wall-alignment error (RMS, matched cells): {100 * res_["wall_error_rms_m"]:.1f} cm '
          f'(target < {100 * args.target:.0f} cm) -> {"PASS" if ok else "FAIL"}')

    # overlay
    scale = 4
    h, w = occ.shape
    img = np.full((h, w, 3), 205, np.uint8)
    img[free] = (255, 255, 255)
    # true geometry, drawn in map coordinates
    tocc_xy = cells_to_xy(truth.occ, truth.res, truth.origin)
    b = transform(transform(tocc_xy - [dx, dy], 0, 0, -dyaw) - args.spawn[:2], 0, 0, -args.spawn[2])
    ci = ((b[:, 0] - origin[0]) / res).astype(int)
    ri = ((b[:, 1] - origin[1]) / res).astype(int)
    ok_i = (ci >= 0) & (ci < w) & (ri >= 0) & (ri < h)
    img[ri[ok_i], ci[ok_i]] = (150, 150, 150)
    r, c = np.nonzero(occ)
    colours = np.where((d < 0.05)[:, None], [40, 160, 40],
                       np.where((d < args.target)[:, None], [0, 150, 255],
                                np.where(wall[:, None], [0, 0, 230], [200, 0, 200])))
    img[r, c] = colours
    img = cv2.resize(img[::-1], (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
    legend = ['grey: true geometry (floor level)', 'green: wall < 5 cm', 'orange: < 10 cm',
              'red: >= 10 cm', 'magenta: no true wall nearby (false)']
    for i, text in enumerate(legend):
        cv2.putText(img, text, (8, 18 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, f'RMS wall error {100 * res_["wall_error_rms_m"]:.1f} cm, false {100 * res_["false_wall_fraction"]:.1f} %, '
                     f'coverage {100 * res_["coverage_of_explored_surfaces"]:.0f} %',
                (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    out = args.out or os.path.splitext(args.map)[0] + '_overlay.png'
    cv2.imwrite(out, img)
    with open(os.path.splitext(out)[0] + '.json', 'w') as f:
        json.dump(res_, f, indent=2)
    print(f'overlay: {out}')


if __name__ == '__main__':
    main()
