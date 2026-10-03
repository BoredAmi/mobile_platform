#!/usr/bin/env python3
"""Phase 1 acceptance helper: do the visual meshes line up with the collision primitives?

Expands the xacro, then for every link compares the axis-aligned bounding box of its visual
meshes (with the visual origins applied) with the bounding box of its collision geometry.
Rotations in origins are limited to what this robot uses (rpy about single axes).
"""
import math
import os
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(el):
    o = el.find('origin')
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if o is not None:
        xyz = np.array([float(v) for v in o.get('xyz', '0 0 0').split()])
        rpy = np.array([float(v) for v in o.get('rpy', '0 0 0').split()])
    return xyz, rpy_matrix(*rpy)


def stl_vertices(path):
    d = open(path, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    return a['v'].reshape(-1, 3).astype(float)


def geometry_points(geom):
    """Corner/extreme points of a primitive in its own frame."""
    if geom.find('box') is not None:
        s = np.array([float(v) for v in geom.find('box').get('size').split()]) / 2
        return np.array([[sx, sy, sz] for sx in (-s[0], s[0]) for sy in (-s[1], s[1]) for sz in (-s[2], s[2])])
    if geom.find('cylinder') is not None:
        c = geom.find('cylinder')
        r, h = float(c.get('radius')), float(c.get('length')) / 2
        ang = np.linspace(0, 2 * math.pi, 72, endpoint=False)
        ring = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)
        return np.vstack([np.c_[ring, np.full(len(ang), z)] for z in (-h, h)])
    if geom.find('sphere') is not None:
        r = float(geom.find('sphere').get('radius'))
        return np.array([[sx * r, sy * r, sz * r] for sx, sy, sz in
                         [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]])
    if geom.find('mesh') is not None:
        return stl_vertices(geom.find('mesh').get('filename').replace('file://', ''))
    raise ValueError('unknown geometry')


def bbox(elements):
    pts = []
    for el in elements:
        xyz, R = origin(el)
        pts.append((R @ geometry_points(el.find('geometry')).T).T + xyz)
    if not pts:
        return None
    p = np.vstack(pts)
    return p.min(0), p.max(0)


def main():
    share = get_package_share_directory('jgb_rover_description')
    xacro_args = sys.argv[1:]
    urdf = subprocess.check_output(['xacro', os.path.join(share, 'urdf', 'jgb_rover.urdf.xacro')] + xacro_args)
    root = ET.fromstring(urdf)
    print(f'{"link":20s} {"":9s} {"min x,y,z [mm]":>26s}   {"max x,y,z [mm]":>26s}')
    for link in root.findall('link'):
        vis = bbox(link.findall('visual'))
        col = bbox(link.findall('collision'))
        if vis is None and col is None:
            continue
        for label, bb in (('visual', vis), ('collision', col)):
            if bb is None:
                print(f'{link.get("name"):20s} {label:9s} (none)')
                continue
            lo, hi = bb[0] * 1e3, bb[1] * 1e3
            print(f'{link.get("name"):20s} {label:9s} '
                  f'{lo[0]:8.1f}{lo[1]:8.1f}{lo[2]:8.1f}   {hi[0]:8.1f}{hi[1]:8.1f}{hi[2]:8.1f}')
        if vis is not None and col is not None:
            d = np.maximum(np.abs(vis[0] - col[0]), np.abs(vis[1] - col[1])) * 1e3
            print(f'{"":20s} {"max diff":9s} {d[0]:8.1f}{d[1]:8.1f}{d[2]:8.1f}  (x, y, z mm)')


if __name__ == '__main__':
    main()
