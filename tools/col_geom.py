#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""col_geom.py — 碰撞点几何真身 + 撞拍速度（离线，只读）。

用法: cd <ws> && python3 tools/col_geom.py <batch_dir> ...

对每起碰撞: 撞点 → 最近树的真实几何（trunk_r/trunk_h/crown_z/crown_h）/
到树干轴水平距离 vs 判据 (trunk_r+drone_radius) / z 是否落在树干带 /
该点附近真枝命中数（球-线段同几何）/ 撞拍 CPN 指令速度（probe tl vwx,vwy）。
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "zx2026_common", "scripts"))

from zx2026_common import config as cfg          # noqa: E402
from zx2026_common import geometry as geo       # noqa: E402
from zx2026_common.scene import Scene            # noqa: E402

COLL = re.compile(r'^([0-9.]+).*drone (\d+) COLLISION #(\d+) kind=(\w+) at '
                  r'\(([^)]+)\)')


def main():
    scene = Scene()
    trees = [ob for ob in scene.obstacles if ob.kind == "tree"]
    dr = float(scene.drone_radius)
    print("drone_radius=%.3f  n_trees=%d  ground_z=%.2f"
          % (dr, len(trees), scene.venue["ground_z"]))
    for base in sys.argv[1:]:
        print("\n########## %s" % os.path.basename(base.rstrip('/')))
        for cname in sorted(os.listdir(base)):
            cp = os.path.join(base, cname, "collisions.txt")
            if not os.path.exists(cp):
                continue
            pp = os.path.join(base, cname, "probe.json")
            ddbg = (json.load(open(pp)).get("defop_debug", {})
                    if os.path.exists(pp) else {})
            print("=== %s" % cname)
            for ln in open(cp, errors="ignore"):
                m = COLL.search(ln)
                if not m:
                    continue
                t = float(m.group(1))
                d = int(m.group(2))
                kind = m.group(4)
                x, y, z = [float(v) for v in m.group(5).split(',')]
                best = min(trees, key=lambda ob: math.hypot(x - ob.cx,
                                                            y - ob.cy))
                dxy = math.hypot(x - best.cx, y - best.cy)
                rz = best.trunk_r + dr
                in_trunk_z = (z - dr <= best.trunk_h)
                contact = dxy <= rz and in_trunk_z
                nbr = 0
                for seg in (best.branches or []):
                    sx, sy, sz, ex, ey, ez, br = seg
                    if geo.point_segment_dist((x, y, z), (sx, sy, sz),
                                              (ex, ey, ez)) <= br + dr:
                        nbr += 1
                if best.crown_z is not None and best.crown_h:
                    cz = "[%.2f,%.2f]" % (best.crown_z - best.crown_h / 2.0,
                                          best.crown_z + best.crown_h / 2.0)
                else:
                    cz = "-"
                ent = ddbg.get(str(d)) or {}
                rows = [r for r in ent.get("tl", [])
                        if isinstance(r, list) and len(r) >= 14
                        and abs(r[0] - t) <= 0.6]
                v = None
                if rows:
                    r0 = min(rows, key=lambda r: abs(r[0] - t))
                    v = math.hypot(r0[11] or 0.0, r0[12] or 0.0)
                print(" t=%7.2f d%d %-6s z=%5.2f | 近树(%7.2f,%7.2f) "
                      "tr=%4.2f th=%5.2f dxy=%5.3f/(%4.2f) %-4s z带%s | 冠%-16s "
                      "| 近枝%2d | v=%s"
                      % (t, d, kind, z, best.cx, best.cy, best.trunk_r,
                         best.trunk_h, dxy, rz,
                         "触干" if dxy <= rz else "离轴",
                         "内" if in_trunk_z else "外", cz, nbr,
                         ("%.2f" % v) if v is not None else "-"))


if __name__ == '__main__':
    main()
