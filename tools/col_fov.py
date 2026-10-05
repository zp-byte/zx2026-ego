#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""col_fov.py — 碰撞点「树在航向的哪一侧」离线核算（无 ROS）。

用法: cd <ws> && python3 tools/col_fov.py <batch_dir> ...

对每起碰撞: 取撞点真方位（撞点 -> 最近树心）+ 撞拍真航向（probe tl col 10）
=> 相对方位角 dpsi（-180..180）。|dpsi| > hfov/2 (43.5°) === 树在相机
半视场外 === 「根本看不见」的结构性失效，与 dmin 无关。
"""
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "zx2026_common", "scripts"))

from zx2026_common import config as cfg      # noqa: E402
from zx2026_common.scene import Scene        # noqa: E402

COLL = re.compile(r'^([0-9.]+).*drone (\d+) COLLISION #(\d+) kind=(\w+) at '
                  r'\(([^)]+)\)')


def main():
    settings = cfg.load("sim_settings.yaml")
    hfov2 = float(settings["sensor"]["depth"]["hfov_deg"]) / 2.0
    scene = Scene()
    trees = [ob for ob in scene.obstacles if ob.kind == "tree"]
    print("hfov/2 = %.1f deg  n_trees=%d" % (hfov2, len(trees)))
    for base in sys.argv[1:]:
        print("\n########## %s" % os.path.basename(base.rstrip('/')))
        for cname in sorted(os.listdir(base)):
            cp = os.path.join(base, cname, "collisions.txt")
            if not os.path.exists(cp):
                continue
            pp = os.path.join(base, cname, "probe.json")
            ddbg = (json.load(open(pp)).get("defop_debug", {})
                    if os.path.exists(pp) else {})
            n_out = n_in = 0
            print("=== %s" % cname)
            for ln in open(cp, errors="ignore"):
                m = COLL.search(ln)
                if not m:
                    continue
                t = float(m.group(1))
                d = int(m.group(2))
                kind = m.group(4)
                x, y, z = [float(v) for v in m.group(5).split(',')]
                tr = min(trees, key=lambda ob: math.hypot(x - ob.cx,
                                                          y - ob.cy))
                brg = math.degrees(math.atan2(tr.cy - y, tr.cx - x))
                ent = ddbg.get(str(d)) or {}
                rows = [r for r in ent.get("tl", [])
                        if isinstance(r, list) and len(r) >= 14
                        and abs(r[0] - t) <= 0.6]
                if not rows:
                    print(" t=%7.2f d%d %-6s z=%5.2f 方位=%7.1f | 无逐拍"
                          % (t, d, kind, z, brg))
                    continue
                r0 = min(rows, key=lambda r: abs(r[0] - t))
                yaw = r0[10] or 0.0
                dpsi = (brg - yaw + 180.0) % 360.0 - 180.0
                out = abs(dpsi) > hfov2
                n_out += out
                n_in += (not out)
                v = math.hypot(r0[11] or 0.0, r0[12] or 0.0)
                print(" t=%7.2f d%d %-6s z=%5.2f yaw=%7.1f 方位=%7.1f "
                      "dpsi=%7.1f %-8s | CPN dmin=%-7s veto=%-3s v=%.2f"
                      % (t, d, kind, z, yaw, brg, dpsi,
                         "视场外" if out else "视场内",
                         r0[8], r0[3], v))
            if n_out + n_in:
                print(" >>> 视场外 %d / 视场内 %d" % (n_out, n_in))


if __name__ == '__main__':
    main()
