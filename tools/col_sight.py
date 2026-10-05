#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""col_sight.py — 撞拍「CPN 实见 vs 真值渲染」并排对质（离线）。

用法: cd <ws> && python3 tools/col_sight.py <batch_dir> ...

对每起碰撞取撞拍那一拍的 CPN 遥测（dmin/dnear/az/yaw/veto/safe/cod/v），
再用**同一时刻的真实位姿/真航向**离线渲染深度图，给真值 dmin/dnear/中心带。
两者并排 ⇒ 判「CPN 是被喂了错图（感知）」还是「看见了仍撞（决策）」。
"""
import glob
import json
import math
import os
import random
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "zx2026_common", "scripts"))
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_sensor", "scripts"))

from zx2026_common import config as cfg          # noqa: E402
from zx2026_common.scene import Scene            # noqa: E402
import depth_sim_node as dsn                     # noqa: E402

COLL = re.compile(r'^([0-9.]+).*drone (\d+) COLLISION #(\d+) kind=(\w+) at '
                  r'\(([^)]+)\)')


def main():
    settings = cfg.load("sim_settings.yaml")
    dcfg = dsn.make_depth_cfg(settings["sensor"]["depth"])
    scene = Scene()
    cam_dirs, inv_len = dsn.ray_dirs(dcfg)
    w = dcfg["width"]
    fx = (w / 2.0) / math.tan(math.radians(dcfg["hfov_deg"]) / 2.0)
    half = int(round(fx * math.tan(math.radians(8.0))))
    c0, c1 = max(0, w // 2 - half), min(w, w // 2 + half)
    dr = float(scene.drone_radius)
    print("drone_radius=%.2f crown_shape=%s" % (dr, dcfg.get("crown_shape")))
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
                ent = ddbg.get(str(d)) or {}
                rows = [r for r in ent.get("tl", [])
                        if isinstance(r, list) and len(r) >= 14
                        and abs(r[0] - t) <= 0.6]
                if not rows:
                    print(" t=%7.2f d%d %-6s %s | CPN 无逐拍" % (t, d, kind,
                                                                  (x, y, z)))
                    continue
                r0 = min(rows, key=lambda r: abs(r[0] - t))
                yaw = math.radians(r0[10] or 0.0)
                # 真值渲染：撞点位姿 + 撞拍真航向
                img = dsn.render_depth(scene, (x, y, z), yaw, dcfg,
                                       random.Random(4242), cam_dirs, inv_len)
                dd = img.astype(np.float64) / 1000.0
                val = np.isfinite(dd) & (dd > 0.0)
                rdmin = float(dd[val].min()) if val.any() else None
                rdnear = int(np.count_nonzero(val & (dd < 1.0)))
                cen = dd[:, c0:c1]
                cval = cen[np.isfinite(cen) & (cen > 0.0)]
                rcmin = float(cval.min()) if cval.size else None
                print(" t=%7.2f d%d %-6s z=%5.2f yaw=%6.1f cod=%s veto=%-3s "
                      "safe=%-3s | CPN dmin=%-7s dnear=%-6s az=%-7s v=%s"
                      % (t, d, kind, z, math.degrees(yaw), r0[1], r0[3], r0[4],
                         r0[8], r0[6], r0[9],
                         round(math.hypot(r0[11] or 0.0, r0[12] or 0.0), 2)))
                print(" %22s 真值 dmin=%-7s dnear=%-6d 中心=%-7s"
                      % ("", ('%.2f' % rdmin) if rdmin is not None else 'None',
                         rdnear,
                         ('%.2f' % rcmin) if rcmin is not None else 'None'))


if __name__ == '__main__':
    main()
