#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""col_render.py — 碰撞点位姿的离线深度渲染对质（无 ROS，纯真值几何）。

用法: cd <ws> && python3 tools/col_render.py <batch_dir|cell_dir> ...

对每起碰撞：机心 = 撞点 + dir·R（dir=弹开方向=树→机），朝向 = -dir（面向树）；
渲染该位姿深度图 → dmin(cpn 口径) / dnear(<1.0m) / 正前方 ±8° 中心带最小深度
⇒ 与实测 tl 的 dmin/dnear 对质，判「撞的那棵树在不在深度图上」。
R 两档：0.5（球-柱判据接触位）、1.0（半程）。
注意碰撞判据是**软件球-柱**（drone_radius 0.35 + trunk_r），非物理盒。
"""
import glob
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

PAT = re.compile(r'^([0-9.]+).*drone (\d+) COLLISION #(\d+) kind=(\w+) at '
                 r'\(([^)]+)\) dir=\(([^)]+)\)')


def main():
    settings = cfg.load("sim_settings.yaml")
    dcfg = dsn.make_depth_cfg(settings["sensor"]["depth"])
    scene = Scene()
    cam_dirs, inv_len = dsn.ray_dirs(dcfg)
    w, h = dcfg["width"], dcfg["height"]
    tmax = dcfg["max_range"]
    fx = (w / 2.0) / math.tan(math.radians(dcfg["hfov_deg"]) / 2.0)
    half = int(round(fx * math.tan(math.radians(8.0))))
    c0, c1 = max(0, w // 2 - half), min(w, w // 2 + half)

    for batch in sys.argv[1:]:
        cells = sorted(glob.glob(os.path.join(batch, 'collisions.txt')))
        if not cells:
            cells = sorted(glob.glob(os.path.join(batch, '*',
                                                  'collisions.txt')))
        print('\n########## %s' % os.path.basename(batch.rstrip('/')))
        for cf in cells:
            print('=== %s' % os.path.basename(os.path.dirname(cf)))
            for ln in open(cf):
                m = PAT.search(ln)
                if not m:
                    continue
                ts, d, _n, kind, xyz, ddd = m.groups()
                t = float(ts)
                x, y, z = [float(v) for v in xyz.split(',')]
                dx, dy = [float(v) for v in ddd.split(',')[:2]]
                nn = math.hypot(dx, dy) or 1.0
                dx, dy = dx / nn, dy / nn
                yaw = math.atan2(-dy, -dx)      # 面向树（dir 是树→机）
                out = []
                for R in (0.5, 1.0):
                    pos = (x + dx * R, y + dy * R, z)
                    img = dsn.render_depth(scene, pos, yaw, dcfg,
                                           random.Random(4242),
                                           cam_dirs, inv_len)
                    dd = img.astype(np.float64) / 1000.0
                    val = np.isfinite(dd) & (dd > 0.0)      # cpn 口径
                    dmin = float(dd[val].min()) if val.any() else None
                    dnear = int(np.count_nonzero(val & (dd < 1.0)))
                    cen = dd[:, c0:c1]
                    cval = cen[np.isfinite(cen) & (cen > 0.0)
                               & (cen < tmax - 1e-9)]
                    cmin = float(cval.min()) if cval.size else None
                    out.append('R%.1f: dmin=%s dnear=%-6d 中心=%s'
                               % (R, ('%.2f' % dmin) if dmin is not None
                                  else 'None', dnear,
                                  ('%.2f' % cmin) if cmin is not None
                                  else 'None'))
                print(' t=%6.1f d%s %-6s (%6.2f,%6.2f,%5.2f) | %s'
                      % (t, d, kind, x, y, z, '  |  '.join(out)))


if __name__ == '__main__':
    main()
