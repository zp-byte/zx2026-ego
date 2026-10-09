#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ego_col_fov.py — ego py 后端碰撞「树在航向哪一侧」离线核算（只读，无 ROS）。

用法: cd <ws> && python3 tools/ego_col_fov.py <uuid> ...

背景：本仓三型法证（col_fov/col_geom/col_sight）吃的是 DeFoP 线的
`drone N COLLISION #N kind=W at` 格式；ego py 后端发的是
`drone N COLLIDED obstacle at (x,y,z)` ⇒ 本器照 col_geom.py 的 Scene 取法复刻，
真航向取 nav_node 自己的 HEADING-ALIGN 遥测（`yaw_deg`）。

判据：
  ① |dpsi| > hfov/2 (43.5°) ⇒ 撞的那棵树**在相机半视场外**（结构性看不见）。
    这是 yaw 冻结时代的签名；strict 若真治了病，① 占比应显著下降。
  ② 触干 vs 离轴：撞点是否落在 (trunk_r + drone_radius) 判据内。

⚠ 诚实边界：HEADING-ALIGN 是 loginfo_throttle(1.0) ⇒ ~1Hz，撞前最近一拍最多
  早 1.0s；门控中 yaw 速率上限 1.5 rad/s ⇒ 极端可差 86°。故 **gated=0 的撞
  （yaw 近静止）结论可信；gated=1 的撞只作参考** —— 输出里带 gated 与 Δt。
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "zx2026_common", "scripts"))

from zx2026_common.scene import Scene      # noqa: E402

CO = re.compile(r'drone (\d+) COLLIDED obstacle at \(([-\d.]+),([-\d.]+),([-\d.]+)\)')
HA = re.compile(r'drone (\d+) HEADING-ALIGN mode=\w+ gated=(\d) align_s=([\d.]+) '
                r'yaw_deg=(-?[\d.]+) rate=(-?[\d.]+) vh=([\d.]+)')
HB = re.compile(r'(?:^|\s)ha=([a-z]+)')
HFOV_HALF = 43.5


def wrap180(a):
    while a <= -180.0:
        a += 360.0
    while a > 180.0:
        a -= 360.0
    return a


def parse(path):
    ha, col, mode = {}, [], None
    for ln in open(path, errors='ignore'):
        if mode is None:
            m = HB.search(ln)
            if m:
                mode = m.group(1)
        m = HA.search(ln)
        if m:
            i = ln.find(' ')
            try:
                t = float(ln[:i])
            except ValueError:
                continue
            ha.setdefault(int(m.group(1)), []).append(
                (t, int(m.group(2)), float(m.group(4)), float(m.group(6))))
            continue
        m = CO.search(ln)
        if m:
            i = ln.find(' ')
            try:
                t = float(ln[:i])
            except ValueError:
                continue
            col.append((t, int(m.group(1)), float(m.group(2)),
                        float(m.group(3)), float(m.group(4))))
    return ha, col, mode


def main():
    scene = Scene()
    trees = [ob for ob in scene.obstacles if ob.kind == "tree"]
    dr = float(scene.drone_radius)
    print("drone_radius=%.3f  n_trees=%d" % (dr, len(trees)))
    agg = {}
    for u in sys.argv[1:]:
        p = '/home/ubuntu/.ros/log/%s/rosout.log' % u
        if not os.path.exists(p):
            print("(%s 缺)" % u[:8])
            continue
        ha, col, mode = parse(p)
        mode = mode or '无'
        print("\n=== %s  ha=%s  %d 撞 ===" % (u[:8], mode, len(col)))
        st = agg.setdefault(mode, [0, 0, 0, 0, 0])   # n,①,②,无航向,非树干
        for (tc, d, x, y, z) in col:
            best = min(trees, key=lambda ob: math.hypot(x - ob.cx, y - ob.cy))
            dxy = math.hypot(x - best.cx, y - best.cy)
            rz = best.trunk_r + dr
            # 2026-10-08 补洞：判据 world_node.py:330 是 scene.collides(...,地面内建)。
            # 树场 x∈[-23.3,29.3] ⇒ 树场外或 dxy 远超 (trunk_r+dr) 的「撞」不可能
            # 是撞树；z≈drone_radius 的更是贴地。这些若混进来会把 ①② 统计污染。
            if dxy > rz + 0.15:
                why = "非树干接触(z=%.2f,dxy=%.2f)" % (z, dxy)
                print("  t=%+.1f d%d (%6.1f,%6.1f) z=%.2f | 近树(%6.1f,%6.1f) "
                      "dxy=%.2f/%.2f ⇒ %s" % (tc - col[0][0], d, x, y, z,
                                              best.cx, best.cy, dxy, rz, why))
                st[4] += 1
                continue
            touch = "触干" if dxy <= rz else "离轴"
            brg = math.degrees(math.atan2(best.cy - y, best.cx - x))
            s = ha.get(d, [])
            j = None
            for k in range(len(s) - 1, -1, -1):
                if s[k][0] <= tc:
                    j = k
                    break
            if j is None:
                print("  t=%+.1f d%d (%6.1f,%6.1f) z=%.2f | 近树(%6.1f,%6.1f) "
                      "dxy=%.3f/%.2f %-4s | 真方位%7.1f° | 无航向遥测"
                      % (tc - col[0][0], d, x, y, z, best.cx, best.cy, dxy, rz,
                         touch, brg))
                st[3] += 1
                continue
            dt, gated, yaw, vh = s[j]
            dpsi = wrap180(brg - yaw)
            out = abs(dpsi) > HFOV_HALF
            st[0] += 1
            st[1 if out else 2] += 1
            print("  t=%+.1f d%d (%6.1f,%6.1f) z=%.2f | 近树(%6.1f,%6.1f) "
                  "dxy=%.3f/%.2f %-4s | 真方位%7.1f° yaw=%7.1f° ⇒ dpsi=%7.1f°  "
                  "%s  [gated=%d Δt=%.2f vh=%.2f]"
                  % (tc - col[0][0], d, x, y, z, best.cx, best.cy, dxy, rz, touch,
                     brg, yaw, dpsi, "①视场外" if out else "②视场内", gated,
                     tc - dt, vh))
    print("\n########## 汇总（①=半视场外结构性看不见 / ②=视场内；真树干接触）")
    for k in ('无', 'off', 'track', 'strict'):
        if k in agg:
            n, o, i, g, nb = agg[k]
            if n:
                print("  %-6s 真树干 n=%2d  ① %2d (%4.1f%%)  ② %2d (%4.1f%%)"
                      "   | 非树干(贴地/树场外) %2d   无航向 %d"
                      % (k, n, o, 100.0 * o / n, i, 100.0 * i / n, nb, g))
            else:
                print("  %-6s 真树干 n=0  | 非树干 %d   无航向 %d" % (k, nb, g))


if __name__ == '__main__':
    main()
