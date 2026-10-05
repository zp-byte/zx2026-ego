#!/usr/bin/env python3
"""③ 几何法证：从 verify.txt 还原撞前位姿/速度/航向，与碰撞点、树位置对质。

用法: python3 tools/sup33_pose.py run_logs/<批> <cell> <drone> <t_collision>
"""
import math
import os
import re
import sys

POSE = re.compile(r"^\s*t=(\d+) (.*)$")
DR = re.compile(r"drone(\d+)=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)")


def poses(path, drone):
    out = []
    if not os.path.exists(path):
        return out
    for ln in open(path, errors="ignore"):
        m = POSE.match(ln)
        if not m:
            continue
        t = int(m.group(1))
        for dm in DR.finditer(m.group(2)):
            if int(dm.group(1)) == drone:
                out.append((t, float(dm.group(2)), float(dm.group(3)),
                            float(dm.group(4))))
    return sorted(out)


def main():
    base, cell, d, tc = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
    ps = poses(os.path.join(base, cell, "verify.txt"), d)
    print("drone%d 位姿样本 %d 条" % (d, len(ps)))
    if len(ps) < 3:
        return
    # 碰撞原文
    cp = os.path.join(base, cell, "collisions.txt")
    if os.path.exists(cp):
        for ln in open(cp, errors="ignore"):
            if ("drone %d COLLISION" % d) in ln:
                mm = re.search(r"at \(([-\d.]+),([-\d.]+),([-\d.]+)\) dir=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", ln)
                if mm:
                    cx, cy, cz = (float(mm.group(i)) for i in (1, 2, 3))
                    dx, dy = float(mm.group(4)), float(mm.group(5))
                    # dir = 树轴 -> 机；机在树的 dir 方向上
                    print("  碰撞点 (%.2f, %.2f, %.2f)  dir=(%.2f,%.2f)" % (cx, cy, cz, dx, dy))
                    if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                        n = math.hypot(dx, dy)
                        ux, uy = dx / n, dy / n
                        for off in (0.4, 0.6, 0.8):
                            print("    树轴约在 (%.2f, %.2f)  [假设 dir*%.1fm]"
                                  % (cx - ux * off, cy - uy * off, off))
    print()
    print("     t      x        y       z     v(m/s)   航向(deg, 由位移得)")
    for i, (t, x, y, z) in enumerate(ps):
        if not (tc - 9 <= t <= tc + 2):
            continue
        if i > 0 and t - ps[i - 1][0] > 0:
            dt = t - ps[i - 1][0]
            vx, vy = (x - ps[i - 1][1]) / dt, (y - ps[i - 1][2]) / dt
            v = math.hypot(vx, vy)
            hd = math.degrees(math.atan2(vy, vx))
        else:
            v, hd, vx, vy = float("nan"), float("nan"), float("nan"), float("nan")
        tag = "   <== 碰撞" if abs(t - tc) < 3 else ""
        print("  %5d  %8.2f %8.2f %6.2f   %6.2f    %8.1f%s" % (t, x, y, z, v, hd, tag))
    # 撞点反推：最近的一秒里，机与碰撞点连线方向 vs 航向
    near = [p for p in ps if tc - 3 <= p[0] <= tc]
    if len(near) >= 2 and cp and os.path.exists(cp):
        for ln in open(cp, errors="ignore"):
            if ("drone %d COLLISION" % d) in ln:
                mm = re.search(r"at \(([-\d.]+),([-\d.]+),", ln)
                if mm:
                    cx, cy = float(mm.group(1)), float(mm.group(2))
                    x0, y0 = near[0][1], near[0][2]
                    print("\n  撞前 3s: 起点(%.2f,%.2f) 木标(%.2f,%.2f) 位移 %.2f m"
                          % (x0, y0, cx, cy, math.hypot(cx - x0, cy - y0)))


if __name__ == "__main__":
    main()
