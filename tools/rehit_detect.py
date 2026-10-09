#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同格再撞检测（起 2 判据仪，2026-10-08）：撞后回到刚撞过的同一障碍再撞。

口径（本仓血训固化，勿改）：
  * 世界层真值 token = **`COLLIDED obstacle`**（world_node.py:335）。
    **绝不**用 `grep -c COLLISION` —— 该子串出现在 COLLISION_THRESHOLD 里，
    恒返回 0（2026-10-07 我的口径事故：报了「碰撞 0」而真值 6）。
  * 只读 rosout.log（epoch 时间戳）；nav_node_*.log 是墙钟，**不可与撞点对齐**。

「同格再撞」定义（= 起 2 的判据，窗与 deadend_escape.contact_ttl_s 同源）：
  同一机两次撞击满足  Δt ≤ ttl_s  且  水平间距 ≤ r_m
  ⇒ 撞后返回/被推回同一障碍再撞。这是「撞后逃逸把机推回同一棵树」的直接读数，
    比 Σcol 敏感得多（Σcol 里三类机制混在一起）。

用法:
  python3 tools/rehit_detect.py                  # 最新一局
  python3 tools/rehit_detect.py <uuid 前缀>
  python3 tools/rehit_detect.py --list           # 列近 12 局
  python3 tools/rehit_detect.py <uuid> --ttl 12 --r 1.0
退出码：恒 0（纯读数仪，不做判分）。
"""
import io
import os
import re
import sys

LOG_ROOT = "/home/ubuntu/.ros/log"
RE_HIT = re.compile(
    r"^(\d{10}\.\d+)\s+\w+\s+/\S+\s+\[[^\]]*\]\s+\[topics:[^\]]*\]\s+"
    r"drone (\d+) COLLIDED obstacle at \((-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)\)")


def list_runs(n=12):
    if not os.path.isdir(LOG_ROOT):
        return []
    out = []
    for d in os.listdir(LOG_ROOT):
        p = os.path.join(LOG_ROOT, d)
        r = os.path.join(p, "rosout.log")
        if os.path.isfile(r):
            out.append((os.path.getmtime(r), d))
    out.sort(reverse=True)
    return [d for _, d in out[:n]]


def resolve(arg):
    cands = list_runs(500)
    for d in cands:
        if d.startswith(arg):
            return d
    return None


def parse_hits(uuid):
    """→ [(t, drone, x, y, z)]，按 (drone,t,xy) 去重（跨源副本只留一份）。"""
    path = os.path.join(LOG_ROOT, uuid, "rosout.log")
    hits, seen = [], set()
    with io.open(path, encoding="utf-8", errors="ignore") as f:
        for ln in f:
            m = RE_HIT.match(ln)
            if not m:
                continue
            t = float(m.group(1))
            d = int(m.group(2))
            x, y, z = (float(m.group(i)) for i in (3, 4, 5))
            key = (d, round(t, 3), round(x, 3), round(y, 3))
            if key in seen:
                continue
            seen.add(key)
            hits.append((t, d, x, y, z))
    hits.sort()
    return hits


def main(argv):
    ttl, r_m, arg = 12.0, 1.0, None
    i = 1
    while i < len(argv):
        a = argv[i]
        if a == "--list":
            for d in list_runs():
                print(d)
            return 0
        if a == "--ttl":
            i += 1
            ttl = float(argv[i])
        elif a == "--r":
            i += 1
            r_m = float(argv[i])
        elif not a.startswith("-"):
            arg = a
        i += 1

    uuid = resolve(arg) if arg else (list_runs(1) or [None])[0]
    if uuid is None:
        print("REHIT: 找不到可用的局（%s）" % LOG_ROOT)
        return 0

    hits = parse_hits(uuid)
    print("局 %s  ——  同格再撞检测  ttl=%.1fs  r=%.2fm" % (uuid, ttl, r_m))
    print("=" * 74)
    print("真值撞点 %d 起（token = `COLLIDED obstacle`）：" % len(hits))
    for t, d, x, y, z in hits:
        print("   t=%10.3f  drone %d  (%.2f, %.2f, %.2f)" % (t, d, x, y, z))

    t0 = hits[0][0] if hits else 0.0
    pairs, seen = [], set()
    for j in range(1, len(hits)):
        tj, dj, xj, yj, _ = hits[j]
        for i2 in range(j - 1, -1, -1):
            ti, di, xi, yi, _ = hits[i2]
            if tj - ti > ttl:
                break
            if di != dj:
                continue
            dist = ((xj - xi) ** 2 + (yj - yi) ** 2) ** 0.5
            if dist > r_m:
                continue
            key = (di, i2, j)
            if key in seen:
                continue
            seen.add(key)
            pairs.append((dj, ti, tj, dist, xi, yi))

    print("\n同格再撞（同机  Δt≤%.0fs  且  间距≤%.2fm）：**%d 起**"
          % (ttl, r_m, len(pairs)))
    for d, ti, tj, dist, xi, yi in pairs:
        print("   drone %d  前撞 t=%+7.2f → 再撞 t=%+7.2f  (Δt=%5.1fs, 间距=%.2fm)"
              "  于 (%.2f, %.2f)" % (d, ti - t0, tj - t0, tj - ti, dist, xi, yi))
    if not pairs:
        print("   （无）")

    per = {}
    for _t, d, _x, _y, _z in hits:
        per[d] = per.get(d, 0) + 1
    print("\n按机分布：" + ("  ".join("d%d=%d" % kv for kv in sorted(per.items()))
                            if per else "（无撞击）"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
