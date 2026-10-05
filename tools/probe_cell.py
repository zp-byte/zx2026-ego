#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_cell.py <cell_dir> — 单格深度判读：ALT 时序 + 判分 + 碰撞。"""
import io
import os
import re
import sys

PAT = re.compile(r"drone (\d+) ALT z=([-\d.]+) gz=([-\d.]+) cmd=([-\d.]+) cpn=(\d)")


def pct(xs, q):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))]


cell = sys.argv[1]
print("=" * 84)
print("格: %s" % os.path.basename(cell))
print("=" * 84)

# ---- 判分 ----
sy = os.path.join(cell, "score.yaml")
if os.path.isfile(sy):
    print("--- score.yaml ---")
    print(io.open(sy, encoding="utf-8").read().strip()[:1200])

cx = os.path.join(cell, "collisions.txt")
if os.path.isfile(cx):
    txt = io.open(cx, encoding="utf-8", errors="ignore").read().strip()
    print("\n--- collisions.txt (%d 行) ---" % len(txt.split("\n")))
    print(txt[:900])

# ---- ALT 时序 ----
ros = os.path.join(cell, "rosout.log")
if not os.path.isfile(ros):
    print("\n(无 rosout.log)")
    sys.exit(0)

rows = []
for ln in io.open(ros, encoding="utf-8", errors="ignore"):
    m = PAT.search(ln)
    if m:
        rows.append((int(m.group(1)), float(m.group(2)), float(m.group(3)),
                     float(m.group(4)), int(m.group(5))))
print("\n--- ALT 行数 %d ---" % len(rows))

# 每机：cpn=1 段的 z 分位 + cpn=1 出现区间（按行序近似时序）
print("%-8s %-46s %-46s" % ("drone", "cpn=1 段  n/z均值/zp50/zp90/max", "cpn=0 段  n/z均值/zp50/zp90/max"))
for d in sorted(set(r[0] for r in rows)):
    one = [r for r in rows if r[0] == d and r[4] == 1]
    zer = [r for r in rows if r[0] == d and r[4] == 0]
    def f(g):
        z = [r[1] for r in g]
        if not z:
            return "n=0"
        return "n=%d m=%.2f p50=%.2f p90=%.2f mx=%.2f" % (
            len(z), sum(z) / len(z), pct(z, .5), pct(z, .9), max(z))
    print("d%-7d %-46s %-46s" % (d, f(one), f(zer)))

# cpn 段的时序占比（分十档）
print("\n--- cpn=1 占比时序（把每机 ALT 序列十等分，每档 cpn=1 比例 / z 中位）---")
for d in sorted(set(r[0] for r in rows)):
    seq = [r for r in rows if r[0] == d]
    n = len(seq)
    if n < 10:
        continue
    out = []
    for k in range(10):
        seg = seq[k * n // 10:(k + 1) * n // 10]
        c1 = sum(1 for r in seg if r[4] == 1) / float(len(seg))
        out.append("%.0f%%/%.2f" % (c1 * 100, pct([r[1] for r in seg], .5)))
    print("d%-3d %s" % (d, " ".join(out)))
print("\n(每档 = 该机序列的 1/10，格式: cpn=1占比 / 该档 z 中位)")
