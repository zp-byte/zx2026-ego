#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_alt_timeline.py <cell_dir> [<cell_dir> ...] — 高度 vs 目标高度 时间线（只读）。

回答一个具体问题：CPN 持笔期的那个"掉高度"，是**连续下压**，还是在
**换腿的瞬间阶梯式掉**？两者的根因完全不同：
  * 连续下压  => CPN 的基元选择本身有向下偏置（要动基元库/代价函数）
  * 阶梯式掉  => executor 每腿只发一次 goal（_goto L692），而 _cruise_z_goal()
                 取的是"发目标那一刻的高度" => 目标随高度一起往下棘轮
把每机的 ALT 序列十等分，逐档印 z 中位 + goal z 中位（只看 cpn=1）。
"""
import io
import os
import re
import sys

PAT = re.compile(
    r"drone (\d+) ALT z=([-\d.]+) gz=([-\d.]+) cmd=([-\d.]+) cpn=(\d)")


def med(xs):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[len(xs) // 2]


for cell in sys.argv[1:]:
    ros = os.path.join(cell, "rosout.log")
    if not os.path.isfile(ros):
        print("!! %s 无 rosout.log" % cell)
        continue
    per = {}
    for ln in io.open(ros, encoding="utf-8", errors="ignore"):
        m = PAT.search(ln)
        if m and int(m.group(5)) == 1:
            per.setdefault(int(m.group(1)), []).append(
                (float(m.group(2)), float(m.group(3)), float(m.group(4))))
    print("=" * 100)
    print("格 %s   （格式: z / goal_z / nav_cmd_z  中位；每档 = 该机 cpn=1 序列的 1/10）"
          % os.path.basename(cell))
    print("=" * 100)
    for d in sorted(per):
        seq = per[d]
        if len(seq) < 20:
            print("  d%d  CPN拍数仅 %d，跳过" % (d, len(seq)))
            continue
        cells = []
        for k in range(10):
            seg = seq[k * len(seq) // 10:(k + 1) * len(seq) // 10]
            if not seg:
                continue
            cells.append("%.2f/%.2f/%+.2f"
                         % (med([s[0] for s in seg]), med([s[1] for s in seg]),
                            med([s[2] for s in seg])))
        print("  d%d  n=%4d  %s" % (d, len(seq), "  ".join(cells)))
    print("")
