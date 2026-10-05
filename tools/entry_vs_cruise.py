#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""entry_vs_cruise.py <归档...> — 分「进林带瞬间高度」与「巡航稳态高度」。

回答一个具体问题：解绑把 nav 的靶抬上去了，但机体到底有没有被抬高？
做法：对每机取自己的 ALT 序列，只看 cpn=1（CPN 持笔）段，
  * 入口高度 = CPN 段**前 10%** 拍的 z 中位
  * 稳态高度 = CPN 段**后 70%** 拍的 z 中位
若入口抬了而稳态没抬 ⇒ CPN 自己把机子压下去了（nav 靶无效）。
"""
import glob
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


def onecell(cell):
    ros = os.path.join(cell, "rosout.log")
    if not os.path.isfile(ros):
        return None
    per = {}
    for ln in io.open(ros, encoding="utf-8", errors="ignore"):
        m = PAT.search(ln)
        if m and int(m.group(5)) == 1:
            per.setdefault(int(m.group(1)), []).append(float(m.group(2)))
    if not per:
        return None
    ent, sus, gz = [], [], []
    for d, z in per.items():
        if len(z) < 20:
            continue
        k = max(1, len(z) // 10)
        ent.append(pct(z[:k], 0.5))
        sus.append(pct(z[7 * len(z) // 10:], 0.5))
    return ent, sus


for arc in sys.argv[1:]:
    cells = sorted(glob.glob(os.path.join(arc, "*_defop_seed*")))
    print("=" * 76)
    print("归档 %s  (%d 格)" % (os.path.basename(arc), len(cells)))
    print("=" * 76)
    print("%-22s %-26s %s" % ("cell", "CPN段 入口 z 中位", "CPN段 稳态 z 中位(后70%)"))
    E, S = [], []
    for c in cells:
        r = onecell(c)
        if not r:
            print("%-22s (无数据)" % os.path.basename(c))
            continue
        e, s = r
        E += e
        S += s
        print("%-22s %-26s %s" % (os.path.basename(c),
                                  "%.2f (n=%d机)" % (pct(e, .5), len(e)),
                                  "%.2f (n=%d机)" % (pct(s, .5), len(s))))
    if E:
        print("  >>> 全臂合计  入口 %.2f   稳态 %.2f   （差 %+.2f）"
              % (pct(E, .5), pct(S, .5), pct(S, .5) - pct(E, .5)))
