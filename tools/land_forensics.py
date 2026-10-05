#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""land_forensics.py <归档...> — 逐格拆 landing 缺口：谁没落、为什么。"""
import glob
import io
import os
import re
import sys

try:
    import yaml
except ImportError:
    yaml = None


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))]


PAT = re.compile(r"drone (\d+) ALT z=([-\d.]+) gz=([-\d.]+) cmd=([-\d.]+) cpn=(\d)")

for arc in sys.argv[1:]:
    cells = sorted(glob.glob(os.path.join(arc, "*_defop_seed*")))
    print("=" * 90)
    print("归档 %s" % os.path.basename(arc))
    print("=" * 90)
    for c in cells:
        sy = os.path.join(c, "score.yaml")
        if not os.path.isfile(sy):
            continue
        d = yaml.safe_load(io.open(sy, encoding="utf-8"))
        landed = d.get("landed_n")
        retired = d.get("retired") or []
        drops = d.get("drops") or {}
        got = [k for k, v in drops.items() if v]
        miss = [k for k, v in drops.items() if not v]
        print("%-22s landed=%s retired=%s  投成=%s 投缺=%s"
              % (os.path.basename(c), landed, retired, got, miss))
        # 没落地的机：末段 z 轨迹
        ros = os.path.join(c, "rosout.log")
        if not os.path.isfile(ros):
            continue
        per = {}
        for ln in io.open(ros, encoding="utf-8", errors="ignore"):
            m = PAT.search(ln)
            if m:
                per.setdefault(int(m.group(1)), []).append(
                    (float(m.group(2)), int(m.group(5))))
        for dd, seq in sorted(per.items()):
            tail = seq[-8:]
            zs = [t[0] for t in tail]
            cps = [t[1] for t in tail]
            print("     d%d  末8拍 z=%s  cpn=%s  最低=%.2f"
                  % (dd, " ".join("%.2f" % z for z in zs),
                     "".join(str(x) for x in cps),
                     min(z for z, _ in seq)))
    print("")
