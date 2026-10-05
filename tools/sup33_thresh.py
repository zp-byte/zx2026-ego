#!/usr/bin/env python3
"""③ 候选修法的**离线核查**：否决率 `veto/256` 在全跑中的分布。

用途：为「死端门加一条监督器条件 `veto_frac >= X`」选 X。
- 若 X 取小了 ⇒ 大面积误入死端（批 3 的"整表变保守"教训）；
- 若 X 取大了 ⇒ 盖不住实测的 ③ 现场（9/9 落在 0.59~0.69）。

输出：全体拍 vs 撞前 1.5 s 拍的 veto_frac 分位/直方图。
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+)")


def cells(base):
    for c in sorted(os.listdir(base)):
        p = os.path.join(base, c, "probe.json")
        if os.path.exists(p):
            yield c, json.load(open(p)).get("defop_debug", {})


def pct(xs, q):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def main():
    base = sys.argv[1]
    allv, crashv, deadv = [], [], []
    hist = [0] * 11
    n_crash = 0
    for cname, dbg in cells(base):
        cp = os.path.join(base, cname, "collisions.txt")
        colls = []
        if os.path.exists(cp):
            for ln in open(cp, errors="ignore"):
                m = COLL.search(ln)
                if m:
                    colls.append((float(m.group(1)), int(m.group(2))))
        for k, v in dbg.items():
            tl = v.get("tl") or []
            marks = [(t, d) for (t, d) in colls if str(d) == str(k)]
            for r in tl:
                veto, safe = (r[3:5] + [None, None])[:2]
                if veto is None or safe is None:
                    continue
                tot = veto + safe
                if tot <= 0:
                    continue
                f = veto / float(tot)
                allv.append(f)
                hist[min(10, int(f * 10))] += 1
                cod = r[1]
                if cod:
                    deadv.append(f)
                for (t, _d) in marks:
                    if t - 1.5 <= r[0] <= t:
                        crashv.append(f)
                        n_crash += 1
                        break
    print("全跑拍数=%d" % len(allv))
    print("  veto_frac 分位: p10=%.2f p25=%.2f p50=%.2f p75=%.2f p90=%.2f p99=%.2f max=%.2f"
          % tuple(pct(allv, q) for q in (0.1, 0.25, 0.5, 0.75, 0.9, 0.99, 1.0)))
    print("  直方图 (0.1 一档):")
    for i, n in enumerate(hist):
        print("    [%d.%d,%d.%d) %7d  %5.1f%%"
              % (i, 0, i + 1, 0, n, 100.0 * n / max(1, len(allv))))
    tot = len(allv)
    for X in (0.40, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80):
        n = sum(1 for f in allv if f >= X)
        print("  veto_frac >= %.2f 的拍占比 = %5.1f%%  (%d/%d)"
              % (X, 100.0 * n / max(1, tot), n, tot))
    print()
    print("撞前 1.5 s 拍数=%d   veto_frac: min=%.2f p50=%.2f max=%.2f"
          % (len(crashv), min(crashv) if crashv else -1,
             pct(crashv, 0.5), max(crashv) if crashv else -1))
    print("死端拍数=%d       veto_frac: min=%.2f p50=%.2f max=%.2f"
          % (len(deadv), min(deadv) if deadv else -1,
             pct(deadv, 0.5), max(deadv) if deadv else -1))


if __name__ == "__main__":
    main()
