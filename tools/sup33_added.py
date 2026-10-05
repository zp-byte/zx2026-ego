#!/usr/bin/env python3
"""③ 候选修法离线核查（第二刀）：**若把「veto_frac >= X」并入死端高条件，
会新增多少次死端进入回合？**

只数「上升沿」：上一拍 `cod==0 且 veto_frac<X`、本拍 `cod==0 且 veto_frac>=X`
⇒ 该拍会被新条件推进死端 = 一次新增回合。已在死端中的拍不算（本来就在）。
"""
import json
import os
import sys


def cells(base):
    for c in sorted(os.listdir(base)):
        p = os.path.join(base, c, "probe.json")
        if os.path.exists(p):
            yield c, json.load(open(p)).get("defop_debug", {})


def main():
    base = sys.argv[1]
    Xs = [0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75]
    added = {X: 0 for X in Xs}
    added_s = {X: 0.0 for X in Xs}     # 新增回合的 veto_frac>=X 累计时长（上界）
    n_ticks = 0
    for _cname, dbg in cells(base):
        for k in sorted(dbg, key=lambda x: int(x)):
            tl = dbg[k].get("tl") or []
            per = {X: (False, 0.0) for X in Xs}   # (上拍是否越线, 上次 t)
            for r in tl:
                n_ticks += 1
                cod = r[1]
                veto, safe = (r[3:5] + [None, None])[:2]
                if veto is None or safe is None or (veto + safe) <= 0:
                    continue
                f = veto / float(veto + safe)
                t = r[0]
                for X in Xs:
                    on, pt = per[X]
                    if cod == 0 and f >= X:
                        if not on:
                            added[X] += 1
                        if pt is not None and t > pt:
                            added_s[X] += t - pt
                        per[X] = (True, t)
                    else:
                        per[X] = (False, None)
    print("总拍数 %d（3 格 × 6 机）" % n_ticks)
    print("  X     新增死端回合数   越线累计时长(s，上界)   每机每格新增")
    for X in Xs:
        print("  %.2f   %8d          %10.1f            %.1f"
              % (X, added[X], added_s[X], added[X] / 18.0))


if __name__ == "__main__":
    main()
