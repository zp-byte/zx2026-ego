#!/usr/bin/env python3
"""③ 候选修法**离线核查**：在同一批数据上扫候选门谓词（代价 vs 收益）。

对每个谓词 P 报三件事：
  1. `新增回合` = 上升沿数（cod==0 且 P 由假变真）⇒ **代价**；
  2. `越线时长` = 这些上升沿内 P 为真的累计秒 ⇒ 额外的死端预算占用；
  3. `覆盖` = 有多少起碰撞在 [t_coll-3, t_coll] 内至少有一拍满足 P ⇒ **收益**；
     并给出「最后满足 P 的拍」到碰撞的秒数（提前量，够不够转）。

tl 行（14 列）= [t, cod, col_prob, veto, safe, forced,
                dnear, dnear2, dmin, near_az, yaw, vwx, vwy, nix1]
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+)")


def load(base):
    out = []
    for cname in sorted(os.listdir(base)):
        p = os.path.join(base, cname, "probe.json")
        if not os.path.exists(p):
            continue
        dbg = json.load(open(p)).get("defop_debug", {})
        cp = os.path.join(base, cname, "collisions.txt")
        colls = []
        if os.path.exists(cp):
            for ln in open(cp, errors="ignore"):
                m = COLL.search(ln)
                if m:
                    colls.append((float(m.group(1)), int(m.group(2))))
        for k, v in dbg.items():
            out.append((k, v.get("tl") or [],
                        [t for (t, d) in colls if str(d) == str(k)]))
    return out


def vf(r):
    veto, safe = (r[3:5] + [None, None])[:2]
    if veto is None or safe is None or (veto + safe) <= 0:
        return None
    return veto / float(veto + safe)


def analyze(data, pred, label):
    added, added_s, n_true = 0, 0.0, 0
    for _k, tl, _c in data:
        on, pt = False, None
        for r in tl:
            f = vf(r)
            if f is None:
                continue
            p = pred(r, f)
            t = r[0]
            if r[1] == 0 and p:
                n_true += 1
                if not on:
                    added += 1
                if pt is not None and t > pt:
                    added_s += t - pt
                on, pt = True, t
            else:
                on, pt = False, None
    cov, leads, tot = 0, [], 0
    for _k, tl, cts in data:
        for tc in cts:
            tot += 1
            near = [r for r in tl if tc - 3.0 <= r[0] <= tc
                    and vf(r) is not None and pred(r, vf(r))]
            if near:
                cov += 1
                leads.append(tc - near[-1][0])
    print("  %-22s 新增回合=%-4d 越线=%6.1fs 触发拍=%-5d 覆盖=%d/%d 提前量=%s"
          % (label, added, added_s, n_true, cov, tot,
             ("%.2f~%.2f" % (min(leads), max(leads))) if leads else "-"))


def main():
    base = sys.argv[1]
    data = load(base)
    print("批 %s  机数=%d  碰撞=%d"
          % (os.path.basename(base), len(data),
             sum(len(c) for _, _, c in data)))
    print("  --- A: veto_frac >= X ---")
    for X in (0.50, 0.55, 0.60, 0.625, 0.65, 0.70):
        analyze(data, lambda r, f, X=X: f >= X, "A veto_frac>=%.3f" % X)
    print("  --- C: nix1 <= X ---")
    for X in (0, 1, 2, 3, 5, 8):
        analyze(data,
                lambda r, f, X=X: (r[13] if len(r) > 13 else None) is not None
                and r[13] <= X, "C nix1<=%d" % X)
    print("  --- A∧C 组合（两条件任一）---")
    for X in (0.55, 0.60, 0.65):
        analyze(data,
                lambda r, f, X=X: f >= X or (
                    (r[13] if len(r) > 13 else None) is not None and r[13] == 0),
                "A>=%.2f ∨ nix1==0" % X)


if __name__ == "__main__":
    main()
