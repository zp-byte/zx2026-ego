#!/usr/bin/env python3
"""③ 离线核查（第三刀）：**决策窗**逐拍对质。

前两刀按「撞前 3s 内任意一拍满足谓词」算覆盖 ⇒ 被**撞拍自己**（veto_frac 已高）
灌水，提前量普遍 0.00。真正要问的是：

  在**还有反应余地**的那些拍上（`dmin` 在 [0.35, 1.2] m 且机体还在动），
  监督器说了什么？CPN 说了什么？

对每起碰撞打印决策窗（`dmin < 1.2` 的拍）逐拍行，并给出**首个**满足
`dmin < 0.8` 的拍的各量 ⇒ 这才是各候选门**能不能提前咬合**的现场。
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+)")


def load(base):
    for cname in sorted(os.listdir(base)):
        p = os.path.join(base, cname, "probe.json")
        if not os.path.exists(p):
            continue
        dbg = json.load(open(p)).get("defop_debug", {})
        cp = os.path.join(base, cname, "collisions.txt")
        colls = {}
        if os.path.exists(cp):
            for ln in open(cp, errors="ignore"):
                m = COLL.search(ln)
                if m:
                    colls.setdefault(m.group(2), []).append(float(m.group(1)))
        yield cname, dbg, colls


def vf(r):
    if len(r) < 5 or r[3] is None or r[4] is None:
        return None
    tot = r[3] + r[4]
    return (r[3] / float(tot)) if tot else None


def main():
    base = sys.argv[1]
    rad = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8
    rows = []
    for cname, dbg, colls in load(base):
        for k in sorted(dbg, key=lambda x: int(x)):
            tl = dbg[k].get("tl") or []
            for tc in colls.get(k, []):
                win = [r for r in tl if tc - 3.5 <= r[0] <= tc + 0.3]
                near = [r for r in win if (r[8] is not None and r[8] < 1.2)]
                if not near:
                    rows.append((cname, k, tc, None, []))
                    continue
                first = next((r for r in near if r[8] < rad), None)
                rows.append((cname, k, tc, first, near))

    print("=== 决策窗：`dmin<0.8` 的首拍（若有）===")
    print("%-5s d%s  t撞     t决策  Δt    dmin  dnear  veto/256  vf    cp    nix1  az    vw")
    hits = 0
    for cname, k, tc, first, near in rows:
        if first is None:
            print("%-5s d%s  %7.2f  —— 撞前无 dmin<%.2f 的拍（近场空/被 min_range 清零）"
                  % (cname[3:], k, tc, rad))
            continue
        hits += 1
        v = vf(first)
        print("%-5s d%s  %7.2f %7.2f %5.2f  %5.2f  %6s  %3s/%-3s %-5s %-5s %-5s %-5s (%s,%s)"
              % (cname[3:], k, tc, first[0], tc - first[0], first[8], first[6],
                 first[3], first[4], ("%.3f" % v) if v is not None else "-",
                 first[2], first[13], first[9], first[11], first[12]))
    print("⇒ %d/%d 起在撞前存在可反应的近场拍" % (hits, len(rows)))

    print()
    print("=== 决策窗逐拍（dmin<1.2）===")
    for cname, k, tc, first, near in rows:
        if not near:
            continue
        print("--- %s d%s  撞 t=%.2f ---" % (cname[3:], k, tc))
        for r in near:
            v = vf(r)
            print("    t=%7.2f cod=%d cp=%-6s veto=%-4s safe=%-4s vf=%-5s dmin=%-5s "
                  "dnr=%-6s az=%-6s nix1=%-4s vw=(%s,%s)"
                  % (r[0], r[1], r[2], r[3], r[4],
                     ("%.3f" % v) if v is not None else "-",
                     r[8], r[6], r[9], r[13], r[11], r[12]))


if __name__ == "__main__":
    main()
