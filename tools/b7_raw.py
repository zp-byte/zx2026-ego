#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""撞前逐拍原样打印（末 N 拍），看**指令 vs 机体运动**。

tl 行：`[t, cod, col_prob, veto, safe, forced, dnear, dnear2, dmin, az, yaw, vwx, vwy, nix1]`
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+)")


def main(base, want=None, n=14):
    ddbg = {}
    for cname in sorted(os.listdir(base)):
        p = os.path.join(base, cname, "probe.json")
        if os.path.exists(p):
            ddbg[cname] = json.load(open(p)).get("defop_debug", {})
    for cname in sorted(os.listdir(base)):
        cp = os.path.join(base, cname, "collisions.txt")
        if not os.path.exists(cp):
            continue
        for ln in open(cp, errors="ignore"):
            m = COLL.search(ln)
            if not m:
                continue
            t_hit, d = float(m.group(1)), int(m.group(2))
            if want and (cname.split('/')[0], d) not in want:
                continue
            ent = ddbg.get(cname, {}).get(str(d)) or {}
            rows = [r for r in ent.get('tl', [])
                    if isinstance(r, list) and len(r) >= 14 and r[0] <= t_hit]
            print('\n[%s] t=%.2f d%d  撞前 %d 拍' % (cname, t_hit, d, n))
            print('      t      cod  colp  vf    dmin   az    yaw    vwx   vwy   nix1  |v|')
            for r in rows[-n:]:
                v = ((r[11] or 0.0) ** 2 + (r[12] or 0.0) ** 2) ** 0.5
                tot = (r[3] or 0) + (r[4] or 0)
                vf = (r[3] / float(tot)) if tot else -1
                print('  %7.2f  %s  %5s %5.3f %6.3f %5s %6s %5.2f %5.2f %5s  %.2f'
                      % (r[0], r[1], r[2] if r[2] is not None else '-', vf,
                         r[8], r[9] if r[9] is not None else '-',
                         r[10] if r[10] is not None else '-',
                         r[11] or 0.0, r[12] or 0.0,
                         r[13] if r[13] is not None else '-', v))


if __name__ == '__main__':
    main(sys.argv[1])
