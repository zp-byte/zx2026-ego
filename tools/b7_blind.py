#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""近距致盲量化：`dmin` 很小时，监督器到底还看没看见？

若 `depth_sim_node.py:517`（`Z<min_range=0.2 → 0`）搭
`obstacle_mask = (depth>0)&(depth<min_dist)` 真的造成致盲，则
**`dmin` 越小，`vf` 应越低**（障碍滑进 0.2m 内 ⇒ 像素被清零 ⇒ 掩膜空 ⇒ 全判安全）。
若无致盲，`vf` 应随 `dmin` 减小而**升高**（越近越该否决）。

本工具按 `dmin` 分箱统计 `vf=veto/(veto+safe)` 与 `n_safe=col13`。
"""
import json
import os
import sys

BINS = [(0.0, 0.25), (0.25, 0.5), (0.5, 0.8), (0.8, 1.2), (1.2, 2.0), (2.0, 99)]


def main(bases):
    # n, Σvf, Σnix1, n_low, Σdnear, n_over5pct
    st = {b: [[0, 0.0, 0.0, 0, 0.0, 0] for _ in BINS] for b in bases}
    for b in bases:
        for cname in sorted(os.listdir(b)):
            p = os.path.join(b, cname, "probe.json")
            if not os.path.exists(p):
                continue
            ddbg = json.load(open(p)).get("defop_debug", {})
            for ent in ddbg.values():
                for r in ent.get('tl', []):
                    if not (isinstance(r, list) and len(r) >= 14):
                        continue
                    dm, v, s = r[8], r[3], r[4]
                    if not isinstance(dm, (int, float)) or dm <= 0:
                        continue
                    if not isinstance(v, (int, float)) or not isinstance(s, (int, float)):
                        continue
                    tot = v + s
                    if tot <= 0:
                        continue
                    for i, (lo, hi) in enumerate(BINS):
                        if lo <= dm < hi:
                            a = st[b][i]
                            a[0] += 1
                            a[1] += v / float(tot)
                            a[2] += (r[13] if isinstance(r[13], (int, float)) else 0)
                            a[3] += 1 if v / float(tot) < 0.2 else 0
                            a[4] += (r[6] if isinstance(r[6], (int, float)) else 0)
                            a[5] += 1 if (isinstance(r[6], (int, float))
                                          and r[6] >= 0.05 * 129600) else 0
                            break
    print('=' * 78)
    print('近距检验：按 dmin 分箱')
    print('  ① 致盲检验：无致盲 ⇒ vf 随 dmin 减小而升高')
    print('  ② 细树干探测：`dnear` = 深度 < sup_min_dist(1.0m) 的像素数；')
    print('     图 480x270 = 129600 px ⇒ sup_min_ratio=0.05 需 dnear>=6480 才算"挡路"')
    print('=' * 78)
    for b in bases:
        print('\n---- %s ----' % b)
        print('  dmin 箱        拍数    均 vf   均 n_safe  vf<0.2  均 dnear  dnear>=5%')
        for i, (lo, hi) in enumerate(BINS):
            n, sv, sn, nl, sd, no = st[b][i]
            if n == 0:
                print('  [%.2f,%.2f)    %-7d  -       -         -        -        -'
                      % (lo, hi, n))
                continue
            print('  [%.2f,%.2f)    %-7d  %.3f  %-8.1f  %5.0f%%  %-8.0f %.0f%%'
                  % (lo, hi, n, sv / n, sn / n, 100.0 * nl / n, sd / n,
                     100.0 * no / n))


if __name__ == '__main__':
    main(sys.argv[1:])
