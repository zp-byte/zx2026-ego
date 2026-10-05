#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""碰撞**接触形态**：撞前 2s 的 dmin 轨迹 / 速度 / 是否触到 0.20m 地板。

用来区分两种形态（本仓已有先例，见 zx2026-dam-drift-margin）：
  · **撞入**：dmin 单调下降、撞前速度高（>0.6 m/s）⇒ 制动距离不够
  · **贴树挤入**：dmin 已在 0.2~0.5 徘徊 ≥0.5s、撞前速度低（<0.35 m/s）
    ⇒ 深度地板（`min_range=0.2`）让监督器致盲，低速蹭进去
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+)")
W = 2.0


def main(base):
    ddbg = {}
    for cname in sorted(os.listdir(base)):
        p = os.path.join(base, cname, "probe.json")
        if os.path.exists(p):
            ddbg[cname] = json.load(open(p)).get("defop_debug", {})
    print('=' * 78)
    print('批 %s — 接触形态（撞前 %.1fs）' % (base, W))
    print('=' * 78)
    n_in = n_squeeze = 0
    for cname in sorted(os.listdir(base)):
        cp = os.path.join(base, cname, "collisions.txt")
        if not os.path.exists(cp):
            continue
        for ln in open(cp, errors="ignore"):
            m = COLL.search(ln)
            if not m:
                continue
            t_hit, d = float(m.group(1)), int(m.group(2))
            ent = ddbg.get(cname, {}).get(str(d)) or {}
            rows = [r for r in ent.get('tl', [])
                    if isinstance(r, list) and len(r) >= 14
                    and t_hit - W <= r[0] <= t_hit]
            if not rows:
                print('  t=%7.2f d%d 无逐拍线' % (t_hit, d))
                continue
            dm = [r[8] for r in rows if isinstance(r[8], (int, float))]
            sp = [((r[11] or 0.0) ** 2 + (r[12] or 0.0) ** 2) ** 0.5 for r in rows]
            last = rows[-1]
            v_end = sp[-1]
            frac_floor = sum(1 for x in dm if x <= 0.21) / float(len(dm))
            if v_end < 0.35 or frac_floor > 0.4:
                form = '贴树挤入'
                n_squeeze += 1
            else:
                form = '撞入'
                n_in += 1
            print('  t=%7.2f d%d  %-6s  dmin %.2f→%.2f (地板拍 %.0f%%)  '
                  '末速 %.2f  max速 %.2f  vf末 %s/no-%s  nix1末 %s'
                  % (t_hit, d, form, dm[0], dm[-1], 100 * frac_floor,
                     v_end, max(sp), last[3], last[4], last[13]))
    print('\n本批: 撞入=%d  贴树挤入=%d' % (n_in, n_squeeze))


if __name__ == '__main__':
    for b in sys.argv[1:]:
        main(b)
