#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""出口批法证仪（第一刀）：逐拍 `cod` 轨迹。

拆开「② 遇撞时在死端」的两种互斥读法：

  (A) **一次进门、减速漂入**：`cod` 从 0 变 2/1，之后**连续非零**直到撞拍
      ⇒ 病在**入口**（进门太晚 / 停不下来），出口闸无用。
  (B) **退出→再进→撞回**：`cod` 在撞前出现过 **0**（退出），然后又变非零
      ⇒ 病在**出口**（判"干净"太早，出门即撞回），出口闸是靶子。

工具对每起碰撞打印撞前 `W` 秒的 `cod` 串（`_`=0 · `S`=1 stop · `R`=2 rotate）
与各段边界时刻、以及每段的 `dmin/vf/nix1` 范围。

tl 行（14 列）：`[t, cod, col_prob, veto, safe, forced, dnear, dnear2, dmin,
near_az_deg, yaw_deg, vwx, vwy, n_indx1]`（`n_indx1` = 真机喂给 n_safe 的那个）。
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+)")
W = 8.0          # 撞前回看窗（秒）
GAP = 0.6        # 采样间隔超过此值视为断档（不连段）


def colls_of(path):
    out = []
    if not os.path.exists(path):
        return out
    for ln in open(path, errors="ignore"):
        m = COLL.search(ln)
        if m:
            out.append((float(m.group(1)), int(m.group(2))))
    return out


def trace(rows, t_hit, w=W):
    win = [r for r in rows if t_hit - w <= r[0] <= t_hit]
    if not win:
        return None
    sym = {0: '_', 1: 'S', 2: 'R'}
    # 串（按拍）
    s = ''.join(sym.get(int(r[1]), '?') for r in win)
    # 分段：cod 变化的边界
    segs = []
    cur = None
    for r in win:
        c = int(r[1])
        if cur is None or c != cur[0]:
            if cur is not None:
                cur[3] = prev_t
                segs.append(cur)
            cur = [c, r[0], None, None, []]
        cur[4].append(r)
        prev_t = r[0]
    if cur is not None:
        cur[3] = prev_t
        segs.append(cur)

    def rng(rs, i):
        v = [x[i] for x in rs if isinstance(x[i], (int, float)) and x[i] > -1e6]
        if not v:
            return '-'
        return '%.2f~%.2f' % (min(v), max(v))

    lines = ['    cod 串(%.1fs 前 → 撞)： %s' % (w, s)]

    # 预警余量：首个 dmin < 1.0（= sup_min_dist，监督器唯一看得见的门）的拍
    hit1 = [r for r in win if isinstance(r[8], (int, float)) and r[8] < 1.0]
    if hit1:
        t1 = hit1[0][0]
        sp = [((r[11] or 0.0) ** 2 + (r[12] or 0.0) ** 2) ** 0.5
              for r in win if r[0] >= t1]
        lines.append('      预警(1.0m→撞) = %5.2fs   dmin@1.0m 入 = %.2f m   '
                     '均速 %.2f m/s   ⇒ 需要 %.2fs 才走得完 1.0m'
                     % (t_hit - t1, hit1[0][8],
                        (sum(sp) / len(sp)) if sp else -1,
                        1.0 / (sum(sp) / len(sp)) if sp and sum(sp) > 0 else -1))
    else:
        lines.append('      预警(1.0m→撞) = n/a（窗内 dmin 从未 <1.0）')

    for c, t0, _, t1, rs in segs:
        name = {0: '非死端', 1: 'stop  ', 2: 'rotate'}.get(c, '?')
        lines.append('      %-7s %7.2f→%7.2f (%4.2fs)  dmin %-12s vf %-12s nix1 %s'
                     % (name, t0, t1, t1 - t0, rng(rs, 8), rng(rs, 3), rng(rs, 13)))
    return lines, segs, win


def main(base):
    ddbg = {}
    for cname in sorted(os.listdir(base)):
        p = os.path.join(base, cname, "probe.json")
        if os.path.exists(p):
            ddbg[cname] = json.load(open(p)).get("defop_debug", {})
    print('=' * 78)
    print('批 %s' % base)
    print('=' * 78)
    tally = {'A': 0, 'B': 0, 'NA': 0}
    for cname in sorted(os.listdir(base)):
        cp = os.path.join(base, cname, "collisions.txt")
        cs = colls_of(cp)
        if not cs:
            continue
        print('\n[%s]  碰撞 %d 起' % (cname, len(cs)))
        tl_all = ddbg.get(cname, {})
        for t_hit, d in cs:
            ent = tl_all.get(str(d)) or tl_all.get(d) or {}
            rows = ent.get('tl', []) if isinstance(ent, dict) else ent
            rows = [r for r in rows if isinstance(r, list) and len(r) >= 14]
            if not rows:
                print('  t=%7.2f d%d  无逐拍线' % (t_hit, d))
                tally['NA'] += 1
                continue
            res = trace(rows, t_hit)
            if res is None:
                print('  t=%7.2f d%d  窗外无拍' % (t_hit, d))
                tally['NA'] += 1
                continue
            lines, segs, win = res
            # 判 A / B：撞前是否出现过 cod==0（退出）之后又非零
            nonz = [i for i, r in enumerate(win) if int(r[1]) != 0]
            last_z = None
            verdict = 'A 一次进门·减速漂入'
            if nonz:
                first_nz = nonz[0]
                # 在 first_nz 之后是否又回到 0
                back = [i for i in nonz if i > first_nz and int(win[i][1]) == 0]
                # 是否 first_nz 之前有 0（即窗内先非零后归零）——只关心 0→非零→0→非零
                seen_nz = False
                seen_zero_after = False
                for r in win:
                    c = int(r[1])
                    if c != 0:
                        if seen_zero_after:
                            verdict = 'B 退出→再进→撞回'
                            break
                        seen_nz = True
                    else:
                        if seen_nz:
                            seen_zero_after = True
            tally['B' if verdict.startswith('B') else 'A'] += 1
            print('  t=%7.2f d%d   → %s' % (t_hit, d, verdict))
            for L in lines:
                print(L)
    print('\n本批小计: A(一次进门·减速漂入)=%d  B(退出→再进)=%d  不可判=%d'
          % (tally['A'], tally['B'], tally['NA']))
    return tally


if __name__ == '__main__':
    for b in sys.argv[1:]:
        main(b)
