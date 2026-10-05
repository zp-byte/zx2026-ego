#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_deadend_forensics.py [run_dir] — 死端案批 1 取证与对拍。

两路证据
--------
(a) **planner 自报**（1 Hz `SHADOW bearing` 行）：replica 直测的
    `t_rot_cur` / `t_rot_beta`（状态 1 的 sim 秒积分）与回合时长
    `h_*`（n / p50 / max）。**同帧对照**：同一张深度图跑两套掩膜。
(b) **真值**（`probe.json` 的 `defop_debug`，来自 `/drone_N/defop/debug`
    逐拍 JSON，**零改 planner**）：按 mode 归属累计的 `rot_s`/`stop_s`/`ep_n`。
    只覆盖**现状掩膜**（发布路径）。另含 `defop_debug` 的机型键 ⇒ 天然分机。

⚠ 口径（2026-10-02 实测教训，**必须按机分组**）
------------------------------------------------
六台机**各自**打 `SHADOW bearing`，`cyc` 是各机自己的计数器；六条流**并行**、
每机约 1 Hz。因此：

1. **不能"取最后一条/cyc 最大的一条"**——那是跨机混读，看起来 `h_* n` 会
   *回退*（实测踩过：cyc=112 时 n=1，cyc=365 时"取到"n=0）。
2. 分机的唯一权威源是 **`rosout.log`**（行首带 `/drone_N/defop_cpn`）；
   `defop_launch.log` 是裸 rospy stdout，**不带机号**，六机混流不可分。
3. `rosout.log` 在**格末**才落盘 ⇒ 格在跑时本脚本只能看到 `launch.log`，
   此时应报"格未完"，不得出结论。
4. 正确聚合 = **每机取各机自己末条，再对六机求和**。

对拍门（硬）
------------
`Σ_drones rot_s`（真值）应与 replica `Σ t_rot_cur` 同量级；偏差 >45% ⇒
replica 与真机漂移 ⇒ β 的 Δt_rot 判据作废，不得据以判决（案卷硬门）。

用法
----
    cd ~/zx2026_defop_ws && python3 tools/defop_deadend_forensics.py \
        run_logs/defop_ab_20261002_110544
不传则取 run_logs 下最新的 defop_ab_* 目录。
"""
import glob
import json
import os
import re
import sys

WS = '/home/ubuntu/zx2026_defop_ws'
# 行首机号（只在 rosout.log 里有）；未匹配者归 '?'（launch.log 混流）
DRONE = re.compile(r'/drone_(\d+)/defop_cpn')
LINE = re.compile(r'SHADOW bearing cyc=(\d+).*?'
                  r't_rot_cur=([\d.]+) t_rot_beta=([\d.]+) '
                  r't_force_beta=([\d.]+) dt=([-\d.]+) \| '
                  r'h_cur n=(\d+) p50=([\d.]+) max=([\d.]+) \| '
                  r'h_beta n=(\d+) p50=([\d.]+) max=([\d.]+)')
SUMMARY_DT = re.compile(r'SHADOW bearing SUMMARY.*?Δt_rot=([-+\d.]+)s')
# 旧格式（无 replica 字段）的判据：有 SHADOW 行但 LINE 一条都不匹配
MARK = 'SHADOW bearing cyc='


def latest_run_dir():
    ds = sorted(glob.glob(os.path.join(WS, 'run_logs', 'defop_ab_*')))
    return ds[-1] if ds else None


def read_shadow(cell):
    """按机分组 → {drone_key: rec}；每机取该机 **cyc 最大** 的一条。

    返回 (per_drone, state)：
      state ∈ 'ok' | 'old_format' | 'none' | 'pending'
      - 'pending'：`rosout.log` 还没落盘（格在跑），只有 launch.log 混流线
        （不分机，不作为证据）。
    """
    rs = os.path.join(cell, 'rosout.log')
    if not os.path.exists(rs):
        return {}, 'pending'
    per, seen_mark, seen_summary = {}, False, None
    with open(rs, errors='ignore') as f:
        for line in f:
            if 'SHADOW bearing' not in line:
                continue
            if MARK in line:
                seen_mark = True
            s = SUMMARY_DT.search(line)
            if s is not None:
                seen_summary = float(s.group(1))
            m = LINE.search(line)
            if m is None:
                continue
            dm = DRONE.search(line)
            key = dm.group(1) if dm else '?'
            g = m.groups()
            rec = {'cyc': int(g[0]), 't_rot_cur': float(g[1]),
                   't_rot_beta': float(g[2]), 't_force_beta': float(g[3]),
                   'dt': float(g[4]), 'h_cur': (int(g[5]), float(g[6]), float(g[7])),
                   'h_beta': (int(g[8]), float(g[9]), float(g[10]))}
            if key not in per or rec['cyc'] > per[key]['cyc']:
                per[key] = rec                      # 同机取 cyc 最大 = 该机末态
    if not per:
        return {}, ('old_format' if seen_mark else 'none')
    if seen_summary is not None:
        per['_summary_dt'] = seen_summary
    return per, 'ok'


def read_probe(cell):
    p = os.path.join(cell, 'probe.json')
    if not os.path.exists(p):
        return None
    try:
        j = json.load(open(p))
    except Exception:
        return None
    dd = j.get('defop_debug') or {}
    if not dd:
        return None
    agg = {'n': 0, 'rot_s': 0.0, 'stop_s': 0.0, 'forced_s': 0.0, 'ep_n': 0,
           'live': 0}
    for _d, r in sorted(dd.items()):
        agg['n'] += int(r.get('n', 0))
        agg['rot_s'] += float(r.get('rot_s', 0.0))
        agg['stop_s'] += float(r.get('stop_s', 0.0))
        agg['forced_s'] += float(r.get('forced_s', 0.0))
        agg['ep_n'] += int(r.get('ep_n', 0))
        if int(r.get('n', 0)) > 0:
            agg['live'] += 1
    return agg


def cell_sums(per):
    """六机求和（跳过 '_summary_dt' 这类非机键）。"""
    tot = {'t_rot_cur': 0.0, 't_rot_beta': 0.0, 't_force_beta': 0.0,
           'h_cur_n': 0, 'h_beta_n': 0, 'h_cur_max': 0.0, 'h_beta_max': 0.0,
           'h_cur_p50': 0.0, 'h_beta_p50': 0.0, 'drones': 0}
    for k, r in per.items():
        if k.startswith('_'):
            continue
        tot['t_rot_cur'] += r['t_rot_cur']
        tot['t_rot_beta'] += r['t_rot_beta']
        tot['t_force_beta'] += r['t_force_beta']
        tot['h_cur_n'] += r['h_cur'][0]
        tot['h_beta_n'] += r['h_beta'][0]
        tot['h_cur_max'] = max(tot['h_cur_max'], r['h_cur'][2])
        tot['h_beta_max'] = max(tot['h_beta_max'], r['h_beta'][2])
        tot['h_cur_p50'] = max(tot['h_cur_p50'], r['h_cur'][1])
        tot['h_beta_p50'] = max(tot['h_beta_p50'], r['h_beta'][1])
        tot['drones'] += 1
    return tot


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else latest_run_dir()
    if not root or not os.path.isdir(root):
        raise SystemExit('用法: defop_deadend_forensics.py <run_dir>')
    cells = sorted(d for d in glob.glob(os.path.join(root, '*_defop_*'))
                   if os.path.isdir(d))
    print('=' * 100)
    print('死端案批 1 取证  root=%s  (%d 格)' % (root, len(cells)))
    print('=' * 100)

    T = {'cur': 0.0, 'beta': 0.0, 'force': 0.0, 'true_rot': 0.0,
         'ep_true': 0, 'ep_cur': 0, 'ep_beta': 0}
    pending = 0
    for c in cells:
        name = os.path.basename(c)
        per, st = read_shadow(c)
        pr = read_probe(c)
        print('\n[%s]' % name)
        if st == 'pending':
            pending += 1
            print('  ⏳ 格未完：`rosout.log` 未落盘（launch.log 是六机混流，'
                  '**不可分机、不作证据**）')
        elif st == 'old_format':
            print('  ⚠ SHADOW 行是**旧格式**（无 replica 字段）——非本案验收对象')
        elif st == 'none':
            print('  ⚠ 无 SHADOW 行（未走 shadow 档/节点没起）')
        else:
            for k in sorted(k for k in per if not k.startswith('_')):
                r = per[k]
                print('    drone_%s @cyc=%-5d t_rot cur=%6.1f beta=%6.1f '
                      'Δ=%+5.1f | force=%4.1f | h_cur n=%2d p50=%4.0f max=%5.1f'
                      ' | h_beta n=%2d p50=%4.0f max=%5.1f'
                      % (k, r['cyc'], r['t_rot_cur'], r['t_rot_beta'],
                         r['t_rot_beta'] - r['t_rot_cur'], r['t_force_beta'],
                         r['h_cur'][0], r['h_cur'][1], r['h_cur'][2],
                         r['h_beta'][0], r['h_beta'][1], r['h_beta'][2]))
            s = cell_sums(per)
            print('    Σ(%d 机) t_rot cur=%.1f beta=%.1f Δ=%+.1f | force=%.1f'
                  % (s['drones'], s['t_rot_cur'], s['t_rot_beta'],
                     s['t_rot_beta'] - s['t_rot_cur'], s['t_force_beta']))
            if per.get('_summary_dt') is not None:
                print('    ⤷ SUMMARY 直报 Δt_rot=%+.1fs（应与上式 Δ 一致）'
                      % per['_summary_dt'])
            T['cur'] += s['t_rot_cur']
            T['beta'] += s['t_rot_beta']
            T['force'] += s['t_force_beta']
            T['ep_cur'] += s['h_cur_n']
            T['ep_beta'] += s['h_beta_n']
        if pr is None:
            print('  ⚠ probe.json 无 defop_debug（探针旧版/话题没发/格未完）')
        else:
            print('  ✓ 真值 /defop/debug（现状掩膜，%d/6 机在线）: '
                  'rot_s=%.1fs stop_s=%.1fs forced_s=%.1fs ep_n=%d'
                  % (pr['live'], pr['rot_s'], pr['stop_s'], pr['forced_s'],
                     pr['ep_n']))
            T['true_rot'] += pr['rot_s']
            T['ep_true'] += pr['ep_n']

    print('\n' + '=' * 100)
    print('汇总')
    if pending:
        print('  ⏳ 有 %d 格未完（rosout.log 未落盘）⇒ **本表不作判决依据**，'
              '等格跑完重跑本脚本' % pending)
    print('  回合数: 真值(/defop/debug)=%d  replica h_cur=%d  h_beta=%d'
          % (T['ep_true'], T['ep_cur'], T['ep_beta']))
    print('  零前进(sim s): cur=%.1f  beta=%.1f  Δ(β−cur)=%+.1f'
          % (T['cur'], T['beta'], T['beta'] - T['cur']))
    print('  强制放宽(sim s): beta=%.1f（批 1 参数 0 ⇒ 应恒 0）' % T['force'])
    print()
    if T['cur'] > 0:
        d = abs(T['true_rot'] - T['cur']) / T['cur']
        print('  【对拍门】真值 Σrot_s=%.1f  vs  replica Σt_rot_cur=%.1f'
              '  ⇒ 偏差 %.0f%%  %s'
              % (T['true_rot'], T['cur'], d * 100,
                 'PASS（replica 可用，Δt_rot 判据有效）' if d <= 0.45 else
                 'FAIL（replica 漂移 ⇒ Δ 判据作废，不得判决）'))
    else:
        print('  ⚠ 分母为 0，无法对拍（本方无死端轨迹？）')
    print()
    print('  【离线定值参考】(deadend_max_s, deadend_exit_hyst_s)')
    print('    ⚠ 日志行只带 h_* 的 n/p50/max（12 档直方图未上行）⇒ 一阶切分')
    print('      · max_s 应 **> p50**（别截断典型回合）且 **< max**（砍长尾）')
    print('      · 天花板 12 s：30°/s 单向扫满 360°，再转不新增信息（案卷已记）')
    print('      · hyst_s 对准抖动时标：法证 F3/F5 实测复入 0.6/0.8/0.9 s')
    print('        ⇒ ~1.0 s 可吸收抖动而不显著加长每回合')
    print('=' * 100)


if __name__ == '__main__':
    main()
