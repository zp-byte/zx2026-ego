#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ha_census.py <label> <yaw.csv> <rosout.log> —— 机头对齐离线普查（只读）。

回答两个问题（本波 A/B 的实证核心）：
  Q1 航向到底动没动？—— off 臂 total_turn 应 ≈ 0（证明「yaw 冻在正东」这条
     此前只有代码级证据的根因），strict 臂应显著 > 0。
  Q2 接线是否真通？—— 把「指令非零的拍」与「航向真的变了」分开统计：
     wr_frac 高但 turn_when_cmd ≈ 0 ⇒ 上游发了、下游没执行（后端不消费）。

口径与限制（诚实边界）：
  * yaw.csv 由 /tmp/ha_yaw_probe.py 产出（墙钟 + odom 姿态真值 + vel_cmd
    angular.z），**非** nav_node 自报。
  * nav_node 的 HEADING-ALIGN 行时间戳属 rosout 域，与本文件墙钟域不保证
    可对齐 ⇒ 只做「该机累计 align_s」与「行数」的独立读数，不做时间连接。
  * total_turn 用 wrap 后逐差累加（原值在 [-pi,pi] 会假性大跳）。
"""
import math
import re
import sys

if len(sys.argv) < 4:
    sys.exit("usage: ha_census.py <label> <yaw.csv> <log...>")
LABEL, YCSV = sys.argv[1], sys.argv[2]
LOGS = sys.argv[3:]

WR_MIN = 0.05          # rad/s；「指令非零」判据
FROZEN_DEG = 1.0       # total_turn 低于此值判「整局未转」


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- Q1/Q2：探针 CSV ---------------------------------------------------------
samples = {}
for ln in open(YCSV, errors="ignore"):
    if ln.startswith("#") or not ln.strip():
        continue
    a = ln.split()
    if len(a) != 10:
        continue
    try:
        d = int(a[1])
        samples.setdefault(d, []).append(
            (float(a[0]), float(a[5]), float(a[9])))   # wall, yaw, yaw_rate_cmd
    except ValueError:
        continue

print("=" * 78)
print("机头对齐普查  [%s]" % LABEL)
print("=" * 78)
print("  探针样本：%s，合计 %d 行" % (YCSV, sum(len(v) for v in samples.values())))

tot_all = 0.0
rows = []
for d in sorted(samples):
    rows_d = samples[d]
    turn = 0.0
    turn_cmd = 0.0
    unwrapped = [rows_d[0][1]]
    un = rows_d[0][1]
    wr_hi = 0
    wr_max = 0.0
    for k in range(1, len(rows_d)):
        _, yaw, _ = rows_d[k]
        _, _, wr_prev = rows_d[k - 1]
        step = wrap_pi(yaw - rows_d[k - 1][1])
        turn += abs(step)
        un += step
        unwrapped.append(un)
        if abs(wr_prev) > WR_MIN:
            turn_cmd += abs(step)
        if abs(rows_d[k][2]) > WR_MIN:
            wr_hi += 1
        wr_max = max(wr_max, abs(rows_d[k][2]))
    span = math.degrees(max(unwrapped) - min(unwrapped))
    rows.append((d, len(rows_d), math.degrees(turn), span,
                 math.degrees(wr_max), 100.0 * wr_hi / max(1, len(rows_d))))
    tot_all += math.degrees(turn)

print()
print("  逐机：累计转角 / 展开角幅 / 指令峰值速率 / 指令非零占比")
print("  %-5s %7s %12s %11s %11s %9s" %
      ("机", "样本", "total_turn", "展开幅", "|wr|max", "wr非零%"))
for (d, n, turn, span, wrm, frac) in rows:
    flag = "  ★整局未转" if turn < FROZEN_DEG else ""
    print("  d%-4d %7d %11.1f° %10.1f° %8.2f°/s %8.1f%%%s"
          % (d, n, turn, span, wrm, frac, flag))

fr = [r[0] for r in rows if r[2] < FROZEN_DEG]
print()
print("  ★ 冻结机（total_turn < %.0f°）= %d / %d" % (FROZEN_DEG, len(fr), len(rows)))
if rows:
    print("  ★ 平均累计转角 = %.1f°   （off 臂应 ≈ 0；strict 臂应显著大于 0）"
          % (tot_all / len(rows)))

# ---- nav_node 自报（独立读数，不做时间连接）---------------------------------
print()
print("  ---- nav_node 自报读数（读 %d 个日志：rosout + 逐节点 stdout）----"
      % len(LOGS))
al = {}
nline = 0
yaw_deg = {}
import glob as _glob
files = []
for pat in LOGS:
    files.extend(_glob.glob(pat))
files = sorted(set(files))
for fp in files:
    try:
        fh = open(fp, errors="ignore")
    except OSError:
        continue
    for ln in fh:
        if "HEADING-ALIGN" not in ln:
            continue
        nline += 1
        m = re.search(r'drone (\d+) HEADING-ALIGN mode=(\S+) gated=(\d+) '
                      r'align_s=([0-9.]+) yaw_deg=(-?[0-9.]+)', ln)
        if not m:
            continue
        al[m.group(1)] = (m.group(2), float(m.group(4)))
        yaw_deg.setdefault(m.group(1), []).append(float(m.group(5)))
print("  读入 %d 个文件；HEADING-ALIGN 行数 = %d（mode=off 时应为 0）"
      % (len(files), nline))
for d in sorted(al):
    mode, s = al[d]
    ys = yaw_deg[d]
    print("    drone %s: mode=%s align_s=%.2f  yaw_deg 自报幅 %.1f°（含环绕，仅参考）"
          % (d, mode, s, max(ys) - min(ys)))
print("=" * 78)
