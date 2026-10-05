#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""反应窗仪器：对每起碰撞，判它落在哪一档反应机制里。

为什么需要它：杀线（P4 余量 / landed / Σcol）只判**代价**，判不了
「机理是否被扳动」。批 3（β 轴向）与批 4（yaw_rate_spot）都需要一个把
碰撞归因到具体机制的**可复现**量具。

判据的物理语义（读码定谳，见 defop_cpn_node.py:_deadend_step_logic）：
    hi_ok = (min_score < deadend_high(6.5)) and n_safe > 0   -> 常规跟随
    lo_ok = (min_score < deadend_low(5.0))  and n_safe > 0   -> 退出死端
  ⇒ **min_score 是碰撞分，越低越安全**（不是"越低越危险"）。
  死端既可由 `min_score >= 6.5`（分太高）触发，也可由 `n_safe == 0` 触发。

判定次序（先写死，再见数）：
  ① 反应太晚    : 0 <= dt < 0.8 s，dt = 碰撞 − 该机**最近一次真机** `DEADEND enter`
                  （撞速 ~1.3 m/s、树边界距轴 ~0.65 m ⇒ 几何窗 ~0.5 s 量级）
  ② 死端中仍撞  : 碰撞时**正处于**死端（stop/rotate 任一档）
  ③ 监督层未拦住: 碰撞时不在死端，而监督层当时判定"安全"（ms < deadend_low）
  ? 证据不足    : 占用不可判，且最近的 min_score 读数滞后 > 5 s

证据源两级（自动择优）：
  Tier 2  探针逐拍线 probe.json["defop_debug"][d]["tl"]（2~5Hz，sim 秒，
          `tools/defop_ab_run.py` 的 TL_MAX 段）。**逐拍** mode/col_prob/veto/safe
          ⇒ dt/ms/rot 都是瞬时量，滞后 ~0.3s。批 4 起可用。
  Tier 1  rosout：dt 取自真机 `DEADEND enter`；ms 取自 loginfo_throttle 的
          `[defop_cpn] drone_N veto=.. safe=.. min_score=..`（实测每机每格仅
          14~19 条、最大间隔 46s ⇒ 必须设 lag 门）；占用取自影子档 1Hz
          `SHADOW bearing ... t_rot_cur=` 的**尾部窗增量**。

⚠ 副本（replica）可信域：`SHADOW bearing ... t_rot_cur=` 是**影子档**里与真机
共用一个 `_deadend_step_logic` 的复刻，喂的是**真实**逐拍 n/min_score。
本案只用它的**占用增量**（Δ>0 即该秒内有旋转），**不用其时长**——
时长曾因"间隙充电"虚高一个量级（已由 §0b `_clamp_dt` 修，但本案不依赖）。
边沿计数已与真机 `DEADEND enter` 三源对拍一致（docs/deadend_timeout_case.md）。
`SHADOW bearing SUMMARY` 行是收尾汇总，本器**排除**。

⚠ 退化路径：`sup_bearing_mode: off` 时既无 SHADOW 行也无逐拍线 ⇒ 占用不可知，
本器退回纯 ms 规则。

⚠ 已知分辨率：占用判定按 1 Hz 影子行的**尾部窗**（碰撞前最后两个采样）做，
故分辨率 ~1 s——撞前 1 s 内刚退出旋转档的，会被判成"未在转"。Tier 2 探针
（2–5 Hz）落地后此模糊即可消。

用法: python3 tools/defop_reactwin.py <批目录> [<批目录> ...]
"""
import glob
import json
import os
import re
import sys

COLL = re.compile(
    r"^([\d.]+) .*drone (\d+) COLLISION #(\d+) (?:kind=(\w+) )?"
    r"at \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
ENTER = re.compile(r"^([\d.]+) .*drone_(\d+)/defop_cpn.*DEADEND enter")
SUP = re.compile(
    r"^([\d.]+) .*drone_(\d+)/defop_cpn.*\[defop_cpn\] drone_\d+ "
    r"veto=(\d+) safe=(\d+) min_score=([-\d.]+)")
FR = re.compile(r"^([\d.]+) .*drone_(\d+)/defop_cpn.*DEADEND forced-relax")
SHADOW = re.compile(
    r"^([\d.]+) .*drone_(\d+)/defop_cpn.*\[defop_cpn\] SHADOW bearing "
    r"cyc=\d+ .*?\| t_rot_cur=([\d.]+)")

DEADEND_HIGH = 6.5
DEADEND_LOW = 5.0
LATE_S = 0.8
LAG_MAX = 5.0
BRACKET_MAX = 2.5          # 夹逼区间宽于此 ⇒ 占用不可判

CLS_LATE = "① 反应太晚"
CLS_INDE = "② 死端中仍撞"
CLS_SUP = "③ 监督层未拦住"
CLS_GAP = "? 证据不足"


def in_rot(sh, t, bracket_max=BRACKET_MAX):
    """副本占用判定：**碰撞前最后一个 1 Hz 采样窗**内 t_rot_cur 是否在涨。

    为什么不用"跨碰撞的夹逼区间"：碰撞后进入 settle HOLD，plan_once 停摆
    ⇒ 影子行稀疏，夹逼区间常宽达 3 s 以上（批 1 bis cell00 的 d4 正是
    3.06 s）⇒ 反而不可判。**尾部窗**（碰撞前最后两个采样）总是紧的，
    且正是我们要问的那一秒："撞之前它还在转吗"。

    sh = 该机 [(t, t_rot_cur)] 升序。返回 True/False/None（None = 不可判）。
    """
    le = [s for s in sh if s[0] <= t]
    if len(le) < 2:
        return None
    if (le[-1][0] - le[-2][0]) > bracket_max:      # 窗内本就没有连续采样
        return None
    if (t - le[-1][0]) > bracket_max:              # 最后一次采样离碰撞太远
        return None
    return le[-1][1] > le[-2][1]


def classify(dt, ms, lag, rot=None):
    """纯函数：三机制归类。dt/ms/lag/rot 均可为 None。"""
    if dt is not None and 0.0 <= dt < LATE_S:
        return CLS_LATE
    if rot is True:
        return CLS_INDE
    if rot is False:
        # 副本证明撞时不在旋转档 ⇒ 监督层未拦住（不论 ms 高低）
        return CLS_SUP
    if ms is not None and ms < DEADEND_LOW:
        if lag is not None and lag > LAG_MAX:
            return CLS_GAP
        return CLS_SUP
    return CLS_INDE


def from_ticks(rows, t, stale_max=2.0):
    """Tier 2：从探针逐拍线取 (dt, ms, lag, rot)。

    行 = [sim_t, cod, col_prob, veto_cnt, safe_cnt, forced]（见
    tools/defop_ab_run.py 的 TL_MAX 注释）。cod>0 = 真机死端拍。
    dt 取**上升沿**（与 `DEADEND enter` 同义）；ms/lag 取 t 前最后一个非 None
    的 col_prob；rot 取 t 前最后一拍的瞬时占用。
    """
    prev_cod, last_enter = 0, None
    cp_t = cp_v = None
    last = None
    for r in rows:
        if r[0] > t:
            break
        last = r
        if r[1] > 0 and prev_cod == 0:
            last_enter = r[0]
        prev_cod = r[1]
        if r[2] is not None:
            cp_t, cp_v = r[0], r[2]
    dt = (t - last_enter) if last_enter is not None else None
    lag = (t - cp_t) if cp_t is not None else None
    rot = None
    if last is not None and (t - last[0]) <= stale_max:
        rot = last[1] > 0
    return dt, cp_v, lag, rot


def probe_ticks(cell):
    """读 probe.json 的逐拍线（Tier 2）。缺/坏 => {}（退回 Tier 1）。"""
    p = os.path.join(cell, "probe.json")
    if not os.path.exists(p):
        return {}
    try:
        j = json.load(open(p))
    except Exception:                                   # noqa: BLE001
        return {}
    out = {}
    for k, v in (j.get("defop_debug") or {}).items():
        rows = v.get("tl") or []
        if rows:
            out[int(k)] = rows
    return out


def parse(cell):
    colls, enters, sups, frs, shad = [], {}, {}, {}, {}
    p = os.path.join(cell, "rosout.log")
    if not os.path.exists(p):
        return colls, enters, sups, frs, shad
    for ln in open(p, errors="ignore"):
        m = COLL.match(ln)
        if m:
            g = m.groups()
            colls.append((float(g[0]), int(g[1]), g[3] or "?",
                          (float(g[4]), float(g[5]), float(g[6]))))
            continue
        m = SHADOW.match(ln)
        if m:
            shad.setdefault(int(m.group(2)), []).append(
                (float(m.group(1)), float(m.group(3))))
            continue
        m = ENTER.match(ln)
        if m:
            enters.setdefault(int(m.group(2)), []).append(float(m.group(1)))
            continue
        m = SUP.match(ln)
        if m:
            sups.setdefault(int(m.group(2)), []).append(
                (float(m.group(1)), float(m.group(5)),
                 int(m.group(3)), int(m.group(4))))
            continue
        m = FR.match(ln)
        if m:
            frs.setdefault(int(m.group(2)), []).append(float(m.group(1)))
    for v in shad.values():
        v.sort()
    return colls, enters, sups, frs, shad


def analyze(cell, bracket_max=BRACKET_MAX):
    """逐起碰撞算 dt/ms/lag/rot 并归类。返回 (rows, n_enter, n_fr, n_sh)。

    证据源优先级：Tier 2 探针逐拍线（若 probe.json 有 tl）> Tier 1 rosout。
    逐拍源的字段缺失时**逐字段**回落，并用 src 标出最终用了哪一源。
    """
    colls, enters, sups, frs, shad = parse(cell)
    tls = probe_ticks(cell)
    rows = []
    for t, d, kind, xyz in colls:
        prev_e = [e for e in enters.get(d, []) if e <= t]
        dt_r = (t - prev_e[-1]) if prev_e else None
        prev_s = [s for s in sups.get(d, []) if s[0] <= t]
        ms_r = prev_s[-1][1] if prev_s else None
        lag_r = (t - prev_s[-1][0]) if prev_s else None
        rot_r = in_rot(shad.get(d, []), t, bracket_max)

        dt, ms, lag, rot, src = dt_r, ms_r, lag_r, rot_r, "rosout"
        if d in tls:
            dt_t, ms_t, lag_t, rot_t = from_ticks(tls[d], t)
            if rot_t is not None or dt_t is not None or ms_t is not None:
                src = "tick"
            dt = dt_t if dt_t is not None else dt_r
            ms = ms_t if ms_t is not None else ms_r
            lag = lag_t if lag_t is not None else lag_r
            rot = rot_t if rot_t is not None else rot_r

        rows.append(dict(t=t, d=d, kind=kind, xyz=xyz, dt=dt, ms=ms, lag=lag,
                         rot=rot, src=src, cls=classify(dt, ms, lag, rot)))
    return (rows, sum(len(v) for v in enters.values()),
            sum(len(v) for v in frs.values()),
            sum(len(v) for v in shad.values()))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    overall, total, n_rot_known = {}, 0, 0
    for B in sys.argv[1:]:
        print("=" * 78)
        print("批 %s" % B)
        print("=" * 78)
        tally, allc = {}, 0
        for c in sorted(glob.glob(os.path.join(B, "*"))):
            if not os.path.isdir(c) or not os.path.exists(
                    os.path.join(c, "rosout.log")):
                continue
            rows, n_en, n_fr, n_sh = analyze(c)
            print("\n[%s]  碰撞 %d 起  真机死端入口 %d 次  强制放宽 %d 次  "
                  "影子行 %d 条" % (os.path.basename(c), len(rows), n_en, n_fr, n_sh))
            print("   t       机 kind   dt_死端(s)  遇撞时在死端  min_score(滞后s)  源     归类")
            for r in rows:
                if r["rot"] is not None:
                    n_rot_known += 1
                print("  %7.2f  d%d %-6s %-11s %-13s %-16s %-6s %s"
                      % (r["t"], r["d"], r["kind"],
                         "%.3f" % r["dt"] if r["dt"] is not None else "无入口",
                         {True: "是", False: "否", None: "不可判"}[r["rot"]],
                         "%.2f(%.1fs)" % (r["ms"], r["lag"])
                         if r["ms"] is not None else "无读数",
                         r["src"], r["cls"]))
                tally[r["cls"]] = tally.get(r["cls"], 0) + 1
                overall[r["cls"]] = overall.get(r["cls"], 0) + 1
            allc += len(rows)
        total += allc
        print("\n  本批小计 %d 起: %s"
              % (allc, "  ".join("%s=%d" % kv for kv in sorted(tally.items()))))
    print("\n" + "=" * 78)
    print("合计 %d 起: %s" % (total, "  ".join("%s=%d" % kv
                                              for kv in sorted(overall.items()))))
    print("占用可判（有影子行且夹逼够近）: %d / %d" % (n_rot_known, total))
    return 0


sys.exit(main())
