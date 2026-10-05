#!/usr/bin/env python3
"""③ 离线复算：把候选门 `deadend_veto_frac=X` 灌进**真机那份**
`_deadend_step_logic`（直接 import，不再写第二份实现 —— 本仓在
`_crown_z_range` 上中过"双实现漂移"），在批 5 的逐拍线上重放，量**代价**。

口径（必须写死，防误读）：
  · 重放跑在**已观测的轨迹**上。一旦进了死端，真机该拍会 stop/rotate，
    后续轨迹就与观测分叉 ⇒ **本工具只给"代价"（死端占用增量），不给"收益"**。
    收益（少撞几起 / landed 变不变）只能由同 seed43 三格 A/B 回答。
  · 代价指标 = 死端占用**时长**（ΣΔt，state≠0 的拍），不是上升沿计数：
    批 3 的教训就是「DE 26→62」这种计数掩盖了 rot 62.8→142.8s 的真代价。
  · 基线 = 同一份 tl、同一份状态机、`veto_hi=0.0`（= 关闭 = 逐字节旧行为）。

用法：python3 tools/sup33_replay.py run_logs/defop_ab_20261002_135937
"""
import json
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'defop_nav', 'scripts'))
from defop_cpn_node import _deadend_step_logic, _sup_bad  # noqa: E402


def load(base):
    for cname in sorted(os.listdir(base)):
        p = os.path.join(base, cname, "probe.json")
        if not os.path.exists(p):
            continue
        yield cname, json.load(open(p)).get("defop_debug", {})


def replay(tl, hi, lo, max_s, hyst_s, veto_hi, dwell=0.0):
    """返回 (死端占用秒数, state!=0 的拍数, 拍数)。"""
    st = {'t0': None, 'ok_t0': None}
    step, prev_t = 0, None
    sec, n_de, n = 0.0, 0, 0
    for r in tl:
        _v, _s = r[3], r[4]
        vf = (float(_v) / float(_v + _s)) if (_v is not None and _s is not None
                                              and (_v + _s) > 0) else None
        # ⚠ n_safe 必须取 **col 13 = len(indx1)**，不是 col 4 = safe_cnt：
        # 真机 `_deadend_action(min_score, indx1, ...)` 收的是 indx1
        # （分数窗 ∩ 安全掩膜），而 safe_cnt 是安全掩膜在 256 个基元上的
        # 全量计数 —— 两者不同（2026-10-02 已在节点内注释点明）。
        # 用错会同时压低基线死端占用并抬高"增益"，是本工具的第一个坑。
        n_safe = r[13] if len(r) > 13 and r[13] is not None else 0
        t = r[0]
        dt = 0.05 if prev_t is None else max(0.0, min(0.5, t - prev_t))
        step, _a, _f = _deadend_step_logic(
            step, r[2], n_safe, t, st, hi, lo, max_s, hyst_s,
            veto_frac=vf, veto_hi=veto_hi, veto_dwell_s=dwell)
        n += 1
        if step != 0:
            sec += dt
            n_de += 1
        prev_t = t
    return sec, n_de, n


def replay_marks(tl, hi, lo, max_s, hyst_s, veto_hi, dwell):
    """同 replay，但额外返回**进入死端的时刻表**（供算撞前提前量）。"""
    st = {'t0': None, 'ok_t0': None}
    step, prev_t = 0, None
    enters, sec, n_de = [], 0.0, 0
    for r in tl:
        _v, _s = r[3], r[4]
        vf = (float(_v) / float(_v + _s)) if (_v is not None and _s is not None
                                              and (_v + _s) > 0) else None
        n_safe = r[13] if len(r) > 13 and r[13] is not None else 0
        t = r[0]
        dt = 0.05 if prev_t is None else max(0.0, min(0.5, t - prev_t))
        prev = step
        step, _a, _f = _deadend_step_logic(
            step, r[2], n_safe, t, st, hi, lo, max_s, hyst_s,
            veto_frac=vf, veto_hi=veto_hi, veto_dwell_s=dwell)
        if prev == 0 and step == 1:
            enters.append(t)
        if step != 0:
            sec += dt
            n_de += 1
        prev_t = t
    return sec, n_de, enters


def main():
    base = sys.argv[1]
    hi, lo = 6.5, 5.0
    max_s, hyst = 4.0, 1.0
    cells = list(load(base))
    coll = {}
    for cname, _dbg in cells:
        cp = os.path.join(base, cname, "collisions.txt")
        lst = []
        if os.path.exists(cp):
            for ln in open(cp, errors="ignore"):
                m = re.search(r"(\d+\.\d+).*drone (\d+) COLLISION #\d+", ln)
                if m:
                    lst.append((float(m.group(1)), m.group(2)))
        coll[cname] = lst
    n_coll = sum(len(v) for v in coll.values())

    print("批 %s  格数=%d  碰撞=%d" % (os.path.basename(base), len(cells),
                                       n_coll))
    print("参数 hi=%.1f lo=%.1f max_s=%.1f hyst=%.1f（= 当前默认档）"
          % (hi, lo, max_s, hyst))
    print()
    print("  X      dwell   死端占用(s)  相对基线   撞前已进死端的提前量(s)")
    base_sec = None
    for X in (0.0, 0.55, 0.60, 0.65, 0.70):
        for dw in ((0.0,) if X == 0.0 else (0.0, 0.3, 0.5, 0.8, 1.2)):
            tot, leads = 0.0, []
            for cname, dbg in cells:
                for k in sorted(dbg, key=lambda x: int(x)):
                    s, _d, ent = replay_marks(dbg[k].get("tl") or [], hi, lo,
                                              max_s, hyst, X, dw)
                    tot += s
                    for (tc, dnum) in coll.get(cname, []):
                        if dnum != str(k):
                            continue
                        prev = [e for e in ent if e <= tc]
                        if prev:
                            leads.append(tc - prev[-1])
            if base_sec is None:
                base_sec, lab = tot, "基线(关闭)"
            else:
                lab = "%+.1f%%" % (100.0 * (tot - base_sec) / max(1e-9, base_sec))
            ls = ("%d/%d 起  min=%.2f p50=%.2f max=%.2f"
                  % (len(leads), n_coll, min(leads), sorted(leads)[len(leads) // 2],
                     max(leads))) if leads else "0/%d 起" % n_coll
            print("  %-5s  %-6s %10.1f  %-10s %s"
                  % ("%.2f" % X, "%.2f" % dw, tot, lab, ls))
    print()
    print("⚠ 只看代价 + 覆盖面。收益（碰撞/landed）必须由 A/B 回答：本重放一旦"
          "进死端，后续轨迹就与观测分叉，不可外推。")


if __name__ == "__main__":
    main()
