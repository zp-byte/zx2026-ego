#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_ab_report.py — DeFoP A/B 矩阵报告（读 defop_ab_run.py 的 CSV）。

用法（WSL）：
  cd ~/zx2026_defop_ws && python3 tools/defop_ab_report.py results/defop_ab_p3.csv

口径（P3 定谳，2026-09-29）：
  * **禁单判**：同 seed 同臂跑次有方差（基线 seed43 实测 55/70/80 三档），
    报告必须给 min/max 展布，不给单一均值结论。
  * **ABORT/ERROR/行内 FAIL 区分**：ABORT/ERROR=基建问题不计入臂统计；
    FAIL=真跑出来的比赛口径失败（如退赛），必须计入（它是结果不是噪声）。
  * **cpn_ratio 是 A/B 前提校验**：defop 臂 cpn_live=0 -> 探针没起（该 cell
    结论不可用）；cpn_live>0 而 cpn_ratio≈0 -> 旗恒 False（cpn 静默失效，
    该 cell 实际跑的是基线行为，必须从 defop 臂里剔除或标注）。
  * 缺失列（旧 CSV）按空串读，不进统计。
"""
import csv
import os
import sys
from collections import OrderedDict

NUM = ("score", "landed", "col", "minclr", "done_t", "rtf", "vel_cmd_hz",
       "latent_hz", "latent_live", "cpn_live", "cpn_duty_s", "cpn_ratio")


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _fleet_dist(cell_dir):
    """nav_metrics.yaml 逐机 dist_m 求和（跨臂公平口径：碰撞率=col/100m）。

    defop 臂巡航 1.0 m/s vs 基线 2.0 m/s——同任务窗内 defop 飞得更短更慢，
    原始碰撞计数不可直接跨臂比；碰撞/里程才是同一把尺。cell_dir 缺失或
    yaml 坏 -> None（如实缺席，不猜）。
    """
    p = os.path.join(cell_dir or "", "nav_metrics.yaml")
    if not os.path.isfile(p):
        return None
    try:
        import yaml
        with open(p, encoding="utf-8") as f:
            j = yaml.safe_load(f)
        tot = 0.0
        for v in (j.get("drones") or {}).values():
            tot += float(v.get("dist_m", 0.0) or 0.0)
        return tot if tot > 0 else None
    except Exception:
        return None


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in NUM:
            r[k] = fnum(r.get(k))
        r["dist_m"] = _fleet_dist(r.get("cell_dir"))
        c, d = r.get("col"), r.get("dist_m")
        r["col_per_100m"] = (round(100.0 * c / d, 2)
                             if (c is not None and c >= 0 and d) else None)
    return rows


def usable(r):
    """ABORT/ERROR=基建不计；其余（含 FAIL）计入。"""
    return r.get("verdict") not in ("ABORT", "ERROR")


def stats(rows, key, want_nonneg=True):
    vals = [r[key] for r in rows
            if isinstance(r.get(key), float) and (r[key] >= 0 or not want_nonneg)]
    if not vals:
        return None
    vals.sort()
    return (len(vals), sum(vals) / len(vals), vals[0], vals[-1])


def fmt(st, f="%.2f"):
    if st is None:
        return "n=0"
    n, m, lo, hi = st
    return ("n=%d mean=" + f + " [min=" + f + " max=" + f + "]") % (n, m, lo, hi)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "results/defop_ab_p3.csv"
    rows = [r for r in load(path) if usable(r)]
    if not rows:
        print("无可用行（全 ABORT/ERROR 或空文件）: %s" % path)
        return 1
    arms = list(OrderedDict.fromkeys(r["arm"] for r in rows))
    seeds = sorted(set(int(r["seed"]) for r in rows))

    print("A/B 报告: %s   可用行 %d（ABORT/ERROR 已剔除）" % (path, len(rows)))
    print("臂=%s  seed=%s" % (",".join(arms), seeds))

    # ---- 臂级 ----
    print("\n===== 臂级 =====")
    for a in arms:
        ar = [r for r in rows if r["arm"] == a]
        pv = sum(1 for r in ar if r["verdict"] == "PASS")
        print("[%s] n=%d PASS=%d" % (a, len(ar), pv))
        for k, f in (("score", "%.1f"), ("landed", "%.2f"), ("col", "%.2f"),
                     ("col_per_100m", "%.2f"), ("dist_m", "%.0f"),
                     ("minclr", "%.3f"), ("done_t", "%.0f"), ("rtf", "%.2f"),
                     ("vel_cmd_hz", "%.1f"), ("latent_hz", "%.2f")):
            print("    %-12s %s" % (k, fmt(stats(ar, k), f)))
        if a == "defop":
            print("    %-10s %s   <- A/B 前提校验"
                  % ("cpn_live", fmt(stats(ar, "cpn_live"), "%.1f")))
            print("    %-10s %s   <- 持笔占比（>0=cpn 真持过 cruise）"
                  % ("cpn_ratio", fmt(stats(ar, "cpn_ratio"), "%.3f")))
            bad = [r for r in ar
                   if r.get("cpn_live") == 0
                   or (r.get("cpn_live") and r.get("cpn_ratio") is not None
                       and 0 <= r["cpn_ratio"] < 0.05)]
            if bad:
                print("    !! %d cell 前提校验未过（探针没起/旗恒 False）:"
                      % len(bad))
                for r in bad:
                    print("       seed%s r%s live=%s ratio=%s %s"
                          % (r["seed"], r["run"], r.get("cpn_live"),
                             r.get("cpn_ratio"), r.get("note", "")))

    # ---- 按 seed 配对 ----
    print("\n===== 按 seed（每格 min-max 展布，禁单判）=====")
    for s in seeds:
        for a in arms:
            ar = [r for r in rows if r["arm"] == a and int(r["seed"]) == s]
            if not ar:
                continue
            sc = [r["score"] for r in ar if isinstance(r["score"], float)
                  and r["score"] >= 0]
            co = [r["col"] for r in ar if isinstance(r["col"], float)
                  and r["col"] >= 0]
            ld = [r["landed"] for r in ar if isinstance(r["landed"], float)
                  and r["landed"] >= 0]
            print("seed%-3s %-8s score=%s  col=%s  landed=%s"
                  % (s, a,
                     ("%s" % sorted(sc)) if sc else "-",
                     ("%s" % sorted(co)) if co else "-",
                     ("%s" % sorted(ld)) if ld else "-"))

    # ---- note 汇总（退赛/污染/门失败等如实列出）----
    notes = [(r["arm"], r["seed"], r["run"], r["note"]) for r in rows
             if r.get("note")]
    if notes:
        print("\n===== note（非空行）=====")
        for a, s, ru, n in notes:
            print("  %s seed%s r%s: %s" % (a, s, ru, n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
