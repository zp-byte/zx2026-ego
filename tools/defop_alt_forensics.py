#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_alt_forensics.py — 巡航高度判读器（只读）。

用法:  python3 tools/defop_alt_forensics.py run_logs/<归档> [<归档2> ...]

判读 nav 的 ALT 探针（2026-10-01 装）：
    nav_node: drone %d ALT z=%.2f gz=%.2f cmd=%.2f cpn=%d

按 **cpn 旗**分组给 z 的分位——cpn=1 即 CPN 持笔期，**这段的高度就是
"林带巡航高度"**（CPN 不读 goal z，接手后保持）。cpn=0 是 nav 持笔期
（terminal 投放 / departure 离台），两段都在起降台或投放台上方。

判读口径（防误读）：
  * 探针 1Hz 节流×每机，故一行 ≈ 1 秒。样本数 ≈ 机数 × 秒数。
  * 这是**行为读数**，不是判分——分数照旧看 score.yaml。
  * 若处置臂 cpn=1 中位与 A 臂无差 ⇒ **改动没生效**，先查因别看分数。

【2026-10-01 修：nav 持笔列的"停尸污染"】
初版把 cpn=0 的所有拍子都算进 nav 列，结果**已退赛/已落地坐在跑道上的机
仍以 1Hz 报数**（实测 d3 z=0.16、d4 z=0.20 这类），把 nav 列中位一路拖到
0.20——看着像"nav 在压高度"，其实是一堆尸体的读数。本版两处修正：
  ① cpn=0 列默认丢掉 z < MIN_Z(0.5) 的拍子，并**同时印出丢了多少**；
  ② 逐格点名"停尸机"（cpn=0 拍子里 ≥80% 低于 MIN_Z 且总拍数 ≥10 者）。
cpn=1 列**不做任何过滤**——那是本判读器的主读数，不许动。
"""
import glob
import io
import os
import re
import sys

PAT = re.compile(
    r"drone (\d+) ALT z=([-\d.]+) gz=([-\d.]+) cmd=([-\d.]+) cpn=(\d)")

MIN_Z = 0.50        # 地面判定：低于此值的拍子视为"落地/停尸"，不进 nav 列
PARK_FRAC = 0.80    # 某机 cpn=0 拍子里低于 MIN_Z 的比例超过它就点名
PARK_MIN_N = 10     # 且总拍数不少于这个数才点名（防两拍的偶发）


def pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    i = min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))
    return xs[i]


def scan_cell(cell):
    rosout = os.path.join(cell, "rosout.log")
    by = {0: [], 1: []}          # 0 列已过滤；1 列原样
    n0_raw = 0
    gz1 = []
    per_drone = {}
    per_drone0 = {}
    if os.path.isfile(rosout):
        for ln in io.open(rosout, encoding="utf-8", errors="ignore"):
            m = PAT.search(ln)
            if not m:
                continue
            d, z, gz, _cmd, cpn = (int(m.group(1)), float(m.group(2)),
                                   float(m.group(3)), float(m.group(4)),
                                   int(m.group(5)))
            if cpn == 0:
                n0_raw += 1
                per_drone0.setdefault(d, []).append(z)
                if z >= MIN_Z:
                    by[0].append(z)
            else:
                by[1].append(z)
                gz1.append(gz)
                per_drone.setdefault(d, []).append(z)
    parked = []
    for d, zs in per_drone0.items():
        if len(zs) >= PARK_MIN_N:
            lo = sum(1 for z in zs if z < MIN_Z) / float(len(zs))
            if lo >= PARK_FRAC:
                parked.append((d, lo, pct(zs, 0.5)))
    return dict(cell=os.path.basename(cell), by=by, gz1=gz1,
                drones=per_drone, n0_raw=n0_raw, parked=sorted(parked))


def fmt(xs):
    if not xs:
        return "    -   "
    return "%.2f/%.2f/%.2f/%.2f" % (pct(xs, 0.0), pct(xs, 0.5),
                                    pct(xs, 0.9), max(xs))


def main():
    dirs = sys.argv[1:]
    if not dirs:
        print(__doc__)
        return 1
    for d in dirs:
        cells = sorted(glob.glob(os.path.join(d, "*_defop_seed*")))
        if not cells:
            print("!! %s 下没找到格子" % d)
            continue
        print("=" * 96)
        print("归档 %s  (%d 格)" % (d, len(cells)))
        print("=" * 96)
        print("%-22s %6s %-12s  %-22s  %-22s  %s"
              % ("cell", "n1", "n0 raw/kept", "CPN持笔期 z  min/p50/p90/max",
                 "nav持笔期 z（已滤停尸）", "CPN期goal z p50"))
        for c in cells:
            r = scan_cell(c)
            print("%-22s %6d %5d/%-6d  %-22s  %-22s  %s"
                  % (r["cell"], len(r["by"][1]), r["n0_raw"], len(r["by"][0]),
                     fmt(r["by"][1]), fmt(r["by"][0]),
                     ("%.2f" % pct(r["gz1"], 0.5)) if r["gz1"] else "-"))
        # 停尸点名
        for c in cells:
            r = scan_cell(c)
            if not r["parked"]:
                continue
            print("    %s  停尸机（cpn=0 拍子里 %.0f%% 低于 %.2f m）: %s"
                  % (r["cell"], PARK_FRAC * 100, MIN_Z,
                     "  ".join("d%d(%.0f%%,z~%.2f)" % (k, f * 100, z)
                               for k, f, z in r["parked"])))
        # 分机 CPN 期中位（看是否六机一致，或个别机没跟上）
        for c in cells:
            r = scan_cell(c)
            if not r["drones"]:
                continue
            s = "  ".join("d%d:%.2f" % (k, pct(v, 0.5))
                          for k, v in sorted(r["drones"].items()))
            print("    %s  CPN期分机中位  %s" % (r["cell"], s))
        print("")
    print("判读：A 臂(cruise_free=false) CPN 期中位应 ≈1.2；B 臂 ≈1.5；C 臂 ≈2.4；D 臂 ≈3.0。")
    print("      处置臂中位与 A 臂相同 = 改动没生效，先查因。")
    print("      注意：CPN 会把 entry 高度往下压，入口 vs 稳态要看 entry_vs_cruise.py。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
