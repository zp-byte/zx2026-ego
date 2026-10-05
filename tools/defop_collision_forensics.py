#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_collision_forensics.py <run_logs 归档目录> [...] — 碰撞归因仪（只读）。

A/B 的判据不能只看分。本工具把一个跑批归档（run_logs/defop_ab_<ts>/<cell>/）
拆成可对质的四类证据，全部从**已落盘的文件**里取，不重跑、不猜：

  1) 结局      score.yaml      -> total / landed_n / s1 / s2 / retired
  2) 机体指标  nav_metrics.yaml-> 每机 collisions / min_clear_m / flips /
                                 stuck_s / speed / max_speed
  3) 碰撞解剖  rosout.log      -> 逐条 COLLISION，**第一刀 = kind**（monitor 直接
                                 判出的 trunk/branch/crown），次级按**弹向 dz** 与
                                 **距平台**分类：
                                 dz==0 -> 水平化弹向（树干/树枝）
                                 dz!=0 -> 机-机/灌木/围栏（保留垂直分量）
                                 距任一平台 <2.0m -> 平台邻域（投放段）
                                 —— 区分"树林穿越撞"与"平台挤压撞"是历次
                                    归因的第一刀（两者的修法完全不同）
                                 ⚠ 解析数 < 原文 `COLLISION #` 条数即 SystemExit
                                   （正则与日志脱节时**不许静默少报**，2026-10-02 修）
  4) 感知速率  rosout.log      -> depth_fill 帧计数 -> 每条流实测 Hz
                                 （论证"反应不过来"必须用实测速率，不能用名义值）
  5) 监督层    rosout.log      -> veto/safe 读数分布（拦了没有/拦了多少）
  6) 退赛      rosout.log      -> RETIRED 行（级联引信）

用法：
  python3 tools/defop_collision_forensics.py run_logs/defop_ab_20260930_224047
  python3 tools/defop_collision_forensics.py A/00_x B/00_x      # 多格对照
"""
import glob
import os
import re
import sys

import yaml

PLATFORMS = [(27.0, -9.0), (26.8, -0.2), (27.0, 9.0)]   # 三平台心（约值，仅供邻域分类）


def _read(path):
    try:
        return open(path, encoding="utf-8", errors="replace").read()
    except IOError:
        return ""


def score_of(cell):
    p = os.path.join(cell, "score.yaml")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return yaml.safe_load(f)


def metrics_of(cell):
    p = os.path.join(cell, "nav_metrics.yaml")
    if not os.path.exists(p):
        return {}
    with open(p, encoding="utf-8") as f:
        return (yaml.safe_load(f) or {}).get("drones", {}) or {}


# 2026-10-02 修：日志行自 `kind=` 字段落地后变成
#   `drone 1 COLLISION #1 kind=trunk at (x,y,z) dir=(dx,dy,dz)`
# 旧正则不含该段 ⇒ **静默解析 0 条**（110544 批真值 5、旧版报 0）。
# 故：① `kind=` 设为**可选组**（历史归档无此字段，保持可读）；
#     ② 加**防假阴性断言**（见 collisions_of）—— 解析数 < 原文条数即硬错。
COLL = re.compile(
    r"^([0-9.]+) .*drone (\d+) COLLISION #(\d+) (?:kind=(\w+) )?"
    r"at \(([-0-9.]+),([-0-9.]+),([-0-9.]+)\) "
    r"dir=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)", re.M)

_COLL_RAW = re.compile(r"COLLISION #\d+")


def collisions_of(txt):
    out = []
    for m in COLL.finditer(txt):
        t, i, n, kind, x, y, z, dx, dy, dz = m.groups()
        x, y, z, dz = float(x), float(y), float(z), float(dz)
        near_pf = min(((x - px) ** 2 + (y - py) ** 2) ** 0.5 for px, py in PLATFORMS)
        out.append(dict(t=float(t), drone=int(i), n=int(n),
                        kind=(kind or "?"), xyz=(x, y, z),
                        dz=dz, near_pf=near_pf))
    # 防假阴性：正则与日志格式脱节时必须**响**，不许静默少报（本仓已中一次）。
    n_raw = len(_COLL_RAW.findall(txt))
    if len(out) < n_raw:
        raise SystemExit(
            "[forensics] 解析 %d 条 / 原文 %d 条 —— 正则与日志格式已脱节，"
            "拒绝给出结论（先对齐 COLL 正则）" % (len(out), n_raw))
    return out


FRAME = re.compile(r"^([0-9.]+) .*/drone_(\d+)/defop_depth_fill.*frame (\d+),", re.M)


def depth_rate_of(txt):
    """每条流：首末 (t, frame) 线性斜率 -> Hz（实测，非名义）。"""
    per = {}
    for m in FRAME.finditer(txt):
        t, i, n = float(m.group(1)), int(m.group(2)), int(m.group(3))
        per.setdefault(i, []).append((t, n))
    out = {}
    for i, pts in per.items():
        if len(pts) < 2:
            continue
        (t0, n0), (t1, n1) = pts[0], pts[-1]
        if t1 > t0:
            out[i] = (n1 - n0) / (t1 - t0)
    return out


VETO = re.compile(r"drone_(\d+) veto=(\d+) safe=(\d+) min_score=([^ ]*)")


def veto_of(txt):
    acc = {}
    for m in VETO.finditer(txt):
        i, v, s = int(m.group(1)), int(m.group(2)), int(m.group(3))
        a = acc.setdefault(i, {"n": 0, "veto": [], "safe": []})
        a["n"] += 1
        a["veto"].append(v)
        a["safe"].append(s)
    return acc


RETIRE = re.compile(r"^([0-9.]+) .*drone (\d+) RETIRED \(([^)]*)\)", re.M)


def report(cell):
    print("=" * 78)
    print("CELL %s" % cell)
    sc = score_of(cell)
    txt = _read(os.path.join(cell, "rosout.log"))

    if sc:
        print("  结局: total=%s landed=%s/%s s1=%s s2=%s retired=%s"
              % (sc.get("total"), sc.get("landed_n"), sc.get("s2_eligible_n"),
                 sc.get("s1_total"), sc.get("s2"), sc.get("retired")))
    else:
        print("  结局: (无 score.yaml — 未跑完/失败)")

    m = metrics_of(cell)
    if m:
        print("  机体指标:")
        print("    %-4s %-5s %-9s %-6s %-7s %-6s %-6s"
              % ("机", "碰撞", "min_clr", "flips", "stuck_s", "speed", "max_v"))
        for i in sorted(m, key=lambda k: int(k)):
            d = m[i]
            print("    %-4s %-5s %-9s %-6s %-7s %-6s %-6s"
                  % (i, d.get("collisions"), d.get("min_clear_m"),
                     d.get("flips"), d.get("stuck_s"), d.get("speed"),
                     d.get("max_speed")))

    cs = collisions_of(txt)
    if cs:
        # 第一刀 = kind（真枝入物理后，trunk/branch/crown 比 dz 更能直接指路）；
        # dz 与"平台邻域"降为次级分类。
        byk = {}
        for c in cs:
            byk[c["kind"]] = byk.get(c["kind"], 0) + 1
        tree_n = sum(v for k, v in byk.items() if k in ("trunk", "branch", "crown"))
        other = [c for c in cs if abs(c["dz"]) >= 1e-6]
        pf = [c for c in cs if c["near_pf"] < 2.0]
        print("  碰撞解剖: 共 %d  | 树类 %d  | 按 kind: %s  | 非零弹向 dz!=0 %d  | 平台邻域(<2m) %d"
              % (len(cs), tree_n,
                 " ".join("%s=%d" % (k, v) for k, v in sorted(byk.items())),
                 len(other), len(pf)))
        byd = {}
        for c in cs:
            byd[c["drone"]] = byd.get(c["drone"], 0) + 1
        print("    每机: %s" % " ".join("d%d=%d" % (k, v) for k, v in sorted(byd.items())))
        print("    首次碰撞 t=%.2f  末次 t=%.2f" % (cs[0]["t"], cs[-1]["t"]))
        for c in cs:
            x, y, z = c["xyz"]
            print("      t=%7.2f d%d #%d %-7s (%7.2f,%7.2f,%5.2f) dz=%+5.2f 距台%.2f"
                  % (c["t"], c["drone"], c["n"], c["kind"], x, y, z,
                     c["dz"], c["near_pf"]))

    dr = depth_rate_of(txt)
    if dr:
        vals = ", ".join("d%d=%.2fHz" % (k, v) for k, v in sorted(dr.items()))
        print("  补洞深度流实测速率: %s" % vals)

    vo = veto_of(txt)
    if vo:
        parts = []
        for k in sorted(vo):
            vs = vo[k]["veto"]
            parts.append("d%d n=%d veto %d-%d" % (k, vo[k]["n"], min(vs), max(vs)))
        print("  监督层读数: %s" % " | ".join(parts))

    for m2 in RETIRE.finditer(txt):
        print("  退赛: t=%.2f drone %s (%s)" % (float(m2.group(1)), m2.group(2),
                                                m2.group(3)))


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    cells = []
    for a in args:
        if os.path.isdir(os.path.join(a, "rosout.log")) or os.path.exists(
                os.path.join(a, "score.yaml")):
            cells.append(a)
        else:
            cells.extend(sorted(glob.glob(os.path.join(a, "*"))))
    for c in cells:
        if os.path.isdir(c):
            report(c)
    return 0


sys.exit(main())
