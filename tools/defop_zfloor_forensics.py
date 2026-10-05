#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_zfloor_forensics.py — z_floor_post 判读器（只读）。

用法:  python3 tools/defop_zfloor_forensics.py run_logs/defop_ab_<ts> [<ts2> ...]

对每个归档逐格统计三层指标 + 退赛 + 分数：

  LOWZ  窗口命中  z<0.9 且 cmd[2]<0      （病态窗口"低空+向下"）
  VIOL  真破地板  final < floor          （反应层把 cmd[2] 推到地板以下）
  DEEP  踩线      z < contact_z 0.30     （退赛前驱；宽限 1.5s 内收回则不判）

关键：VIOL 是**真违约**，LOWZ 只是窗口。2026-09-30 已知阳性档案
defop_ab_20260930_234339/00_defop_seed43_r0 基准 = LOWZ 36 / VIOL 8 / DEEP 1 /
zmin 0.24，同格 100 分 6/6 落地 retired=[] —— 病态常见、退赛是尾部。
"""
import glob
import io
import os
import re
import sys

PAT = re.compile(
    r"LOWZ z=([-\d.]+) goalz=([-\d.]+) base=([-\d.]+) floor=([-\d.]+) final=([-\d.]+)")
# COLLISION #n at (x,y,z) —— 抓第 3 个分量 z，用于按高度带切碰撞
COL_PAT = re.compile(
    r"COLLISION #\d+ at \(([-\d.]+),([-\d.]+),([-\d.]+)\)")
CONTACT_Z = 0.30
EPS = 1e-6


def read_score_yaml(path):
    """极简 score.yaml 读取（避开 yaml 依赖差异）：只要 total/landed_n/retired。"""
    d = {"total": None, "landed_n": None, "retired": None}
    try:
        txt = io.open(path, encoding="utf-8").read()
    except IOError:
        return d
    m = re.search(r"^total:\s*(\d+)", txt, re.M)
    if m:
        d["total"] = int(m.group(1))
    m = re.search(r"^landed_n:\s*(\d+)", txt, re.M)
    if m:
        d["landed_n"] = int(m.group(1))
    m = re.search(r"^retired:\s*\[(.*?)\]", txt, re.M)
    if m:
        s = m.group(1).strip()
        d["retired"] = len([x for x in s.split(",") if x.strip()]) if s else 0
    return d


def scan_collisions(cell):
    """碰撞计数 + 高度带分布。

    z_floor_post 只在 z<0.5 咬合，故必须分开数 z<0.5 的碰撞——那是本补丁
    唯一可能相互作用的带（dam v2 教训：速度地板=暴露时长调节器，减速可能
    反而延长暴露）。巡航带 z>1.0 的碰撞结构上够不到，用作"应无变化"的哨兵。
    """
    out = {"col": 0, "col_lo": 0, "zlo": [], "zhi": []}
    for name in ("collisions.txt", "rosout.log"):
        p = os.path.join(cell, name)
        if not os.path.isfile(p):
            continue
        seen = 0
        for ln in io.open(p, encoding="utf-8", errors="ignore"):
            m = COL_PAT.search(ln)
            if not m:
                continue
            seen += 1
            z = float(m.group(3))       # (x,y,z) 的第 3 分量
            if z < 0.5:
                out["col_lo"] += 1
                out["zlo"].append(z)
            else:
                out["zhi"].append(z)
        if seen:            # collisions.txt 优先；已有则不从 rosout 重复计
            out["col"] = seen
            break
    return out


def scan_cell(cell):
    rosout = os.path.join(cell, "rosout.log")
    tot = viol = deep = 0
    zmin = 99.0
    worst = None
    unauth = 0
    if os.path.isfile(rosout):
        for ln in io.open(rosout, encoding="utf-8", errors="ignore"):
            # 只认规则监视器的权威判决行（邻居释放"releasing its platform"
            # 等旁听行也含同一串，会 1 判决膨胀成 1+N，故必须锁 /rule_monitor）
            if "rule_monitor" in ln and "RETIRED (UNAUTHORIZED_LANDING)" in ln:
                unauth += 1
            m = PAT.search(ln)
            if not m:
                continue
            z, gz, base, floor, final = [float(x) for x in m.groups()]
            tot += 1
            if z < zmin:
                zmin = z
            if z < CONTACT_Z:
                deep += 1
            if final < floor - EPS:
                viol += 1
                dd = floor - final
                if worst is None or dd > worst[0]:
                    worst = (dd, z, floor, final)
    sc = read_score_yaml(os.path.join(cell, "score.yaml"))
    co = scan_collisions(cell)
    return dict(cell=os.path.basename(cell), lowz=tot, viol=viol, deep=deep,
                zmin=(zmin if tot else None), worst=worst, unauth=unauth,
                col=co["col"], col_lo=co["col_lo"], zlo=co["zlo"], zhi=co["zhi"],
                **sc)


def main():
    dirs = sys.argv[1:]
    if not dirs:
        print(__doc__)
        return 1
    grand = []
    for d in dirs:
        cells = sorted(glob.glob(os.path.join(d, "*_defop_seed*")))
        if not cells:
            print("!! %s 下没找到格子" % d)
            continue
        print("=" * 78)
        print("归档 %s  (%d 格)" % (d, len(cells)))
        print("=" * 78)
        print("%-22s %5s %5s %5s %6s %5s %4s %5s  %-5s %-5s %s"
              % ("cell", "LOWZ", "VIOL", "DEEP", "zmin", "UNAU", "COL", "COL<.5",
                 "score", "land", "retired"))
        rows = []
        for c in cells:
            r = scan_cell(c)
            rows.append(r)
            grand.append(r)
            print("%-22s %5d %5d %5d %6s %5d %4d %5d  %-5s %-5s %s"
                  % (r["cell"], r["lowz"], r["viol"], r["deep"],
                     ("%.2f" % r["zmin"]) if r["zmin"] is not None else "-",
                     r["unauth"], r["col"], r["col_lo"],
                     r["total"] if r["total"] is not None else "-",
                     r["landed_n"] if r["landed_n"] is not None else "-",
                     r["retired"] if r["retired"] is not None else "-"))
        print("-" * 78)
        n = len(rows)
        avg = lambda k: (sum(r[k] for r in rows if r[k] is not None)
                         / max(1, len([r for r in rows if r[k] is not None])))
        print("%-22s %5.1f %5.1f %5.1f %6s %5.1f %4.1f %5.1f  %-5.1f %-5.1f"
              % ("均值/格", avg("lowz"), avg("viol"), avg("deep"), "",
                 avg("unauth"), avg("col"), avg("col_lo"),
                 avg("total"), avg("landed_n")))
        zhi = [z for r in rows for z in r["zhi"]]
        zlo = [z for r in rows for z in r["zlo"]]
        if zhi or zlo:
            print("碰撞高度带: 巡航(z>=0.5) %d 次 z=%.2f~%.2f  |  低空(z<0.5) %d 次%s"
                  % (len(zhi), min(zhi) if zhi else 0, max(zhi) if zhi else 0,
                     len(zlo), (" z=%s" % ["%.2f" % z for z in zlo]) if zlo else ""))
        ws = [r["worst"] for r in rows if r["worst"]]
        if ws:
            w = max(ws, key=lambda t: t[0])
            print("最深违约: 深度 %.2f m/s @ z=%.2f (floor=%.2f final=%.2f)"
                  % w)
        print("")
    if grand:
        print("=" * 78)
        print("总计 %d 格: LOWZ=%d  VIOL=%d  DEEP=%d  UNAUTHORIZED_LANDING=%d"
              % (len(grand), sum(r["lowz"] for r in grand),
                 sum(r["viol"] for r in grand), sum(r["deep"] for r in grand),
                 sum(r["unauth"] for r in grand)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
