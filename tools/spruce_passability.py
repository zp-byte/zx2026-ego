#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云杉林通过性核算：冠升级成硬障碍后，林带在每个高度层还剩多少可走。

随场景替换（杨树→云杉）新增，是本轮「干+冠都碰撞」决策的**唯一判据**：
杨树时代冠全在巡航带之上，冠参不参与碰撞都一样；云杉冠底中位 1.22m 正压
巡航带，若冠一进碰撞/占用就封死林带，方案不成立。

口径（与 nav 一致）：
  - 栅格 0.5 m（sim_settings 的 grid resolution），场地 [-30,30]²
  - 膨胀 pad = drone_radius 0.35 + inflation 0.4 = 0.75 m
  - 判定 = Scene.collides_xy 的带式语义：树干圆盘 ∩ [地面, 干顶]、
    冠包络圆盘 ∩ [冠底, 树顶]，各自与查询带重叠才封格
  - 连通 = 逐列洪水填充，西边界(x=-30) → 东边界(x=+30)

用法: python3 tools/spruce_passability.py [table路径]
退出码 0=断言全过，1=有层 BLOCK 或巡航层非 PASS
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
DEF = os.path.join(WS, "src/zx2026_common/config/forest_spruce_table.yaml")

RES = 0.5
LO, HI = -30.0, 30.0
NX = int(round((HI - LO) / RES))
DRONE_R = 0.35
INFLATION = 0.4
PAD = DRONE_R + INFLATION          # 0.75
CRUISE_Z = 1.2                     # competition_rules.heights.cruise_z
GZ = 0.0


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["trees"]


def make_grid(trees):
    """返回 (gx, GX, GY)：格中心坐标的 meshgrid。"""
    gx = [LO + (i + 0.5) * RES for i in range(NX)]
    return gx


def occupied(trees, z_lo, z_hi, pad, use_crown=True):
    """布尔栅格：整格中心落在障碍内即占用（与 nav 的 cell-center 采样同口径）。"""
    occ = [[False] * NX for _ in range(NX)]
    gx = [LO + (i + 0.5) * RES for i in range(NX)]
    for t in trees:
        bot = t["crown_z"] - t["crown_h"] / 2.0
        top = t["crown_z"] + t["crown_h"] / 2.0
        tr_r = t["trunk_r"] + pad
        cr_r = t["crown_r"] + pad
        trunk_on = not (GZ + t["trunk_h"] < z_lo or GZ > z_hi)
        crown_on = use_crown and not (top < z_lo or bot > z_hi)
        if not (trunk_on or crown_on):
            continue
        rmax = max(tr_r if trunk_on else 0.0, cr_r if crown_on else 0.0)
        ix0 = max(0, int((t["x"] - rmax - LO) / RES) - 1)
        ix1 = min(NX - 1, int((t["x"] + rmax - LO) / RES) + 1)
        iy0 = max(0, int((t["y"] - rmax - LO) / RES) - 1)
        iy1 = min(NX - 1, int((t["y"] + rmax - LO) / RES) + 1)
        for iy in range(iy0, iy1 + 1):
            dy = gx[iy] - t["y"]
            for ix in range(ix0, ix1 + 1):
                dx = gx[ix] - t["x"]
                d2 = dx * dx + dy * dy
                if trunk_on and d2 < tr_r * tr_r:
                    occ[iy][ix] = True
                elif crown_on and d2 < cr_r * cr_r:
                    occ[iy][ix] = True
    return occ


def connected(occ):
    """西边界 → 东边界的 4 邻域连通（BFS）。"""
    free = [[not occ[iy][ix] for ix in range(NX)] for iy in range(NX)]
    seen = [[False] * NX for _ in range(NX)]
    stack = []
    for iy in range(NX):
        if free[iy][0]:
            seen[iy][0] = True
            stack.append((iy, 0))
    while stack:
        iy, ix = stack.pop()
        if ix == NX - 1:
            return True
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            jy, jx = iy + dy, ix + dx
            if 0 <= jy < NX and 0 <= jx < NX and free[jy][jx] and not seen[jy][jx]:
                seen[jy][jx] = True
                stack.append((jy, jx))
    return False


def free_frac(occ):
    n = sum(1 for row in occ for v in row if not v)
    return 100.0 * n / (NX * NX)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEF
    trees = load(path)
    print("表: %s  (%d 棵)" % (path, len(trees)))
    print("栅格 %dx%d @%.1fm，场地 [%.0f,%.0f]²，膨胀 %.2f m（drone %.2f + infl %.2f）"
          % (NX, NX, RES, LO, HI, PAD, DRONE_R, INFLATION))

    bands = [(0.70, 2.60, "map_z_band（现役证据窗）"),
             (0.70, 1.60, "低空"),
             (0.70, 1.30, "低空更窄"),
             (0.90, 1.50, "投放回收段"),
             (0.70, 0.90, "贴地"),
             (2.60, 4.50, "冠上半段（飞越体制候选）")]
    fails = []
    print("\n%-9s %-30s %10s %10s %8s" % ("z 带", "说明", "只认干", "干+冠", "东西连通"))
    for zlo, zhi, lab in bands:
        o1 = occupied(trees, zlo, zhi, PAD, use_crown=False)
        o2 = occupied(trees, zlo, zhi, PAD, use_crown=True)
        conn = connected(o2)
        print("[%.2f,%.2f] %-30s %9.1f%% %9.1f%% %8s"
              % (zlo, zhi, lab, free_frac(o1), free_frac(o2),
                 "PASS" if conn else "BLOCK"))
        if not conn:
            fails.append("z 带 [%.2f,%.2f] 东西不连通" % (zlo, zhi))

    # 巡航层必检（本轮硬判据）
    print("\n--- 巡航层硬判据（cruise_z=%.2f）---" % CRUISE_Z)
    o = occupied(trees, CRUISE_Z - 0.5, CRUISE_Z + 0.5, PAD)
    conn = connected(o)
    print("  z 带 [%.2f,%.2f] 自由格 %.1f%%  东西连通 %s"
          % (CRUISE_Z - 0.5, CRUISE_Z + 0.5, free_frac(o),
             "PASS" if conn else "BLOCK"))
    if not conn:
        fails.append("巡航层 z=%.2f 东西不连通 —— 冠升级方案不成立" % CRUISE_Z)

    print("\n--- 冠底分布（决定带上限立论）---")
    bot = sorted(t["crown_z"] - t["crown_h"] / 2.0 for t in trees)
    n = len(trees)
    print("  冠底 < %s 的树: %d / %d"
          % (CRUISE_Z, sum(1 for b in bot if b < CRUISE_Z), n))
    print("  冠底 > 2.60（带顶）的树: %d" % sum(1 for b in bot if b > 2.60))

    print()
    if fails:
        print("== FAIL %d 项 ==" % len(fails))
        for m in fails:
            print("   - " + m)
        return 1
    print("== 全绿：每层都东西连通，巡航层过线 ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())
