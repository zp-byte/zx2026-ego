#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高度带通过性核查：巡航带到 [2.0, 4.9] 后，林带在每个高度层还剩多少可走。

**为什么需要**：2026-09-23 的「冠下穿行」体制（cruise 1.2）从未让机体进过枝区；
把带抬到 2.0~4.9 后，机体将进入**真枝层**（世界系最低枝 < 4 m，且**每棵树都挂枝**
——203 棵全有，见 check_poplar_branches.py:163）。真枝是硬碰撞体，密度若高到封死
走廊，则整套「带体制」不成立。这是计划 §六.3 的**唯一判据**，离线可答。

**口径**：直接复用 `Scene.collides_xy`（唯一实现，含干+冠门控+真枝），
不写第二份几何逻辑——本仓在双实现上翻过两次车（`_crown_z_range`、
`_deadend_step_logic`）。栅格 0.5 m、膨胀 pad = drone_radius + inflation。

**可失败断言**（防空集假 PASS，本仓踩过）：
  [A] 基线层 z=1.2 必须东西连通（现役体制可用性自证）
  [B] 占用随 z 抬升**单调不减**（干全高 5.2~9.5 覆盖全域，枝只做加法）
  [C] 受检格数 > 0 且占用格数 > 0（空场景会让 [A][B] 全过 = 假绿）
  [D] 至少有一棵树在带内 2.0~4.9 有枝（否则「进枝区」是空谈）

用法: python3 tools/band_feasibility.py [res] [z1,z2,...]
退出码 0=断言全过，1=有断言失败
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WS, "src", "zx2026_common", "scripts"))

TBL = os.path.join(WS, "src", "zx2026_common", "config",
                   "forest_poplar_table.yaml")

LO, HI = -30.0, 30.0
DRONE_R = 0.35
INFLATION = 0.4
PAD = DRONE_R + INFLATION          # 0.75，与 nav 栅格膨胀同口径
DEFAULT_RES = 0.5
DEFAULT_ZS = [1.2, 2.0, 2.4, 2.6, 3.0, 3.5, 4.0, 4.5, 4.9]

FAILS = []


def chk(name, cond, detail=""):
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        FAILS.append(name)


def connected(occ, nx, ny):
    """西边界 → 东边界的 4 邻域连通（BFS）。"""
    free = [[not occ[iy][ix] for ix in range(nx)] for iy in range(ny)]
    seen = [[False] * nx for _ in range(ny)]
    stack = [(iy, 0) for iy in range(ny) if free[iy][0]]
    for iy, _ in stack:
        seen[iy][0] = True
    while stack:
        iy, ix = stack.pop()
        if ix == nx - 1:
            return True
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            jy, jx = iy + dy, ix + dx
            if 0 <= jy < ny and 0 <= jx < nx and free[jy][jx] and not seen[jy][jx]:
                seen[jy][jx] = True
                stack.append((jy, jx))
    return False


def main():
    res = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_RES
    zs = ([float(v) for v in sys.argv[2].split(",")]
          if len(sys.argv) > 2 else DEFAULT_ZS)
    nx = int(round((HI - LO) / res))
    gx = [LO + (i + 0.5) * res for i in range(nx)]

    from zx2026_common.scene import Scene
    s = Scene()
    trees = [o for o in s.obstacles if o.kind == "tree"]

    print("=" * 78)
    print("高度带通过性核查 —— 场景 %d 障碍（树 %d）" % (len(s.obstacles), len(trees)))
    print("栅格 %dx%d @%.2fm 场地 [%.0f,%.0f]²  膨胀 %.2f m (drone %.2f + infl %.2f)"
          % (nx, nx, res, LO, HI, PAD, DRONE_R, INFLATION))
    print("判据 = Scene.collides_xy 带式语义（干圆盘∩[地,干顶] + 冠门控 + 真枝线段）")
    print("=" * 78)

    # ---- 枝的世界系 z 分布（决定「带抬到哪才开始进枝区」）----
    allseg = [(seg, t) for t in trees for seg in (t.branches or [])]
    if allseg:
        zlo_seg = [min(sg[2], sg[5]) for sg, _ in allseg]
        zhi_seg = [max(sg[2], sg[5]) for sg, _ in allseg]
        ntrees_branch = len(set(id(t) for _, t in allseg))
        print("\n真枝（世界系）：%d 段 / %d 棵（共 %d 棵）"
              % (len(allseg), ntrees_branch, len(trees)))
        print("  段 z 下沿 min=%.2f  段 z 上沿 max=%.2f" % (min(zlo_seg), max(zhi_seg)))
        # 分位（下沿）
        zs_sorted = sorted(zlo_seg)
        q = lambda p: zs_sorted[int(p * (len(zs_sorted) - 1))]
        print("  下沿分位: p05=%.2f p25=%.2f p50=%.2f p75=%.2f p95=%.2f"
              % (q(.05), q(.25), q(.50), q(.75), q(.95)))
        # 逐层：有多少棵树的枝与 [z,z] 重叠
        print("\n  逐层枝覆盖（树中有枝落在该 z 的棵数 / %d）：" % len(trees))
        for z in zs:
            n = sum(1 for t in trees
                    if any(min(sg[2], sg[5]) <= z <= max(sg[2], sg[5])
                           for sg in (t.branches or [])))
            print("    z=%.2f  %3d 棵 (%.0f%%)" % (z, n, 100.0 * n / len(trees)))
    else:
        print("\n!! 无真枝段 —— 场景未挂 branch_file？")

    # ---- 逐层占用 / 连通 ----
    print("\n%-8s %10s %10s %8s" % ("z 带", "自由格%", "占用格", "东西连通"))
    rows = []
    occs = []
    for z in zs:
        occ = [[False] * nx for _ in range(nx)]
        nfree = 0
        for iy in range(nx):
            y = gx[iy]
            for ix in range(nx):
                if s.collides_xy(gx[ix], y, z, z, pad=PAD):
                    occ[iy][ix] = True
                else:
                    nfree += 1
        nocc = nx * nx - nfree
        conn = connected(occ, nx, nx)
        rows.append((z, nfree, nocc, conn))
        occs.append(occ)
        print("[%.2f] 单层 %7.1f%%  %9d  %8s"
              % (z, 100.0 * nfree / (nx * nx), nocc, "PASS" if conn else "BLOCK"))

    # ---- 带内垂直自由度（**选高器真正依赖的量**）----
    # 单层连通过 ≠ 带可飞：机体要能「此处被挡就升/降一格」。真正的问题是
    # **每个 (x,y) 在带内还剩几档自由**，以及「存在一条通道，其每格都至少有一档自由」。
    print("\n--- 带内垂直自由度（%d 档 z 切片）---" % len(zs))
    nz = len(zs)
    cnt_free = [[sum(1 for k in range(nz) if not occs[k][iy][ix])
                 for ix in range(nx)] for iy in range(nx)]
    n_any = sum(1 for iy in range(nx) for ix in range(nx) if cnt_free[iy][ix] > 0)
    n_all = sum(1 for iy in range(nx) for ix in range(nx) if cnt_free[iy][ix] == nz)
    print("  至少一档自由 : %6.1f%%   （其余格 = 全带被封，选高器必回落 nominal）"
          % (100.0 * n_any / (nx * nx)))
    print("  全档自由     : %6.1f%%" % (100.0 * n_all / (nx * nx)))
    # 至少一档自由的格的连通性（带内通道存在性）
    union_occ = [[all(occs[k][iy][ix] for k in range(nz)) for ix in range(nx)]
                 for iy in range(nx)]
    conn_band = connected(union_occ, nx, nx)
    print("  带内通道东西连通（每格至少一档自由）: %s"
          % ("PASS" if conn_band else "BLOCK"))
    hist = {}
    for iy in range(nx):
        for ix in range(nx):
            hist[cnt_free[iy][ix]] = hist.get(cnt_free[iy][ix], 0) + 1
    print("  自由档数分布: " + "  ".join(
        "%d档=%d" % (k, hist[k]) for k in sorted(hist, reverse=True)))

    # ---- 断言 ----
    print("\n--- 断言 ---")
    d = dict((z, (nf, no, cn)) for z, nf, no, cn in rows)
    # [C] 防空集假绿
    total = sum(no for _, _, no, _ in rows)
    chk("[C] 受检非空（占用格合计 > 0）", total > 0, "占用格合计 %d" % total)
    # [A] 基线可走
    if 1.2 in d:
        chk("[A] 基线 z=1.2 东西连通（现役体制自证）", d[1.2][2],
            "自由 %.1f%%" % (100.0 * d[1.2][0] / (nx * nx)))
    # [B] 单调不减 —— **只在 z>=3.0 段**：干全高覆盖该段，枝只做加法 ⇒ 必不减。
    #     z<3.0 段有意排除：屏障网(kind=net, z 0~2.50)与围栏(top 2.80)是
    #     **有限高盒障碍**，在各自顶部以上释放格 ⇒ 占用会**下降**（实测
    #     z=2.4 占 539 → z=2.6 占 525，差的 14 格正是 net 让出的）。
    #     这不是缺陷而是关键事实：**巡航 ≥2.6 时 net 不再封路**。
    zs_mono = [z for z in zs if z >= 3.0]
    bad = []
    for a, b in zip(zs_mono, zs_mono[1:]):
        if d[b][1] < d[a][1] - 1:      # 容 1 格数值噪声
            bad.append("z=%.2f 占用 %d < z=%.2f 的 %d" % (b, d[b][1], a, d[a][1]))
    chk("[B] z>=3.0 段占用单调不减（干全覆盖，枝只做加法）", not bad, "; ".join(bad[:3]))
    # [D] 带内确实有枝
    if allseg:
        n_in = sum(1 for t in trees
                   if any(min(sg[2], sg[5]) <= 4.9 and max(sg[2], sg[5]) >= 2.0
                          for sg in (t.branches or [])))
        chk("[D] 带 [2.0,4.9] 内有枝的树 > 0（进枝区非空谈）", n_in > 0, "%d 棵" % n_in)

    print()
    if FAILS:
        print("== CHECK FAIL %d 项 ==" % len(FAILS))
        for f in FAILS:
            print("  -", f)
        return 1
    print("== 断言全过 ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())
