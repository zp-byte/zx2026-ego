#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云杉林树表体检：结构断言 + 清理带/投放点净空 + 冠底树顶分位。

随场景替换（杨树→云杉，见 tools/import_spruce_world.py）新增。换场地时先跑
本脚本再跑 tools/spruce_passability.py，两者都绿才动仿真。

用法: python3 tools/check_spruce_table.py [table路径]
退出码 0=全绿，1=有 FAIL
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
DEF = os.path.join(WS, "src/zx2026_common/config/forest_spruce_table.yaml")

# 与 scene_topology.yaml / sim_settings.yaml 同源的口径（改动须三处同步）
DROP_CLEAR = 1.5
CLEARING_X = -23.3          # x < 此值不得有树（起降区清理带）
BAND_LO, BAND_HI = 0.70, 2.60   # sim_settings closed_loop.branch_handling.map_z_band
EXPECT_N = 160

fails = []


def bad(msg):
    fails.append(msg)
    print("  FAIL " + msg)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEF
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    trees = doc["trees"]
    meta = doc.get("meta") or {}

    print("表: %s" % path)
    print("meta.mesh_kind = %r  variant_pool = %s" % (meta.get("mesh_kind"),
                                                      meta.get("variant_pool")))

    print("\n--- 1. 结构断言 ---")
    if meta.get("mesh_kind") != "spruce":
        bad("meta.mesh_kind != 'spruce'（world_builder 会走错发射路径）")
    else:
        print("  OK   mesh_kind=spruce")
    if len(trees) != EXPECT_N:
        bad("树数 %d != %d" % (len(trees), EXPECT_N))
    else:
        print("  OK   n_trees=%d" % EXPECT_N)
    need = ("x", "y", "yaw", "variant", "scale", "trunk_r", "trunk_h",
            "crown_r", "crown_z", "crown_h")
    miss = set()
    for t in trees:
        miss |= {k for k in need if t.get(k) is None}
    if miss:
        bad("字段缺失: %s" % sorted(miss))
    else:
        print("  OK   10 个字段齐全")

    # 派生量自洽：trunk_h == 冠底，crown_z ± crown_h/2 == [冠底, 树顶]
    # 容差 2e-3：导入器把 crown_z/crown_h/trunk_h 各自 round 到 3 位小数，
    # 端点重算回来有 ≤1e-3 的舍入差（实测最大正好 0.001），不是几何错。
    TOL = 2e-3
    dmax = max(abs((t["crown_z"] - t["crown_h"] / 2.0) - t["trunk_h"])
               for t in trees)
    bad_deriv = [t for t in trees if t["crown_h"] <= 0 or
                 abs((t["crown_z"] - t["crown_h"] / 2.0) - t["trunk_h"]) > TOL]
    if bad_deriv:
        bad("%d 行的 trunk_h/crown_z/crown_h 不自洽（最大偏差 %.4f m）"
            % (len(bad_deriv), dmax))
    else:
        print("  OK   trunk_h == crown_z - crown_h/2（干顶即冠底），"
              "最大舍入偏差 %.4f m ≤ %.0e" % (dmax, TOL))

    bad_g = [t for t in trees if t["crown_r"] <= t["trunk_r"]]
    if bad_g:
        bad("%d 行 crown_r <= trunk_r（几何退化）" % len(bad_g))
    else:
        print("  OK   crown_r > trunk_r 恒成立")

    # 与 meshes/spruce/spruce_variants.json 的高度档对得上（逐棵 scale 后）
    vj = os.path.join(WS, "src/arena_world_gazebo/meshes/spruce/spruce_variants.json")
    if os.path.isfile(vj):
        import json
        with open(vj, encoding="utf-8") as f:
            geo = {int(k): v for k, v in json.load(f).items()
                   if not k.startswith("_")}
        worst = 0.0
        for t in trees:
            g = geo[t["variant"]]
            worst = max(worst, abs(t["trunk_h"] - g["trunk_z"] * t["scale"]))
        print("  OK   trunk_h vs json 最大偏差 %.2e m（≤3 位小数舍入量级）" % worst)
        if worst > 2e-3:
            bad("trunk_h 与 spruce_variants.json 不一致（偏差 %.4f m）" % worst)

    print("\n--- 2. 清理带（起降区）无树 ---")
    west = [t for t in trees if t["x"] < CLEARING_X]
    if west:
        bad("清理带 (x<%.1f) 内有 %d 棵树" % (CLEARING_X, len(west)))
    else:
        print("  OK   x<%.1f 无树（最近树 x=%.2f）"
              % (CLEARING_X, min(t["x"] for t in trees)))

    print("\n--- 3. 投放平台净空 ---")
    drops = meta.get("drop_candidates") or []
    if not drops:
        bad("meta.drop_candidates 为空，无法核对投放点")
    for (tx, ty) in drops:
        c = min(math.hypot(tx - t["x"], ty - t["y"]) - t["trunk_r"] for t in trees)
        cc = min(math.hypot(tx - t["x"], ty - t["y"]) - t["crown_r"] for t in trees)
        ok = c >= DROP_CLEAR and cc >= 0.0
        print("  %s 投放点 (%.2f,%.2f) 树干净空 %.2f m / 冠净空 %.2f m"
              % ("OK  " if ok else "FAIL", tx, ty, c, cc))
        if c < DROP_CLEAR:
            bad("投放点 (%.1f,%.1f) 树干净空 %.2f < %.1f" % (tx, ty, c, DROP_CLEAR))
        if cc < 0.0:
            bad("投放点 (%.1f,%.1f) 落在某树冠内（冠净空 %.2f）" % (tx, ty, cc))

    print("\n--- 4. 冠底 / 树顶分位（换场景须重读的体制参数）---")
    bot = sorted(t["crown_z"] - t["crown_h"] / 2.0 for t in trees)
    top = sorted(t["crown_z"] + t["crown_h"] / 2.0 for t in trees)
    n = len(trees)
    q = lambda a, p: a[min(n - 1, int(p * n))]
    print("  冠底  p10 %.2f p25 %.2f 中位 %.2f p75 %.2f p90 %.2f max %.2f"
          % (q(bot, .10), q(bot, .25), bot[n // 2], q(bot, .75), q(bot, .90), bot[-1]))
    print("  树顶  p10 %.2f p25 %.2f 中位 %.2f p75 %.2f p90 %.2f max %.2f"
          % (q(top, .10), q(top, .25), top[n // 2], q(top, .75), q(top, .90), top[-1]))

    in_band = sum(1 for t in trees
                  if not (t["crown_z"] + t["crown_h"] / 2.0 < BAND_LO
                          or t["crown_z"] - t["crown_h"] / 2.0 > BAND_HI))
    print("  冠包络与 map_z_band[%.2f,%.2f] 重叠的树 %d/%d (%.0f%%)"
          % (BAND_LO, BAND_HI, in_band, n, 100.0 * in_band / n))
    print("  ^ 杨树时代此值=0（冠全在带外）；云杉冠进带是正确语义（冠已是真障碍），")
    print("    故 sim_settings 的旧硬不变量「带上限 < 冠下缘」已随场景作废改立论。")

    print("\n--- 5. 三角面预算 ---")
    tris = meta.get("n_tris")
    if tris:
        print("  估计 %d 三角面 (%.1fM)。>15M 请实测 RTF，必要时切 "
              "import_spruce_world.py 的 VARIANT_POOL=[4,5,6] 重生成。"
              % (tris, tris / 1e6))

    print()
    if fails:
        print("== 共 %d 项 FAIL ==" % len(fails))
        for m in fails:
            print("   - " + m)
        return 1
    print("== 全绿 ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())
