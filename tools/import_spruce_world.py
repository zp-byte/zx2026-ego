#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DeFoP forest_spruce_only.world → forest_spruce_table.yaml 一次性转换器。

源：DeFoP 原生仿真的云杉林世界（`forest` 模型，每条 <link> = 一棵 picea_abies），
结构与 tools/import_poplar_sdf.py 同构 —— 输出 Scene table 生成器消费的显式树表。
差别只在几何来源：杨树表从源 SDF 的 c_trunk/c_crown 圆柱读，云杉源里碰撞体是
整树 mesh，故几何改从 meshes/spruce/spruce_variants.json（实测值）取，再乘逐棵 scale。

用法: python3 tools/import_spruce_world.py [world路径] [输出路径]
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)

SRC = (sys.argv[1] if len(sys.argv) > 1 else
       "/home/ubuntu/defop_ws/src/simulate/lmf_gazebo_models/rmf_sim/worlds/"
       "forest_spruce_only.world")
OUT = (sys.argv[2] if len(sys.argv) > 2 else
       os.path.join(WS, "src/zx2026_common/config/forest_spruce_table.yaml"))
VARIANTS = os.path.join(WS, "src/arena_world_gazebo/meshes/spruce/spruce_variants.json")

DROP_TARGETS = [(27.0, -9.0), (27.0, 0.0), (27.0, 9.0)]   # = scene_topology.drop_points
DROP_CLEAR = 1.5          # 平台位要求的最小树干表面净空 (m)，同 scene_topology.drop_clearance
CLEARING_X = -23.3        # 起降区清理带：x < 此值不得有树
BAND_LO, BAND_HI = 0.70, 2.60   # = sim_settings.branches.map_z_band

# 变体池：全 6 变体共 24.7M 三角面（picea_1 单棵 609K × 28 实例）。若 RTF 不够，
# 改成 [4, 5, 6]（各 11.9K / 11.6K / 4.5K）→ 全林约 1.5M，与杨树同量级。
# 收窄池时按「高度最接近」重映射，保持森林的高度分布不变。
VARIANT_POOL = [1, 2, 3, 4, 5, 6]

MESH_KIND = "spruce"


def load_variants():
    with open(VARIANTS, encoding="utf-8") as f:
        raw = json.load(f)
    return {int(k): v for k, v in raw.items() if not k.startswith("_")}


def remap_variant(v, geo, pool):
    """把变体 v 映射到 pool 内高度最接近的一个（保持树的实际高度不变）。"""
    if v in pool:
        return v
    hv = geo[v]["height"]
    return min(pool, key=lambda p: abs(geo[p]["height"] - hv))


def main():
    geo = load_variants()
    pool = sorted(VARIANT_POOL)
    assert pool, "VARIANT_POOL 不能为空"

    root = ET.parse(SRC).getroot()
    world = root.find("world")
    forest = None
    for m in world.findall("model"):
        if m.get("name") == "forest":
            forest = m
            break
    assert forest is not None, "源 world 里找不到名为 forest 的模型"

    trees, skipped = [], 0
    for L in forest.findall("link"):
        pose_el = L.find("pose")
        pose = [float(v) for v in (pose_el.text or "").split()]
        if len(pose) < 6:
            skipped += 1
            continue
        vis = L.find("visual")
        mesh = vis.find("geometry/mesh") if vis is not None else None
        if mesh is None:
            skipped += 1
            continue
        uri = mesh.findtext("uri") or ""
        if "picea_abies_" not in uri:
            skipped += 1
            continue
        src_v = int(uri.rsplit("picea_abies_", 1)[1].split(".")[0])
        scale_txt = mesh.findtext("scale") or "1 1 1"
        s = float(scale_txt.split()[0])

        v = remap_variant(src_v, geo, pool)
        g = geo[v]
        top = g["height"] * s           # 树顶
        bot = g["trunk_z"] * s          # 冠底 = 光干顶
        crown_bot, crown_top = min(bot, top), max(bot, top)
        trees.append(dict(
            x=round(pose[0], 3), y=round(pose[1], 3), yaw=round(pose[5], 3),
            variant=v, scale=round(s, 4),
            trunk_r=round(g["trunk_r"] * s, 4),
            trunk_h=round(crown_bot, 3),                       # 干圆柱：地面→冠底
            crown_r=round(g["crown_r"] * s, 4),
            crown_z=round((crown_bot + crown_top) / 2.0, 3),   # 冠包络圆心
            crown_h=round(crown_top - crown_bot, 3),
        ))
    if skipped:
        print("  跳过 %d 条非树 link" % skipped)

    trees.sort(key=lambda t: (round(t["x"], 2), round(t["y"], 2)))

    # ---- 统计与健康检查 -----------------------------------------------------
    n = len(trees)
    xs = [t["x"] for t in trees]
    ys = [t["y"] for t in trees]
    bot = sorted(t["crown_z"] - t["crown_h"] / 2.0 for t in trees)
    top = sorted(t["crown_z"] + t["crown_h"] / 2.0 for t in trees)
    print("trees=%d  x[%.2f,%.2f] y[%.2f,%.2f]" % (n, min(xs), max(xs), min(ys), max(ys)))
    print("trunk_r [%.3f,%.3f]  crown_r [%.3f,%.3f]"
          % (min(t["trunk_r"] for t in trees), max(t["trunk_r"] for t in trees),
             min(t["crown_r"] for t in trees), max(t["crown_r"] for t in trees)))
    print("冠底  min %.2f 中位 %.2f max %.2f   |  树顶 min %.2f 中位 %.2f max %.2f"
          % (bot[0], bot[n // 2], bot[-1], top[0], top[n // 2], top[-1]))

    west = [t for t in trees if t["x"] < CLEARING_X]
    assert not west, "清理带 (x<%.1f) 内发现 %d 棵树" % (CLEARING_X, len(west))

    # 【换场景必查】sim_settings.yaml 明令：map_z_band 上限 vs 冠下缘
    in_band = sum(1 for t in trees
                  if not (t["crown_z"] + t["crown_h"] / 2.0 < BAND_LO
                          or t["crown_z"] - t["crown_h"] / 2.0 > BAND_HI))
    print("冠包络与 map_z_band[%.2f,%.2f] 有重叠的树: %d / %d  (%.0f%%)"
          % (BAND_LO, BAND_HI, in_band, n, 100.0 * in_band / n))
    print("  ^ 杨树时代此值为 0（冠全在带外）。云杉冠底中位 %.2f，冠进带内是"
          "新场景的正常形态（冠已是真障碍），不再是异常。" % bot[n // 2])

    # ---- 投放平台净空（对照 scene_topology 的固定投送点）----------------------
    def min_clear(px, py):
        return min(math.hypot(px - t["x"], py - t["y"]) - t["trunk_r"] for t in trees)

    drops = []
    for (tx, ty) in DROP_TARGETS:
        c = min_clear(tx, ty)
        drops.append([tx, ty])
        print("drop (%.2f,%.2f) 树干净空 = %.2f %s"
              % (tx, ty, c, "OK" if c >= DROP_CLEAR else "!! 不足 %.1f" % DROP_CLEAR))
        assert c >= DROP_CLEAR, "投送点 (%.1f,%.1f) 净空不足" % (tx, ty)
    for i in range(len(drops)):
        for j in range(i + 1, len(drops)):
            d = math.hypot(drops[i][0] - drops[j][0], drops[i][1] - drops[j][1])
            print("drop pair dist=%.2f" % d)

    tris = sum(geo[t["variant"]]["n_tris"] for t in trees)
    print("三角面预算: %d 棵共 %.2fM  (池 %s)" % (n, tris / 1e6, pool))

    # ---- 输出树表 ------------------------------------------------------------
    doc = dict(
        meta=dict(source=SRC, mesh_kind=MESH_KIND, variant_pool=pool,
                  n_trees=n, n_tris=tris,
                  area=[40.0, 40.0], drop_candidates=drops,
                  note="DeFoP picea_abies 云杉林；几何见 meshes/spruce/spruce_variants.json"),
        trees=trees)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False)
    print("wrote %s (%d trees)" % (OUT, n))


if __name__ == "__main__":
    main()
