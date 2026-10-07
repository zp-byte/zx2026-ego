#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""杨树树体校验（2026-10-07 换场重钉）：冠=物理+视觉、真枝=无。

沿革：本文件原名/原职是**真枝模板校验**（枝段端点落在 wood STL 包围盒内等）。
2026-10-07 用户裁决「场景换成 zx2026_ws（arena）场景」⇒ 树体 = 干 c_trunk +
冠 c_crown + wood mesh + leaf mesh，**无真枝**（表缺 branch_file 键）。故本文件
按新档反转：删去全部真枝断言，改为**冠**的物理/判官/感知三口径对质，并保留两条
与原档同源的独立证据链：

  · consumer_checks(1)(2)(3)：冠杠杆 `_crown_z_range` 的**两份实现**（Scene 方法
    与判官同名静态方法）必须逐树一致，且碰撞路径读冠几何处均经该杠杆；
  · world_checks：scene.py 与 world_builder.py 两个**独立**坐标/几何实现
    （Scene 的 crown z 跨度 vs 世界文件 c_crown 的 pose/length）必须算到同一处。

原「真枝」断言在 defop 线仍有意义 —— 见 tools/branch_selftest.py（硬编码
~/zx2026_defop_ws，与本档无关，勿在本仓跑）。
"""
import ast
import importlib
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WS, "src", "zx2026_common", "scripts"))

CFG = os.path.join(WS, "src", "zx2026_common", "config")
TBL = os.path.join(CFG, "forest_poplar_table.yaml")
TREE_NAME = "zx2026_ws (arena) 场景：干 + 冠 + wood/leaf mesh，无真枝"

FAILS = []


def chk(name, cond, detail=""):
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        FAILS.append(name)


def _load_judge():
    mdir = os.path.join(WS, "src", "arena_world_gazebo", "scripts")
    sys.path.insert(0, mdir)
    return importlib.import_module("gazebo_collision_monitor").CollisionMonitor


def main():
    print("== 杨树树体校验（%s）==" % TREE_NAME)
    d = yaml.safe_load(open(TBL, encoding="utf-8"))
    m = d.get("meta") or {}
    trees = d.get("trees", [])

    # ---- 表 meta ------------------------------------------------------
    chk("表 meta crown_collidable=True（冠=物理）",
        m.get("crown_collidable") is True, "crown_collidable=%r" % m.get("crown_collidable"))
    chk("表 meta 无 branch_file（真枝断源）", not m.get("branch_file"),
        "branch_file=%r" % m.get("branch_file"))
    chk("表 n_trees == 实际树数", m.get("n_trees") == len(trees),
        "meta=%r 实际=%d" % (m.get("n_trees"), len(trees)))
    miss_geo = [i for i, r in enumerate(trees)
                if r.get("crown_r") is None or r.get("crown_z") is None
                or r.get("crown_h") is None]
    chk("全树冠几何齐备（crown_r/z/h 非 None）", not miss_geo,
        "缺 %d 棵" % len(miss_geo))

    # ---- 巡航带关系（冠是上方结构 ⇒ 冠底 vs 巡航带）--------------------
    zb = [(float(r["crown_z"]) - float(r["crown_h"]) / 2.0) for r in trees]
    zlo = min(zb)
    print("      全林冠底 z 最低 = %.2f（巡航 cruise_z=2.4 ⇒ 机顶 %.2f，裕度 %+.2f）"
          % (zlo, 2.4 + 0.35, zlo - (2.4 + 0.35)))
    print("      ⚠ 换场后上方结构=冠（无真枝）。map_z_band 上界 2.60 的旧依据"
          "「低于最低真枝 2.88」已失效，须另行裁决（本脚本不作断言）。")

    scene_checks(d)
    consumer_checks()
    world_checks()

    print()
    if FAILS:
        print("CHECK FAIL: %d 项" % len(FAILS))
        for f in FAILS:
            print("  -", f)
        return 1
    print("CHECK PASS")
    return 0


def scene_checks(d):
    """Scene 侧：冠**是**障碍（四层同口径）、且无真枝。
    对抗对照翻杠杆证明几何仍在表、是杠杆在开它（防空集假 PASS）。"""
    print("\n== Scene 实例化对质 ==")
    from zx2026_common.scene import Scene
    s = Scene()
    trees = [o for o in s.obstacles if o.kind == "tree"]
    chk("Scene 树数 = 表树数", len(trees) == len(d["trees"]), "n=%d" % len(trees))

    # 冠 z 跨度必须全在，且等于 (crown_z ± crown_h/2)
    bad = []
    for t in trees:
        zr = s._crown_z_range(t)
        exp = (t.crown_z - t.crown_h / 2.0, t.crown_z + t.crown_h / 2.0)
        if zr is None or abs(zr[0] - exp[0]) > 1e-6 or abs(zr[1] - exp[1]) > 1e-6:
            bad.append((t.id, zr, exp))
    chk("全树 _crown_z_range = (crown_z ± crown_h/2)（冠入碰撞）", not bad,
        "异常 %d 棵 %s" % (len(bad), bad[:3]))
    chk("全树 crown_collidable=True", all(t.crown_collidable for t in trees))

    # 对抗对照：翻回 False ⇒ 跨度全消失（证明是杠杆在开，不是几何本来就无）
    saved = [t.crown_collidable for t in trees]
    for t in trees:
        t.crown_collidable = False
    try:
        n_off = sum(1 for t in trees if s._crown_z_range(t) is not None)
    finally:
        for t, v in zip(trees, saved):
            t.crown_collidable = v
    chk("对抗对照：杠杆翻 False ⇒ 跨度全消失（冠几何仍在表）",
        n_off == 0, "仍非 None=%d/%d" % (n_off, len(trees)))

    # 无真枝
    nb = [t for t in trees if getattr(t, "branches", None)]
    chk("全树无真枝（branches 空）", not nb, "带枝 %d 棵" % len(nb))

    # 冠包络内空点（离干远、半径 0）。选点须**在关掉全林冠后仍不被任何他树挡**
    # ——否则邻树冠会污染对抗对照（换场后林密，实测 1/25 被邻树罩住）。
    saved = [t.crown_collidable for t in trees]
    for t in trees:
        t.crown_collidable = False
    try:
        probes = []
        for t in trees:
            if not t.crown_h or not t.crown_r:
                continue
            r_off = (t.trunk_r + t.crown_r) / 2.0
            if r_off <= t.trunk_r + 0.05:
                continue
            p = (t.cx + r_off, t.cy, t.crown_z)
            if s.collides(p, 0.0):      # 关冠后仍被挡 ⇒ 非干净探针（他树干/冠），弃
                continue
            probes.append((t, p))
            if len(probes) >= 25:
                break
    finally:
        for t, v in zip(trees, saved):
            t.crown_collidable = v
    blocked = sum(1 for t, p in probes if s.collides(p, 0.0))
    chk("冠包络内空点全被挡（冠=障碍）",
        len(probes) > 0 and blocked == len(probes),
        "%d 探针，命中 %d" % (len(probes), blocked))
    saved = [t.crown_collidable for t in trees]
    for t in trees:                     # 全林关冠（与选点同条件）——只关本树会漏邻树冠
        t.crown_collidable = False
    try:
        freed = sum(1 for t, p in probes if not s.collides(p, 0.0))
    finally:
        for t, v in zip(trees, saved):
            t.crown_collidable = v
    chk("对抗对照：杠杆翻 False ⇒ 同点全放行（探针确在冠内）",
        freed == len(probes), "放行 %d/%d" % (freed, len(probes)))


def consumer_checks():
    """冠杠杆消费者一致性：`_crown_z_range` 两份实现必须同步；碰撞路径读冠几何处
    必须经杠杆；三口径（物理/感知/判官）在同一点必须同时命中。"""
    print("\n== 冠杠杆消费者一致性 ==")
    from zx2026_common.scene import Scene
    s = Scene()
    trees = [o for o in s.obstacles if o.kind == "tree"]
    CM = _load_judge()

    # (1) 判官第二实现与 Scene 逐树逐位一致
    mism = [t.id for t in trees if CM._crown_z_range(t) != s._crown_z_range(t)]
    chk("判官 _crown_z_range 与 Scene 逐树一致（两份实现未漂移）", not mism,
        "不一致 %d 棵，前 5=%s" % (len(mism), mism[:5]))

    # (2) 碰撞路径里读到冠几何的函数，必须同时经 `_crown_z_range` 取跨度
    coll = ["src/arena_nav/scripts/nav_node.py",
            "src/arena_world_gazebo/scripts/gazebo_collision_monitor.py",
            "src/arena_mission/scripts/mission_executor_node.py"]
    bad = []
    for rel in coll:
        p = os.path.join(WS, rel)
        if not os.path.exists(p):
            bad.append("%s 不存在" % rel)
            continue
        src = open(p, encoding="utf-8").read()
        lines = src.splitlines()
        for fn in [n for n in ast.walk(ast.parse(src))
                   if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
            body = "\n".join(lines[fn.lineno - 1:getattr(fn, "end_lineno", fn.lineno)])
            if ("crown_r" in body or "crown_z" in body) \
                    and "_crown_z_range" not in body:
                bad.append("%s:%d %s()" % (rel, fn.lineno, fn.name))
    chk("碰撞路径读冠几何处均经杠杆", not bad, "; ".join(bad[:6]))

    # (3) 渲染路径须经同一杠杆
    cr = os.path.join(WS, "src/arena_sensor/scripts/arena_sensor/camera_render.py")
    src_cr = open(cr, encoding="utf-8").read()
    chk("camera_render 冠入口同读 crown_collidable 杠杆",
        'getattr(ob, "crown_collidable"' in src_cr,
        "文件内 lever 引用 %d 处" % src_cr.count("crown_collidable"))

    # (4) 三口径同点对质：冠包络内点（离干够远）上，collides（物理）/ raycast
    #     （感知/lidar）/ 判官 _crown_hit（计分）必须**同时命中**。
    mon = CM.__new__(CM)
    mon.dr = s.drone_radius
    n = hit_c = hit_r = hit_m = 0
    for t in trees:
        if not t.crown_h or not t.crown_r:
            continue
        r_off = (t.trunk_r + t.crown_r) / 2.0
        if r_off <= t.trunk_r + mon.dr + 0.05:
            continue
        p = (t.cx + r_off, t.cy, t.crown_z)
        org = (t.cx + r_off + 3.0, t.cy, t.crown_z)   # 冠外水平点
        dvec = (-1.0, 0.0, 0.0)
        n += 1
        if s.collides(p, mon.dr):
            hit_c += 1
        if s.raycast(org, dvec, max_t=6.0, ground=False) is not None:
            hit_r += 1
        if mon._crown_hit(p[0], p[1], p[2], t) is not None:
            hit_m += 1
    chk("三口径对质样本非空", n > 100, "样本 %d 棵" % n)
    chk("三口径同点对质：collides 全中（物理）", hit_c == n, "%d/%d" % (hit_c, n))
    chk("三口径同点对质：判官 _crown_hit 全中（计分）", hit_m == n, "%d/%d" % (hit_m, n))
    chk("三口径同点对质：raycast 全被挡（感知）", hit_r == n, "%d/%d" % (hit_r, n))


def world_checks():
    """世界文件侧：Scene 的冠 z 跨度与 world_builder 写出的 c_crown（pose z +
    length）两个**独立**实现必须一致；计数须为 冠=棵数 / 枝=0。"""
    wf = os.path.join(WS, "src", "arena_world_gazebo", "worlds", "forest_world.world")
    if not os.path.exists(wf):
        print("\n== 世界文件对质 ==（%s 不存在，跳过）" % wf)
        return
    print("\n== 世界文件对质（两独立几何实现互证）==")
    from zx2026_common.scene import Scene
    s = Scene()
    root = ET.fromstring(open(wf, encoding="utf-8").read())
    mdl = {}
    for mm in root.iter("model"):
        nm = mm.get("name", "")
        if nm.startswith("tree_"):
            mdl[int(nm.split("_")[1])] = mm
    maxd = 0.0
    ncmp = 0
    ncrown = nbranch = 0
    for t in s.obstacles:
        if t.kind != "tree" or t.id not in mdl:
            continue
        mm = mdl[t.id]
        cc = [c for c in mm.iter("collision") if c.get("name") == "c_crown"]
        ncrown += len(cc)
        nbranch += sum(1 for c in mm.iter("collision")
                       if c.get("name", "").startswith("c_branch_"))
        if not cc:
            maxd = 1e9
            continue
        c = cc[0]
        pz = float(c.find("pose").text.split()[2])
        ln = float(c.find("geometry/cylinder/length").text)
        zr = s._crown_z_range(t)
        maxd = max(maxd, abs(pz - t.crown_z), abs(ln - (zr[1] - zr[0])))
        ncmp += 1
    chk("Scene 冠跨度与 world c_crown 一致（z/length 差 < 5mm）", maxd < 5e-3,
        "比对 %d 棵，最大差 %.3e" % (ncmp, maxd))
    chk("世界 c_crown 数 = 树数", ncrown == len(mdl), "%d vs %d" % (ncrown, len(mdl)))
    chk("世界 c_branch 数 = 0（本档无真枝）", nbranch == 0, "c_branch=%d" % nbranch)


if __name__ == "__main__":
    sys.exit(main())
