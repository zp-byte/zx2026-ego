#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""杨树枝模板校验：结构 + 与 wood 网格的几何对质 + 巡航带关系。

对质思路：`extract_poplar_branches.py` 是**重放**生成器，自证无意义。这里的
独立证据是**盘上的 STL 本身**——枝段端点必须落在同变体 `tree_N_wood.stl` 的
包围盒内（枝是干网格的一部分，不可能跑到网格外）。这条能抓住「重放错位」
「变体对错文件」「模板被手改坏」三类事故。

另外报告枝的 z 分布与巡航带的关系——枝若整体落在巡航带内，本改动就不是
「几乎行为中性」而是会真改行为，须重新评估（见计划第三节）。
"""
import json
import os
import struct
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(WS, "src", "zx2026_common", "scripts"))

BR = os.path.join(WS, "src", "zx2026_common", "config",
                  "forest_poplar_branches.json")
TBL = os.path.join(WS, "src", "zx2026_common", "config",
                   "forest_poplar_table.yaml")
MESH = os.path.join(WS, "src", "arena_world_gazebo", "meshes", "poplar")

FAILS = []


def chk(name, cond, detail=""):
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           ("  " + detail) if detail else ""))
    if not cond:
        FAILS.append(name)


def stl_bbox(path):
    """二进制 STL 包围盒 + 三角数。"""
    with open(path, "rb") as fp:
        fp.read(80)
        n = struct.unpack("<I", fp.read(4))[0]
        data = fp.read(50 * n)
    assert len(data) == 50 * n, (path, "STL 截断")
    lo = [1e18] * 3
    hi = [-1e18] * 3
    for i in range(n):
        off = i * 50 + 12          # 跳过法向(12B)
        for v in range(3):
            xyz = struct.unpack_from("<3f", data, off + v * 12)
            for a in range(3):
                lo[a] = min(lo[a], xyz[a])
                hi[a] = max(hi[a], xyz[a])
    return lo, hi, n


def main():
    print("== 杨树枝模板校验 ==")
    doc = json.load(open(BR, encoding="utf-8"))
    tbl = open(TBL, encoding="utf-8").read()
    d = yaml.safe_load(tbl)
    m = d.get("meta") or {}

    chk("模板含 tree_0/1/2", sorted(doc.keys()) == ["tree_0", "tree_1", "tree_2"],
        str(sorted(doc.keys())))

    # 表 meta 是否已挂 branch_file（P4 后应为 True）
    chk("表 meta 有 branch_file", bool(m.get("branch_file")),
        "branch_file=%r" % m.get("branch_file"))

    # 表里出现过的 variant 都必须有模板（空表须显式判负，否则 all() 空过）
    variants = set(int(r["variant"]) for r in d["trees"])
    chk("表非空且 variant 全有模板",
        bool(variants) and all(("tree_%d" % v) in doc for v in variants),
        "表 %d 棵 variants=%s" % (len(d["trees"]), sorted(variants)))

    for name in sorted(doc):
        vi = int(name.split("_")[1])
        b = doc[name]["branches"]
        chk("%s 主枝数 ∈[7,10]" % name, 7 <= len(b) <= 10, "n=%d" % len(b))

        lo, hi, ntri = stl_bbox(os.path.join(MESH, "tree_%d_wood.stl" % vi))
        # 端点必须落在同变体 wood 网格包围盒内（容 1cm 抗浮点/取整）
        tol = 0.01
        bad = []
        for k, seg in enumerate(b):
            for p in ((seg[0], seg[1], seg[2]), (seg[3], seg[4], seg[5])):
                for a in range(3):
                    if p[a] < lo[a] - tol or p[a] > hi[a] + tol:
                        bad.append((k, a, round(p[a], 3), round(lo[a], 3),
                                    round(hi[a], 3)))
        chk("%s 端点全在 wood 包围盒内" % name, not bad,
            "wood tris=%d bbox x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]%s"
            % (ntri, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2],
               ("  越界:%s" % bad[:4]) if bad else ""))

        rmin = min(s[6] for s in b)
        rmax = max(s[6] for s in b)
        chk("%s 枝半径 ∈[0.02,0.06]" % name, 0.02 <= rmin and rmax <= 0.06,
            "r∈[%.3f, %.3f]" % (rmin, rmax))

        up = all(s[5] >= s[2] - 1e-9 for s in b)
        chk("%s 主枝均上举" % name, up)

        z0 = [s[2] for s in b]
        print("      枝基 z∈[%.2f, %.2f]（局部系，未缩放）" % (min(z0), max(z0)))

    # ---- 巡航带关系（决定本改动的行为影响）----
    tgt = []
    for r in d["trees"]:
        s = float(r["scale"])
        for seg in doc["tree_%d" % int(r["variant"])]["branches"]:
            tgt.append((min(seg[2], seg[5]) * s, max(seg[2], seg[5]) * s))
    zmin = min(a for a, _ in tgt)
    zmax = max(b for _, b in tgt)
    print("      全林枝段 z（世界系，已乘 scale）∈[%.2f, %.2f]，共 %d 段"
          % (zmin, zmax, len(tgt)))
    print("      巡航 cruise_z=2.4 / map_z_band=[0.70,2.60] ⇒ 枝%s巡航带"
          "（机顶 2.75 vs 枝底 %.2f，物理裕度 %+.2f m）"
          % ("不侵入" if zmin > 2.60 else "**侵入**，须重估行为影响",
             zmin, zmin - 2.75))
    if zmin <= 2.60:
        chk("枝不侵入巡航带", False, "最低枝 z=%.2f ≤ 2.60" % zmin)
    else:
        print("  [OK] 枝不侵入巡航带（最低 %.2f > 2.60）" % zmin)

    scene_checks()
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


def scene_checks():
    """Scene 侧：冠确实**不再是障碍**（四层同口径）、枝确实进了世界系且可被命中。

    这是本项目「物理在挡、判官不报」静默失配的既定检法（计划 §5 步 5）：
    在**同一枝段中点**上，scene.collides 必须命中。枝/冠的碰撞证据各自在
    摘除对方后取得（冠会罩住整段枝，不摘冠则枝的命中证据被冠遮蔽）。
    """
    print("\n== Scene 实例化对质 ==")
    from zx2026_common.scene import Scene
    s = Scene()
    trees = [o for o in s.obstacles if o.kind == "tree"]
    chk("Scene 树数 = 表 n_trees", len(trees) == len(
        yaml.safe_load(open(TBL, encoding="utf-8"))["trees"]), "n=%d" % len(trees))

    # 冠四层（2026-10-02 晚裁决「冠物理、视觉都删，只保留树枝」）：单一口径
    # `_crown_z_range` 必须对全树返回 None。但「全 None」也可能因冠几何缺失而
    # 假过（几何没了，杠杆自然无事可关）⇒ 附加**对抗对照**：把杠杆在内存里翻回
    # True，跨度必须全部回来——证明几何仍在表里，是杠杆在关它。
    n_off = sum(1 for t in trees if s._crown_z_range(t) is not None)
    chk("全树 _crown_z_range 为 None（冠不入碰撞）", n_off == 0,
        "非 None=%d/%d" % (n_off, len(trees)))
    chk("全树 crown_collidable=False", not any(t.crown_collidable for t in trees))
    saved_cc = [t.crown_collidable for t in trees]
    for t in trees:
        t.crown_collidable = True
    try:
        n_on = sum(1 for t in trees if s._crown_z_range(t) is not None)
    finally:
        for t, v in zip(trees, saved_cc):
            t.crown_collidable = v
    chk("对抗对照：杠杆翻回 True 跨度全回来（冠几何仍在表）",
        n_on == len(trees), "非 None=%d/%d" % (n_on, len(trees)))

    # 枝进入世界系
    nb = [t for t in trees if t.branches]
    chk("有枝树 = 全树", len(nb) == len(trees), "%d/%d" % (len(nb), len(trees)))
    zmin = min(min(seg[2] for seg in t.branches) for t in nb)
    dr = s.drone_radius
    print("      世界系枝最低 z=%.2f（巡航 cruise_z=2.4 ⇒ 机顶 %.2f，裕度 %+.2f）"
          "——冠删除后此为最高处障碍的最近下缘"
          % (zmin, 2.4 + dr, zmin - 2.4 - dr))

    # 三口径之物理口径：枝段中点必须被判碰撞。默认档冠已不参与（见上），保留
    # 摘冠是保险：若哪天杠杆翻回 True，本断言测的仍是枝本体，而非被冠包络罩住
    # 的假命中。
    saved_cc = [t.crown_collidable for t in trees]
    for t in trees:
        t.crown_collidable = False
    try:
        miss = 0
        tested = 0
        for t in nb[:30]:
            for seg in t.branches:
                mid = ((seg[0] + seg[3]) / 2.0, (seg[1] + seg[4]) / 2.0,
                       (seg[2] + seg[5]) / 2.0)
                tested += 1
                if not s.collides(mid, dr):
                    miss += 1
    finally:
        for t, v in zip(trees, saved_cc):
            t.crown_collidable = v
    chk("枝段中点 scene.collides 命中（摘冠取证）", miss == 0,
        "受检 %d 段，漏检 %d" % (tested, miss))

    # 冠删除的**反向证据 + 对抗对照**（2026-10-02 晚逐条翻转旧断言）：探针取在
    # 冠包络内、离干与所有枝都够远、且以**半径 0** 测 ⇒ 过去唯一可能挡它的东西
    # 就是冠（半径 0 时干/枝的判据是纯心距，均远于探针）。默认档必须**全部
    # 放行**；把杠杆翻回 True 后同点必须**全部重新被挡**——后者证明探针确实落在
    # 冠包络里，「放行」不是探针选偏导致的空过（本项目踩过的「空集假 PASS」同型）。
    import math
    from zx2026_common import geometry as geo
    probes = []
    for t in nb:
        if not t.crown_h:
            continue
        for i in range(1, 40):
            rr = 0.30 + i * 0.04
            if rr >= t.crown_r:
                break
            if rr <= 0.16 * (t.mesh_scale or 1.0) + 0.02:
                continue
            for k in range(16):
                a = 2 * math.pi * k / 16
                p = (t.cx + rr * math.cos(a), t.cy + rr * math.sin(a), t.crown_z)
                if any(geo.point_segment_dist(p, (x, y, z), (X, Y, Z)) <= r + 0.35
                       for (x, y, z, X, Y, Z, r) in t.branches):
                    continue
                probes.append(p)
                break
        if len(probes) >= 25:
            break
    blocked = sum(1 for p in probes if s.collides(p, 0.0))
    chk("冠包络内空点全部放行（冠已非障碍）",
        len(probes) > 0 and blocked == 0,
        "%d 个探针，命中 %d" % (len(probes), blocked))
    saved_cc = [t.crown_collidable for t in trees]
    for t in trees:
        t.crown_collidable = True
    try:
        blocked_on = sum(1 for p in probes if s.collides(p, 0.0))
    finally:
        for t, v in zip(trees, saved_cc):
            t.crown_collidable = v
    chk("对抗对照：杠杆翻回 True 同点全被挡（探针确在冠内）",
        blocked_on == len(probes), "命中 %d/%d" % (blocked_on, len(probes)))


def consumer_checks():
    """冠杠杆的**消费者一致性**：`_crown_z_range` 是「冠入不入碰撞」的单一口径，
    但它有**两份实现**（Scene 的方法 + 判官的同名静态方法，判官只拿 Scene 的
    obstacles，不 import 方法），外加若干直接读 `crown_r/crown_z` 的函数。

    2026-10-01 实测揪出：判官那份只看 `crown_r is None`，不认表级 `crown_collidable`
    ⇒ 杨树档删掉 `c_crown` 后，**物理不挡、判官照报**的静默失配。本函数把这一类
    永久钉死：改杠杆时两份实现不同步、或新增调用点绕过杠杆，都在这里红。
    """
    print("\n== 冠杠杆消费者一致性 ==")
    import ast
    from zx2026_common.scene import Scene
    s = Scene()
    trees = [o for o in s.obstacles if o.kind == "tree"]

    # (1) 判官的同名第二实现必须与 Scene 逐树逐位一致
    mdir = os.path.join(WS, "src", "arena_world_gazebo", "scripts")
    sys.path.insert(0, mdir)
    import importlib
    CM = importlib.import_module("gazebo_collision_monitor").CollisionMonitor
    mism = [t.id for t in trees if CM._crown_z_range(t) != s._crown_z_range(t)]
    chk("判官 _crown_z_range 与 Scene 逐树一致（两份实现未漂移）", not mism,
        "不一致 %d 棵，前 5=%s" % (len(mism), mism[:5]))

    # (2) 碰撞路径里读到冠几何的函数，必须同时经 `_crown_z_range` 取跨度
    #     ——否则它读的是「冠几何存在与否」，而非「冠入不入碰撞」
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

    # (3) 渲染路径**不在此断言内**，但须经同一个杠杆：冠视觉/物理已删
    #     （2026-10-02 晚），任何仍按 `render_crowns` 直画冠的入口都是漏改。
    cr = os.path.join(WS, "src/arena_sensor/scripts/arena_sensor/camera_render.py")
    src_cr = open(cr, encoding="utf-8").read()
    # 断言的是**门控表达式本身**，不是「文件里出现过这个词」（注释也算——
    # 那会是一条能被注释满足的假断言）。
    chk("camera_render 冠入口同读 crown_collidable 杠杆",
        'getattr(ob, "crown_collidable"' in src_cr,
        "文件内 lever 引用 %d 处" % src_cr.count("crown_collidable"))

    # (4) 三口径同点对质（计划 §5 步 5）：同一枝段中点上，collides（物理）/
    #     raycast（感知/lidar）/ 判官 `_branch_hit`（计分）必须**同时命中**。
    #     这是本项目「物理在挡、判官不报」静默失配的既定检法。
    import math
    nb = [t for t in trees if t.branches]
    dr = s.drone_radius
    # 判官 _branch_hit 只要 self.dr ⇒ 绕过 __init__（那会 init_node 连 master）
    mon = CM.__new__(CM)
    mon.dr = dr
    _rng = __import__("random").Random(20261001)
    hit_c, hit_r, hit_m, n = 0, 0, 0, 0
    exact = 0
    for t in nb:
        for seg in t.branches:
            # 随机抽面：从枝段中点沿水平方向退 3m 取射线原点，避开固定朝向的偶然性
            a = _rng.uniform(0, 2 * math.pi)
            mid = ((seg[0] + seg[3]) / 2.0, (seg[1] + seg[4]) / 2.0,
                   (seg[2] + seg[5]) / 2.0)
            org = (mid[0] + 3.0 * math.cos(a), mid[1] + 3.0 * math.sin(a), mid[2])
            d = (math.cos(a + math.pi), math.sin(a + math.pi), 0.0)
            n += 1
            if s.collides(mid, dr):
                hit_c += 1
            tt = s.raycast(org, d, max_t=6.0, ground=False)
            if tt is not None:
                hit_r += 1
                if abs(tt - 3.0) <= seg[6] + 0.05:
                    exact += 1
            if mon._branch_hit(mid[0], mid[1], mid[2], t) is not None:
                hit_m += 1
    # 防空集假过：n=0 时三条全中都是 0==0（本项目踩过的「空集假 PASS」）
    chk("三口径对质样本非空", n > 100, "样本 %d 段" % n)
    chk("三口径同点对质：collides 全中", hit_c == n, "%d/%d" % (hit_c, n))
    chk("三口径同点对质：判官 _branch_hit 全中", hit_m == n, "%d/%d" % (hit_m, n))
    chk("三口径同点对质：raycast 全部被挡", hit_r == n, "%d/%d" % (hit_r, n))
    # 上面的断言只看「被挡」，不看「挡在枝面」。冠删除（2026-10-02 晚）后多数
    # 射线应直达枝面（≈3.0−r）；残余未命中=他树干/枝在半路挡住，属正常遮挡。
    print("      raycast 命中距离≈枝面（非他物遮挡）%d/%d（余下为他树遮挡）"
          % (exact, n))


def world_checks():
    """世界文件侧：scene.py 与 world_builder.py 两个**独立**的坐标变换实现
    必须算到同一个点（模型 yaw 少反旋一次 ⇒ 双重旋转 ⇒ 差 ~1.8m，实测揪出过）。
    """
    wf = os.path.join(WS, "src", "arena_world_gazebo", "worlds",
                      "forest_world.world")
    if not os.path.exists(wf):
        print("\n== 世界文件对质 ==（%s 不存在，跳过）" % wf)
        return
    print("\n== 世界文件对质（两独立变换实现互证）==")
    import math
    import xml.etree.ElementTree as ET
    from zx2026_common.scene import Scene
    s = Scene()
    root = ET.fromstring(open(wf, encoding="utf-8").read())
    mdl = {}
    for m in root.iter("model"):
        nm = m.get("name", "")
        if nm.startswith("tree_"):
            mdl[int(nm.split("_")[1])] = m
    maxd = 0.0
    ncmp = 0
    for t in s.obstacles:
        if t.kind != "tree" or not t.branches or t.id not in mdl:
            continue
        m = mdl[t.id]
        p = [float(v) for v in m.find("pose").text.split()]
        ct, st = math.cos(p[5]), math.sin(p[5])
        cols = {int(c.get("name").split("_")[2]): c for c in m.iter("collision")
                if c.get("name", "").startswith("c_branch_")}
        for k, seg in enumerate(t.branches):
            if k not in cols:
                maxd = 1e9
                continue
            q = [float(v) for v in cols[k].find("pose").text.split()]
            wx = p[0] + ct * q[0] - st * q[1]
            wy = p[1] + st * q[0] + ct * q[1]
            wz = p[2] + q[2]
            maxd = max(maxd, abs(wx - (seg[0] + seg[3]) / 2),
                       abs(wy - (seg[1] + seg[4]) / 2),
                       abs(wz - (seg[2] + seg[5]) / 2))
            ncmp += 1
    # 容差 5mm：世界文件坐标以 %.3f 写出，本身就有 ≤0.5mm 量化
    chk("Scene 与 world_builder 变换一致（同点差 < 5mm）", maxd < 5e-3,
        "比对 %d 段，最大差 %.3e m" % (ncmp, maxd))
    chk("世界文件 c_branch 数与 Scene 枝段数相等",
        sum(1 for m in mdl.values() for c in m.iter("collision")
            if c.get("name", "").startswith("c_branch_")) == ncmp)


if __name__ == "__main__":
    sys.exit(main())
