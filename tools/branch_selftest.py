#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/branch_selftest.py — 仿真树枝层离线自测（不依赖 ROS 运行栈）。

用法（WSL）：
  cd ~/zx2026_defop_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/branch_selftest.py

结果写 /tmp/branch_selftest.txt（UTF-8，规避 GBK 控制台）。

覆盖（2026-10-02 真枝档重基；旧程序枝期望值随杨树林真枝导入作废）：
  0. 真枝装载：表 meta.branch_file 装载器返回模板、Scene 全树带枝；
     回滚杠杆：装载器对 None/'' 返回 None（摘键=无枝基线）
  1. 确定性：两次实例化，树枝元组逐位相同（真枝来自 JSON 重放，天然确定）
  2. 生成合理：总枝数>0、每树 ∈[7,10]（模板 tree_0/1=10、tree_2=8）
  2b. 枝参数范围：世界系 r∈[0.02,0.07]、段长∈[0.9,3.8]（模板 r∈[0.02,0.06]
      × mesh_scale；2026-10-02 实测 r[0.028,0.055] 长[1.03,3.62]）
  3. 稀疏回波：72az×5el lidar 射线（同 lidar_node 模式），枝命中数>0
     且占比低（细枝特征，同真机 2cm 枝 @5m ~4% 量级复现）
  4. 硬碰撞：球(0.35) 置枝中点 collides=True；沿枝轴端点外 1.2m False
  5. A* 占用：collides_xy 枝 z 带内 True、带外（**全树顶之上**）False、
     树干外侧点仅因枝判占（隔离树干混淆）。
     带外探针高度由场景实测导出（max(全林顶点)+0.5..+1.5），**不再硬编码
     3.5**——2026-10-01 冠升级成四层一等公民后 collides_xy 认冠，硬编码
     3.5 落进冠包络里（云杉 37% 树顶 > 3.5；杨树冠 z 3.10-7.83 同样覆盖
     3.5），断言即"拿冠去测枝层"，与枝层 z 门控无关（场景保持不变的 A/B
     已证：两种林子里都假，病因是冠语义变更本身）
  6. 对照一致性：摘枝场景逐射线对照，有枝命中距离 ≤ 无枝
     （枝只会更近不会更远）

**摘冠取证（2026-10-02）**：冠物理存在后（用户裁决）其包络（z 2.81-12.9、
半径 1.65-2.97）罩住枝 z 带 [2.88,12.73]，4/5/3 的命中证据全部可由冠满足
⇒ 被测对象（枝层）必须摘冠测量，否则测的是冠（摘冠取证同
tools/check_poplar_branches.py，同一杠杆非另立机制）。
"""
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "zx2026_common", "scripts"))

from zx2026_common import geometry as geo
from zx2026_common.scene import Scene

WS = os.path.expanduser("~/zx2026_defop_ws")
TBL = WS + "/src/zx2026_common/config/forest_poplar_table.yaml"
OUT = "/tmp/branch_selftest.txt"

LIDAR_AZ = 72
LIDAR_EL = [-0.5, -0.25, 0.0, 0.25, 0.5]
RANGE = 5.0

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))


def crowns_blind(*scenes):
    """摘冠（内存置 crown_collidable=False）——被测对象是枝层，见头注。"""
    for s in scenes:
        for ob in s.obstacles:
            if ob.kind == "tree":
                ob.crown_collidable = False


def lidar_hits(scene, pos):
    """复刻 lidar_node 射线模式（72az × 5el，yaw=0），返回 [(t, az, el),...]。"""
    hits = []
    for az_i in range(LIDAR_AZ):
        az = az_i * (2.0 * math.pi / LIDAR_AZ)
        for el in LIDAR_EL:
            d = geo.normalize((math.cos(el) * math.cos(az),
                               math.cos(el) * math.sin(az), math.sin(el)))
            t = scene.raycast(pos, d, max_t=RANGE)
            if t is not None:
                hits.append((t, az, el))
    return hits


def main():
    # ---- 0 真枝装载与回滚杠杆 ----
    s0 = Scene()
    n_trees = sum(1 for ob in s0.obstacles if ob.kind == "tree")
    n_wb = sum(1 for ob in s0.obstacles if ob.kind == "tree" and ob.branches)
    check("0 真枝装载（表 meta.branch_file）", n_wb == n_trees and n_trees >= 20,
          "trees=%d tree_with_branches=%d" % (n_trees, n_wb))
    tmeta = yaml.safe_load(open(TBL, encoding="utf-8")).get("meta", {})
    ok_lever = (s0._load_branch_template(None) is None
                and s0._load_branch_template("") is None
                and isinstance(s0._load_branch_template(tmeta.get("branch_file")),
                               dict))
    check("0b 回滚杠杆（None/''→无枝，真键→模板）", ok_lever,
          "branch_file=%s" % tmeta.get("branch_file"))

    # ---- 开枝：确定性 / 生成合理 / 碰撞 / 占用 ----
    sA = Scene()
    sB = Scene()
    brA = [ob.branches for ob in sA.obstacles if ob.kind == "tree"]
    brB = [ob.branches for ob in sB.obstacles if ob.kind == "tree"]
    check("1 确定性", brA == brB,
          "trees=%d" % len(brA))
    tot = sum(len(b) for b in brA if b)
    per = [len(b) for b in brA if b]
    check("2 枝数", tot > 0 and all(7 <= n <= 10 for n in per),
          "total=%d per_tree=%s" % (tot, sorted(set(per))))
    ok_r = True
    n_br = 0
    for b in brA:
        if not b:
            continue
        for (sx, sy, sz, ex, ey, ez, r) in b:
            n_br += 1
            if not (0.02 - 1e-9 <= r <= 0.07 + 1e-9):
                ok_r = False
            ln = geo.dist((sx, sy, sz), (ex, ey, ez))
            if not (0.9 - 1e-6 <= ln <= 3.8 + 1e-6):
                ok_r = False
    check("2b 枝参数范围", ok_r and n_br > 0, "branches=%d" % n_br)

    # 全林顶点（冠顶/干顶/枝梢取最大）——测试 5 带外探针基线。须在摘冠前
    # 算：摘冠后 _crown_z_range 返回 None，会退到干顶而偏低。
    tops = []
    for ob in sA.obstacles:
        if ob.kind != "tree":
            continue
        zr = sA._crown_z_range(ob)
        t0 = zr[1] if zr else sA.venue["ground_z"] + ob.trunk_h
        if ob.branches:
            t0 = max(t0, max(max(seg[2], seg[5]) for seg in ob.branches))
        tops.append(t0)
    z_top = max(tops) if tops else 5.0

    # 以下证据只可能来自干/枝（摘冠，见头注）
    crowns_blind(sA, sB)

    # 4 硬碰撞（选长枝 ln>=0.8，中点=球心）
    coll_ok = ctrl_ok = ctrl_skip = 0
    for ob in sA.obstacles:
        if ob.kind != "tree" or not ob.branches:
            continue
        for (sx, sy, sz, ex, ey, ez, br) in ob.branches:
            if geo.dist((sx, sy, sz), (ex, ey, ez)) < 0.8:
                continue
            mid = ((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0)
            if sA.collides(mid, 0.35):
                coll_ok += 1
            else:
                check("4 中点碰撞", False, "tree%d br@%.2f,%.2f,%.2f" % (
                    ob.id, mid[0], mid[1], mid[2]))
            u = geo.normalize((ex - sx, ey - sy, ez - sz))
            probe = (ex + u[0] * 1.2, ey + u[1] * 1.2, ez + u[2] * 1.2)
            if sA.collides(probe, 0.35):
                ctrl_skip += 1  # 可能贴近邻枝/树干，跳过对照组
            else:
                ctrl_ok += 1
            if ctrl_ok >= 3:
                break
        if ctrl_ok >= 3:
            break
    check("4 硬碰撞", coll_ok > 0 and ctrl_ok > 0,
          "mid_hit=%d ctrl_clear=%d ctrl_skip=%d" % (coll_ok, ctrl_ok, ctrl_skip))

    # 5 A* 占用（隔离树干：探针取枝段起点沿段向 0.45m 处）
    # 带外探针 z 由场景实测导出（z_top 于摘冠前算好，含冠顶/干顶/枝梢三者
    # 最大）：那才是"整棵树都在脚下"的真空带（硬编码 3.5 会落进冠包络，
    # 见头注 5）。
    # 探针位置（2026-10-02 重基）：真枝是多段折线，段起点未必贴树干面，
    # 旧"干轴 trunk_r+0.45 沿段方位角"投影会落到段外（程序枝=单段贴干
    # 假设已作废）；与干轴距 ≤trunk_r+0.2 的段跳过（证据会与干混淆）。
    out_z0, out_z1 = z_top + 0.5, z_top + 1.5
    occ_ok = occ_out = 0
    for ob in sA.obstacles:
        if ob.kind != "tree" or not ob.branches:
            continue
        for (sx, sy, sz, ex, ey, ez, br) in ob.branches:
            ln = geo.dist((sx, sy, sz), (ex, ey, ez))
            if ln < 0.6:
                continue
            az = math.atan2(ey - sy, ex - sx)
            zmin, zmax = min(sz, ez), max(sz, ez)
            x0 = sx + math.cos(az) * 0.45
            y0 = sy + math.sin(az) * 0.45
            if math.hypot(x0 - ob.cx, y0 - ob.cy) <= ob.trunk_r + 0.2:
                continue
            in_band = sA.collides_xy(x0, y0, zmin, zmax, pad=0.0)
            out_band = sA.collides_xy(x0, y0, out_z0, out_z1, pad=0.0)
            if in_band and not out_band:
                occ_ok += 1
            else:
                check("5 占用", False,
                      "tree%d br@%.2f,%.2f z=[%.2f,%.2f] out=[%.2f,%.2f] "
                      "in=%s out=%s" % (ob.id, x0, y0, zmin, zmax,
                                        out_z0, out_z1, in_band, out_band))
            if occ_ok >= 3:
                break
        if occ_ok >= 3:
            break
    check("5 A* 占用", occ_ok > 0,
          "isolated_branch_occ=%d (带外探针 z=[%.2f,%.2f] 由全林顶点 %.2f 导出)"
          % (occ_ok, out_z0, out_z1, z_top))

    # ---- 3/6 稀疏回波 + 对照一致性（摘枝场景对照；两侧均已摘冠） ----
    sOff = Scene()
    for ob in sOff.obstacles:
        if ob.kind == "tree":
            ob.branches = None
    crowns_blind(sOff)
    poses = []
    for ob in sA.obstacles:
        if ob.kind != "tree" or not ob.branches:
            continue
        br = ob.branches[0]
        az = math.atan2(br[4] - br[1], br[3] - br[0])
        px = ob.cx + math.cos(az + math.pi) * 4.0
        py = ob.cy + math.sin(az + math.pi) * 4.0
        # 视点高度=现役巡航（2026-10-02 裁决 cruise_z=2.4）。枝 z 带
        # [2.88,12.73] 高于旧程序枝（0.8-2.6），低视点（旧 1.5）的
        # ±28.6° el 扇够不着枝带 ⇒ branch_hits 恒 0 假阴性。
        pz = 2.4
        pos = (px, py, pz)
        # 站位须净空（两场景同查，避免站在别的树里）
        if not sA.collides(pos, 0.0) and not sOff.collides(pos, 0.0):
            poses.append(pos)
        if len(poses) >= 8:
            break
    bhits = 0
    thits = 0
    anomalies = []
    for pos in poses:
        hA = lidar_hits(sA, pos)
        hO = lidar_hits(sOff, pos)
        tA = {(a, e): t for (t, a, e) in hA}
        tO = {(a, e): t for (t, a, e) in hO}
        for k, t in tA.items():
            if k in tO:
                thits += 1
                if t < tO[k] - 1e-9:
                    bhits += 1  # 有枝场景更近 → 枝命中
                elif t > tO[k] + 1e-9:
                    anomalies.append((pos, k, t, tO[k]))
            else:
                bhits += 1  # 有枝命中、无枝场景无命中（被枝挡在树干前）
    frac = bhits / max(thits + bhits, 1)
    check("6 对照一致性", len(anomalies) == 0,
          "anomalies=%d" % len(anomalies))
    check("3 稀疏回波", bhits > 0 and frac <= 0.15,
          "branch_hits=%d/%d frac=%.3f poses=%d" % (bhits, bhits + thits, frac, len(poses)))

    npass = sum(1 for (_, ok, _) in RESULTS if ok)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("branch_selftest 结果 (%d/%d PASS)\n" % (npass, len(RESULTS)))
        for (name, ok, detail) in RESULTS:
            f.write("  [%s] %s%s\n" % ("PASS" if ok else "FAIL", name,
                                       (" — " + detail) if detail else ""))
    rc = 0 if npass == len(RESULTS) else 1
    print("branch_selftest rc=%d (%d/%d)" % (rc, npass, len(RESULTS)))
    sys.exit(rc)


if __name__ == "__main__":
    main()
