#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_depth_selftest.py — 深度渲染器盒原语（AABB）离线自测（无 ROS 运行栈）。

用法（WSL）：
  cd ~/zx2026_defop_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/defop_depth_selftest.py

T1 2026-09-29 验收（depth_sim_node 盒原语 + 配置键扩容）：
  0) 配置契约：sensor.depth.render_boxes=true 装订（段 enabled 默认 false
     =基线零侵入）；make_depth_cfg 映射（键缺失→代码缺省 false=旧行为）；
     顶层 planner.source=internal（P3 shadow 开关键位装订，当前无消费方）
  1) 真实 Scene 盒入画：kind=net 西林缘网（场景唯一静态网盒，必须实例化
     Scene 取真盒，walk yaml=假阴性）在 pose B 视野内，渲染值=盒真值的
     像素 >= NET_MIN——排除"另造等价测试盒"
  2) 全帧真值对质（pose B / pose C，噪声/dropout 关、射线方向复用渲染器
     同一 float32 基组合）：隔行全像素 vs scene.raycast，容差 ±1mm，0 失配
     ——盒/树干/枝/地原语统一 min-t 口径。冠两侧**同时排除**（渲染侧由
     cfg_truth 的 render_crowns=False + crowns_blind() 摘物理侧）：隔离
     非冠原语的走查；含冠同源对质见 2b)。
  2b) 含冠同源对质（2026-10-02「冠渲染 球→柱」修复的正证据）：pose A/B
     冠不摘、render_crowns=True，全帧对质仍须 0 失配——渲染器与
     geo.ray_cylinder 逐字同源（侧面求交、端盖忽略、柱面足迹内取远根）。
     修复前（渲染冠=球近似，非同源几何）此断言必失败，是"能失败"的对质。
     ⚠ 2026-10-02 晚起表档 crown_collidable=false ⇒ 本项**必须临时把杠杆
     强制 True**，否则两侧同时无冠、断言退化成空过（本项目踩过的「空集
     假 PASS」同型）；[6] 的默认钉已把「无冠档」逐位钉住。
  3) 盒在树后→树胜（min-t）：pose C 西林缘真实树挡网，中心像素=树干命中
     且严格近于盒单独 t；树侧邻像素=网命中且更远
  4) 盒出 FOV 无伪影：pose A（pad0 巡航代表位）与 pose D（网/围栏全在身后
     或量程外）render_boxes on/off 逐位一致——覆盖"围栏包围球含原点→
     全掩膜"与"薄轴一轴判死"两条早退路径
  5) 同 seed 逐位确定性（pose B 盒入画最重载位姿两帧逐位一致；异 seed
     不同=噪声流在场的旁证）
  6) 冠删除钉 + 双杠杆对抗断言（2026-10-02 晚「冠物理、视觉双删」重钉）：
     pose A/B boxes-off 默认档帧 sha256 == PIN_A/PIN_B（该值逐字节等于旧
     冠门 GATE_PIN_*，冠不入深度图）；回放杠杆各自逐位复现历史钉——① 强制
     全树 crown_collidable=True == CROWN_PIN_*（冠物理时代柱档）；② 再叠加
     crown_shape="sphere" == SPHERE_PIN_*（球档旧钉）。双向正证据：默认档
     render_crowns 开关是 no-op（冠不在画中），杠杆 True 下同开关必须改变帧
     且关掉即回到 PIN_*（跨门恒等式：两个独立门切同一批冠像素）。杠杆=唯一
     行为增量的永久证明
  7) 性能：pose A 盒开 <30ms/帧（任务书门）；pose B 附报盒增量 on-off
     （<5ms；注意改前基线在"网充满视野"的 pose B 已 31.8ms>30ms，
     P1 门原为 80ms——见交付 concerns）
  8) 树级预筛 tree_prefilter（2026-10-01 渲染提速）：六位姿（巡航 A / 网前 B /
     挡网树 C / 两林内重载位 / 林缘）on/off **逐字节一致** —— 预筛是纯提速，
     不是"更接近真值"，故判据只能是"同一张图且更快"；并断言最慢位姿
     <80ms/帧（六机 8 核下达 15Hz 的必要条件）。

注意 [0] 的 yaml enabled 断言已于 2026-10-01 松绑：该键是在跑的运行档
（DeFoP 臂须 true），"段缺失=零发布者"的契约由**代码缺省**承载（见下方
make_depth_cfg(None) 断言），不是由 yaml 现值承载。
"""
import hashlib
import math
import os
import random
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "zx2026_common", "scripts"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "arena_sensor", "scripts"))

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene
import depth_sim_node as dsn

# ---- 钉值（2026-10-07 换场「冠=物理+视觉、无真枝」重钉；默认噪声 cfg） -------
# 历史沿革（同一条 `crown_collidable` + 真枝有无的杠杆在动）：
#   ① 10-01 冠门时代（冠不入深度图）= 旧 GATE_PIN_*
#   ② 10-02 上半天 冠照画（球近似）= 旧 SPHERE_PIN_*
#   ③ 10-02 「球→柱」修复（渲染冠对齐 scene.raycast/geo.ray_cylinder）= 旧 PIN_*
#   ④ 10-02 晚「冠物理、视觉双删」——旧钉（默认==冠盲帧）
#   ⑤ **10-07 换场至 zx2026_ws 场景**：冠回来（表 crown_collidable=true）、
#      真枝全去（表无 branch_file）⇒ **旧钉整体作废**（旧帧含 1888 段真枝像素，
#      换场后真枝几何不存在）。按新基线重钉，三档（pose 固定，render_boxes=False）：
#        CROWN_ON    = 默认（crown_collidable=true ⇒ 冠在画中，cylinder）
#        CROWN_BLIND = 冠不在画中（lever False，或默认 + render_crowns=False）
#        SPHERE      = lever 开 + crown_shape="sphere"（形状杠杆档）
# 判别力不靠常数本身，而靠 [6] 的**三档两两不等** + 跨门恒等式 + 杠杆对抗——
# 常数只是新基线的比特快照，故意换场后按新基线重取，不削弱本测。
CROWN_ON_A = "350177603182cbf14d708ad46b5851c8f15d5de1789bfb2c18a3e1b2da624070"
# poseA=(-27.75,-3.75,1.0) yaw=0 seed=42，默认档（表 crown_collidable=true）
CROWN_ON_B = "57806b01ee7d16212297614bbc736223cb81ed2288a54dbbdea5e553a826d4f8"
# poseB=(-25.5,8.0,1.2) yaw=0 seed=424242，同上（实测与旧 CROWN_PIN_B 恰好相同）
CROWN_BLIND_A = "3c5792bba53eb3a5cd7a7512016028f0cb175ceeecee65a4f7bdd5a5c7929cc4"
# lever False（或默认 + render_crowns=False）——冠不入画的 poseA
CROWN_BLIND_B = "2bcc090f4e691df8beb47dae6f2abce1debacfc3bd47623d23f149b97ce24047"
# 同上 poseB
SPHERE_PIN_A = "4d3d0ab6a7415fc5ccfcb143ea43ceceffc9d1fd31e335dbd4996b2e43ccc7bb"
# lever 开 + crown_shape="sphere"（球档历史钉；换场后实测未变）poseA
SPHERE_PIN_B = "6e2f0942035804d1477834e2d5878b4a9d783c2dd25408887586ec43a2b0623d"
# 同上 poseB

POSE_A = ((-27.75, -3.75, 1.0), 0.0, 42)      # pad0 巡航代表位（P1 自测同款）
POSE_B = ((-25.5, 8.0, 1.2), 0.0, 424242)     # 网正前 2.7m（网充满视野）
POSE_D = ((-20.5, 8.0, 1.2), 0.0, 424242)     # 网正后 2.3m（盒全出 FOV/量程）

NET_MIN = 80000          # pose B 网可见像素下限（实测 ~11 万）


def frame(scene, pos, yaw, seed, dcfg, cam_dirs, inv_len):
    return dsn.render_depth(scene, pos, yaw, dcfg, random.Random(seed),
                            cam_dirs, inv_len)


def sha(img):
    return hashlib.sha256(img.tobytes()).hexdigest()


def crowns_blind(scene):
    """[2]/[3] 对质期间把物理侧的冠也摘掉（渲染侧由 cfg_truth 的
    render_crowns=False）——冠两侧同盲，对质聚焦盒/干/枝原语，失败归因
    更干净。含冠同源对质由 [2b] 承担（2026-10-02 球→柱修复后渲染冠=
    raycast 冠=竖圆柱，同一模型已可同框对质）。返回还原闭包（finally 调用）。

    ⚠ 2026-10-07 换场后表档 crown_collidable=**true**（冠回来），故本函数在默认
    档是**真摘冠**（把物理侧冠拿掉，与 cfg_truth 的 render_crowns=False 两侧同盲）。
    """

    trees = [ob for ob in scene.obstacles if ob.kind == "tree"]
    saved = [t.crown_collidable for t in trees]
    for t in trees:
        t.crown_collidable = False

    def restore():
        for t, v in zip(trees, saved):
            t.crown_collidable = v
    return restore


def main():
    # ---- 0) 配置契约 ----
    settings = cfg.load("sim_settings.yaml")
    dsec = settings.get("sensor", {}).get("depth")
    assert dsec is not None, "sim_settings.sensor.depth 段缺失"
    # 2026-10-01：yaml 的 enabled 是**在跑的运行档**，不再恒为 false——DeFoP
    # 臂须 true，由 tools/defop_ab_run.py 的 cell 装订写入并留在工作区。原断言
    # 写于该键刚立项时，已与运行档脱节。契约的真正载体是**代码缺省**：
    # make_depth_cfg(None)/键缺失 → false（见下方断言），段缺失即零发布者。
    assert isinstance(dsec.get("enabled"), bool), \
        "sensor.depth.enabled 必须是布尔（got %r）" % (dsec.get("enabled"),)
    assert dsec.get("render_boxes") is True, \
        "sensor.depth.render_boxes 必须装订为 true（段默认关故基线零侵入）"
    m = dsn.make_depth_cfg(dsec)
    assert m["render_boxes"] is True, "make_depth_cfg 未映射 yaml render_boxes"
    assert dsn.make_depth_cfg({})["render_boxes"] is False, \
        "键缺失时代码缺省必须 false=旧行为（默认关纪律）"
    assert dsn.make_depth_cfg(None)["enabled"] is False
    assert m["tree_prefilter"] is True, "make_depth_cfg 未映射 yaml tree_prefilter"
    assert dsn.make_depth_cfg({})["tree_prefilter"] is True, \
        "tree_prefilter 代码缺省须 true（逐位无损的提速；false 只是 A/B 回退档）"
    # 2026-10-02「冠渲染 球→柱」：渲染冠口径对齐 scene.raycast/geo.ray_cylinder
    # （真值三处——raycast/判官/物理——本就同源，渲染器是唯一错位方）。
    # 缺省 cylinder 不是"新行为"：键缺失即按真值口径渲染；sphere 仅为
    # [6] 回放旧钉的杠杆档，勿在运行档装订。
    assert m["crown_shape"] == "cylinder", \
        "make_depth_cfg 未映射 crown_shape（got %r）" % (m["crown_shape"],)
    assert dsn.make_depth_cfg({})["crown_shape"] == "cylinder", \
        "crown_shape 代码缺省须 cylinder（对齐真值口径非新行为，sphere=回放档）"
    # 2026-10-01 同 [0] 的松绑：planner.source 也是**运行档**——DeFoP 臂由
    # defop_ab_run.py 装订为 "defop" 并留在工作区。契约=键存在且取值合法，
    # 不是"恒为 internal"。
    planner = settings.get("planner")
    assert isinstance(planner, dict) and planner.get("source") in ("internal",
                                                                   "defop", "ego"), \
        "顶层 planner.source 必须是 internal|defop|ego（got %r）" % (
            planner.get("source") if isinstance(planner, dict) else planner,)
    # 2026-10-01：本行原为硬编码字面量 "depth.enabled=false"，但 yaml 里那个
    # 键是**在跑的运行档**（defop_ab_run.py 会装订 true 并留在工作区）——打印
    # 与事实不符，且正是"读了假象去归因"那类坑。改为回显实数。
    print("[0] config OK: depth.enabled=%s(运行档) render_boxes=%s(yaml)/"
          "false(代码缺省), planner.source=%s"
          % (dsec.get("enabled"), m["render_boxes"], planner["source"]))

    # ---- 场景事实（必须实例化 Scene） ----
    scene = Scene()
    nets = [ob for ob in scene.obstacles if ob.kind == "net"]
    assert len(nets) == 1, "场景必须恰有一个 kind=net 真盒（got %d）" % len(nets)
    net = nets[0]
    n_tree = sum(1 for ob in scene.obstacles if ob.kind == "tree")
    print("    scene: %d obstacles (%d trees, 1 net lo=%s hi=%s)"
          % (len(scene.obstacles), n_tree, net.lo, net.hi))
    netlo = np.asarray(net.lo, dtype=np.float64)
    nethi = np.asarray(net.hi, dtype=np.float64)

    dcfg = dsn.make_depth_cfg({"enabled": True})
    # 对质/计数用：噪声/dropout 关 + 冠关 + 盒开（对质对象含 net 盒真值）。
    # 冠关的理由：把 [2]/[3] 的遮挡集聚焦在盒/干原语，失败归因更干净；
    # 物理侧由 crowns_blind() 同步摘冠（换场后表 true，本函数是全林摘冠的有效操作）。
    # 含冠同源对质=[2b]（2026-10-02 球→柱修复后渲染冠=raycast 冠）。冠渲染路径由
    # [6] 钉值逐位保护。
    cfg_truth = dsn.make_depth_cfg({"enabled": True, "noise_std": 0.0,
                                    "dropout": 0.0, "render_crowns": False,
                                    "render_boxes": True})
    cam_dirs, inv_len = dsn.ray_dirs(dcfg)
    W, H = dcfg["width"], dcfg["height"]
    N = W * H
    t_max = dcfg["max_range"]

    def world_dirs(yaw):
        """与 render_depth 逐位同源的射线方向（float32 基组合）。"""
        rgt, dwn, fwd = dsn.cam_basis(yaw, 0.0)
        return cam_dirs @ np.stack([rgt, dwn, fwd]).astype(np.float32)

    def confront(pos, yaw, stride, tag, dcfg_use=None):
        """全帧 vs scene.raycast 真值对质，返回 (失配数, 网可见像素数)。
        dcfg_use 覆盖渲染 cfg（默认 cfg_truth=摘冠档；[2b] 用含冠档）。"""
        o = np.asarray(pos, dtype=np.float64)
        dworld = world_dirs(yaw).astype(np.float64)
        img = frame(scene, pos, yaw, 1, dcfg_use or cfg_truth, cam_dirs, inv_len)
        mm = img.astype(np.int64).ravel()
        mism = 0
        net_vis = 0
        for i in range(0, N, stride):
            dw = tuple(dworld[i])
            tt = scene.raycast(pos, dw, max_t=t_max * 1.5)
            tn = geo.ray_aabb(o, dw, netlo, nethi)
            v = int(mm[i])
            if tt is None or tt > t_max:
                exp = 10000
            else:
                z = tt * inv_len[i]
                exp = 0 if z < dcfg["min_range"] else int(np.rint(z * 1000.0))
            if abs(v - exp) > 1:
                mism += 1
                if mism <= 5:
                    print("    MISMATCH %s px=%d v=%d exp=%d t=%s row=%d col=%d"
                          % (tag, i, v, exp, tt, i // W, i % W))
            if (tn is not None and 0 < tn <= t_max and tt is not None
                    and abs(tt - tn) < 1e-6 and abs(v - int(np.rint(tn * inv_len[i] * 1000.0))) <= 1):
                net_vis += 1
        return mism, net_vis

    # ---- 1)+2) 真盒入画 + 全帧对质（pose B：网正前） ----
    posB, yawB, seedB = POSE_B
    restore = crowns_blind(scene)
    try:
        mismB, net_vis = confront(posB, yawB, 2, "poseB")
    finally:
        restore()
    print("[1] real-net in view (pose B): net-visible pixels=%d (stride2, "
          "expect >=%d)" % (net_vis, NET_MIN / 2))
    assert net_vis * 2 >= NET_MIN, "网可见像素不足（真盒未入画？）%d" % net_vis
    print("[2] full-frame confrontation vs scene.raycast (crowns off, "
          "noise off, stride2):")
    print("    poseB mismatches=%d / %d rays" % (mismB, N // 2))
    assert mismB == 0, "poseB 对质失配 %d" % mismB

    # ---- 2b) 含冠同源对质（2026-10-02「冠渲染 球→柱」修复的正证据） ----
    # 修复前：渲染冠=球近似（半径 max(crown_r, crown_h/2)）、raycast 冠=竖圆柱
    # （crown_r × 包络高，c_crown）——非同源几何，含冠对质必失配（球档实测：
    # 冠占 poseA/B 全帧 ~51% 像素）。修复后渲染器逐字同 geo.ray_cylinder
    # （侧面求交/端盖忽略/柱面足迹内取远根）⇒ 两侧都不摘冠、全帧仍须 0 失配。
    # 反向证明见 [6]：sphere 档回放帧 != 柱档钉（形状杠杆唯一增量）。
    cfg_truth_crown = dict(cfg_truth)
    cfg_truth_crown["render_crowns"] = True
    posA2, yawA2, seedA2 = POSE_A
    # 表档 crown_collidable=false（10-02 晚）⇒ 必须临时把杠杆强制 True，
    # 否则渲染侧与 raycast 侧同时无冠，"0 失配"退化成空过。
    trees_all = [ob for ob in scene.obstacles if ob.kind == "tree"]
    saved_cc2 = [t.crown_collidable for t in trees_all]
    for t in trees_all:
        t.crown_collidable = True
    try:
        mismAcr, _ = confront(posA2, yawA2, 2, "poseA-crown", cfg_truth_crown)
        mismBcr, _ = confront(posB, yawB, 2, "poseB-crown", cfg_truth_crown)
        img_crownlive = frame(scene, posA2, yawA2, seedA2, cfg_truth_crown,
                              cam_dirs, inv_len)
    finally:
        for t, v in zip(trees_all, saved_cc2):
            t.crown_collidable = v
    # 反空过护栏：杠杆 True 下 render_crowns 开关必须改变帧（冠确在画中）。
    # 缺此护栏时，渲染冠若整条路径失效，本项仍会 0 失配地静默通过。
    img_crownblind = frame(scene, posA2, yawA2, seedA2,
                           dict(cfg_truth_crown, render_crowns=False),
                           cam_dirs, inv_len)
    assert img_crownlive.tobytes() != img_crownblind.tobytes(), \
        "杠杆 True 下 render_crowns 开关未改变帧——冠不在画中，[2b] 是空过"
    print("[2b] crown-live confrontation (lever forced on, stride2): poseA=%d "
          "poseB=%d mismatches / %d rays" % (mismAcr, mismBcr, N // 2))
    assert mismAcr == 0 and mismBcr == 0, \
        "含冠对质失配 poseA=%d poseB=%d（渲染冠与 raycast 冠非同源？）" % (
            mismAcr, mismBcr)

    # ---- 3) 盒在树后→树胜（min-t，pose C：西林缘真实树挡网） ----
    blk = [ob for ob in scene.obstacles if ob.kind == "tree"
           and net.lo[0] - 0.6 <= ob.cx < net.lo[0]
           and net.lo[1] - 1.0 <= ob.cy <= net.lo[1] + 1.0]
    assert blk, "网前缘西侧无真实挡网树（场景事实变更？）"
    blk.sort(key=lambda ob: ob.cy)   # 距网南缘最近者
    tree = blk[0]
    posC = (tree.cx - 2.85, tree.cy - 4.27, 1.2)
    yawC = math.atan2(tree.cy - posC[1], tree.cx - posC[0])
    oC = np.asarray(posC, dtype=np.float64)
    fwC = dsn.cam_basis(yawC, 0.0)[2]
    t_trunk = scene.raycast(posC, tuple(fwC), max_t=15.0)
    assert t_trunk is not None, "poseC 中心射线无命中"
    hp = oC + t_trunk * fwC
    d_ax = math.hypot(hp[0] - tree.cx, hp[1] - tree.cy)
    assert d_ax <= tree.trunk_r + 0.05, \
        "poseC 中心命中非挡网树 id=%d (d_ax=%.3f)" % (tree.id, d_ax)
    t_net_axis = geo.ray_aabb(oC, tuple(fwC), netlo, nethi)
    assert t_net_axis is not None and t_net_axis - t_trunk > 0.3, \
        "挡网树与网无前后间隔差 (%s vs %s)" % (t_trunk, t_net_axis)
    imgC = frame(scene, posC, yawC, 7, cfg_truth, cam_dirs, inv_len)
    ci = (H // 2) * W + (W // 2)     # 中心像素=正对树干（yawC 对准）
    v_center = int(imgC.ravel()[ci])
    exp_center = int(np.rint(t_trunk * inv_len[ci] * 1000.0))
    assert abs(v_center - exp_center) <= 2, \
        "poseC 中心像素 %d != 树干真值 %d" % (v_center, exp_center)
    assert v_center < (t_net_axis - 0.3) * 1000.0, \
        "树未胜盒：中心 %d vs 网单独 t %.3f" % (v_center, t_net_axis)
    # 树侧邻像素=网命中且更远（确定性扫描取第一个网真值像素）
    RC = np.stack(list(dsn.cam_basis(yawC, 0.0)))
    dworldC = (cam_dirs @ RC.astype(np.float32)).astype(np.float64)
    du_net = None
    for du in list(range(6, 120)) + list(range(-6, -120, -1)):
        i = (H // 2) * W + (W // 2 + du)
        dw = tuple(dworldC[i])
        tt = scene.raycast(posC, dw, max_t=t_max * 1.5)
        tn = geo.ray_aabb(oC, dw, netlo, nethi)
        if tt is not None and tn is not None and abs(tt - tn) < 1e-6:
            du_net = (du, i, tt)
            break
    assert du_net is not None, "poseC 树侧未找到网真值像素"
    du, i_net, t_net_px = du_net
    v_net = int(imgC.ravel()[i_net])
    exp_net = int(np.rint(t_net_px * inv_len[i_net] * 1000.0))
    assert abs(v_net - exp_net) <= 2, \
        "poseC 网像素 %d != 盒真值 %d" % (v_net, exp_net)
    assert v_net > v_center, "网像素应远于树像素（min-t 树胜）"
    restore = crowns_blind(scene)
    try:
        mismC, _ = confront(posC, yawC, 2, "poseC")
    finally:
        restore()
    print("[3] box-behind-tree (pose C, tree id=%d cx=%.3f cy=%.3f): center=%d"
          " (trunk t=%.3f) < net-only t=%.3f; net px du=%+d val=%d (t=%.3f); "
          "full-frame mismatches=%d"
          % (tree.id, tree.cx, tree.cy, v_center, t_trunk, t_net_axis,
             du, v_net, t_net_px, mismC))
    assert mismC == 0, "poseC 对质失配 %d" % mismC

    # ---- 4) 盒出 FOV 无伪影（on/off 逐位一致） ----
    cfg_on = dict(dcfg)
    cfg_on["render_boxes"] = True
    cfg_off = dict(dcfg)
    cfg_off["render_boxes"] = False
    for pos, yaw, seed in (POSE_A, POSE_D):
        img_on = frame(scene, pos, yaw, seed, cfg_on, cam_dirs, inv_len)
        img_off = frame(scene, pos, yaw, seed, cfg_off, cam_dirs, inv_len)
        assert img_on.tobytes() == img_off.tobytes(), \
            "%s 盒开关两图不逐位一致（FOV 外伪影）" % (pos,)
        print("[4] boxes on/off bitwise identical at pose %s (%dx%d)"
              % (pos, W, H))

    # ---- 5) 同 seed 逐位确定性（盒入画最重载位姿） ----
    img_b1 = frame(scene, posB, yawB, seedB, cfg_on, cam_dirs, inv_len)
    img_b2 = frame(scene, posB, yawB, seedB, cfg_on, cam_dirs, inv_len)
    assert img_b1.tobytes() == img_b2.tobytes(), "同 seed 两帧不逐位一致"
    img_b3 = frame(scene, posB, yawB, seedB + 1, cfg_on, cam_dirs, inv_len)
    assert img_b1.tobytes() != img_b3.tobytes(), "异 seed 两帧逐位相同（噪声流失效？）"
    print("[5] determinism OK: same seed bitwise identical, diff seed differs")

    # ---- 6) 换场钉 + 双杠杆对抗断言（2026-10-07：冠=物理+视觉、无真枝） ----
    posA, yawA, seedA = POSE_A
    img_a_off = frame(scene, posA, yawA, seedA, cfg_off, cam_dirs, inv_len)
    img_b_off = frame(scene, posB, yawB, seedB, cfg_off, cam_dirs, inv_len)
    sa, sb = sha(img_a_off), sha(img_b_off)
    assert sa == CROWN_ON_A, "poseA 默认档 sha 不符换场钉\n  got %s\n  pin %s" % (
        sa, CROWN_ON_A)
    assert sb == CROWN_ON_B, "poseB 默认档 sha 不符换场钉\n  got %s\n  pin %s" % (
        sb, CROWN_ON_B)
    # 正证据①：默认档（表 crown_collidable=true）⇒ 冠**在画中**，故 render_crowns
    # 开关**必须改变帧**，且关掉后逐位复现 CROWN_BLIND 钉（冠盲帧）。
    cfg_cr_off = dict(cfg_off)
    cfg_cr_off["render_crowns"] = False
    img_a_nc = frame(scene, posA, yawA, seedA, cfg_cr_off, cam_dirs, inv_len)
    img_b_nc = frame(scene, posB, yawB, seedB, cfg_cr_off, cam_dirs, inv_len)
    assert sha(img_a_nc) == CROWN_BLIND_A and sha(img_b_nc) == CROWN_BLIND_B, \
        "默认档 render_crowns 关 != CROWN_BLIND 钉（冠盲帧）"
    assert sha(img_a_nc) != sa and sha(img_b_nc) != sb, \
        "默认档 render_crowns 开关未改变帧——冠不在画中（表 crown_collidable 未生效？）"
    # 形状杠杆（「球→柱」修复的唯一增量证明）：默认 lever 开 + crown_shape
    # ="sphere" 必须逐位复现球档钉 SPHERE_PIN_*，且 != cylinder 默认帧。
    cfg_sph = dict(cfg_off)
    cfg_sph["crown_shape"] = "sphere"
    img_a_sph = frame(scene, posA, yawA, seedA, cfg_sph, cam_dirs, inv_len)
    img_b_sph = frame(scene, posB, yawB, seedB, cfg_sph, cam_dirs, inv_len)
    assert sha(img_a_sph) == SPHERE_PIN_A, \
        "形状杠杆回放失败 poseA\n  got %s\n  sphere pin %s" % (
            sha(img_a_sph), SPHERE_PIN_A)
    assert sha(img_b_sph) == SPHERE_PIN_B, \
        "形状杠杆回放失败 poseB\n  got %s\n  sphere pin %s" % (
            sha(img_b_sph), SPHERE_PIN_B)
    assert sha(img_a_sph) != sa and sha(img_b_sph) != sb, \
        "sphere 与 cylinder 两档图像逐位相同（形状开关未生效？）"
    # 冠杠杆对抗证明（永久化）：强制全树 crown_collidable=False ⇒ 须逐位复现
    # 冠盲钉 CROWN_BLIND_*；且 lever-off + render_crowns 关 走**跨门恒等式**
    # （两个独立门 crown_collidable / render_crowns 切同一批冠像素）。破 ⇒ 杠杆
    # 之外另有行为差，或冠渲染路径被改坏——两种都须查。
    trees = [ob for ob in scene.obstacles if ob.kind == "tree"]
    saved = [getattr(t, "crown_collidable", True) for t in trees]
    for t in trees:
        t.crown_collidable = False
    try:
        img_a_blind = frame(scene, posA, yawA, seedA, cfg_off, cam_dirs, inv_len)
        img_b_blind = frame(scene, posB, yawB, seedB, cfg_off, cam_dirs, inv_len)
        img_a_gate = frame(scene, posA, yawA, seedA, cfg_cr_off, cam_dirs, inv_len)
        img_b_gate = frame(scene, posB, yawB, seedB, cfg_cr_off, cam_dirs, inv_len)
    finally:
        for t, v in zip(trees, saved):
            t.crown_collidable = v
    s_ab, s_bb = sha(img_a_blind), sha(img_b_blind)
    assert s_ab == CROWN_BLIND_A and s_bb == CROWN_BLIND_B, \
        "杠杆 False 回放失败 poseA=%s poseB=%s（期望 CROWN_BLIND）" % (s_ab, s_bb)
    assert s_ab != sa and s_bb != sb, \
        "杠杆开关未改变帧（crown_collidable 未生效？）"
    assert sha(img_a_gate) == s_ab and sha(img_b_gate) == s_bb, \
        "lever 关 + render_crowns 关 != CROWN_BLIND（两门切不同像素集？）"
    img_b_on = img_b1     # 盒开、网在视野内：必须不同于钉（正证据）
    assert sha(img_b_on) != CROWN_ON_B, "poseB 盒开仍与钉逐位相同（盒未入画？）"
    print("[6] default == CROWN_ON pins (冠在画中); default+render_crowns-off == "
          "CROWN_BLIND pins 且 != default; lever-off == CROWN_BLIND 且 != default;"
          " default+sphere == SPHERE pins 且 != cylinder; lever-off+render_crowns-"
          "off == CROWN_BLIND (跨门恒等式); boxes-on differs")

    # ---- 7) 性能 ----
    rng_t = random.Random(20260929)
    for _ in range(3):
        frame(scene, posA, yawA, 1, cfg_on, cam_dirs, inv_len)
    NF = 30
    t0 = time.perf_counter()
    for _ in range(NF):
        frame(scene, posA, yawA, 1, cfg_on, cam_dirs, inv_len)
    ms_a = (time.perf_counter() - t0) / NF * 1000.0
    # 绝对毫秒门只在**静默宿主**上有效：同宿另跑（如 arena gzserver 满载）会把
    # 每帧抬到 40-60ms——正是「负载读成回归」的坑。宿主不静默 ⇒ **响亮跳过**
    # 绝对门（打印 SKIP+loadavg），保留相对/逐位判据（[8] 的比值与逐位一致抗争用）。
    # 绝不静默通过、也不误报 FAIL。阈值 load1/核数 < 0.5（静默实测 ~0.06）。
    _la = float(open("/proc/loadavg").read().split()[0])
    _ncpu = os.cpu_count() or 1
    _quiet = (_la / _ncpu) < 0.5
    if _quiet:
        assert ms_a < 30.0, "poseA 单帧 %.1f ms 超过 30ms 门" % ms_a
    for _ in range(3):
        frame(scene, posB, yawB, 1, cfg_on, cam_dirs, inv_len)
        frame(scene, posB, yawB, 1, cfg_off, cam_dirs, inv_len)
    t0 = time.perf_counter()
    for _ in range(20):
        frame(scene, posB, yawB, 1, cfg_on, cam_dirs, inv_len)
    ms_b_on = (time.perf_counter() - t0) / 20 * 1000.0
    t0 = time.perf_counter()
    for _ in range(20):
        frame(scene, posB, yawB, 1, cfg_off, cam_dirs, inv_len)
    ms_b_off = (time.perf_counter() - t0) / 20 * 1000.0
    delta = ms_b_on - ms_b_off
    # 预算 8ms：pose B=网充满视野的最重载位姿（85% 像素盒命中，float32
    # slab 实测 ~6ms）；DeFoP 契约 15Hz=66.7ms 帧预算、P1 模块门 80ms，
    # 均有余量；任务书 30ms 门由 pose A（巡航代表位）把关。
    if _quiet:
        assert delta < 8.0, "盒增量 %.1f ms 超过 8ms 预算" % delta
        print("[7] perf: poseA %.1f ms (<30 gate); poseB on=%.1f off=%.1f "
              "(box delta %.1f ms, <8)" % (ms_a, ms_b_on, ms_b_off, delta))
    else:
        print("[7] SKIP 绝对毫秒门：宿主不静默 load1=%.1f/%d核=%.2f（阈0.5）"
              " —— 实测 poseA %.1f ms、盒增量 %.1f ms 仅供参考"
              % (_la, _ncpu, _la / _ncpu, ms_a, delta))

    # ---- 8) 树级预筛 on/off：逐位一致 + 提速（2026-10-01 渲染提速） ----
    # 预筛是**纯提速**：外包球严格包含各原语包围球 ⇒ 子集只多不少 ⇒ 同一张
    # 图。故判据是"逐字节相同且更快"，而不是"更接近真值"。位姿覆盖巡航位、
    # 网前、林缘、两个林内重载位（林内是真枝预筛收益最大的地方）。
    cfg_pf_on = dict(dcfg)
    cfg_pf_on["tree_prefilter"] = True
    cfg_pf_off = dict(dcfg)
    cfg_pf_off["tree_prefilter"] = False
    POSES_PF = [("A", posA, yawA, seedA), ("B", posB, yawB, seedB),
                ("C", posC, yawC, 7),
                ("mid1", (-15.0, -15.0, 2.0), 0.7, 43),
                ("mid2", (10.0, 10.0, 2.4), 2.1, 43),
                ("edge", (-28.0, -28.0, 1.5), 3.9, 44)]
    tot_on = tot_off = 0.0
    pf_on_ms = []
    for tag, pos, yaw, seed in POSES_PF:
        for _ in range(2):
            frame(scene, pos, yaw, seed, cfg_pf_on, cam_dirs, inv_len)
            frame(scene, pos, yaw, seed, cfg_pf_off, cam_dirs, inv_len)
        n = 12
        t0 = time.perf_counter()
        for _ in range(n):
            i_on = frame(scene, pos, yaw, seed, cfg_pf_on, cam_dirs, inv_len)
        t_on = (time.perf_counter() - t0) / n * 1000.0
        t0 = time.perf_counter()
        for _ in range(n):
            i_off = frame(scene, pos, yaw, seed, cfg_pf_off, cam_dirs, inv_len)
        t_off = (time.perf_counter() - t0) / n * 1000.0
        assert i_on.tobytes() == i_off.tobytes(), \
            ("pose %s: tree_prefilter on/off 两图不逐位一致"
             "（子集丢像素=包围球未严格包含原语）" % tag)
        tot_on += t_on
        tot_off += t_off
        pf_on_ms.append(t_on)
        print("    %-5s on=%7.2f  off=%7.2f ms  %+.2fx  bitwise identical"
              % (tag, t_on, t_off, t_off / t_on))
    print("[8] tree_prefilter on/off 六位姿逐位一致；合计 on=%.1f off=%.1f ms"
          " (%.2fx)" % (tot_on, tot_off, tot_off / tot_on))
    assert tot_on < tot_off, \
        "tree_prefilter 未提速（on %.1f >= off %.1f ms）" % (tot_on, tot_off)
    # 六机 15Hz 的算力账：每帧 <=80ms（模块门）×6 机 = 480ms CPU/帧周期，
    # 8 核并行 → 60ms 周期 ⇒ 16.7Hz。故"最慢位姿 <=80ms"是 15Hz 的必要条件。
    if _quiet:
        assert max(pf_on_ms) < 80.0, \
            "最慢位姿 %.1f ms 超 80ms/帧模块门（六机 8 核下 15Hz 需 <=80ms）" % max(pf_on_ms)
    else:
        print("[8] SKIP 绝对毫秒门（宿主不静默）：最慢位姿 %.1f ms 仅供参考"
              % max(pf_on_ms))

    print("ALL SELFTEST PASS")


if __name__ == "__main__":
    main()
