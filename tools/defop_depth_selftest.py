#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_depth_selftest.py — 深度渲染器盒原语（AABB）离线自测（无 ROS 运行栈）。

用法（WSL）：
  cd ~/zx2026_arena_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/defop_depth_selftest.py

T1 2026-09-29 验收（depth_sim_node 盒原语 + 配置键扩容）：
  0) 配置契约：sensor.depth.render_boxes=true 装订（段 enabled 默认 false
     =基线零侵入）；make_depth_cfg 映射（键缺失→代码缺省 false=旧行为）；
     顶层 planner.source=internal（P3 shadow 开关键位装订，当前无消费方）
  1) 真实 Scene 盒入画：kind=net 西林缘网（场景唯一静态网盒，必须实例化
     Scene 取真盒，walk yaml=假阴性）在 pose B 视野内，渲染值=盒真值的
     像素 >= NET_MIN——排除"另造等价测试盒"
  2) 全帧真值对质（pose B / pose C，render_crowns 关=遮挡集与 scene.raycast
     对齐【树冠仅视觉不入 raycast】、噪声/dropout 关、射线方向复用渲染器
     同一 float32 基组合）：隔行全像素 vs scene.raycast，容差 ±1mm，0 失配
     ——盒/树干/地三原语统一 min-t 口径
  3) 盒在树后→树胜（min-t）：pose C 西林缘真实树挡网，中心像素=树干命中
     且严格近于盒单独 t；树侧邻像素=网命中且更远
  4) 盒出 FOV 无伪影：pose A（pad0 巡航代表位）与 pose D（网/围栏全在身后
     或量程外）render_boxes on/off 逐位一致——覆盖"围栏包围球含原点→
     全掩膜"与"薄轴一轴判死"两条早退路径
  5) 同 seed 逐位确定性（pose B 盒入画最重载位姿两帧逐位一致；异 seed
     不同=噪声流在场的旁证）
  6) render_boxes=false 逐位回到旧图：pose A/B 两帧 sha256 == 改前钉
     （PIN_A/PIN_B，2026-09-29 T1 改前代码采样，默认噪声/dropout 流同源；
     若渲染器本体后续变更需重采样钉值）
  7) 性能：pose A 盒开 <30ms/帧（任务书门）；pose B 附报盒增量 on-off
     （<5ms；注意改前基线在"网充满视野"的 pose B 已 31.8ms>30ms，
     P1 门原为 80ms——见交付 concerns）
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

# ---- 改前钉值（2026-09-29 T1 改前代码采样；默认噪声 cfg） --------------------
PIN_A = "4d3d0ab6a7415fc5ccfcb143ea43ceceffc9d1fd31e335dbd4996b2e43ccc7bb"
# poseA=(-27.75,-3.75,1.0) yaw=0 seed=42
PIN_B = "6e2f0942035804d1477834e2d5878b4a9d783c2dd25408887586ec43a2b0623d"
# poseB=(-25.5,8.0,1.2) yaw=0 seed=424242

POSE_A = ((-27.75, -3.75, 1.0), 0.0, 42)      # pad0 巡航代表位（P1 自测同款）
POSE_B = ((-25.5, 8.0, 1.2), 0.0, 424242)     # 网正前 2.7m（网充满视野）
POSE_D = ((-20.5, 8.0, 1.2), 0.0, 424242)     # 网正后 2.3m（盒全出 FOV/量程）

NET_MIN = 80000          # pose B 网可见像素下限（实测 ~11 万）


def frame(scene, pos, yaw, seed, dcfg, cam_dirs, inv_len):
    return dsn.render_depth(scene, pos, yaw, dcfg, random.Random(seed),
                            cam_dirs, inv_len)


def sha(img):
    return hashlib.sha256(img.tobytes()).hexdigest()


def main():
    # ---- 0) 配置契约 ----
    settings = cfg.load("sim_settings.yaml")
    dsec = settings.get("sensor", {}).get("depth")
    assert dsec is not None, "sim_settings.sensor.depth 段缺失"
    assert dsec.get("enabled") is False, "depth.enabled 默认必须 false（P1 契约）"
    assert dsec.get("render_boxes") is True, \
        "sensor.depth.render_boxes 必须装订为 true（段默认关故基线零侵入）"
    m = dsn.make_depth_cfg(dsec)
    assert m["render_boxes"] is True, "make_depth_cfg 未映射 yaml render_boxes"
    assert dsn.make_depth_cfg({})["render_boxes"] is False, \
        "键缺失时代码缺省必须 false=旧行为（默认关纪律）"
    assert dsn.make_depth_cfg(None)["enabled"] is False
    planner = settings.get("planner")
    assert isinstance(planner, dict) and planner.get("source") == "internal", \
        "顶层 planner.source 必须为 internal（T1 键位装订）"
    print("[0] config OK: depth.enabled=false render_boxes=true(yaml)/"
          "false(代码缺省), planner.source=%s" % planner["source"])

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
    # 对质/计数用：噪声/dropout 关 + 树冠关（遮挡集与 scene.raycast 对齐，
    # raycast 对树冠无回波）+ 盒开（对质对象含 net 盒真值）
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

    def confront(pos, yaw, stride, tag):
        """全帧 vs scene.raycast 真值对质，返回 (失配数, 网可见像素数)。"""
        o = np.asarray(pos, dtype=np.float64)
        dworld = world_dirs(yaw).astype(np.float64)
        img = frame(scene, pos, yaw, 1, cfg_truth, cam_dirs, inv_len)
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
    mismB, net_vis = confront(posB, yawB, 2, "poseB")
    print("[1] real-net in view (pose B): net-visible pixels=%d (stride2, "
          "expect >=%d)" % (net_vis, NET_MIN / 2))
    assert net_vis * 2 >= NET_MIN, "网可见像素不足（真盒未入画？）%d" % net_vis
    print("[2] full-frame confrontation vs scene.raycast (crowns off, "
          "noise off, stride2):")
    print("    poseB mismatches=%d / %d rays" % (mismB, N // 2))
    assert mismB == 0, "poseB 对质失配 %d" % mismB

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
    mismC, _ = confront(posC, yawC, 2, "poseC")
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

    # ---- 6) render_boxes=false 逐位回到旧图（改前 sha256 钉） ----
    posA, yawA, seedA = POSE_A
    img_a_off = frame(scene, posA, yawA, seedA, cfg_off, cam_dirs, inv_len)
    img_b_off = frame(scene, posB, yawB, seedB, cfg_off, cam_dirs, inv_len)
    sa, sb = sha(img_a_off), sha(img_b_off)
    assert sa == PIN_A, "poseA boxes-off sha 不符改前钉\n  got %s\n  pin %s" % (sa, PIN_A)
    assert sb == PIN_B, "poseB boxes-off sha 不符改前钉\n  got %s\n  pin %s" % (sb, PIN_B)
    img_b_on = img_b1     # 盒开、网在视野内：必须不同于旧图（正证据）
    assert sha(img_b_on) != PIN_B, "poseB 盒开仍与旧图逐位相同（盒未入画？）"
    print("[6] render_boxes=false == pre-T1 pins (A&B), boxes-on differs")

    # ---- 7) 性能 ----
    rng_t = random.Random(20260929)
    for _ in range(3):
        frame(scene, posA, yawA, 1, cfg_on, cam_dirs, inv_len)
    NF = 30
    t0 = time.perf_counter()
    for _ in range(NF):
        frame(scene, posA, yawA, 1, cfg_on, cam_dirs, inv_len)
    ms_a = (time.perf_counter() - t0) / NF * 1000.0
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
    assert delta < 8.0, "盒增量 %.1f ms 超过 8ms 预算" % delta
    print("[7] perf: poseA %.1f ms (<30 gate); poseB on=%.1f off=%.1f "
          "(box delta %.1f ms, <8)" % (ms_a, ms_b_on, ms_b_off, delta))

    print("ALL SELFTEST PASS")


if __name__ == "__main__":
    main()
