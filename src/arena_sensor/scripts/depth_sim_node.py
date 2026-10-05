#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::depth_sim_node — 前视深度相机合成渲染（每机一个，D435 式）。

DeFoP 导航节点的图像源：480x270 @15Hz 16UC1 毫米制深度图（真机 D435 契约）。
发布 /drone_<id>/depth/image_raw (16UC1, mm, 0=无效) + /drone_<id>/depth/camera_info。
传感器模型哲学同 lidar_node / camera_sim_node：读 Scene 真值几何 + 真值 odom，
产出带噪声的观测。渲染（本文件）与消费（defop_nav，P2）分离——本节点保持
无真值泄漏边界：图像里只有几何深度，不含任何任务语义。

自门控（新约定，camera 段没有的）：sim_settings.sensor.depth 段缺失或
enabled=false 时启动即退、零发布者，INFO 日志一次。默认关=零侵入基线，
DeFoP 联调时手动开。

几何渲染（numpy 向量化，两级"包围球掩膜 + 子集精确求交"，无逐像素 python
循环）：先按**树**切一次射线子集（g 条），再在子集内逐原语掩膜+求交。
单帧 CPU 目标 <=80ms —— 2026-10-01 实测破线：真枝 1888 段（每棵 8–10 段）
把逐原语掩膜次数从 406 推到 2294，林内位姿 165–185ms/帧（超 2.5×），
六机分摊后 depth 链降到 ~1.5Hz；g 条即为此而立。
  a) 地面平面 z=ground_z 相交；
  b) 树干=竖直圆柱（有限高 trunk_h 从 ground_z 起）；预筛：机位水平距离
     > max_range + margin（margin=trunk_r+max(crown_r,冠半高)，≥冠圆柱水平
     半径）的树整棵跳过；
  c) 树冠=竖直圆柱（2026-10-02 由【球近似】改为与真值同源）：半径 crown_r、
     z∈[crown_z-crown_h/2, crown_z+crown_h/2]（杨树 table；crown_h=None 旧
     场景 → z∈[crown_z-crown_r, crown_z+crown_r]）——口径逐字同
     scene._crown_z_range + geo.ray_cylinder（侧面求交、端盖忽略；相机在柱面
     足迹内时近根在身后、远根=内壁命中，同 ray_cylinder 两根循环）。改前
     球近似（心=(cx,cy,crown_z)、半径=max(crown_r, crown_h/2)）在冠包络底
     以下 0.35m（=机高带）横向半宽归零：物理/判官（c_crown 竖圆柱）在撞击
     高度能接住机顶而图上无冠信号（diag24 三起冠撞的图像读数因此乐观
     0.4–0.8m）。sphere 档仅保留为旧钉回放杠杆（depth_cfg.crown_shape），
     非运行档；
  d) 树枝=世界系线段 → 有限圆柱（轴线 A→B、半径 r）精确求交（同
     geo.ray_finite_cylinder 模型）；同样按包围球（中点、半径=|AB|/2+r）预筛。
     细枝在 10m 外亚像素属正常，不做抗锯齿；
  e) 盒障碍=AABB（网 kind=net / 围栏 / 灌木等静态盒；render_boxes 门控，
     代码缺省关=旧图逐位）：slab 法向量化（轴序=边长升序薄轴先行，t_near
     单调增 / t_far 单调减 → 逐轴中途全灭即整盒出局），并入同一"预筛→
     精确求交→min-t"管线；模型同 geo.ray_aabb（scene.raycast 对盒的真值
     口径），命中判据 t>eps 与其他原语同口径（原点在盒内不产生命中，
     scene.raycast 此情形返回 t=0，真值对质仅在盒外位姿做）；标量预筛=
     盒面最近点距离超量程整盒跳过（射线命中点必在盒面上）。2026-09-29
     T1：西林缘网原本不入画，DeFoP 臂对网盲——P3 A/B 双臂须同世界认知
     （真机网失明是 P4/P5 装订兜底项，与本键无关）；
  f) 每像素取最近命中（深度缓冲取 min），无命中 = max_range（10000 毫米）；
  g) 树级两级预筛（2026-10-01，sensor.depth.tree_prefilter）：
     ① 视锥剔除——整树/单个原语的包围球若整个在视锥外、超量程或在身后，
        直接跳过不求（_frustum_visible）。原预筛只看水平距离，相机身后的
        树照样吃一整套全幅掩膜；真枝又多在平视 58° 垂直视场的上缘之外。
     ② 射线子集——每棵候选树算一个罩住 干+冠+全枝 的保守外包球
        （_tree_bound），一次掩膜切出子集 dsub/tsub，b/c/d 三步的掩膜与
        求交都只在子集上做（掩膜是 O(N)=129600 的全幅扫描，此前每棵树的
        每个原语各扫一遍，真枝导入后林内单帧达 2294 遍）。
     两道都**只做排除**：外包球严格包含各原语包围球（半径带 _TREE_PAD），
     视锥判据用半径把球整体算进去（冗余 _CULL_PAD），故输出**逐位无损**。
     关掉则无任何剔除、子集=全幅，表达式与改前逐位同算（tools/
     defop_depth_selftest.py [8] 断言 on/off 两图逐字节一致）。

相机模型：机体系原位、朝机头水平前视；cam_pitch_deg 正=机头上仰。图像行
=ROS 光学系口径（y 朝下、row 0=画面顶）：地平线以上成像于 row<cy
（2026-09-29 翻转修：原实现 y 基向量误用世界上方向，画面上下颠倒）。
fx=(W/2)/tan(hfov/2)、fy=(H/2)/tan(vfov/2) 各轴精确导出（480x270@87/58 下
fx≠fy≈4%，非方像素 plumb_bob 合法；camera_info K 是唯一权威，消费方禁用
fx=fy 假设）。cx=W/2、cy=H/2、畸变全零。深度=Z 深度（沿光轴分量，D435 同
口径），非斜距。frame_id=drone_<id>/depth_optical；本节点不广播 TF（sim 全栈
世界系 odom 哲学，外参=机体系原位、yaw 对齐，消费方自行与 odom 融合）。

噪声模型（模拟 D435 主动双目，参数化 sim_settings.sensor.depth，每机独立
rng=run_seed+id*7919，同 seed 逐位复现——lidar/camera 同纪律）：
  命中距离 + 高斯噪声 noise_std（米）；
  Z < min_range 或 Z > max_range → 0（无效）；
  命中像素随机 dropout 置 0；
  量化到毫米 uint16。无命中像素=10000 毫米（max_range 哨兵）不参与噪声。

odom 断流 >0.5s 停发（cloud_adapter ODOM_TIMEOUT 同语义）。
"""
import math
import random

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray

from zx2026_common import config as cfg
from zx2026_common.scene import Scene

ODOM_TIMEOUT = 0.5  # s，odom 断流停发（同 cloud_adapter）


def make_depth_cfg(settings_depth):
    """sim_settings.sensor.depth 段 → 渲染参数 dict（缺省同 yaml 注释）。"""
    c = settings_depth or {}
    return {
        "enabled": bool(c.get("enabled", False)),
        "fps": float(c.get("fps", 15.0)),
        "width": int(c.get("width", 480)),
        "height": int(c.get("height", 270)),
        "hfov_deg": float(c.get("hfov_deg", 87.0)),
        "vfov_deg": float(c.get("vfov_deg", 58.0)),
        "max_range": float(c.get("max_range", 10.0)),
        "noise_std": float(c.get("noise_std", 0.02)),
        "min_range": float(c.get("min_range", 0.2)),
        "dropout": float(c.get("dropout", 0.02)),
        "render_crowns": bool(c.get("render_crowns", True)),
        # 树冠渲染形状（2026-10-02）："cylinder"=与真值同源的竖圆柱
        # （scene.raycast/判官/Gazebo 物理三处同款，见模块 docstring c 条）；
        # "sphere"=旧球近似——仅自检回放旧钉用，勿在运行档装订（球档在冠底
        # 高度带图上盲，是 diag24 冠撞图像读数乐观的病灶）。键缺失→cylinder：
        # 渲染器对齐真值口径不是"新行为"。
        "crown_shape": str(c.get("crown_shape", "cylinder")),
        "render_boxes": bool(c.get("render_boxes", False)),
        # 树级包围球预筛（2026-10-01）：每棵候选树先切一次射线子集，干/冠/枝
        # 的掩膜与求交全在子集上做。**纯提速、逐位无损**（见 _tree_bound 与
        # render_depth 的 g 条）；false = 子集退化为全幅 = 逐位回到改前算路。
        # 代码缺省 True：它不是"新行为"，而是把旧行为原样算得更快。
        "tree_prefilter": bool(c.get("tree_prefilter", True)),
        "cam_pitch_deg": float(c.get("cam_pitch_deg", 0.0)),
    }


def intrinsics(depth_cfg):
    """(fx, fy, cx, cy)：各轴 FOV 精确导出（见模块 docstring，fx≠fy 合法）。"""
    w, h = float(depth_cfg["width"]), float(depth_cfg["height"])
    fx = (w / 2.0) / math.tan(math.radians(depth_cfg["hfov_deg"]) / 2.0)
    fy = (h / 2.0) / math.tan(math.radians(depth_cfg["vfov_deg"]) / 2.0)
    return fx, fy, w / 2.0, h / 2.0


def cam_basis(yaw, pitch):
    """(right, down, fwd) 正交基（世界系）。right 同 camera_render 口径
    （sin(yaw), -cos(yaw), 0）；pitch 正=前向轴上仰。down=相机光学系 y 轴
    （REP-103 口径：y 朝下、row 0=画面顶），图像 v=(row-cy)/fy 增大=画面
    下方；(rgt, down, fwd) 右手系。"""
    cp, sp = math.cos(pitch), math.sin(pitch)
    fwd = np.array([cp * math.cos(yaw), cp * math.sin(yaw), sp])
    rgt = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    dwn = np.cross(fwd, rgt)  # pitch=0 时 = (0,0,-1)
    return rgt, dwn, fwd


def ray_dirs(depth_cfg):
    """相机系单位射线 (N,3) float32（N=H*W，行主序 v 主循环）+ 每像素
    1/|cam_vec|（=单位化方向与 fwd 的点积，Z 深度换算用）。固定内参只算一次。"""
    w, h = depth_cfg["width"], depth_cfg["height"]
    fx, fy, cx, cy = intrinsics(depth_cfg)
    us = (np.arange(w, dtype=np.float64) - cx) / fx          # (W,)
    vs = (np.arange(h, dtype=np.float64) - cy) / fy          # (H,)
    u = np.broadcast_to(us, (h, w))
    v = np.broadcast_to(vs.reshape(h, 1), (h, w))
    d = np.stack([u, v, np.ones((h, w))], axis=2).reshape(-1, 3)  # (N,3)
    n = np.linalg.norm(d, axis=1, keepdims=True)
    return (d / n).astype(np.float32), (1.0 / n).astype(np.float32).ravel()


def _sphere_mask(o, dirs, center, radius):
    """角半径包围球掩膜：射线指向以 center 为心 radius 球内 → True。
    机在球内时全 True。mask 全 O(N) 但仅 3 次乘加，子集求交才是重活。"""
    u = center - o
    dist = float(np.linalg.norm(u))
    if dist <= radius:
        return np.ones(dirs.shape[0], dtype=bool)
    cos_a = math.sqrt(max(0.0, 1.0 - (radius / dist) ** 2))
    return dirs @ (u / dist) > cos_a


_TREE_PAD = 1e-3  # m，树级包围球半径冗余（见 _tree_bound）


def _sub_idx(tsub, mask):
    """子集掩膜 → 全幅像素索引。tsub=None 表示全幅（tree_prefilter 关），
    此时表达式与改前的 np.nonzero(mask)[0] 逐位相同。"""
    return np.nonzero(mask)[0] if tsub is None else tsub[mask]


def _crown_prim(ob):
    """冠圆柱原语 (cx, cy, r, z0, z1, r_bb)（2026-10-02 与真值同源）。

    半径/高度口径逐字同 scene._crown_z_range + geo.ray_cylinder：
    crown_h 给包络高（杨树 table）→ z∈[crown_z-h/2, crown_z+h/2]；
    crown_h=None 旧语义 → z∈[crown_z-r, crown_z+r]（球冠包络也按竖圆柱
    求交，raycast 同款）。r_bb=sqrt(r²+(span/2)²) 为圆柱外包球半径——
    sqrt≥max(r, span/2) ⇒ 同时含 sphere 回放档的球，故 tree_prefilter 的
    外包球对两档都成立（逐位无损前提不变）。本函数与 _tree_bound 的冠项、
    render_depth 的冠块必须逐字一致（两处口径分家=历史上"冻结 r_crown"
    同类事故的温床）。
    """
    r = float(ob.crown_r or 0.0)
    zc = float(ob.crown_z or 0.0)
    if ob.crown_h:
        h = float(ob.crown_h)
        z0, z1 = zc - h / 2.0, zc + h / 2.0
    else:
        z0, z1 = zc - r, zc + r
    r_bb = math.sqrt(r * r + ((z1 - z0) / 2.0) ** 2)
    return float(ob.cx), float(ob.cy), r, z0, z1, r_bb


def _tree_bound(ob, brs, gz_o):
    """树的保守外包球 (center, radius) + 各原语包围球 (C, R)。

    各原语包围球 (c_i, r_i) → 合并 AABB → 心 C=AABB 心 →
    radius = max_i(|c_i-C| + r_i) + _TREE_PAD。

    该球**严格包含**任一原语的包围球（PAD 项），故"枝掩膜 True ⇒ 树掩膜
    True"在浮点下也成立——掠射切线上不会因 `dirs @ u_hat > cos_a` 的浮点
    翻转而丢掉一个像素。这是 tree_prefilter 逐位无损的前提；PAD 只多放
    几条射线进子集，它们照常走精确求交、无命中，输出不变。

    C/R 按 **干、冠（仅当 r_crown>0）、枝***的顺序返回，供调用方做逐原语
    视锥剔除（_frustum_visible）；render_depth 里靠 `nhead` 对齐同一顺序。

    原语包围球口径必须与 render_depth 的 b/c/d 三步逐字一致（干=干高中点、
    半径 sqrt(r²+(h/2)²)；冠=圆柱外包球（心=冠包络中点、半径 _crown_prim
    的 r_bb=sqrt(r²+(span/2)²)，≥max(r,span/2) 故同时含 sphere 回放档的
    球）；枝=轴线中点、半径 |AB|/2+br），否则外包球不成立。
    """
    cs = []
    rs = []
    th = float(ob.trunk_h or 0.0)
    tr = float(ob.trunk_r or 0.0)
    cs.append((float(ob.cx), float(ob.cy), gz_o + th / 2.0))
    rs.append(math.sqrt(tr * tr + (th / 2.0) ** 2))
    crown_h = float(ob.crown_h) if ob.crown_h is not None else 0.0
    r_crown = max(float(ob.crown_r or 0.0), crown_h / 2.0)
    if r_crown > 0.0:
        # 冠项=圆柱外包球（2026-10-02 球→柱后与 render_depth 冠块同口径）；
        # 心=(z0+z1)/2=crown_z、半径 r_bb（见 _crown_prim docstring）。
        ccx, ccy, _cr, cz0, cz1, cr_bb = _crown_prim(ob)
        cs.append((ccx, ccy, (cz0 + cz1) / 2.0))
        rs.append(cr_bb)
    for (sx, sy, sz, ex, ey, ez, br) in brs:
        cs.append(((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0))
        rs.append(math.hypot(ex - sx, ey - sy, ez - sz) / 2.0 + float(br))
    C = np.asarray(cs, dtype=np.float64)
    R = np.asarray(rs, dtype=np.float64)
    ctr = ((C - R[:, None]).min(axis=0) + (C + R[:, None]).max(axis=0)) / 2.0
    rad = float(np.max(np.linalg.norm(C - ctr, axis=1) + R)) + _TREE_PAD
    return ctr, rad, C, R


_CULL_PAD = 1e-2  # m，视锥剔除的保守冗余（见 _frustum_visible）
_EMPTY_MASK = np.zeros(0, dtype=bool)  # 原语被视锥剔除时的空掩膜（.any() 恒 False）


def _frustum_planes(depth_cfg, rgt, dwn, fwd):
    """视锥四个侧面的**内法线** (4,3)，过机位（世界系）。

    相机系下四侧面 u=±um、v=±vm（um=tan(hfov/2)、vm=tan(vfov/2)，由
    intrinsics 精确导出）。侧面的内法线在 (rgt,dwn,fwd) 基下为
    (-1,0,um)、(1,0,um)、(0,-1,vm)、(0,1,vm) 各自单位化——校验：内法线对
    前向轴 (0,0,1) 的点积 = um/sqrt(1+um²) > 0，即前向在视锥内。

    只在树级预筛开启时调用（每帧一次），平面与射线方向都以机位为原点，
    故原语侧用 (c - o) 直接代入，无需平移。
    """
    fx, fy, cx, cy = intrinsics(depth_cfg)
    um, vm = cx / fx, cy / fy
    su, sv = math.sqrt(1.0 + um * um), math.sqrt(1.0 + vm * vm)
    loc = ((-1.0 / su, 0.0, um / su), (1.0 / su, 0.0, um / su),
           (0.0, -1.0 / sv, vm / sv), (0.0, 1.0 / sv, vm / sv))
    return np.array([a * rgt + b * dwn + c * fwd for a, b, c in loc])


def _frustum_visible(fwd, planes, D, R, t_max):
    """D=(M,3) 原语中心相对机位、R=(M,) 包围球半径 → keep(M,) bool。
    False = 该原语整个在视锥外/超量程/在身后，**不可能**被任何画面射线命中。

    三条判据（都用半径把球整体算进去，故保守）：
      · 身后  fwd·d + r < -PAD          → 全在相机背后；
      · 超量程 fwd·d - r > t_max + PAD  → 沿射线走 t>t_max，被 _solve_min_t 滤掉
        （球上任意点 p 有 fwd·(p-o) > t_max，而 t·(fwd·d̂) = fwd·(p-o) 且
        fwd·d̂ ≤ 1 ⇒ t > t_max）；
      · 侧外  任一 n_i·d < -(r+PAD)     → 球整体在半空间外；画面射线全程在该
        半空间内侧（n_i·(t d̂) ≥ 0），故永不进入球的所在侧。
    PAD 兜住 float32 射线方向相对 float64 平面的 ~1e-7 相对误差（10m 处
    ~1μm，比 PAD 小四个数量级），确保"剔除 ⇒ 无像素差"。

    判据只做**排除**，永不引入命中，故剔除与否不改变输出（自测 [8] 断言
    on/off 逐字节一致）。
    """
    f = D @ fwd
    keep = (f + R >= -_CULL_PAD) & (f - R <= t_max + _CULL_PAD)
    if not keep.any():
        return keep
    p = D @ planes.T                                  # (M,4)
    keep &= ~np.any(p < -(R + _CULL_PAD)[:, None], axis=1)
    return keep


def _solve_min_t(dirs_sub, quad_a, quad_b, quad_c, t_max, eps=1e-6):
    """单位化射线二次方程 a t^2 + b t + c = 0 取近根，过滤 t<=eps / >t_max /
    无实根；返回 (ok_mask, t数组)（无有效解时 ok 全 False）。a=0（轴向掠射）
    丢弃可接受——包围球掩膜已保证正对子集非空。"""
    disc = quad_b * quad_b - 4.0 * quad_a * quad_c
    ok = (quad_a > eps) & (disc >= 0.0)
    if not ok.any():
        return ok, None
    sq = np.sqrt(np.where(ok, disc, 0.0))
    t = (-quad_b - sq) / (2.0 * np.where(ok, quad_a, 1.0))
    ok &= (t > eps) & (t <= t_max)
    return ok, t


def render_depth(scene, pos, yaw, depth_cfg, rng, cam_dirs=None, inv_len=None,
                 pitch=None):
    """渲染一帧前视深度图。

    scene: zx2026_common.scene.Scene（真值几何）
    pos/yaw: 机体真值位姿（世界系）
    depth_cfg: make_depth_cfg 产物
    rng: random.Random（每机独立，种子=run_seed+id*7919；每帧消耗 1 个
         randint 派生 numpy 流，dropout/噪声同流，帧序确定性）
    cam_dirs/inv_len: ray_dirs 产物缓存（None 则内部现算）
    pitch: 俯仰角 rad（None 则取 cfg cam_pitch_deg）
    render_boxes（depth_cfg 内，缺省关）：盒障碍 AABB 入画开关（见模块
    docstring e 条；关时盒路径零进入，旧图逐位不变）
    返回 (H,W) uint16 毫米；无命中=max_range 哨兵，无效=0。
    """
    h, w = depth_cfg["height"], depth_cfg["width"]
    t_max = depth_cfg["max_range"]
    if cam_dirs is None:
        cam_dirs, inv_len = ray_dirs(depth_cfg)
    if pitch is None:
        pitch = math.radians(depth_cfg["cam_pitch_deg"])
    rgt, dwn, fwd = cam_basis(yaw, pitch)
    o = np.asarray(pos, dtype=np.float64)
    # 世界系射线：d = x_cam*right + y_cam*up + z_cam*fwd（正交基无需再归一化）
    dirs = cam_dirs @ np.stack([rgt, dwn, fwd]).astype(np.float32)  # (N,3)
    dz = dirs[:, 2].astype(np.float64)
    df = inv_len                                  # dot(d_unit, fwd) = 1/|cam|

    best = np.full(h * w, t_max, dtype=np.float64)

    # ---- a) 地面 z=ground_z ----
    gz = scene.venue["ground_z"]
    m = dz < -1e-6
    if m.any():
        t = (gz - o[2]) / dz[m]
        mm = (t > 1e-6) & (t <= t_max)
        if mm.any():
            idx = np.nonzero(m)[0][mm]
            best[idx] = np.minimum(best[idx], t[mm])

    # ---- 预筛：机位水平距离 <= max_range + margin 的树 ----
    gz_o = scene.venue["ground_z"]
    prefilt = bool(depth_cfg.get("tree_prefilter", True))
    cand = []
    box_cand = []  # 盒障碍候选 [(lo,hi),...]（render_boxes 开时收集）
    for ob in scene.obstacles:
        if ob.kind != "tree" or ob.cx is None:
            # 盒障碍（网 kind=net/围栏/灌木等 AABB）：render_boxes 开时入画
            # （2026-09-29 T1，见模块 docstring e 条）。标量预筛：盒面最近点
            # 距离 > 量程整盒跳过——单位射线命中点必在盒面上，命中 t 有
            # 下界=最近点距离，故超量程盒不可能产生任何入画命中，远端
            # 围栏零逐像素成本。
            if depth_cfg.get("render_boxes", False):
                lo = np.asarray(ob.lo, dtype=np.float64)
                hi = np.asarray(ob.hi, dtype=np.float64)
                nearest = np.minimum(np.maximum(o, lo), hi)
                if float(np.linalg.norm(nearest - o)) <= t_max:
                    box_cand.append((lo, hi))
            continue
        dxy = math.hypot(ob.cx - o[0], ob.cy - o[1])
        crown_r = float(ob.crown_r or 0.0)
        crown_h = float(ob.crown_h) if ob.crown_h is not None else 0.0
        r_crown = max(crown_r, crown_h / 2.0)
        margin = float(ob.trunk_r) + r_crown
        if dxy > t_max + margin:
            continue
        brs = list(ob.branches or [])
        if brs:
            far = max(math.hypot(b[0] - b[3], b[1] - b[4]) for b in brs)
            if dxy > t_max + margin + far:
                brs = []
        if prefilt:
            tctr, tR, C, R = _tree_bound(ob, brs, gz_o)
        else:
            tctr = tR = C = R = None
        cand.append((ob, brs, tctr, tR, C, R))

    # ---- b) 树干圆柱 + c) 树冠球 + d) 树枝有限圆柱 ----
    # 树级预筛（两级）：
    #   ① 视锥剔除——整树在视锥外/超量程/身后则连掩膜都不算（原预筛只看水平
    #      距离，相机身后的树照样吃一整套全幅掩膜）；
    #   ② 掩膜定子集——一次全幅掩膜 → (tsub, dsub)，b/c/d 的掩膜与求交全在
    #      子集上做，命中索引经 _sub_idx(tsub, mask) 映射回全幅。
    # 再对每个原语做一次视锥剔除（keep），跳过画面之上/之外的干枝（真枝在
    # z 4.2-8.3，平视 58° 垂直视场下大量在画面上缘之外）。
    # prefilt 关时 tsub=None / dsub=dirs（全幅）且无任何剔除，表达式与改前逐位同算。
    planes = _frustum_planes(depth_cfg, rgt, dwn, fwd) if prefilt else None
    for ob, brs, tctr, tR, C, R in cand:
        if prefilt:
            if not bool(_frustum_visible(fwd, planes, tctr - o, np.array([tR]),
                                         t_max)[0]):
                continue
            tmask = _sphere_mask(o, dirs, tctr, tR)
            if not tmask.any():
                continue
            tsub = np.nonzero(tmask)[0]
            dsub = dirs[tsub]
        else:
            tsub = None
            dsub = dirs
        # 冠参数逐树重算（verify blocker 修复 2026-09-29：原版沿用预筛循环
        # 最后一棵候选树的冻结 r_crown，13 候选 12 错配、58.7% 像素被幻影
        # 冠面污染——冠块三处引用全部受害）
        crown_r = float(ob.crown_r or 0.0)
        crown_h = float(ob.crown_h) if ob.crown_h is not None else 0.0
        r_crown = max(crown_r, crown_h / 2.0)
        # 逐原语视锥剔除（C/R 由 _tree_bound 按 干、冠(仅当 r_crown>0)、枝 的顺序
        # 返回，nhead 即"头两个原语"是否含冠的边界）。被剔除的原语用空掩膜短路，
        # 与改前逐位同算的路径不受影响（prefilt 关时 do_* 恒真、brs_vis 即 brs）。
        if prefilt:
            keep = _frustum_visible(fwd, planes, C - o, R, t_max)
            nhead = 2 if r_crown > 0.0 else 1
            do_trunk = bool(keep[0])
            do_crown = bool(keep[1]) if nhead == 2 else False
            brs_vis = [b for b, k in zip(brs, keep[nhead:]) if bool(k)]
        else:
            do_trunk = do_crown = True
            brs_vis = brs
        # 树干：包围球（干高中点、半径 sqrt(r^2+(h/2)^2)）掩膜 → 子集精确求交
        zm = gz_o + ob.trunk_h / 2.0
        ctr = np.array([ob.cx, ob.cy, zm])
        r_bb = math.sqrt(ob.trunk_r ** 2 + (ob.trunk_h / 2.0) ** 2)
        mask = _sphere_mask(o, dsub, ctr, r_bb) if do_trunk else _EMPTY_MASK
        if mask.any():
            d = dsub[mask].astype(np.float64)
            dxm = o[0] - ob.cx
            dym = o[1] - ob.cy
            a = d[:, 0] ** 2 + d[:, 1] ** 2
            b = 2.0 * (dxm * d[:, 0] + dym * d[:, 1])
            c = dxm * dxm + dym * dym - ob.trunk_r ** 2
            ok, t = _solve_min_t(d, a, b, c, t_max)
            if ok.any():
                zh = o[2] + t[ok] * d[ok, 2]
                inz = (zh >= gz_o) & (zh <= gz_o + ob.trunk_h)
                if inz.any():
                    idx = _sub_idx(tsub, mask)[ok][inz]
                    best[idx] = np.minimum(best[idx], t[ok][inz])
        # 树冠：竖直圆柱（2026-10-02 球→柱，见模块 docstring c 条与
        # _crown_prim；球近似档 crown_shape="sphere" 仅回放旧钉）。
        # 冠门（2026-10-02）：与 scene.raycast / collides_xy / world_builder
        # 同口径——只画「物理存在」的冠（crown_collidable）。杨树档 10-02 晚
        # 裁决「冠物理、视觉都删」后该键为 False ⇒ **冠不入画**（上半天「冠=
        # 物理存在」档照画，10-01 去碰撞冠档亦不画）。
        # 门保留为语义闸：非碰撞冠（若有）不入深度图，防「物理/点云/A* 都不
        # 存在」的冠画进 DeFoP 监督器唯一感知源（幻影墙，W5 同型）。
        # getattr 兜底程序化树（与 world_builder:176 同款）。
        if (depth_cfg["render_crowns"] and r_crown > 0.0
                and getattr(ob, "crown_collidable", True)):
            if depth_cfg.get("crown_shape", "cylinder") == "sphere":
                # 旧球近似（回放杠杆）：球心=(cx,cy,crown_z)、半径=
                # max(crown_r, crown_h/2)——球底恰=包络底，冠底以下机高带
                # 图上半宽归零（冠撞无信号病灶，见 docstring c 条）。
                cctr = np.array([ob.cx, ob.cy, ob.crown_z])
                mask = _sphere_mask(o, dsub, cctr, r_crown) if do_crown else _EMPTY_MASK
                if mask.any():
                    d = dsub[mask].astype(np.float64)
                    u = o - cctr
                    a = np.ones(d.shape[0])          # 单位射线
                    b = 2.0 * (d @ u)
                    c = float(u @ u) - r_crown ** 2
                    ok, t = _solve_min_t(d, a, b, c, t_max)
                    if ok.any():
                        idx = _sub_idx(tsub, mask)[ok]
                        best[idx] = np.minimum(best[idx], t[ok])
            else:
                # 圆柱侧面（同 geo.ray_cylinder）：外包球掩膜（r_bb 口径见
                # _crown_prim）→ 二次方程两根全判——相机在柱面足迹内是常态
                # （冠下穿行），此时近根在身后、远根=内壁命中，不许像树干
                # _solve_min_t 那样丢远根 → 命中点 z 带过滤。
                ccx, ccy, cr, cz0, cz1, cr_bb = _crown_prim(ob)
                cctr = np.array([ccx, ccy, (cz0 + cz1) / 2.0])
                mask = _sphere_mask(o, dsub, cctr, cr_bb) if do_crown else _EMPTY_MASK
                if mask.any():
                    d = dsub[mask].astype(np.float64)
                    dxm = o[0] - ccx
                    dym = o[1] - ccy
                    a = d[:, 0] ** 2 + d[:, 1] ** 2
                    b = 2.0 * (dxm * d[:, 0] + dym * d[:, 1])
                    c = dxm * dxm + dym * dym - cr ** 2
                    disc = b * b - 4.0 * a * c
                    ok = (a > 1e-12) & (disc >= 0.0)
                    if ok.any():
                        sq = np.sqrt(np.where(ok, disc, 0.0))
                        den = 2.0 * np.where(ok, a, 1.0)
                        t1 = (-b - sq) / den
                        t2 = (-b + sq) / den
                        zh1 = o[2] + t1 * d[:, 2]
                        zh2 = o[2] + t2 * d[:, 2]
                        v1 = ok & (t1 > 1e-6) & (zh1 >= cz0) & (zh1 <= cz1)
                        v2 = ok & (t2 > 1e-6) & (zh2 >= cz0) & (zh2 <= cz1)
                        tsel = np.where(v1, t1, t2)   # t1<t2：v1 命中即取近
                        vsel = (v1 | v2) & (tsel <= t_max)
                        if vsel.any():
                            idx = _sub_idx(tsub, mask)[vsel]
                            best[idx] = np.minimum(best[idx], tsel[vsel])
        # 树枝：世界系线段 → 有限圆柱（同 geo.ray_finite_cylinder 模型）
        for (sx, sy, sz, ex, ey, ez, br) in brs_vis:
            A = np.array([sx, sy, sz])
            B = np.array([ex, ey, ez])
            mvec = B - A
            L2 = float(mvec @ mvec)
            if L2 < 1e-12:
                continue
            mid = (A + B) / 2.0
            r_bb = math.sqrt(L2) / 2.0 + br
            mask = _sphere_mask(o, dsub, mid, r_bb)
            if not mask.any():
                continue
            d = dsub[mask].astype(np.float64)
            wv = o - A
            dm = d @ mvec
            wm = float(wv @ mvec)
            a = 1.0 - (dm * dm) / L2
            b = 2.0 * ((d @ wv) - dm * (wm / L2))
            c = float(wv @ wv) - (wm * wm) / L2 - br * br
            ok, t = _solve_min_t(d, a, b, c, t_max)
            if ok.any():
                s_ax = (t[ok] * dm[ok] + wm) / L2
                inseg = (s_ax >= 0.0) & (s_ax <= 1.0)
                if inseg.any():
                    idx = _sub_idx(tsub, mask)[ok][inseg]
                    best[idx] = np.minimum(best[idx], t[ok][inseg])

    # ---- e) 盒障碍 AABB（slab 法，向量化；render_boxes 门控，缺省关） ----
    # 模型同 geo.ray_aabb（scene.raycast 对盒的真值口径，逐轴 1/d 取倒再乘
    # 同型）：t_near=max 轴入点、t_far=min 轴出点，t_near<=t_far 命中取
    # t_near。轴序=边长升序（薄轴先行：背对薄墙第一轴即整批判死）；t_near
    # 单调增 / t_far 单调减 → 逐轴中途 (t_near>t_far | t_far<=eps | t_near>
    # t_max) 全灭即整盒出局，存活子集逐轴收缩（每轴只 gather 一列，拒绝
    # 早的盒不做 (M,3) 整块拷贝）。轴向平行（d_i==0；极小 |d| 除出 ±inf/
    # huge t 与平行同判）按几何展开：原点在带内=该轴不约束、带外=判死。
    # 算术全程 float32（dirs 本征 float32；t 相对误差 ~6e-8 → 3m 处 0.2μm，
    # 毫米量化不可见，真值对质 ±1mm 容差内），仅存活命中升 float64 写 best。
    # 命中判据 t_near>eps 与树干/树冠/树枝同口径（原点在盒内不产生命中；
    # geo.ray_aabb 此情形返回 t=0，真值对质仅在盒外位姿做）。t 写入同一
    # best 缓冲 → 与树/地 min-t 合流，Z 深度/哨兵/0/dropout 全走既有 f) 步。
    for lo, hi in box_cand:
        ctr = (lo + hi) / 2.0
        r_bb = float(np.linalg.norm(hi - lo)) / 2.0      # 角包围球=半对角
        mask = _sphere_mask(o, dirs, ctr, r_bb)
        if not mask.any():
            continue
        sub = np.nonzero(mask)[0]
        t_near = np.full(sub.shape[0], -np.inf, dtype=np.float32)
        t_far = np.full(sub.shape[0], np.inf, dtype=np.float32)
        for ax in np.argsort(hi - lo):
            dv = dirs[sub, ax]                           # float32 列 gather
            with np.errstate(divide="ignore"):
                ood = np.reciprocal(dv)                  # dv==0 → ±inf（IEEE）
            t1 = (lo[ax] - o[ax]) * ood
            t2 = (hi[ax] - o[ax]) * ood
            if o[ax] == lo[ax] or o[ax] == hi[ax]:
                # 原点恰在盒面（0×inf=nan）：平行且带内，该轴不约束
                nan = np.isnan(t1) | np.isnan(t2)
                if nan.any():
                    t1 = np.where(nan, np.float32(-np.inf), t1)
                    t2 = np.where(nan, np.float32(np.inf), t2)
            tlo = np.minimum(t1, t2)
            thi = np.maximum(t1, t2)
            t_near = np.maximum(t_near, tlo, out=t_near)
            t_far = np.minimum(t_far, thi, out=t_far)
            alive = (t_near <= t_far) & (t_far > 1e-6) & (t_near <= t_max)
            if not alive.any():
                t_near = None
                break
            if not alive.all():
                sub = sub[alive]
                t_near = t_near[alive]
                t_far = t_far[alive]
        if t_near is None:
            continue
        ok = t_near > 1e-6
        if ok.any():
            best[sub[ok]] = np.minimum(best[sub[ok]], t_near[ok])

    # ---- f) t → Z 深度（沿光轴分量）----
    Z = best * df

    # ---- 噪声（只打真命中像素；单 RandomState 双用途保帧序确定性）----
    # 真命中判据在 t 域（best<t_max）：Z 域会把无命中像素（best=t_max 哨兵、
    # Z=t_max*df<max_range）误判成命中吃噪声/丢失，哨兵值被拉成 t_max*df
    # （角部像素 ~6.7m 假墙）——2026-09-29 渲染对质修。
    hit = best < t_max
    if hit.any():
        rs = np.random.RandomState(rng.randint(0, 2 ** 31 - 1))
        if depth_cfg["noise_std"] > 0:
            Z[hit] += rs.normal(0.0, depth_cfg["noise_std"], size=int(hit.sum()))
        if depth_cfg["dropout"] > 0:
            drop = hit & (rs.random_sample(Z.shape[0]) < depth_cfg["dropout"])
            Z[drop] = 0.0
    # 无命中 → max_range 哨兵（不参与噪声；恰=max_range 不被下行无效化清零）
    Z[~hit] = t_max
    # 无效化：<min_range 或 >max_range → 0
    Z[(Z < depth_cfg["min_range"]) | (Z > t_max)] = 0.0

    mm = np.rint(Z * 1000.0)
    mm[mm < 0.0] = 0.0
    mm[mm > 65535.0] = 65535.0
    return mm.astype(np.uint16).reshape(h, w)


def project_point(pos, yaw, depth_cfg, P, pitch=None):
    """世界系点 P → (u, v, Z)。测试/对质用，与 render_depth 同一相机模型。"""
    if pitch is None:
        pitch = math.radians(depth_cfg["cam_pitch_deg"])
    fx, fy, cx, cy = intrinsics(depth_cfg)
    rgt, dwn, fwd = cam_basis(yaw, pitch)
    v = np.asarray(P, dtype=np.float64) - np.asarray(pos, dtype=np.float64)
    zc = float(v @ fwd)
    if zc <= 1e-6:
        return None
    u = cx + fx * float(v @ rgt) / zc
    vv = cy + fy * float(v @ dwn) / zc
    return u, vv, zc


class DepthSimNode:
    def __init__(self):
        rospy.init_node("depth_sim_node", anonymous=True)
        self._active = False
        settings = cfg.load("sim_settings.yaml")
        dcfg_raw = settings.get("sensor", {}).get("depth", None)
        if not dcfg_raw or not bool(dcfg_raw.get("enabled", False)):
            rospy.loginfo("depth_sim_node: disabled (sensor.depth 缺失或 "
                          "enabled=false，自门控)")
            return
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id
        self.cfg = make_depth_cfg(dcfg_raw)
        self.scene = Scene()
        self.cam_dirs, self.inv_len = ray_dirs(self.cfg)
        self.rng = random.Random(int(settings.get("run_seed", 42))
                                 + self.drone_id * 7919)

        self.odom_pos = (0.0, 0.0, 1.0)
        self.odom_yaw = 0.0
        self._last_odom_t = None   # 到达时刻（rospy.Time），断流判据

        self.pub_img = rospy.Publisher(ns + "/depth/image_raw", Image,
                                       queue_size=2)
        self.pub_info = rospy.Publisher(ns + "/depth/camera_info", CameraInfo,
                                        queue_size=1, latch=True)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.loginfo("depth_sim_node: drone %d (%dx%d@%.0ffps hfov=%.0f "
                      "vfov=%.0f max=%.1fm)", self.drone_id, self.cfg["width"],
                      self.cfg["height"], self.cfg["fps"], self.cfg["hfov_deg"],
                      self.cfg["vfov_deg"], self.cfg["max_range"])
        self._active = True

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._last_odom_t = rospy.Time.now()

    def run(self):
        if not self._active:
            return
        self._publish_info()
        rate = rospy.Rate(self.cfg["fps"])
        while not rospy.is_shutdown():
            fresh = (self._last_odom_t is not None
                     and (rospy.Time.now() - self._last_odom_t).to_sec()
                     <= ODOM_TIMEOUT)
            if fresh:
                img = render_depth(self.scene, self.odom_pos, self.odom_yaw,
                                   self.cfg, self.rng, self.cam_dirs,
                                   self.inv_len)
                self._publish_img(img)
            rate.sleep()

    def _publish_img(self, img):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "drone_%d/depth_optical" % self.drone_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

    def _publish_info(self):
        fx, fy, cx, cy = intrinsics(self.cfg)
        info = CameraInfo()
        info.header.frame_id = "drone_%d/depth_optical" % self.drone_id
        info.width = self.cfg["width"]
        info.height = self.cfg["height"]
        info.distortion_model = "plumb_bob"
        info.D = Float64MultiArray(data=[0.0] * 5)   # 大写：精简版消息无小写畸变字段
        info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)


if __name__ == "__main__":
    try:
        node = DepthSimNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
