#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_common.py — 纯 numpy 共享层：帧手术 / 几何监督器 / 基元库 / 安全包络。

设计目的：把所有可单元测试的数学从 ROS 节点里剥出来（不 import rospy/tf/torch，
defop_cpn_node.py 与 /tmp 单元测试共用本模块）。

vendored 出处（版权与许可归属原仓库，BSD-3-Clause）：
  https://github.com/guglielmo610/DeFoP
  - src/planning/ORACLE/evaluate/evaluate.py:
      compute_margin / compute_vertical_margin / filter_actions_by_grid_3 /
      choose_dead_end_rotation / calculate_yaw_ctrl / wrap_yaw /
      基元库构建(run() L1046-1065) / UT sigma 状态(L1156-1179) /
      CPN 打分管线(L1232-1271)
  改动：config.py 模块级常量改为函数入参；方法改模块函数；逻辑逐字保留。

本文件新增（arena 契约，DeFoP 原版没有）：
  - quat_to_ypr / body_vel_to_world / world_vel_to_psi：帧手术
  - apply_safety_envelope：水平限幅 + 高度带 z 钳位
"""
import math
import numpy as np

# ============================ 角度/四元数（新增） ============================

def wrap_angle(a):
    """wrap 到 (-pi, pi]。同 DeFoP evaluate.py wrap_yaw。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_ypr(x, y, z, w):
    """四元数 -> (yaw, roll, pitch)，ZYX 航空序。

    yaw 公式与 vel_bridge 契约一致：atan2(2(wz+xy), 1-2(y^2+z^2))。
    """
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_p = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_p)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw, roll, pitch


# ============================ 帧手术（新增，arena 契约） ============================
#
# DeFoP 真机指令（evaluate.py L1487-1490 + realtime_ros_wrapper_infogain_ardupilot
# .step()）：机体系 Twist
#     linear.x = vel_x            （基元前向速度，CMD_VELOCITY=1.0）
#     linear.y = vel_y = 0.5*tanh(yaw_rel)   （侧向，y 左正）
#     linear.z = vel_z            （垂向）
#     angular.z = yaw_ctrl        （yaw 速率 = clip(k*yaw_rel, ±pi/4)，见
#                                  calculate_yaw_ctrl——wrapper 里 "reference
#                                  yaw angle" 注释是陈旧的，yaw_in_spot 才传角度）
# arena 契约（vel_bridge.py 已核实）：/drone_<id>/vel_cmd 世界系 ENU
#     linear.xyz = 世界系速度，angular.z = yaw_dot。
#
# 正变换（指令侧）：[vx_w, vy_w] = Rz(yaw)·[vx, vy_b]，vz 直传。
# 逆变换（状态侧）：DeFoP 的 velocity_W_psi = R_xy(roll,pitch)·v_body。
#   因 R_full = Rz(yaw)·Ry(pitch)·Rx(roll) 且 v_world = R_full·v_body，
#   故 velocity_W_psi = Rz(yaw)^T · v_world —— 即对世界系速度去 yaw 即可，
#   无需先转到机体系（对 sim 的 WorldLinearVel odom 逐字等价）。

def body_vel_to_world(vx, vy, yaw):
    """机体系 [vx, vy] -> 世界系 [vx_w, vy_w]。机体系前向 = 世界系 (cos yaw, sin yaw)。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


def world_vel_to_psi(vx_w, vy_w, vz_w, yaw):
    """世界系速度 -> yaw 对齐系（DeFoP CPN state 前三维语义）。

    等价于 DeFoP 的 R.from_euler('xy',[roll,pitch]) @ v_body：
    v_psi = Rz(-yaw) · v_world。
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w, vz_w


def apply_safety_envelope(vx_w, vy_w, vz_w, z_pos,
                          vel_max=1.2, z_min=0.3, z_max=4.5, vz_abs_max=1.0):
    """世界系速度安全包络（arena 契约，双保险外层）。

    - 水平模长 <= vel_max（比赛首飞档 1.2，vel_bridge 侧还有全 3D 限幅）；
    - 高度带 z 钳位 [z_min, z_max]：语义是"高度门"不是速度门——当前高度
      低于 z_min 时禁负 vz、高于 z_max 时禁正 vz（z_max=4.5 对齐 arena
      flight_z_ceiling，距 OVER_HEIGHT 5.0 保 0.5m）；
    - |vz| <= vz_abs_max。
    返回 (vx_w, vy_w, vz_w, notes:list[str])。
    """
    notes = []
    h = math.hypot(vx_w, vy_w)
    if h > vel_max > 0.0:
        k = vel_max / h
        vx_w, vy_w = vx_w * k, vy_w * k
        notes.append("h_clip")
    if vz_abs_max > 0.0:
        if vz_w > vz_abs_max:
            vz_w = vz_abs_max
            notes.append("vz_clip")
        elif vz_w < -vz_abs_max:
            vz_w = -vz_abs_max
            notes.append("vz_clip")
    if z_pos <= z_min and vz_w < 0.0:
        vz_w = 0.0
        notes.append("z_floor")
    if z_pos >= z_max and vz_w > 0.0:
        vz_w = 0.0
        notes.append("z_ceiling")
    return vx_w, vy_w, vz_w, notes


# ============================ 基元库（vendored） ============================

def build_action_library(num_vel_x, num_vel_z, num_yaw, action_horizon,
                         cmd_velocity, cam_pitch_deg, fov_h_deg, fov_v_deg,
                         pitch_rad=0.0):
    """构建 256 基元库 [vel_x, vel_z, yaw_rel]，逐字移植 evaluate.py run()。

    vel_z = cmd_velocity * tan(elevation角)，elevation 覆盖相机 vfov（随当前
    pitch 平移），中间一档强制 0；yaw_rel ∈ ±0.45*FOV_h。
    返回 (action_seq (N,horizon,3) float32, yaw_lib (num_yaw,), z_lib (num_vel_z,))。
    """
    fov_rad = np.deg2rad(fov_h_deg)
    fov_pitch = np.deg2rad(fov_v_deg)
    vel_x_lib = np.array([cmd_velocity], dtype=np.float32)

    angle_z_limit = np.array([
        -np.deg2rad(cam_pitch_deg) - pitch_rad - fov_pitch / 2.0,
        -np.deg2rad(cam_pitch_deg) - pitch_rad + fov_pitch / 2.0])
    angle_z_limit = np.clip(angle_z_limit, np.deg2rad(-80.0), np.deg2rad(80.0))
    cmd_angle_z_lib = np.linspace(angle_z_limit[0], angle_z_limit[1],
                                  num_vel_z, dtype=np.float32)
    replace_idx = int(num_vel_z / 2) - 1
    cmd_angle_z_lib[replace_idx] = 0.0  # 保证有一档 vz=0
    cmd_velocity_z_lib = np.array([], dtype=np.float32)
    for _ in range(num_vel_x):
        cmd_velocity_z_lib = np.append(cmd_velocity_z_lib,
                                       vel_x_lib[0] * np.tan(cmd_angle_z_lib))
    cmd_vel_z = np.repeat(cmd_velocity_z_lib, num_yaw)
    cmd_vel_z = np.reshape(cmd_vel_z, (num_vel_x * num_vel_z * num_yaw, 1))

    cmd_vel_x = np.repeat(vel_x_lib, num_vel_z * num_yaw)
    cmd_vel_x = np.reshape(cmd_vel_x, (num_vel_x * num_vel_z * num_yaw, 1))

    cmd_yaw_relative_lib = np.linspace(-0.45 * fov_rad, 0.45 * fov_rad,
                                       num_yaw, dtype=np.float32)
    cmd_yaw_relative = np.tile(cmd_yaw_relative_lib, num_vel_x * num_vel_z)
    cmd_yaw_relative = np.reshape(cmd_yaw_relative,
                                  (num_vel_x * num_vel_z * num_yaw, 1))

    action_seq = np.concatenate((cmd_vel_x, cmd_vel_z, cmd_yaw_relative), axis=1)
    action_seq = np.reshape(action_seq, (num_vel_x * num_vel_z * num_yaw, 1, 3))
    action_seq = np.repeat(action_seq, action_horizon, axis=1)
    action_seq = np.ascontiguousarray(action_seq, dtype=np.float32)
    return action_seq, cmd_yaw_relative_lib, cmd_velocity_z_lib


def time_weight_matrix(action_horizon, time_weight_factor):
    """exp(-f*i) 时权，evaluate.py L188-190。"""
    w = np.zeros((1, action_horizon))
    for i in range(action_horizon):
        w[0, i] = np.exp(-time_weight_factor * i)
    return w


# ============================ 几何监督器（vendored） ============================

def compute_margin(r, d, fov_rad, num_yaw):
    """障碍在距离 d 处张角 2*atan(r/d) 需封锁的 yaw 扇区数。evaluate.py L352-360。"""
    delta_theta = fov_rad / num_yaw
    theta = 2 * np.arctan(r / d)
    return int(np.ceil((theta / 2) / delta_theta))


def compute_vertical_margin(r, d, fov_pitch, num_vel_z):
    """同上的垂直版。evaluate.py L346-350（r 固定 0.2 由调用方传入）。"""
    delta_phi = fov_pitch / num_vel_z
    theta = 2 * np.arctan(r / d)
    return int(np.ceil((theta / 2) / delta_phi))


def filter_actions_by_grid_3(depth_img, fov_yaw, fov_pitch,
                             cmd_yaw_relative, cmd_z_relative,
                             yaw_lib, z_lib,
                             num_yaw=32, num_vel_z=8,
                             min_dist=1.5, min_ratio=0.02, drone_radius=0.32):
    """扇区网格否决（向量化版），逐字移植 evaluate.py L529-623。

    depth_img: float32 米（或 uint16 毫米，函数内转换）。
    返回 (safe_actions_yaw, safe_actions_z, safe_mask (N,),
          blocked_with_margin (num_yaw,num_vel_z), margins_yaw, margins_z)。
    """
    if depth_img.dtype == np.uint16:
        depth_img = depth_img.astype(np.float32) / 1000.0  # mm -> m

    h, w = depth_img.shape

    obstacle_mask = (depth_img > 0) & (depth_img < min_dist)

    sector_w = w // num_yaw
    sector_h = h // num_vel_z

    col_ends = np.arange(0, w + 1, sector_w)
    if len(col_ends) > num_yaw + 1:
        col_ends = col_ends[:num_yaw + 1]
        col_ends[-1] = w

    row_ends = np.arange(0, h + 1, sector_h)
    if len(row_ends) > num_vel_z + 1:
        row_ends = row_ends[:num_vel_z + 1]
        row_ends[-1] = h

    blocked = np.zeros((num_yaw, num_vel_z), dtype=bool)
    margins_yaw = np.zeros((num_yaw, num_vel_z), dtype=int)
    margins_z = np.zeros((num_yaw, num_vel_z), dtype=int)

    for iy in range(num_yaw):
        col_start = col_ends[iy]
        col_end = col_ends[iy + 1]

        for iz in range(num_vel_z):
            row_start = row_ends[iz]
            row_end = row_ends[iz + 1]

            sector = obstacle_mask[row_start:row_end, col_start:col_end]
            ratio = np.sum(sector) / sector.size

            if ratio > min_ratio:
                blocked[iy, iz] = True

                sector_depth = depth_img[row_start:row_end, col_start:col_end]
                close_vals = sector_depth[sector]
                if close_vals.size > 0:
                    d = np.median(close_vals)
                    margins_yaw[iy, iz] = compute_margin(drone_radius, d,
                                                         fov_yaw, num_yaw)
                    margins_z[iy, iz] = compute_vertical_margin(
                        0.2, d, fov_pitch, num_vel_z)

    blocked_with_margin = blocked.copy()

    y_coords, z_coords = np.meshgrid(np.arange(num_yaw), np.arange(num_vel_z),
                                     indexing='ij')

    for iy in range(num_yaw):
        for iz in range(num_vel_z):
            if blocked[iy, iz]:
                yaw_margin = margins_yaw[iy, iz]
                z_margin = margins_z[iy, iz]

                if yaw_margin > 0 or z_margin > 0:
                    y_mask = (np.abs(y_coords - iy) <= yaw_margin)
                    z_mask = (np.abs(z_coords - iz) <= z_margin)
                    affected_mask = y_mask & z_mask

                    blocked_with_margin[affected_mask] = True

    # 原版语义：flip 后再映射动作扇区
    blocked_with_margin = np.flip(blocked_with_margin, axis=(0, 1))

    iy_indices = np.argmin(np.abs(yaw_lib - cmd_yaw_relative.reshape(-1, 1)),
                           axis=1)
    iz_indices = np.argmin(np.abs(z_lib - cmd_z_relative.reshape(-1, 1)),
                           axis=1)

    safe_mask = ~blocked_with_margin[iy_indices, iz_indices]

    safe_actions_yaw = cmd_yaw_relative[safe_mask]
    safe_actions_z = cmd_z_relative[safe_mask]

    return (safe_actions_yaw, safe_actions_z,
            safe_mask, blocked_with_margin, margins_yaw, margins_z)


def choose_dead_end_rotation(depth_img, blocked_with_margin, fov_yaw,
                             num_yaw=32, num_vel_z=8):
    """死端逃逸旋转方向：最近障碍偏左->顺时针(-1)，偏右->逆时针(+1)。

    逐字移植 evaluate.py L695-735（注意入参 blocked_with_margin 是
    filter_actions_by_grid_3 flip 之后的图，与原版调用序一致）。
    """
    h, w = depth_img.shape
    sector_w = w // num_yaw
    sector_h = h // num_vel_z

    min_dist = np.inf
    min_iy = None

    for iy in range(num_yaw):
        for iz in range(num_vel_z):
            if blocked_with_margin[iy, iz]:
                col_start = iy * sector_w
                col_end = (iy + 1) * sector_w if iy < num_yaw - 1 else w
                row_start = iz * sector_h
                row_end = (iz + 1) * sector_h if iz < num_vel_z - 1 else h

                sector_depth = depth_img[row_start:row_end, col_start:col_end]
                valid = sector_depth[sector_depth > 0]
                if valid.size > 0:
                    d = np.median(valid)
                    if d < min_dist:
                        min_dist = d
                        min_iy = iy

    if min_iy is None:
        return +1

    mid = num_yaw // 2
    if min_iy < mid:
        return -1
    else:
        return +1


# ============================ yaw 控制（vendored） ============================

def calculate_yaw_ctrl(yaw_error, k_yaw=1.0, yaw_rate_limit=math.pi / 4):
    """相对 yaw 指令 -> yaw 速率。逐字移植 evaluate.py L848-861。"""
    if abs(yaw_error) > math.pi:
        yaw_error = wrap_angle(yaw_error)
    yaw_rate_cmd = k_yaw * yaw_error
    if yaw_rate_cmd > yaw_rate_limit:
        yaw_rate_cmd = yaw_rate_limit
    elif yaw_rate_cmd < -yaw_rate_limit:
        yaw_rate_cmd = -yaw_rate_limit
    return yaw_rate_cmd


# ============================ UT sigma 状态（vendored） ============================

def ut_weights(alpha, beta, L, k_ukf):
    """Unscented 变换权重与 delta，evaluate.py L204-211 + config.py。"""
    lam = (alpha ** 2) * (L + k_ukf) - L
    w_m_0 = lam / (L + lam)
    w_m_i = 1.0 / (2.0 * (L + lam))
    w_m = np.array([w_m_0] + [w_m_i] * (2 * L), dtype=np.float32)
    w_c_0 = w_m_0 + 1.0 - alpha ** 2 + beta
    w_c = np.array([w_c_0] + [w_m_i] * (2 * L), dtype=np.float32)
    return lam, w_m, w_c


def build_sigma_states(velocity_psi, yaw_rate, roll, pitch,
                       delta_v, use_ut=True, vel_ut_min=0.0):
    """(2L+1, 6) sigma 状态矩阵 [vx,vy,vz,yaw_rate,roll,pitch]。

    逐字移植 evaluate.py L1156-1179（USE_UT=True 主路径）。
    velocity_psi: (3,) yaw 对齐系速度（x 分量已被调用方钳 >=0）。
    """
    vx, vy, vz = float(velocity_psi[0]), float(velocity_psi[1]), float(velocity_psi[2])
    dvx, dvy, dvz = delta_v
    if use_ut:
        if vx - 1.0 * dvx < vel_ut_min:
            x_min = vel_ut_min
            x_max = 2.0 * vx - vel_ut_min
        else:
            x_min = vx - 1.0 * dvx
            x_max = vx + 1.0 * dvx
        return np.array([
            [vx, vy, vz, yaw_rate, roll, pitch],
            [x_min, vy, vz, yaw_rate, roll, pitch],
            [x_max, vy, vz, yaw_rate, roll, pitch],
            [vx, vy - 1.0 * dvy, vz, yaw_rate, roll, pitch],
            [vx, vy + 1.0 * dvy, vz, yaw_rate, roll, pitch],
            [vx, vy, vz - 1.0 * dvz, yaw_rate, roll, pitch],
            [vx, vy, vz + 1.0 * dvz, yaw_rate, roll, pitch]])
    base = [vx, vy, vz, yaw_rate, roll, pitch]
    return np.array([base] * 7)


def cpn_collision_scores(models, state_sigma, latent, action_seq_expand,
                         n_seq, n_sigma, n_e, time_weight,
                         w_m, w_c, std_alpha=0.05):
    """CPN 3 集成 + Unscented 打分，移植 evaluate.py L1186-1271。

    models: list[n_e] 个 InferenceCPNseVAE（call_depth_state_combiner/
            call_recurrent_net 接口，见 defop_cpn_model.py）。
    state_sigma: (2L+1, 6) float
    latent: (1, 128) float
    action_seq_expand: (n_seq*n_sigma, horizon, 3) float32
    返回 (max_col_prob (n_seq,1), best_idx, mean (n_seq,1), std (n_seq,1))。
    """
    di_feature_expand = [
        np.repeat(latent, n_sigma, axis=0).astype('float32')
        for _ in range(n_e)]

    temp_collision_prob = []
    for i in range(n_e):
        di_feature_with_state = models[i].call_depth_state_combiner(
            [state_sigma.astype('float32'), di_feature_expand[i]])
        expand = np.tile(di_feature_with_state, [n_seq, 1])
        # tf.split(expand, 2, axis=1) 的 numpy 等价（原版为 contiguous 才用 tf）
        h = np.ascontiguousarray(expand[:, :expand.shape[1] // 2])
        c = np.ascontiguousarray(expand[:, expand.shape[1] // 2:])
        prob = models[i].call_recurrent_net([h, c, action_seq_expand])
        temp_collision_prob.append(prob)
    temp_collision_prob = np.array(temp_collision_prob)
    # (n_e, n_sigma*n_seq, horizon, 1)

    max_col_prob_sample = np.matmul(time_weight, temp_collision_prob)
    max_col_prob_sample = np.reshape(max_col_prob_sample,
                                     [n_e, n_seq, n_sigma], 'C')

    # UT：sigma 维求均值/方差
    mean = np.dot(max_col_prob_sample, w_m)
    mean = np.reshape(mean, [n_e, n_seq, 1])
    cov = max_col_prob_sample - mean
    cov = np.square(cov)
    cov = np.dot(cov, w_c)
    cov = np.reshape(cov, [n_e, n_seq], 'C')
    cov = np.average(cov, axis=0)
    cov = np.reshape(cov, [1, n_seq])

    # 集成：n_e 平均
    mean = np.reshape(mean, [n_e, n_seq], 'C')
    mc_mean = np.average(mean, axis=0)
    mc_mean = np.reshape(mc_mean, [1, n_seq])
    mc_cov = mean - mc_mean
    mc_cov = np.square(mc_cov)
    mc_cov = np.average(mc_cov, axis=0)
    mc_cov = np.reshape(mc_cov, [1, n_seq])

    total_std = np.sqrt(cov + mc_cov)
    mc_mean = np.reshape(mc_mean, [n_seq, 1])
    total_std = np.reshape(total_std, [n_seq, 1])

    max_col_prob = mc_mean + std_alpha * total_std
    indx_best = int(np.argmin(max_col_prob))
    return max_col_prob, indx_best, mc_mean, total_std
