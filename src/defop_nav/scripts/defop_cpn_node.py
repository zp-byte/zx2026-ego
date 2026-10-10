#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_cpn_node.py — 节点 3/3：CPN 碰撞预测 + 几何监督 + 死端逃逸（DeFoP
seVAE-ORACLE 规划核适配到 arena 契约，sim/真机同码）。

订阅：
  /drone_<id>/latent_space      Float32MultiArray(128)   <- defop_vae_node
  /drone_<id>/odom              nav_msgs/Odometry(世界 ENU)
  /drone_<id>/depth/image_filled Image 16UC1 毫米        <- defop_depth_fill（监督器用）
  /drone_<id>/planning/goal     PoseStamped（goal_source=topic 时）
发布：
  /drone_<id>/vel_cmd           geometry_msgs/Twist（世界系 linear.xyz +
                                angular.z=yaw 率；下游 vel_bridge 契约）
  /drone_<id>/defop/debug       std_msgs/String JSON（基元/碰撞概率/否决数/wp）

规划核逐字移植 ~/DeFoP evaluate.py（BSD-3）：
  基元库/监督器/UT/打分/死端/慢翻转/航点推进（帧手术与参数化部分见
  defop_common.py 头注）。行为树原序保留：deadend -> slowdown -> reach_goal
  ->（direction_service/timeout 对 PLANNING_TYPE=1 恒 SUCCESS，省略）->
  follow_goal 选择 -> 执行 step0 基元。

帧手术（README 有完整推导）：
  DeFoP 真指令 = 机体系 [vx, vel_y=0.5*tanh(yaw_rel), vz] + angular.z=yaw 速率
  -> 世界系 [vx_w,vy_w] = Rz(yaw)·[vx,vel_y]，vz 直传（高度带门控）。
  CPN 状态速度 = Rz(-yaw)·v_world（与原版 R_xy(roll,pitch)·v_body 逐字等价）。

运行环境：~/defop_env/bin/python（TF 2.4.1）；缺 TF/权重时报明确错误并 exit
（缺模型不飞，allow_degraded=true 才降级为纯监督器模式）。
"""
import sys
import os
import math
import json
import threading

import numpy as np


def _ensure_ros():
    """venv python 下补 ROS Noetic dist-packages。"""
    try:
        import rospy  # noqa: F401
        return
    except ImportError:
        pass
    for p in ('/opt/ros/noetic/lib/python3/dist-packages',):
        if p not in sys.path:
            sys.path.append(p)
    import rospy  # noqa: F401


_ensure_ros()

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import defop_common as dc

RESULT_SUCCESS = 'success'
RESULT_RUNNING = 'running'
RESULT_FAIL = 'fail'


def decode_depth_m(msg):
    """16UC1 毫米（或 32FC1 米）-> float32 米（DeFoP 真机 '<H' 直解路径）。"""
    buf = bytes(bytearray(msg.data))
    if msg.encoding == '32FC1':
        return np.frombuffer(buf, dtype='<f4').reshape(msg.height, msg.width).copy()
    return np.frombuffer(buf, dtype='<u2').reshape(msg.height, msg.width).astype(np.float32) * 0.001


def default_cfg():
    """默认参数（与 config/defop_nav.yaml 一致；单测直接用）。"""
    return {
        'drone_id': '0',
        # 基元库 / 网络维数（DeFoP config.py 默认）
        'num_vel_x': 1, 'num_vel_z': 8, 'num_yaw': 32,
        'action_horizon': 15, 'action_shape': 3, 'state_shape': 6,
        'cmd_velocity': 1.0, 'cam_pitch_deg': 0.0,
        'fov_h_deg': 87.0, 'fov_v_deg': 50.0, 'latent_size': 128,
        # 集成 + UT（config.py: N_E=3, USE_UT=True, ALPHA=1, BETA=2, L=3, K=0, P=0.16）
        'n_ensemble': 3, 'use_ut': True,
        'ut_alpha': 1.0, 'ut_beta': 2.0, 'ut_l': 3, 'ut_k': 0.0,
        'p_vx': 0.16, 'p_vy': 0.16, 'p_vz': 0.16,
        # 打分 / 死端
        'time_weight_factor': 0.04, 'collision_threshold': 0.35,
        'std_alpha': 0.05,
        'deadend_high': 6.5, 'deadend_low': 5.0,
        # 几何监督器（evaluate.py 实跑用 1.0/0.05/0.30；这里取函数签名默认档）
        'sup_min_dist': 1.5, 'sup_min_ratio': 0.02, 'sup_drone_radius': 0.32,
        # 指令整形
        'vel_y_gain': 0.5, 'yaw_k': 1.0, 'yaw_rate_limit_deg': 45.0,
        # 航点 / 目标
        'goal_source': 'topic',          # topic | waypoint_file
        'waypoint_file': '',
        'waypoint_distance_threshold': 3.0,
        'waypoint_yaw_threshold_deg': 45.0,
        'allow_yaw_at_waypoint': True,
        'goal_reach_dist': 0.8,
        # 安全包络 / 断流
        'vel_max': 1.2, 'z_min': 0.3, 'z_max': 4.5, 'vz_abs_max': 1.0,
        'stream_timeout': 1.0, 'plan_rate': 10.0,
        'yaw_rate_spot_deg': 30.0,
        'odom_twist_frame': 'world',     # world | body
        # 降级
        'allow_degraded': False,
    }


class DefopCPNPlanner(object):
    """规划核：输入缓存 + 行为树 + 基元执行。ROS 无关（单测可直接 new）。"""

    def __init__(self, cfg, log=None):
        self.cfg = dict(cfg)
        c = self.cfg
        self.log = log or (lambda *a, **k: None)
        self.n_seq = int(c['num_vel_x']) * int(c['num_vel_z']) * int(c['num_yaw'])
        self.n_sigma = 2 * int(c['ut_l']) + 1
        self.fov_h_rad = math.radians(float(c['fov_h_deg']))
        self.fov_v_rad = math.radians(float(c['fov_v_deg']))
        self.yaw_rate_limit_rad = math.radians(float(c['yaw_rate_limit_deg']))
        self.yaw_rate_spot = math.radians(float(c['yaw_rate_spot_deg']))
        self.wp_yaw_thr_rad = math.radians(float(c['waypoint_yaw_threshold_deg']))

        self.lam, self.w_m, self.w_c = dc.ut_weights(
            float(c['ut_alpha']), float(c['ut_beta']), int(c['ut_l']),
            float(c['ut_k']))
        self.delta_v = (math.sqrt((int(c['ut_l']) + self.lam) * float(c['p_vx'])),
                        math.sqrt((int(c['ut_l']) + self.lam) * float(c['p_vy'])),
                        math.sqrt((int(c['ut_l']) + self.lam) * float(c['p_vz'])))
        self.time_weight = dc.time_weight_matrix(int(c['action_horizon']),
                                                 float(c['time_weight_factor']))

        # 模型（set_models 注入；None = 未加载）
        self.models = None

        # 输入缓存
        self.odom = None        # dict(pos, yaw, roll, pitch, twist_world, t)
        self.latent = None      # (1,128) float32
        self.latent_t = None
        self.depth = None       # (h,w) float32 米
        self.depth_t = None

        # 目标
        self.goal_pos = None    # topic 模式
        self.waypoints = np.array([])
        self.wp_idx = 0
        self.num_wp = 0
        if c['goal_source'] == 'waypoint_file' and c['waypoint_file']:
            self.load_waypoints(c['waypoint_file'])

        # 行为树状态（原版字段名保留）
        self.deadend_step = 0
        self.deadend_dir = 0
        self.has_deadend_in_segment = True
        self.isflipping = False
        self.slowdown_step = 0
        self.slowdown_start = None
        self.flip_score = 0.0
        self.prev_yaw_sign = None
        self.done = False

        # 告警一次标志（恢复后复位）
        self._warned = {}

        self._lock = threading.Lock()

    # ------------------------------------------------------------------ 目标

    def load_waypoints(self, path):
        """yaml 航点：{'waypoints': [[x,y,z],...]} 或裸 [[x,y,z],...]。"""
        import yaml
        path = os.path.expanduser(path)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict):
            data = data.get('waypoints', [])
        arr = np.array(data, dtype=float)
        if arr.size == 0:
            self.waypoints = np.array([])
            self.num_wp = 0
            self.log('[defop_cpn] waypoint file empty: %s' % path)
            return
        if arr.ndim != 2 or arr.shape[1] < 3:
            raise ValueError('waypoint file %s: 每行需 [x,y,z]，收到 shape %s'
                             % (path, arr.shape))
        self.waypoints = arr[:, 0:3]
        self.num_wp = self.waypoints.shape[0]
        self.wp_idx = 0
        self.log('[defop_cpn] loaded %d waypoints from %s' % (self.num_wp, path))

    def _current_wp(self):
        if self.cfg['goal_source'] == 'topic':
            return self.goal_pos
        if self.num_wp == 0:
            return None
        return self.waypoints[self.wp_idx]

    def _advance_wp(self):
        self.wp_idx += 1
        self.has_deadend_in_segment = False
        self.log('[defop_cpn] REACH WP %d -> next %s'
                 % (self.wp_idx - 1,
                    self.waypoints[self.wp_idx].tolist()
                    if self.wp_idx < self.num_wp else 'END'))

    def _calc_goal_dir(self):
        """-> (goal_yaw, goal_el)；无目标时维持当前 yaw（原版 num_wp==0 语义）。"""
        wp = self._current_wp()
        if wp is None or self.odom is None:
            return self.odom['yaw'] if self.odom else 0.0, 0.0
        delta = np.asarray(wp, dtype=float) - self.odom['pos']
        goal_yaw = math.atan2(delta[1], delta[0])
        goal_el = math.atan2(delta[2], math.hypot(delta[0], delta[1]))
        return goal_yaw, goal_el

    # ------------------------------------------------------------------ 输入

    def update_odom(self, pos, qx, qy, qz, qw, twist_linear, stamp_recv,
                    yaw_rate=0.0):
        yaw, roll, pitch = dc.quat_to_ypr(qx, qy, qz, qw)
        v = np.asarray(twist_linear, dtype=float)
        if self.cfg['odom_twist_frame'] == 'body':
            v = np.array(dc.body_vel_to_world(v[0], v[1], yaw) + (v[2],))
        self.odom = {'pos': np.asarray(pos, dtype=float), 'yaw': yaw,
                     'roll': roll, 'pitch': pitch, 'twist_world': v,
                     'yaw_rate': float(yaw_rate), 't': stamp_recv}

    def update_latent(self, arr, stamp_recv):
        arr = np.asarray(arr, dtype=np.float32).reshape(1, -1)
        if arr.shape[1] != int(self.cfg['latent_size']):
            if not self._warned.get('latent_shape'):
                self._warned['latent_shape'] = True
                self.log('[defop_cpn] latent dim mismatch: %d != %d, dropped'
                         % (arr.shape[1], int(self.cfg['latent_size'])))
            return
        self.latent = arr
        self.latent_t = stamp_recv

    def update_depth(self, depth_m, stamp_recv):
        self.depth = depth_m
        self.depth_t = stamp_recv

    def update_goal(self, xyz):
        self.goal_pos = np.asarray(xyz, dtype=float)
        if self.done:
            self.done = False  # 新目标重新激活
        self.log('[defop_cpn] goal <- %s' % self.goal_pos.tolist())

    def set_models(self, models):
        self.models = models

    # ------------------------------------------------------------- 行为树肢

    def _deadend_action(self, min_score, safe_acts, rot_dir, dbg):
        """evaluate.py deadend_action 移植。min_score None（无模型降级）时
        以"监督器是否全否决"为准。"""
        c = self.cfg
        if min_score is None:
            min_ok_high = len(safe_acts) > 0
            min_ok_low = len(safe_acts) > 0
        else:
            min_ok_high = min_score < float(c['deadend_high'])
            min_ok_low = min_score < float(c['deadend_low'])
        if (self.deadend_step == 0) and min_ok_high and (len(safe_acts) > 0):
            return RESULT_SUCCESS, None
        elif (self.deadend_step == 1) and min_ok_low and (len(safe_acts) > 0):
            self.deadend_step = 0
            return RESULT_SUCCESS, None
        else:
            if self.deadend_step == 0:
                self.deadend_step = 1
                self.has_deadend_in_segment = True
                self.deadend_dir = rot_dir
                dbg['deadend_dir'] = int(rot_dir)
                self.log('[defop_cpn] DEADEND enter (min_score=%s, safe=%d, dir=%d)'
                         % ('NA' if min_score is None else '%.2f' % min_score,
                            len(safe_acts), rot_dir))
                return RESULT_RUNNING, self._cmd_stop('deadend_stop')
            else:
                return RESULT_RUNNING, self._cmd_rotate(
                    self.yaw_rate_spot * self.deadend_dir, 'deadend_rotate')

    def _slowdown_action(self, now):
        """evaluate.py slowdown_action 移植（flip_score>2.5 触发的原地稳一拍）。"""
        if self.isflipping is False:
            self.slowdown_start = None
            return RESULT_SUCCESS, None
        else:
            if (self.slowdown_step == 0) and (self.slowdown_start is None):
                self.slowdown_start = now
                self.slowdown_step = 1
                return RESULT_RUNNING, self._cmd_stop('slowdown_stop')
            elif (self.slowdown_step == 1) and \
                    ((now - self.slowdown_start) < 1.0):
                return RESULT_RUNNING, self._cmd_stop('slowdown_yaw')
            else:
                self.isflipping = False
                self.slowdown_start = None
                self.slowdown_step = 0
                self.flip_score = 0.0
                return RESULT_SUCCESS, None

    def _reach_goal_action(self, now, dbg):
        """evaluate.py reach_goal_action 移植。topic 模式：到达即原地悬停等新目标。"""
        c = self.cfg
        if self.cfg['goal_source'] == 'topic':
            if self.goal_pos is None:
                return RESULT_SUCCESS, None
            d2 = float(np.linalg.norm(self.odom['pos'][0:2] - self.goal_pos[0:2]))
            if d2 < float(c['goal_reach_dist']):
                dbg['goal_dist'] = d2
                return RESULT_RUNNING, self._cmd_stop('goal_hold')
            return RESULT_SUCCESS, None

        if self.num_wp == 0:
            return RESULT_SUCCESS, None
        wp = self.waypoints[self.wp_idx]
        if np.linalg.norm(self.odom['pos'][0:2] - wp[0:2]) > \
                float(c['waypoint_distance_threshold']):
            return RESULT_SUCCESS, None
        if self.wp_idx >= self.num_wp - 1:  # 最后一个航点
            self.done = True
            return RESULT_FAIL, self._cmd_stop('done')
        if not bool(c['allow_yaw_at_waypoint']):
            self._advance_wp()
            return RESULT_SUCCESS, None
        else:
            # 跳过贴身航点
            while np.linalg.norm(self.odom['pos'][0:2]
                                 - self.waypoints[self.wp_idx + 1][0:2]) <= \
                    float(c['waypoint_distance_threshold']):
                self._advance_wp()
                if self.wp_idx >= self.num_wp - 1:
                    self.done = True
                    return RESULT_FAIL, self._cmd_stop('done')
            delta_vec = self.waypoints[self.wp_idx + 1] - self.odom['pos']
            goal_yaw = math.atan2(delta_vec[1], delta_vec[0])
            delta_yaw = (self.odom['yaw'] - goal_yaw + math.pi) % \
                (2 * math.pi) - math.pi
            if abs(delta_yaw) <= self.wp_yaw_thr_rad:
                self._advance_wp()
                return RESULT_SUCCESS, None
            else:
                # 原版 yaw_in_spot(robot_yaw ± 15°) 是角度语义；arena 契约
                # angular.z=yaw 速率 -> 换算为朝目标航向的限速率旋转
                rate = max(-self.yaw_rate_spot,
                           min(self.yaw_rate_spot, -1.5 * delta_yaw))
                dbg['wp_align_err'] = float(delta_yaw)
                return RESULT_RUNNING, self._cmd_rotate(rate, 'waypoint_yaw')

    # ------------------------------------------------------------ 指令装配

    def _cmd_stop(self, mode):
        return {'type': 'stop', 'mode': mode, 'vx_w': 0.0, 'vy_w': 0.0,
                'vz_w': 0.0, 'yaw_rate': 0.0}

    def _cmd_rotate(self, yaw_rate, mode):
        return {'type': 'rotate', 'mode': mode, 'vx_w': 0.0, 'vy_w': 0.0,
                'vz_w': 0.0, 'yaw_rate': float(yaw_rate)}

    def _cmd_from_primitive(self, idx_best, dbg):
        """执行基元 step0（evaluate.py L1483-1516 移植）+ 帧手术 + 安全包络。"""
        c = self.cfg
        cmd_eval = self._action_seq[idx_best]
        dyaw = float(cmd_eval[0, 2])
        vel_x = float(cmd_eval[0, 0])
        vel_z = float(cmd_eval[0, 1])
        yaw_ctrl = dc.calculate_yaw_ctrl(dyaw, float(c['yaw_k']),
                                         self.yaw_rate_limit_rad)
        vel_y = float(c['vel_y_gain']) * math.tanh(dyaw)

        yaw = self.odom['yaw']
        vx_w, vy_w = dc.body_vel_to_world(vel_x, vel_y, yaw)
        vx_w, vy_w, vz_w, notes = dc.apply_safety_envelope(
            vx_w, vy_w, vel_z, float(self.odom['pos'][2]),
            float(c['vel_max']), float(c['z_min']), float(c['z_max']),
            float(c['vz_abs_max']))
        dbg['primitive'] = [vel_x, vel_z, dyaw]
        dbg['body_cmd'] = [vel_x, vel_y, vel_z]
        dbg['yaw_ctrl'] = float(yaw_ctrl)
        if notes:
            dbg['envelope'] = notes
        return {'type': 'primitive', 'mode': 'plan', 'vx_w': vx_w,
                'vy_w': vy_w, 'vz_w': vz_w, 'yaw_rate': float(yaw_ctrl)}

    # ------------------------------------------------------------ 规划主循环

    def plan_once(self, now):
        """一次规划周期。返回 (cmd dict, debug dict)。now 为秒（接收时钟）。"""
        c = self.cfg
        dbg = {'mode': 'init'}
        st = float(c['stream_timeout'])

        # 1) 输入新鲜度（断流 -> HOLD 零速 + WARN 一次）
        if self.odom is None:
            return self._hold('hold_no_odom', dbg, 'odom')
        if (now - self.odom['t']) > st:  # verify F1：odom 同样须时效门
            return self._hold('hold_stale_odom', dbg, 'odom')
        if self.latent is None or (now - self.latent_t) > st:
            return self._hold('hold_stale_latent', dbg, 'latent')
        if self.depth is None or (now - self.depth_t) > st:
            return self._hold('hold_stale_depth', dbg, 'depth')

        # 2) 目标存在性
        if c['goal_source'] == 'topic' and self.goal_pos is None:
            return self._hold('hold_no_goal', dbg, 'goal')
        if c['goal_source'] != 'topic' and self.num_wp == 0:
            return self._hold('hold_no_waypoints', dbg, 'goal')
        if self.done:
            return self._hold('done', dbg, None)

        # 3) 基元库重建（随当前 pitch 平移 vz 档）
        pitch = self.odom['pitch']
        self._action_seq, yaw_lib, z_lib = dc.build_action_library(
            int(c['num_vel_x']), int(c['num_vel_z']), int(c['num_yaw']),
            int(c['action_horizon']), float(c['cmd_velocity']),
            float(c['cam_pitch_deg']), float(c['fov_h_deg']),
            float(c['fov_v_deg']), pitch_rad=pitch)
        action_seq_expand = np.ascontiguousarray(
            np.repeat(self._action_seq, self.n_sigma, axis=0),
            dtype=np.float32)

        # 4) 几何监督器（evaluate.py L1077-1085 调用形态，参数可配）
        _, _, safe_mask, blocked_with_margin, _, _ = dc.filter_actions_by_grid_3(
            self.depth, self.fov_h_rad, self.fov_v_rad,
            self._action_seq[:, 0, 2], self._action_seq[:, 0, 1],
            yaw_lib, z_lib,
            num_yaw=int(c['num_yaw']), num_vel_z=int(c['num_vel_z']),
            min_dist=float(c['sup_min_dist']),
            min_ratio=float(c['sup_min_ratio']),
            drone_radius=float(c['sup_drone_radius']))
        safe_mask = np.asarray(safe_mask).ravel()
        dbg['safe_cnt'] = int(safe_mask.sum())
        dbg['veto_cnt'] = int((~safe_mask).sum())

        # 5) CPN 打分（模型未加载 -> 降级）
        scores = None
        if self.models is not None:
            v_psi = np.array(dc.world_vel_to_psi(
                self.odom['twist_world'][0], self.odom['twist_world'][1],
                self.odom['twist_world'][2], self.odom['yaw']))
            if v_psi[0] < 0:  # 原版 L1153-1154
                v_psi[0] = 0.0
            state_sigma = dc.build_sigma_states(
                v_psi, self._yaw_rate(), self.odom['roll'], self.odom['pitch'],
                self.delta_v, bool(c['use_ut']))
            with np.errstate(invalid='ignore'):
                scores, indx_best_tmp, _, _ = dc.cpn_collision_scores(
                    self.models, state_sigma, self.latent, action_seq_expand,
                    self.n_seq, self.n_sigma, int(c['n_ensemble']),
                    self.time_weight, self.w_m, self.w_c,
                    float(c['std_alpha']))
            if not np.isfinite(scores).all():
                # NaN 基元 = 打分失败 -> 视为最不安全（与原版 bob 过滤等价）
                scores = np.where(np.isfinite(scores), scores, np.inf)
            indx_best_tmp = int(np.argmin(scores))
            min_score = float(scores[indx_best_tmp])
            indx = np.nonzero(
                scores < scores[indx_best_tmp] +
                float(c['collision_threshold']))[0]
            indx1 = indx[safe_mask[indx].ravel()]
            dbg['col_prob'] = min_score
        else:
            indx1 = np.nonzero(safe_mask)[0]
            min_score = None
            dbg['col_prob'] = None

        # 6) 行为树（原序：deadend -> slowdown -> reach_goal -> [direction/
        #    timeout 对 PLANNING_TYPE=1 恒 SUCCESS，省略]）
        rot_dir = dc.choose_dead_end_rotation(
            self.depth, blocked_with_margin, self.fov_h_rad,
            num_yaw=int(c['num_yaw']), num_vel_z=int(c['num_vel_z']))
        res_dead, cmd = self._deadend_action(min_score, indx1, rot_dir, dbg)
        if res_dead != RESULT_SUCCESS:
            dbg['mode'] = cmd['mode']
            return cmd, dbg

        res_slow, cmd2 = self._slowdown_action(now)
        if res_slow != RESULT_SUCCESS:
            dbg['mode'] = cmd2['mode']
            return cmd2, dbg

        res_goal, cmd3 = self._reach_goal_action(now, dbg)
        if res_goal == RESULT_FAIL:
            dbg['mode'] = cmd3['mode']
            return cmd3, dbg
        if res_goal != RESULT_SUCCESS:
            dbg['mode'] = cmd3['mode']
            return cmd3, dbg

        # 7) follow_goal 选择（evaluate.py L1404-1477 移植）
        goal_yaw, goal_el = self._calc_goal_dir()
        dbg['goal_yaw'] = float(goal_yaw)
        if len(indx1) == 0:
            # 理论不可达（deadend 已接）；保底 HOLD
            return self._hold('hold_no_safe_action', dbg, 'no_safe')
        self._warned.pop('no_safe', None)  # verify F2：恢复后允许下次再告警

        if self.models is None:
            # 降级：纯监督器，目标航向差最小；同档 tie-break 用高度差
            # （与模型路径同一 el_cost；yaw 库 32 档无精确 0，最近档 ±0.022 rad）
            dbg['mode'] = 'plan_degraded'
            yaw_cost = np.abs(
                (self._action_seq[indx1, 0, 2] + self.odom['yaw'] - goal_yaw
                 + math.pi) % (2 * math.pi) - math.pi)
            indx_yaw_best = np.where(yaw_cost == yaw_cost.min())[0]
            indx2 = indx1[indx_yaw_best]
            el_cost = np.abs(
                (np.arctan2(self._action_seq[indx2, 0, 1],
                            self._action_seq[indx2, 0, 0]) - goal_el
                 + math.pi) % (2 * math.pi) - math.pi)
            indx_best = int(indx2[int(np.argmin(el_cost))])
        else:
            dbg['mode'] = 'plan'
            yaw_cost = np.abs(
                (self._action_seq[indx1, 0, 2] + self.odom['yaw'] - goal_yaw
                 + math.pi) % (2 * math.pi) - math.pi)
            current_yaw = self._action_seq[indx1, 0, 2]
            current_signs = np.sign(current_yaw)
            candidate_index = int(np.argmin(yaw_cost))
            chosen_sign = current_signs[candidate_index]

            if self.prev_yaw_sign is not None and \
                    chosen_sign != self.prev_yaw_sign and chosen_sign != 0:
                self.flip_score += 1
            else:
                self.flip_score *= 0.1
            self.prev_yaw_sign = chosen_sign

            positive_mask = current_yaw > 0
            negative_mask = current_yaw < 0
            num_pos = int(np.sum(positive_mask))
            num_neg = int(np.sum(negative_mask))

            if self.flip_score > 2.5:
                self.isflipping = True

            if num_pos < num_neg / 3.0:
                yaw_cost[positive_mask] += 1.0
            if num_neg < num_pos / 3.0:
                yaw_cost[negative_mask] += 1.0

            indx_yaw_best = np.where(yaw_cost == yaw_cost.min())[0]
            indx2 = indx1[indx_yaw_best]
            el_cost = np.abs(
                (np.arctan2(self._action_seq[indx2, 0, 1],
                            self._action_seq[indx2, 0, 0]) - goal_el
                 + math.pi) % (2 * math.pi) - math.pi)
            indx_best = int(indx2[int(np.argmin(el_cost))])
        dbg['flip_score'] = float(self.flip_score)

        # 8) 执行基元 step0 + 帧手术 + 包络
        cmd = self._cmd_from_primitive(indx_best, dbg)
        return cmd, dbg

    # ------------------------------------------------------------------ 辅助

    def _yaw_rate(self):
        """yaw 率：odom twist 是世界系角速度（z 分量即 yaw_dot，sim 插件
        SetAngularVel 只给 z；真机 EKF odom 同语义）。"""
        o = self.odom
        if o is None:
            return 0.0
        return float(o.get('yaw_rate', 0.0))

    def _hold(self, mode, dbg, what=None):
        """断流/缺输入 HOLD：零速；每类事故 WARN 一次（recover_input 复位）。"""
        dbg['mode'] = mode
        if what is not None and not self._warned.get(what):
            self._warned[what] = True
            self.log('[defop_cpn] %s -> HOLD zero vel' % mode)
        return self._cmd_stop(mode), dbg

    def recover_input(self, what):
        """输入恢复时清 WARN 一次标志（节点回调里调）。"""
        self._warned.pop(what, None)


class DefopCPNNode(object):
    def __init__(self):
        drone_id = str(rospy.get_param('~drone_id', '0'))
        c = default_cfg()
        c['drone_id'] = drone_id
        # 参数覆盖（config/defop_nav.yaml 经 <rosparam> 装进私有 ns）
        for k in list(c.keys()):
            if rospy.has_param('~' + k):
                c[k] = rospy.get_param('~' + k)
        for k in ('p_vx', 'p_vy', 'p_vz', 'sup_min_dist', 'sup_min_ratio',
                  'sup_drone_radius'):
            if rospy.has_param('~' + k):
                c[k] = float(rospy.get_param('~' + k))
        for k in ('use_ut', 'allow_yaw_at_waypoint', 'allow_degraded'):
            if rospy.has_param('~' + k):
                c[k] = bool(rospy.get_param('~' + k))

        self.base = '/drone_%s' % drone_id
        self.planner = DefopCPNPlanner(c, log=self._log)
        self.load_waypoints_once()

        # P3 终端交接：cpn 持笔旗（nav 每拍广播 /planning/cpn_active）。
        # 默认 False = nav 持笔、本节点完全静默（旗未到达前不抢帧）。
        self.active = False
        self._flag_t = None   # 旗到达时刻（活性门：>1.0s 未更新=nav 死/停播）

        self.pub_cmd = rospy.Publisher(self.base + '/vel_cmd', Twist,
                                       queue_size=1)
        self.pub_dbg = rospy.Publisher(self.base + '/defop/debug', String,
                                       queue_size=2)

        rospy.Subscriber(self.base + '/latent_space', Float32MultiArray,
                         self.cb_latent, queue_size=1)
        rospy.Subscriber(self.base + '/odom', Odometry, self.cb_odom,
                         queue_size=10)
        rospy.Subscriber(self.base + '/depth/image_filled', Image,
                         self.cb_depth, queue_size=1)
        if c['goal_source'] == 'topic':
            rospy.Subscriber(self.base + '/planning/goal', PoseStamped,
                             self.cb_goal, queue_size=2)
        # P3 终端交接闸门：nav 广播的持笔旗（True=cpn 持笔 cruise 窗口，
        # False=nav 持笔本节点静默），见 cb_cpn_active / on_timer 首行 yield
        rospy.Subscriber(self.base + '/planning/cpn_active', Bool,
                         self.cb_cpn_active, queue_size=1)

        self.models = None
        self._busy = threading.Lock()
        self.load_models_async(c)

        period = rospy.Duration(1.0 / max(0.1, float(c['plan_rate'])))
        rospy.Timer(period, self.on_timer)
        rospy.loginfo('[defop_cpn drone_%s] ready: goal_source=%s plan_rate=%.1f '
                      'vel_max=%.2f z_band=[%.2f,%.2f]',
                      drone_id, c['goal_source'], float(c['plan_rate']),
                      float(c['vel_max']), float(c['z_min']), float(c['z_max']))

    def _log(self, msg):
        rospy.loginfo('%s', msg)

    def load_waypoints_once(self):
        if self.planner.cfg['goal_source'] == 'waypoint_file':
            wf = self.planner.cfg.get('waypoint_file', '')
            if not wf:
                rospy.logerr('[defop_cpn] goal_source=waypoint_file 但 '
                             'waypoint_file 为空')
                return
            try:
                self.planner.load_waypoints(wf)
            except Exception as e:
                rospy.logerr('[defop_cpn] waypoint file load failed: %r', e)

    def load_models_async(self, c):
        """TF + CPN 权重装载（阻塞本线程，不堵回调）；失败按 allow_degraded
        决定 exit 或降级。"""
        def _worker():
            try:
                t0 = rospy.Time.now()
                import tensorflow as tf  # noqa: F401  lazy：缺则报明确错误
                import defop_cpn_model as dcm
                paths = rospy.get_param('~cpn_weight_paths', None)
                if not paths:
                    root = '~/DeFoP/src/planning/ORACLE/model_weights/' \
                           'VAE_EVO_back_to_origin'
                    paths = [root + '/nett5/saved-model.hdf5',
                             root + '/nett5/saved-model.hdf5',
                             root + '/nett6/saved-model.hdf5']
                models = dcm.load_sevae_cpn_ensemble(
                    paths, latent_size=int(c['latent_size']),
                    action_horizon=int(c['action_horizon']),
                    action_shape_evaluate=int(c['action_shape']),
                    state_input_shape=int(c['state_shape']),
                    log=self._log)
                self.planner.set_models(models)
                self.models = models
                rospy.loginfo('[defop_cpn] models ready in %.1fs (N_E=%d)',
                              (rospy.Time.now() - t0).to_sec(), len(models))
            except ImportError as e:
                rospy.logerr('[defop_cpn] 缺依赖（应在 ~/defop_env/bin/python '
                             '下运行）: %r', e)
                if not bool(c['allow_degraded']):
                    rospy.signal_shutdown('missing dependency')
                    os._exit(1)
            except Exception as e:
                rospy.logerr('[defop_cpn] model load failed: %r', e)
                if not bool(c['allow_degraded']):
                    rospy.signal_shutdown('model load failed')
                    os._exit(1)
        threading.Thread(target=_worker, daemon=True).start()

    # ------------------------------------------------------------- 回调

    def cb_latent(self, msg):
        self.planner.update_latent(list(msg.data),
                                   rospy.Time.now().to_sec())
        self.planner.recover_input('latent')

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist.linear
        self.planner.update_odom([p.x, p.y, p.z], q.x, q.y, q.z, q.w,
                                 [t.x, t.y, t.z], rospy.Time.now().to_sec(),
                                 yaw_rate=float(msg.twist.twist.angular.z))
        self.planner.recover_input('odom')

    def cb_depth(self, msg):
        try:
            depth_m = decode_depth_m(msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[defop_cpn] depth decode failed: %r', e)
            return
        if depth_m.shape != (270, 480):
            # 监督器按 480x270 网格切扇区，尺寸异常直接丢帧（不静默缩放）
            rospy.logwarn_throttle(5.0,
                                   '[defop_cpn] depth size %s != (270, 480), dropped',
                                   depth_m.shape)
            return
        self.planner.update_depth(depth_m, rospy.Time.now().to_sec())
        self.planner.recover_input('depth')

    def cb_goal(self, msg):
        p = msg.pose.position
        self.planner.update_goal([p.x, p.y, p.z])
        self.planner.recover_input('goal')

    def cb_cpn_active(self, msg):
        """P3 终端交接：nav 广播的持笔旗（True=cpn 持笔）。跳变才记日志
        （cruise 窗口进出是低频事件）。"""
        act = bool(msg.data)
        if act != self.active:
            rospy.loginfo('[defop_cpn] cpn_active <- %s', act)
        self.active = act
        self._flag_t = rospy.Time.now().to_sec()

    # ------------------------------------------------------------- 定时规划

    def _flag_fresh(self):
        """旗新鲜度（校验 V1-M5 活性门）：≤1.0s 内收到过 cpn_active。

        nav 死/停播后 cpn 不得持笔盲飞——让位静默，gz 插件 0.5s 断链保护
        接管（零速），优于"nav 死后 cpn 续飞到底"。"""
        if self._flag_t is None:
            return False
        return (rospy.Time.now().to_sec() - self._flag_t) <= 1.0

    def on_timer(self, _evt):
        # P3 终端交接：nav 持笔（cpn_active=False）= 完全静默 yield——连 HOLD
        # 零速都不发（nav 拥有指令流，双写抢帧从源头掐掉）；首行前置门。
        # 活性门（校验 V1-M5）：旗超时=nav 停播/死亡，同样让位。
        if not self.active or not self._flag_fresh():
            return
        if not self._busy.acquire(False):
            return  # 上个周期推理还没完（CPU 慢），跳过本拍
        try:
            now = rospy.Time.now().to_sec()
            import time as _time
            t0 = _time.time()
            cmd, dbg = self.planner.plan_once(now)
            # 校验 V1-B1/V2-M3 双写窗：nav 终端接管旗可在本拍推理途中翻转
            # （plan_once 0.2-0.5s），发布前复查——陈旧指令丢弃（return 走
            # finally 释放 _busy），双写窗从"一次推理时长"压到零。
            if not self.active or not self._flag_fresh():
                return
            dbg['t_plan_ms'] = int((_time.time() - t0) * 1000)
            tw = Twist()
            tw.linear.x = float(cmd['vx_w'])
            tw.linear.y = float(cmd['vy_w'])
            tw.linear.z = float(cmd['vz_w'])
            tw.angular.z = float(cmd['yaw_rate'])
            self.pub_cmd.publish(tw)
            dbg['drone_id'] = self.planner.cfg['drone_id']
            dbg['wp'] = int(self.planner.wp_idx)
            if self.planner.cfg['goal_source'] == 'topic' and \
                    self.planner.goal_pos is not None:
                dbg['goal'] = self.planner.goal_pos.tolist()
            elif self.planner.num_wp:
                dbg['goal'] = self.planner.waypoints[
                    min(self.planner.wp_idx, self.planner.num_wp - 1)].tolist()
            try:
                self.pub_dbg.publish(String(data=json.dumps(dbg)))
            except TypeError:
                pass
        except Exception as e:
            rospy.logerr_throttle(5.0, '[defop_cpn] plan_once error: %r', e)
            # 规划异常兜底：零速（fail-safe）
            tw = Twist()
            self.pub_cmd.publish(tw)
        finally:
            self._busy.release()


def main():
    rospy.init_node('defop_cpn')
    DefopCPNNode()
    rospy.spin()


if __name__ == '__main__':
    main()
