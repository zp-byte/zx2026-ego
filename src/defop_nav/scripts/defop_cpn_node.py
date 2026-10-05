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
import time

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

# 刀一.5（pen_authority_case §三.4）：CPN 须读 sim_settings planner.pen_authority
# 决定原始令发 /vel_cmd（nav 档直发=旧拓扑）还是中继话题（cpn_forever 档，
# nav 套 sep 后统一组装）。zx2026_common 非本包 catkin 依赖（package.xml 无
# 声明），roslaunch 环境经 devel PYTHONPATH 可导入；venv 直跑时按
# tools/col_geom.py 模式手动落路径。pkg_config_dir 以模块文件为锚（egg-link
# 指回 src），读到的就是工作区 sim_settings.yaml——nav_node 同款先例。
try:
    from zx2026_common import config as _zx_cfg
except ImportError:
    _zx_ws = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', '..', 'src', 'zx2026_common', 'scripts'))
    if _zx_ws not in sys.path:
        sys.path.insert(0, _zx_ws)
    from zx2026_common import config as _zx_cfg

RESULT_SUCCESS = 'success'
RESULT_RUNNING = 'running'
RESULT_FAIL = 'fail'


def _clamp_dt(dt, cap):
    """影子积分的步长钳位：返回 (step, clamped)。

    ⚠ 2026-10-02 定谳的仪器缺陷：`on_timer` 有两道**早退门**——nav 持笔
    (`not self.active or not self._flag_fresh()`) 与碰撞镇定窗
    (`publish(Twist()); return`)——这些拍 `plan_once` 根本不执行，而影子
    `dt = now - _sh_now` 会把**被跳过的整段间隔一次性记入当前状态**。
    症状：回合**边沿计数**与真机逐机吻合（跳过拍不产生边沿），**时长**却
    虚高一个量级（110544/00：d1 replica `t_rot_cur`=7.7s vs 真机 mode 线
    `rot_s`=0.72s，而该机自己的直方图 max 仅 0.9s —— 自相矛盾）。

    语义：间隙内发生了什么**未知**，最多按一个标称周期计；`cap <= 0` 即
    关闭钳位（旧行为，仅供对拍复现旧缺陷用）。
    """
    if cap > 0.0 and dt > cap:
        return cap, True
    return dt, False


def _sup_bad(veto_frac, veto_hi):
    """③ 监督层条件的**单一实现**（真机 / 影子 replica / 离线复算共用）。

    `True` = 「监督层自己否决了绝大多数基元」⇒ 场景按监督器的口径**不安全**。
    `veto_hi <= 0` 或缺读数 ⇒ `False`（该条件整条关闭 = 逐字节旧行为）。
    """
    if veto_hi <= 0.0 or veto_frac is None:
        return False
    return float(veto_frac) >= float(veto_hi)


def _near_field_shadow(ring, cur, r_near):
    """近场记忆影的**单一实现**（真机 / 自检罪案重现共用）。

    B 案病根（docs/near_field_memory_case.md）：0.2m 拟真清零 + 填充从邻域
    取 min ⇒ 背景穿过树形空洞抹进来，树过 0.22m 一拍「全清」（safe 256、
    col_prob 崩、减速释放）→ 加速撞盲区。本影把 `ring` 里近 K 拍的近事件
    （0<d<r_near 的有效读数）逐像素顶在当前帧前面：

      shadow = min(cur, ring 内各帧同像素近事件)

    - 当前帧**有效且更远**（含填充抹平的假背景）时，记忆读数赢 —— 盲区
      被顶住；当前帧更近时，当前读数赢（真贴近不受旧影遮蔽）。
    - `ring` 只收近事件（远处读数不进环）：常规巡航不积累残影，只有
      真逼近过的像素才被顶住 K 拍后自然老化 ⇒ 旋转逃逸自洽（树转出
      画面 → 残影过期 → safe 重开）。
    - 无效像素语义保持：shadow 无有限值处回 0.0（下游 `depth>0` 掩膜
      行为不变）。
    纯 numpy，无 ROS 依赖 —— defop_common 同款可单测纪律。
    """
    shadow = np.array(cur, dtype=np.float32, copy=True)
    for f in ring:
        np.minimum(shadow, f, out=shadow)
    # 近事件之外（inf）且当前帧也无效处：保持无效语义 0.0
    shadow[~np.isfinite(shadow)] = 0.0
    return shadow


def _near_event_frame(depth, r_near):
    """把一帧深度图折成「近事件」帧：0<d<r_near 保留，其余 inf。"""
    ev = np.full(np.shape(depth), np.inf, dtype=np.float32)
    d = np.asarray(depth)
    m = np.isfinite(d) & (d > 0.0) & (d < float(r_near))
    ev[m] = d[m].astype(np.float32)
    return ev


def _sup_dwell(bad, now, st, dwell_s):
    """`bad` 须**连续**成立 dwell_s 秒才放行（迟滞，与退出侧 hyst 同族）。

    离线复算（批 5 逐拍线，真机同款状态机）测出的**代价结构**：`veto_frac`
    在障碍跨进 `sup_min_dist=1.0m` 那一拍就从 0 跳到 0.45~0.84（阶跃，因为
    掩膜本身就是 `depth < min_dist` 的硬门），任何 0.5~0.7 的门因此咬在**同一
    批拍**上 —— 提高 X 几乎不降代价（X=0.60 vs 0.625：死端占用 143.5 vs 143.2s）。
    真正的代价杠杆是**回合数**：擦身而过（0.9~1.0m 掠过 0.2s）也会触发一次
    进入+迟滞。故加本迟滞，滤掉一闪而过的近场，只留**持续逼近**。
    `dwell_s <= 0` ⇒ 立即放行（逐字节旧行为）。
    """
    if not bad:
        st['bad_t0'] = None
        return False
    if dwell_s <= 0.0:
        return True
    if st.get('bad_t0') is None:
        st['bad_t0'] = now      # 首次见坏：本拍仅开始计时，不放行
        return False
    return (now - st['bad_t0']) >= dwell_s


def _deadend_step_logic(step, min_score, n_safe, now, st, hi, lo,
                        max_s, hyst_s, veto_frac=None, veto_hi=0.0,
                        veto_dwell_s=0.0, geo_bad=False, nsafe_min=0):
    """死端状态机（**单一实现**，真机与影子 replica 共用 —— 防"两份实现"
    漂移，本仓已在 `_crown_z_range` 上中过同族）。

    状态：0 = 正常 / 1 = 原地旋转 / 2 = 强制放宽（超时后接受被否决的最高分
    基元，换"有界前进"）。返回 (new_step, action, forced)：
      action ∈ {'success','stop','rotate','forced'}。
    st = 调用方持有的可变状态 {'t0': float|None, 'ok_t0': float|None}
    （真机一份、影子每档各一份，**互不共享**）。

    `max_s <= 0` ⇒ 永不进状态 2；`hyst_s <= 0` ⇒ 单拍即退 —— 两者缺省即
    **逐字节旧行为**（上游 evaluate.py:309-326 的两态 latch）。
    低条件须**连续**满足 hyst_s：中途断裂则清零重计（时间戳差，同
    arena_nav/nav_node.py:_de_tick 的 _de_ok_t0，非计数器）。

    ③ 监督层条件（2026-10-02 立案，默认 `veto_hi=0.0` = 关闭 = 逐字节旧行为）：
    上游/本仓的 hi_ok、lo_ok **只看 CPN 自己的分**，监督器的否决率被丢弃。
    实测（批 5，9/9 起 ③ 同形）近树时 CPN 分**反而低**（0.50~5.53，全在
    `deadend_high=6.5` 之下，近距离图像对训练分布是 OOD），而监督器已否决
    59~69% 的基元 ⇒ 「监督器说绝大多数方向不安全」这条自证信号完全没进决策。
    故把 `veto_frac` 作为**与 min_score 并列的第二条**并入 hi/lo：
    两者都是"本拍场景是否安全"的判据，任一不满足即不安全。
    ⚠ 必须 hi/lo **同改**：只改 hi 会让 `lo_ok` 仍真 ⇒ 迟滞窗一到就退出、
    退出即重进，变成 <1s 抖动（正是 `deadend_exit_hyst_s` 要治的病）。

    ① 几何触发（2026-10-03 立案 docs/deadend_geo_case.md，`geo_bad` 由
    `_deadend_action` 从 dmin 历史差分算出，**影子两档不传=缺省 False=
    逐字节不变**）：dmin<阈值 且 收拢≥0.3m/s 持续 2 拍 ⇒ 与 ③ 同通道
    同置假。触发后 B2 硬零刹停 ⇒ closing→0 ⇒ geo_bad 自清（不闩死），
    退出仍走 lo 迟滞；活锁兜底=max_s 强制放宽。
    """
    # 候选 1（2026-10-05 立案）：`n_safe > 0` -> `n_safe >= _need`。n_safe =
    # len(indx1) = "CPN 认好 ∧ 监督器放行"的逃逸方向数；实测撞前从 ~100 平滑
    # 塌到 0（dmin 3→0.2m，约 3s），而旧门只认 ==0 ⇒ 贴脸才开。hi/lo **同**用
    # _need（只改 hi 会 lo 仍真 ⇒ 迟滞窗一到就退出，见 ③ veto 的同款教训）。
    # nsafe_min=0 ⇒ _need=1 ⇒ n_safe>=1 ≡ n_safe>0 = 逐字节旧行为。
    _need = max(1, int(nsafe_min))
    sup_bad = _sup_dwell(_sup_bad(veto_frac, veto_hi), now, st, veto_dwell_s)
    hi_ok = (n_safe >= _need) if min_score is None else \
        (min_score < hi and n_safe >= _need)
    lo_ok = (n_safe >= _need) if min_score is None else \
        (min_score < lo and n_safe >= _need)
    if sup_bad:
        hi_ok = False
        lo_ok = False
    if geo_bad:
        hi_ok = False
        lo_ok = False

    if step == 0:
        if hi_ok:
            st['ok_t0'] = None
            return 0, 'success', False
        st['t0'] = now
        st['ok_t0'] = None
        return 1, 'stop', False

    if step == 1:
        if lo_ok:
            if hyst_s <= 0:
                st['ok_t0'] = None
                return 0, 'success', False
            if st.get('ok_t0') is None:
                st['ok_t0'] = now
            if (now - st['ok_t0']) >= hyst_s:
                st['ok_t0'] = None
                return 0, 'success', False
        else:
            st['ok_t0'] = None            # 断裂即清零，迟滞重计
        if max_s > 0 and (now - st['t0']) >= max_s:
            st['ok_t0'] = None
            return 2, 'forced', True
        return 1, 'rotate', False

    # step == 2：只在**高**条件（比进入档更宽）连续满足后才交还常规跟随
    if hi_ok:
        if hyst_s <= 0:
            st['ok_t0'] = None
            return 0, 'success', False
        if st.get('ok_t0') is None:
            st['ok_t0'] = now
        if (now - st['ok_t0']) >= hyst_s:
            st['ok_t0'] = None
            return 0, 'success', False
    else:
        st['ok_t0'] = None
    return 2, 'forced', True


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
        # 死端超时 / 退出迟滞（2026-10-02 立案，默认 0.0 = 逐字节旧行为）。
        # 上游 evaluate.py:309-326 是**无超时无迟滞**的两态 latch；本仓实测
        # 相邻回合最短仅 0.6s（空↔非空抖动反复进出）⇒ 迟滞治抖动、超时把
        # "无界原地旋转"变成"有界旋转 + 有界前进（强制放宽）"。
        # 两键取值由影子批的回合时长直方图离线定，不预设。详见 docs/
        # deadend_timeout_case.md 与 config/defop_nav.yaml 的预登记块。
        # 量纲锚点：yaw_rate_spot_deg=30°/s ⇒ 单向 12s 扫满 360°。
        'deadend_max_s': 0.0, 'deadend_exit_hyst_s': 0.0,
        # ③ 监督层否决率门（2026-10-02 立案，**0.0 = 关闭 = 逐字节旧行为**）。
        # 语义见 _deadend_step_logic 头注：vf >= 本值时 hi_ok/lo_ok **同**置假
        # ⇒ 判死端、且不退出（迟滞不算数），直到监督器松口或 max_s 强制放宽。
        # 取值由离线复算 + 同 seed43 三格 A/B 定，不预设。
        'deadend_veto_frac': 0.0,
        # 上式须**连续**成立该秒数才咬合（滤"擦身而过"的一拍近场）。
        # 0.0 = 无迟滞（但仍受 deadend_veto_frac=0.0 的总开关支配）。
        'deadend_veto_dwell_s': 0.0,
        # ① 几何触发（2026-10-03 立案 docs/deadend_geo_case.md，**0.0=关闭=
        # 逐字节旧行为**）：近树图像是 OOD ⇒ CPN 分低位、现行门只给
        # 0.24~0.86s；本键把「dmin<阈值 且 在收拢」并入 hi/lo（③ veto 同
        # 通道）。离线两局 18 样本：抓 15、提前量中位 1.53s；代价=触发边沿
        # ~1 次/分/机（诚实成本，判别维度降不掉）。
        'deadend_geo_dmin': 0.0,
        # 收拢率阈值（c<=-本值 才算逼近；0.3 防「贴着不动也触发」活锁）。
        'deadend_geo_close_mps': 0.3,
        # 连续拍数（滤单拍噪声；离线口径=2）。
        'deadend_geo_dwell_n': 2,
        # 候选 1（2026-10-05 立案）：死端门的 `n_safe > 0` 改成 `n_safe >= 本值`。
        # n_safe = len(indx1) = "CPN 认好 ∧ 监督器放行"的逃逸方向数。实测撞前
        # 从 ~100 平滑塌到 0（dmin 3→0.2m，约 3s），现行门只认 ==0 ⇒ 贴脸才开。
        # 0 = 关闭 = 逐字节旧行为（n_safe>=1 ⇔ n_safe>0）。
        'deadend_nsafe_min': 0,
        # 候选 2（2026-10-05 立案）：选择代价的"横向间隙"项权重（0.0 = 关闭）。
        # yaw_cost += w · 该基元航向扇区的监督器否决率 ⇒ 把"最朝 goal"与
        # "朝空"折中，治"直线路径贴着偏轴树擦过"（机身半径 0.35m）。
        'follow_clear_w': 0.0,
        # 候选 2 的启用门：len(indx1) < 本值（危险）时才注入；<=0 = 恒启用。
        'follow_clear_nsafe': 0,
        # 近场记忆影（2026-10-03 立案 docs/near_field_memory_case.md，
        # **0 = 关闭 = 逐字节旧行为**）：监督器输入的影像系 min 池记忆。
        # 病根（B 案罪责表 25 起）：0.2m 拟真清零 + 填充背景穿树洞 ⇒ 树过
        # 0.22m 一拍变「全清」（safe 256、col_prob 崩、减速释放）→ 加速撞
        # 盲区。本影把近 K 拍内 <sup_min_dist 的有效读数逐像素顶住填充抹平，
        # 只喂监督器/死端旋转方向/dbg 三处，**VAE 路径零改动**（同话题独立
        # 节点）。窗长定量律（mem6 案先例）：否决带 1.0m→致盲 0.2m、收拢
        # ~1.0m/s ⇒ 窗 ≥0.8s；拍距 ~0.15s ⇒ 6 帧。K 帧后自然老化 = 旋转
        # 逃逸自洽（树转出画面 → 残影过期 → safe 重开时机≈转向完成）。
        'near_mem_frames': 0,
        # 几何监督器（G1 法证：1.0/0.05 是论文配 TRT 10Hz 的参数，本环境
        # CPU 2-5Hz 搭 1.3m/s 后 col 10.8→18.7；原注释称「回退 p3b 档
        # 1.5/0.02/0.32」，但那次回退从未落盘——2026-10-01 按实测订正措辞）。
        # sup_drone_radius 2026-10-01：0.30 → 0.35。0.30 是论文那台 0.4m 机架
        # 的调用点实参；本 sim 机体是 0.5×0.5×0.2 盒（面内外接 0.354），而判据
        # （gazebo_collision_monitor._tree_hit）与 nav 膨胀基同为 0.35。监督层
        # 必须与判据同径，否则是按比实际小 17% 的机体去算扇区裕度。
        # 本缺省只在 yaml 缺 `sup_drone_radius` 键时生效（yaml 是权威来源）。
        'sup_min_dist': 1.0, 'sup_min_ratio': 0.05, 'sup_drone_radius': 0.35,
        # 影子 replica 的积分步长上限（秒）：`plan_once` 在 nav 持笔/碰撞
        # 镇定窗**不执行**，其间隙不得整段充进死端状态时长（见 _clamp_dt 的
        # 定谳说明）。nominal 拍约 0.05~0.1s ⇒ 0.5 已是 5~10 倍余量。
        # <=0 即关闭钳位（旧行为，仅供对拍复现旧缺陷）。
        'sup_de_dt_cap_s': 0.5,
        # ④ 监督层几何口径（2026-10-01）：off | shadow | on，默认 off。
        # off    = 逐字节旧行为（映射轴 yaw_lib ≡ cmd_yaw_relative，恒等映射）
        # shadow = **不改发布指令**，对同一张深度图另算一套 β 档掩膜，只记
        #          veto/死端计数差（影子对质，零行为风险）
        # on     = 发布路径改用 β 档（扇区真方位轴 + 净空锥 asin）
        # 该值经 rosparam 到达时是**字符串**，下面 __init__ 有白名单强转。
        'sup_bearing_mode': 'off',
        # ② 预测性减速（2026-09-30）：前视变速，见 DefopCPNPlanner._gov_scale。
        # 默认关 = 逐字节旧行为；D 档预设开启（tools/defop_w3_gov_preset.py）。
        'gov_enable': False, 'gov_dist': 2.0, 'gov_stop': 0.35,
        'gov_floor': 0.45,
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
        'vel_max': 2.0, 'z_min': 0.3, 'z_max': 4.5, 'vz_abs_max': 1.0,
        # ③碰撞镇定：/collision 上升沿 -> HOLD 零速该时长（对齐恢复链
        # bounce 1.0+cooldown 2.0 窗口，交笔回来再复规划，治同树秒级复撞）
        'collision_settle_s': 2.5,
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

        # ④ 监督层 β 档（off | shadow | on）。shadow 计数器：
        # cyc=周期数，*_dead_cyc=该档下 indx1 为空的周期数，*_ep=死端**回合**数
        # （边沿检测：上拍空/本拍不空 记一次结束）。回合数才是 P4 代价的直接量
        # —— 单次死端约 1.4~2.2s 零前进，与周期数不成比例。
        self.sup_mode = str(c.get('sup_bearing_mode', 'off'))
        self._sh = {'cyc': 0, 'cur_dead': 0, 'beta_dead': 0,
                    'cur_ep': 0, 'beta_ep': 0,
                    'veto_cur': 0, 'veto_beta': 0,
                    'prev_cur_dead': False, 'prev_beta_dead': False,
                    'gap_n': 0, 'gap_s': 0.0}
        self._sh_t = 0.0                     # 影子日志节流（1 Hz）
        # 本案（2026-10-02）：影子档**直测**计量，取代 Δep×1.5s 的估算
        # （代理实测高估约 3×：代理 51/17 vs 真机 DEADEND enter 40/29）。
        #   每档两套 replica，都用同一个 _deadend_step_logic：
        #   · cand 档 = 候选参数 (deadend_max_s, deadend_exit_hyst_s) 的预计
        #     代价 —— t_rot/t_force 是状态 1/2 的 **sim 秒**积分；
        #   · raw 档 = 退化档 (0,0)（上游两态 latch）测"死端条件本身持续
        #     多久" ⇒ 回合时长直方图**与候选参数无关**，一次采集即可离线定阈。
        self._sh_de = {m: {'st': {'t0': None, 'ok_t0': None}, 'step': 0,
                           'raw_st': {'t0': None, 'ok_t0': None}, 'raw_step': 0,
                           't_rot': 0.0, 't_force': 0.0,
                           'ep_n': 0, 'ep_sum': 0.0, 'ep_max': 0.0,
                           'ep_t0': None, 'ep_acc': 0.0, 'hist': [0] * 12}
                       for m in ('cur', 'beta')}
        self._sh_now = None                  # 上一拍 sim 时刻（积分步长）
        # 基元的真实航向 β = atan2(vel_y_gain·tanh(dyaw), cmd_velocity)。
        # 与 _cmd_from_primitive 的 vel_y 同式 —— 那里是权威，此处只镜像。
        # _action_seq 在 plan_once 步骤 3 才建，故此处只置位，惰性求值。
        self._prim_beta = None

        # 行为树状态（原版字段名保留）
        self.deadend_step = 0
        self.deadend_dir = 0
        self.has_deadend_in_segment = True
        # 本案：死端状态机的外部状态（真机一份；影子档另有独立两份，互不共享）
        self._deadend_st = {'t0': None, 'ok_t0': None}
        self._deadend_forced = False
        # ① 几何触发状态（docs/deadend_geo_case.md）：dmin 历史（**位置**
        # 回看恰 2 拍差分收拢率，None 拍占位）+ 连击计数。
        # deadend_geo_dmin=0.0（默认）时采集照走、判定恒假 = 逐字节旧行为。
        self._geo_hist = []
        self._geo_run = 0
        # 近场记忆影状态（docs/near_field_memory_case.md）：近 K 拍的
        # 「近事件」帧环（<sup_min_dist 的有效读数，其余 inf）。K=0 时环
        # 不启用，sup_depth 恒为 self.depth 本尊 = 逐字节旧行为。
        self._near_ring = []
        self._sup_depth = None   # 每拍由 plan 刷新；_gov_scale 与监督器同源
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

    def _gov_scale(self):
        """② 预测性减速：前方中央带最近障碍 -> 水平速度比例因子（1.0=不减速）。

        与监督器共用同一张 self.depth（float32 米），故二者判据同源不打架：
        监督器管"拦不拦"（硬否决，1.0m），本器管"来得多快"（软减速，2.0m 起）。
        中央带取列 1/3..2/3（hfov 87° -> 约 ±14.5°），只测飞行方向会撞到的
        障碍；边缘扇区的障碍不在航迹上，对它减速只会白拖慢。

        抗噪双保险（T8 干测揪出）：①先按 sup_min_ratio 卡"近障碍是否占满
        最小面积比"——单像素/孤立点直接出局；②再取近端 10 分位而非 min，
        残余噪点不主导。直接复用监督器的噪声底，故本器在**任意距离**上都不会
        比监督器更早被噪点触发。缺省 0.05 与 sup_min_ratio 默认一致。
        """
        c = self.cfg
        if not bool(c.get('gov_enable', False)):
            return 1.0
        # 近场记忆影（docs/near_field_memory_case.md）：与监督器**同源**
        # （docstring 不变量）——本器现吃 _sup_depth。否则 0.2m 盲拍时监督
        # 器记忆顶住而本器读假背景释放减速 = 拦的手、油门的手打架。
        d = self._sup_depth if self._sup_depth is not None else self.depth
        if d is None:
            return 1.0
        look = float(c['gov_dist'])
        stop = float(c['gov_stop'])
        if look <= stop:
            return 1.0
        h, w = d.shape
        band = d[:, w // 3: w - w // 3]
        m = (band > 0.0) & (band < look)
        # 噪点门（与监督器同源，2026-09-30 干测 T8 揪出）：近障碍未占满 band
        # 的最小面积比 = 单像素/孤立点，不是墙。监督器用 sup_min_ratio 抗同一
        # 件事，此处复用其值。缺省 0.05 与 sup_min_ratio 默认一致。
        if float(m.mean()) < float(c.get('sup_min_ratio', 0.05)):
            return 1.0
        dn = float(np.percentile(band[m], 10))
        frac = (dn - stop) / (look - stop)
        return float(min(1.0, max(float(c['gov_floor']), frac)))

    def _deadend_action(self, min_score, safe_acts, rot_dir, dbg, now):
        """evaluate.py deadend_action 移植。min_score None（无模型降级）时
        以"监督器是否全否决"为准。

        2026-10-02 新增（单独立案，默认 0.0 = 逐字节旧行为）：
          · deadend_exit_hyst_s：退出低条件须**连续**满足该时长（治 <1s 的
            空↔非空抖动反复进出，日志实测最短相邻回合仅 0.6s）；
          · deadend_max_s：状态 1 持续超时 -> 状态 2 **强制放宽**（本拍让
            plan_once 用 indx 取代 indx1，即接受被监督层否决的最高分基元）。
        上游 evaluate.py:309-326 无超时/迟滞；上游唯一那处 `has_deadend_in_
        segment` 读（:835，抑制 segment 超时的模式抖动）本仓移植时被丢弃，
        本机制即其原意的接续。
        状态机本体在模块级 _deadend_step_logic（单一实现，影子档共用）。
        """
        c = self.cfg
        n_safe = len(safe_acts)
        prev = self.deadend_step
        # ③ 监督器否决率（本拍）：`veto + safe` 恒等于基元总数（256），
        # 故 vf 即"监督器判不安全的方向占比"。缺读数记 None ⇒ 条件自动关闭。
        _v, _s = dbg.get('veto_cnt'), dbg.get('safe_cnt')
        vf = (float(_v) / float(_v + _s)) if (_v is not None and _s is not None
                                              and (_v + _s) > 0) else None
        # ① 几何触发（docs/deadend_geo_case.md）：dmin<阈值 且 收拢≥阈值
        # 持续 dwell_n 拍。收拢率=**位置**回看恰 2 拍的差分（None 拍占位，
        # 与离线 H2 逐拍同构）——比「时间窗内最新样本」抗噪（试过窗选样：
        # 不规则拍距下基线变短吃单拍噪声，3 起撞案熄火）。断档大 dt 自然
        # 稀释 c ≤ 阈值不触发，无需上界。dmin=None 拍（致盲帧）连击清零。
        geo_bad = False
        dmin = dbg.get('dmin')
        self._geo_hist.append((now, dmin))
        if len(self._geo_hist) > 8:
            del self._geo_hist[0]
        clo = None
        if dmin is not None and len(self._geo_hist) >= 3:
            t2, d2 = self._geo_hist[-3]
            dt = now - t2
            if dt > 0.05 and d2 is not None:
                clo = (dmin - d2) / dt
        geo_cap = float(c.get('deadend_geo_dmin', 0.0))
        if geo_cap > 0.0:
            thr = -abs(float(c.get('deadend_geo_close_mps', 0.3)))
            on = (dmin is not None and dmin < geo_cap
                  and clo is not None and clo <= thr)
            self._geo_run = self._geo_run + 1 if on else 0
            geo_bad = self._geo_run >= int(c.get('deadend_geo_dwell_n', 2))
            dbg['geo'] = 1 if geo_bad else 0
            dbg['geo_c'] = round(clo, 2) if clo is not None else None
        step, action, forced = _deadend_step_logic(
            prev, min_score, n_safe, now, self._deadend_st,
            float(c['deadend_high']), float(c['deadend_low']),
            float(c['deadend_max_s']), float(c['deadend_exit_hyst_s']),
            veto_frac=vf, veto_hi=float(c.get('deadend_veto_frac', 0.0)),
            veto_dwell_s=float(c.get('deadend_veto_dwell_s', 0.0)),
            geo_bad=geo_bad,
            nsafe_min=int(c.get('deadend_nsafe_min', 0)))
        self.deadend_step = step
        self._deadend_forced = forced

        if step == 1 and prev == 0:
            self.has_deadend_in_segment = True
            self.deadend_dir = rot_dir
            dbg['deadend_dir'] = int(rot_dir)
            self.log('[defop_cpn] DEADEND enter (min_score=%s, safe=%d, dir=%d, '
                     'vf=%s, geo=%d)'
                     % ('NA' if min_score is None else '%.2f' % min_score,
                        n_safe, rot_dir,
                        'NA' if vf is None else '%.3f' % vf,
                        1 if geo_bad else 0))
        if step == 2 and prev != 2:
            _t0 = self._deadend_st.get('t0')
            self.log('[defop_cpn] DEADEND forced-relax (min_score=%s, safe=%d, '
                     'rot=%.1fs) -- 本拍忽略监督掩膜，接受最高分基元'
                     % ('NA' if min_score is None else '%.2f' % min_score,
                        n_safe, (now - _t0) if _t0 is not None else 0.0))

        if action == 'success':
            return RESULT_SUCCESS, None
        if action == 'stop':
            return RESULT_RUNNING, self._cmd_stop('deadend_stop')
        if action == 'rotate':
            return RESULT_RUNNING, self._cmd_rotate(
                self.yaw_rate_spot * self.deadend_dir, 'deadend_rotate')
        # forced：不改发布指令形态，交回 BT 常规路径（plan_once 步骤 6b 用
        # indx 取代 indx1）。零前进时长由影子档计量。
        # ⚠ 法证标记：强制放宽拍的 dbg['mode'] 会落成 'plan'/'plan_degraded'，
        # 与常规前进**同形**，故必须单列 dbg['forced'] 供跑批探针/对账区分。
        dbg['forced'] = True
        return RESULT_SUCCESS, None

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

    # ------------------------------------------------------------ 影子对质

    def _shadow_tick(self, n_cur, n_beta, veto_cur, veto_beta, min_score, now,
                     vf_cur=None, vf_beta=None):
        """一轮影子记账 + 1 Hz 累计行。**不触碰任何控制状态**。

        n_cur / n_beta = 现状档 / β 档在**同一候选集**里剩几个安全基元。
        `n == 0` 即 _deadend_action 判定死端（另一触发条件与掩膜无关）。
        累计量 cyc/**dead**/veto 用 1 Hz 节流发；**ep（回合）每拍都更新**。
        ⚠ 教训（2026-10-02）：ep 的边沿检测与真机 deadend_step **不是同一
        件事**（已接管回合内的空↔非空抖动也计边沿）⇒ 代理实测高估约 3×
        （代理 51/17 vs 真机 DEADEND enter 40/29）。故本案改用下面的
        _shadow_de_tick **直测**（状态机 replica + sim 秒积分），ep 只作旁证。
        """
        s = self._sh
        s['cyc'] += 1
        cur_dead = (n_cur == 0)
        beta_dead = (n_beta == 0)
        if cur_dead:
            s['cur_dead'] += 1
        if beta_dead:
            s['beta_dead'] += 1
        if cur_dead and not s['prev_cur_dead']:
            s['cur_ep'] += 1
        if beta_dead and not s['prev_beta_dead']:
            s['beta_ep'] += 1
        s['prev_cur_dead'] = cur_dead
        s['prev_beta_dead'] = beta_dead
        s['veto_cur'] += veto_cur
        s['veto_beta'] += veto_beta
        t_plan = self._shadow_de_tick(min_score, n_cur, n_beta, now,
                                      vf_cur, vf_beta)
        if t_plan is not None and (t_plan - self._sh_t) >= 1.0:
            self._sh_t = t_plan
            self.log('[defop_cpn] SHADOW bearing %s' % self._shadow_line())

    def _shadow_de_tick(self, min_score, n_cur, n_beta, now,
                        vf_cur=None, vf_beta=None):
        """死端状态机 replica（**只计量，不发布**）+ 回合时长直方图。

        与真机共用同一个 `_deadend_step_logic` ⇒ `cur` 档轨迹即真机轨迹的
        **复刻**，可与 `/drone_N/defop/debug` 的 mode 线对拍；一旦对拍失配，
        Δ 判据即作废（见 docs/deadend_timeout_case.md 的硬门）。
        积分用 plan_once 的 `now`（**sim 秒**），与 run 时钟同源。
        返回 now 供 1 Hz 节流复用（避免再取一次墙钟）。

        ③（2026-10-02）：`vf_cur / vf_beta` 与真机 `deadend_veto_frac` 门
        **必须喂同一份读数**，否则 `cur` 档不再是复刻 —— 对拍硬门会在门开启
        的第一批就失守。两档各喂自己的掩膜计数（cur 用 safe_mask、beta 用
        safe_mask_b），与真机的分母（基元总数）同源。
        """
        c = self.cfg
        raw_dt = 0.0 if self._sh_now is None else max(0.0, now - self._sh_now)
        # 步长钳位：跳过拍（nav 持笔 / 碰撞镇定窗）的间隙不得整段充进状态时长。
        dt, clamped = _clamp_dt(raw_dt, float(c.get('sup_de_dt_cap_s', 0.5)))
        if clamped:
            self._sh['gap_n'] += 1
            self._sh['gap_s'] += raw_dt
        hi, lo = float(c['deadend_high']), float(c['deadend_low'])
        cand = (float(c['deadend_max_s']), float(c['deadend_exit_hyst_s']))
        vhi = float(c.get('deadend_veto_frac', 0.0))
        vdw = float(c.get('deadend_veto_dwell_s', 0.0))
        for m, n_safe, vf in (('cur', n_cur, vf_cur), ('beta', n_beta, vf_beta)):
            r = self._sh_de[m]
            # (i) 候选参数档：预计代价（状态 1/2 的 sim 秒积分）
            step, _act, _f = _deadend_step_logic(
                r['step'], min_score, n_safe, now, r['st'], hi, lo,
                cand[0], cand[1], veto_frac=vf, veto_hi=vhi, veto_dwell_s=vdw)
            r['step'] = step
            if step == 1:
                r['t_rot'] += dt
            elif step == 2:
                r['t_force'] += dt
            # (ii) 退化档 (0,0)：死端条件本身持续多久 —— 与候选参数无关，
            #      故一次采集即可离线定阈（直方图按整秒 0..11s，11 为溢出档）
            #      ⚠ 本档**不带** ③ 门：它是"死端条件本体"的标尺，加了就没法
            #      再与历史档比较。故 m=='cur' 时用 vf=None 显式关掉。
            rstep, _a2, _f2 = _deadend_step_logic(
                r['raw_step'], min_score, n_safe, now, r['raw_st'], hi, lo,
                0.0, 0.0)
            r['raw_step'] = rstep
            # 回合时长 = **钳位后**的积分（不是 `now - ep_t0` —— 后者含跳过的
            # 间隙）。与 t_rot 同口径 ⇒ 两者必须能互相校验上。
            if rstep == 1:
                if r['ep_t0'] is None:
                    r['ep_t0'] = now
                    r['ep_acc'] = 0.0
                else:
                    r['ep_acc'] += dt
            elif r['ep_t0'] is not None:
                d = r['ep_acc']
                r['ep_t0'] = None
                r['ep_acc'] = 0.0
                r['ep_n'] += 1
                r['ep_sum'] += d
                r['ep_max'] = max(r['ep_max'], d)
                r['hist'][int(min(11, math.floor(d)))] += 1
        self._sh_now = now
        return now

    def _shadow_line(self):
        s = self._sh
        c = max(1, s['cyc'])
        cur, beta = self._sh_de['cur'], self._sh_de['beta']

        def _p50(r):
            half, acc = r['ep_n'] / 2.0, 0
            for i, k in enumerate(r['hist']):
                acc += k
                if acc >= half and half > 0:
                    return float(i)
            return 0.0

        return ('cyc=%d cur_dead=%d beta_dead=%d cur_ep=%d beta_ep=%d '
                'veto_cur=%.1f veto_beta=%.1f | t_rot_cur=%.1f t_rot_beta=%.1f '
                't_force_beta=%.1f dt=%.1f | h_cur n=%d p50=%.0f max=%.1f | '
                'h_beta n=%d p50=%.0f max=%.1f | gap n=%d raw=%.1fs'
                % (s['cyc'], s['cur_dead'], s['beta_dead'],
                   s['cur_ep'], s['beta_ep'],
                   s['veto_cur'] / float(c), s['veto_beta'] / float(c),
                   cur['t_rot'], beta['t_rot'], beta['t_force'],
                   beta['t_rot'] - cur['t_rot'],
                   cur['ep_n'], _p50(cur), cur['ep_max'],
                   beta['ep_n'], _p50(beta), beta['ep_max'],
                   s['gap_n'], s['gap_s']))

    def shadow_summary(self):
        """收尾汇总（节点 on_shutdown 调）。SIGKILL 会吃掉它 -> 1 Hz 行才是
        主证据，本行只是"跑完整"时的加分项。

        ⚠ 本案口径更正（2026-10-02）：**删掉** `Δep × 1.5s` 的估算 ——
        代理 Δep 与真机 deadend_step 不是同一件事（高估约 3×），改报
        replica **直测**的 sim 秒差 Δt_rot（β − cur）。
        """
        if self.sup_mode != 'shadow' or self._sh['cyc'] == 0:
            return None
        cur, beta = self._sh_de['cur'], self._sh_de['beta']
        return ('SHADOW bearing SUMMARY %s | Δt_rot=%+.1fs Δt_force=%+.1fs '
                '(beta-cur, replica 直测)'
                % (self._shadow_line(),
                   beta['t_rot'] - cur['t_rot'],
                   beta['t_force'] - cur['t_force']))

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

        # 近场记忆影（docs/near_field_memory_case.md）：监督器输入的 min 池
        # 记忆，只喂下方监督器/死端旋转方向/dbg 三处；VAE/latent 路径是
        # 独立节点独立话题，零改动。k_mem=0 ⇒ sup_depth 恒为本尊。
        k_mem = int(c.get('near_mem_frames', 0))
        if k_mem > 0:
            self._near_ring.append(_near_event_frame(
                self.depth, float(c['sup_min_dist'])))
            if len(self._near_ring) > k_mem:
                del self._near_ring[0]
            sup_depth = _near_field_shadow(self._near_ring, self.depth,
                                           float(c['sup_min_dist']))
        else:
            sup_depth = self.depth
        self._sup_depth = sup_depth

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

        # 3b) 基元真实航向 β（惰性一次；dyaw 列不随 pitch 变，故只需算一次）
        if self._prim_beta is None:
            _dyaw_lib = self._action_seq[:, 0, 2].astype(np.float64)
            self._prim_beta = np.arctan2(
                float(c['vel_y_gain']) * np.tanh(_dyaw_lib),
                float(c['cmd_velocity']))

        # 4) 几何监督器（evaluate.py L1077-1085 调用形态，参数可配）
        #    sup_bearing_mode: off = 旧口径；on = β 档**发布**；
        #    shadow = 发布走旧口径，另对**同一张深度图**算一套 β 档掩膜
        #    只做计数（绝不改发布出去的指令，零行为风险）。
        _beta_kw = {'margin_fn': dc.compute_margin_cone,
                    'prim_bearing': self._prim_beta}
        _pub_kw = _beta_kw if self.sup_mode == 'on' else {}
        _, _, safe_mask, blocked_with_margin, _, _ = dc.filter_actions_by_grid_3(
            sup_depth, self.fov_h_rad, self.fov_v_rad,
            self._action_seq[:, 0, 2], self._action_seq[:, 0, 1],
            yaw_lib, z_lib,
            num_yaw=int(c['num_yaw']), num_vel_z=int(c['num_vel_z']),
            min_dist=float(c['sup_min_dist']),
            min_ratio=float(c['sup_min_ratio']),
            drone_radius=float(c['sup_drone_radius']), **_pub_kw)
        safe_mask = np.asarray(safe_mask).ravel()
        dbg['safe_cnt'] = int(safe_mask.sum())
        dbg['veto_cnt'] = int((~safe_mask).sum())
        dbg['sup_mode'] = self.sup_mode
        # ③ 监督层未拦住 —— 可观测性仪器（2026-10-02 立案，**纯遥测，不改行为**）
        #   dnear/dnear2/dmin：监督器**输入**（image_filled）里 < sup_min_dist /
        #     < 2m 的有效像素数、最近有效距离 ⇒ 分清「相机根本没看见（dnear=0）」
        #     与「看见了却没拦住（dnear>0 而 veto=0 ⇒ 扇区/掩膜映射 bug）」。
        #   near_az_deg：近像素的平均方位（针孔反投影，与 prim_bearing 同式）
        #     ⇒ 与 yaw_deg 对质即可分清「正对冲进来（1m 窗太短）」还是「侧向
        #     FOV 盲（方位超 ±43.5°）」。
        #   yaw_deg / vw：机体航向与世界速度 ⇒ 撞前接近角/接近速率的真值源
        #     （此前 verify.txt 只记 drone0/1，撞机位姿取不到）。
        _dd = sup_depth
        _val = np.isfinite(_dd) & (_dd > 0.0)
        _near = _val & (_dd < float(c['sup_min_dist']))
        dbg['dnear'] = int(np.count_nonzero(_near))
        dbg['dnear2'] = int(np.count_nonzero(_val & (_dd < 2.0)))
        dbg['dmin'] = round(float(_dd[_val].min()), 3) if _val.any() else None
        if _near.any():
            _w = _dd.shape[1]
            _fx = (_w / 2.0) / math.tan(self.fov_h_rad / 2.0)
            _cols = np.nonzero(_near)[1]
            dbg['near_az_deg'] = round(float(np.degrees(
                -np.arctan((float(_cols.mean()) - _w / 2.0) / _fx))), 1)
            dbg['near_d'] = round(float(_dd[_near].min()), 3)
        dbg['yaw_deg'] = round(math.degrees(self.odom['yaw']), 1)
        if self.odom.get('twist_world') is not None:
            dbg['vw'] = [round(float(x), 2)
                         for x in np.asarray(self.odom['twist_world'])[:3]]
        safe_mask_b = None
        if self.sup_mode == 'shadow':
            _t0 = time.time()
            _, _, safe_mask_b, _, _, _ = dc.filter_actions_by_grid_3(
                self.depth, self.fov_h_rad, self.fov_v_rad,
                self._action_seq[:, 0, 2], self._action_seq[:, 0, 1],
                yaw_lib, z_lib,
                num_yaw=int(c['num_yaw']), num_vel_z=int(c['num_vel_z']),
                min_dist=float(c['sup_min_dist']),
                min_ratio=float(c['sup_min_ratio']),
                drone_radius=float(c['sup_drone_radius']), **_beta_kw)
            safe_mask_b = np.asarray(safe_mask_b).ravel()
            dbg['sh_sup_ms'] = round((time.time() - _t0) * 1000.0, 2)

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
            indx_base = indx                    # 影子对质的同一候选集
        else:
            # 降级：无模型 ⇒ 全体基元就是候选集。indx 必须是**独立变量**
            # （超时强制放宽要拿它取代 indx1，见步骤 6b）；旧代码此处无
            # indx，直接引用会 NameError。
            indx = np.arange(safe_mask.size)
            indx1 = indx[safe_mask[indx].ravel()]
            min_score = None
            dbg['col_prob'] = None
            indx_base = indx

        # 死端门的第二条件（`n_safe > 0` 用的就是 len(indx1)，**不是** safe_cnt）
        # —— 这是 ③a「veto 饱和却照飞」的关键量，必须可观测。
        dbg['n_indx1'] = int(len(indx1))
        dbg['n_indx'] = int(len(indx))

        # 5b) 影子对质：只对**同一候选集**换掩膜，只记数，不改 indx1。
        #     _deadend_action 的两个触发条件里 min_score>=deadend_high 与映射
        #     无关，故 Δ死端**完全**由 len(indx1)==0 贡献 —— 单变量对照。
        if safe_mask_b is not None:
            indx1_b = indx_base[safe_mask_b[indx_base].ravel()]
            # ③ 门的读数必须**原样喂进 replica**，否则 `cur` 档不再是真机的
            # 复刻（对拍硬门当场失守，Δ 判据作废）。分母用掩膜的基元总数，
            # 与真机 `dbg['veto_cnt']+dbg['safe_cnt']` 同源同值。
            _nv = int(safe_mask.size)
            self._shadow_tick(len(indx1), len(indx1_b),
                              int((~safe_mask).sum()),
                              int((~safe_mask_b).sum()), min_score, now,
                              vf_cur=(int((~safe_mask).sum()) / float(_nv))
                              if _nv else None,
                              vf_beta=(int((~safe_mask_b).sum()) / float(_nv))
                              if _nv else None)

        # 6) 行为树（原序：deadend -> slowdown -> reach_goal -> [direction/
        #    timeout 对 PLANNING_TYPE=1 恒 SUCCESS，省略]）
        rot_dir = dc.choose_dead_end_rotation(
            sup_depth, blocked_with_margin, self.fov_h_rad,
            num_yaw=int(c['num_yaw']), num_vel_z=int(c['num_vel_z']))
        res_dead, cmd = self._deadend_action(min_score, indx1, rot_dir, dbg,
                                             now)
        if res_dead != RESULT_SUCCESS:
            dbg['mode'] = cmd['mode']
            return cmd, dbg

        # 6b) 强制放宽（仅 deadend_max_s>0 且旋转超时后置位）：本拍用 indx
        #     （分数阈值集，**含被监督层否决者**）取代 indx1 ⇒ 接受最高分
        #     基元、换"有界前进"。默认档永不置位 = 逐字节旧行为。
        if self._deadend_forced:
            indx1 = indx

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

        # 候选 2（2026-10-05 立案）：选择代价注入"横向间隙"偏向。病根：本步在
        # indx1 里只按 |yaw−goal_yaw| 选（最朝 goal），代价无横向间隙项 ⇒ 直线
        # 路径贴着**偏轴**树（机头方位 20~40°、机身半径 0.35m）擦过（撞拍实测
        # 19/19 树均在 ±43.5° 视场内、最近接近拍方位恒 ±30~42°）。
        # 间隙代理 = 该基元航向所在扇区的**监督器否决率**（全 256 口径：0=该向
        # 全放行 / 1=全否决）；yaw_cost += w·该项 ⇒ 把"朝目标"与"朝空"折中。
        # follow_clear_nsafe>0 时只在 len(indx1)<该值（危险）时启用；<=0 恒启用。
        # w=0.0 = 关闭 = 逐字节旧行为。
        _cw = float(c.get('follow_clear_w', 0.0))
        _clear = None
        if _cw > 0.0:
            _gate = int(c.get('follow_clear_nsafe', 0))
            if _gate <= 0 or len(indx1) < _gate:
                _key = np.round(self._action_seq[:, 0, 2], 5)
                _vet = ~safe_mask
                _fr = {}
                for _k in np.unique(_key):
                    _m = _key == _k
                    _fr[_k] = float(_vet[_m].mean())
                _clear = np.array([_fr[_k] for _k in _key])
                dbg['clear_w'] = _cw
                dbg['clear_bias'] = round(float(_clear[indx1].mean()), 3)

        if self.models is None:
            # 降级：纯监督器，目标航向差最小；同档 tie-break 用高度差
            # （与模型路径同一 el_cost；yaw 库 32 档无精确 0，最近档 ±0.022 rad）
            dbg['mode'] = 'plan_degraded'
            yaw_cost = np.abs(
                (self._action_seq[indx1, 0, 2] + self.odom['yaw'] - goal_yaw
                 + math.pi) % (2 * math.pi) - math.pi)
            if _clear is not None:
                yaw_cost = yaw_cost + _cw * _clear[indx1]
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
            if _clear is not None:
                yaw_cost = yaw_cost + _cw * _clear[indx1]
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
                  'sup_drone_radius', 'gov_dist', 'gov_stop',
                  'gov_floor'):
            if rospy.has_param('~' + k):
                c[k] = float(rospy.get_param('~' + k))
        for k in ('use_ut', 'allow_yaw_at_waypoint', 'allow_degraded',
                  'gov_enable'):
            if rospy.has_param('~' + k):
                c[k] = bool(rospy.get_param('~' + k))
        # sup_bearing_mode 是**枚举字符串**，不是 bool：yaml 经 rosparam 到达
        # 的是 str，若走上面的 bool 分支则 bool('off') == True（静默走 on 档）。
        # 故单列白名单强转，未知值一律回落 off —— 与 defop_cfg_preset.py 的
        # 非数值键处理配套。
        _m = str(c.get('sup_bearing_mode', 'off')).strip().lower()
        c['sup_bearing_mode'] = _m if _m in ('off', 'shadow', 'on') else 'off'

        self.base = '/drone_%s' % drone_id
        self.planner = DefopCPNPlanner(c, log=self._log)
        self.load_waypoints_once()

        # ④ 监督层口径活体回声：静默误解析（yaml 掉了/键名写错 -> 回落 off）
        # 与"本来就该 off"同形，故此行走 stdout 是唯一可对质的证据。
        self._log('[defop_cpn] sup_bearing_mode=%s (off|shadow|on)'
                  % self.planner.sup_mode)
        # 死端超时/迟滞活体回声：静默误解析（yaml 掉了-> 回落 0.0）与
        # "本来就该 0.0"同形，故须显式行作对质证据。
        self._log('[defop_cpn] deadend: max_s=%.1f exit_hyst_s=%.1f veto_frac=%.3f '
                  '(0.0 = 上游两态 latch / 逐字节旧行为；veto_frac 0.0 = ③ 条件关闭)'
                  % (float(c['deadend_max_s']),
                     float(c['deadend_exit_hyst_s']),
                     float(c.get('deadend_veto_frac', 0.0))))
        self._log('[defop_cpn] cand: deadend_nsafe_min=%d follow_clear_w=%.2f '
                  'follow_clear_nsafe=%d (0/0.0/0 = 关闭 = 逐字节旧行为)'
                  % (int(c.get('deadend_nsafe_min', 0)),
                     float(c.get('follow_clear_w', 0.0)),
                     int(c.get('follow_clear_nsafe', 0))))
        rospy.on_shutdown(self._shutdown_summary)

        # P3 终端交接：cpn 持笔旗（nav 每拍广播 /planning/cpn_active）。
        # 默认 False = nav 持笔、本节点完全静默（旗未到达前不抢帧）。
        self.active = False
        self._flag_t = None   # 旗到达时刻（活性门：>1.0s 未更新=nav 死/停播）
        # 族④ cpn_ready 握手：首次真实发布指令后 latched 广播 True——nav 收到
        # 前不交笔（暖机期 nav 照飞，堵 3.6~4.3s 指令真空）。见 nav_node
        # _on_cpn_ready 注释；回滚=git checkout 本文件。
        self._ready = False
        self.pub_ready = rospy.Publisher(self.base + '/planning/cpn_ready',
                                         Bool, queue_size=1, latch=True)
        # pen_authority_case 刀一（2026-10-04）：起飞闸门暖机旗（latched）。
        # models 装载完成 且 odom/depth/latent 三输入各见过 ≥1 帧即置位——
        # 与 goal/active 无关（active 依赖 has_goal，起飞前恒 False；闸门若
        # 等 cpn_ready = 结构性死锁：无 goal ⇒ CPN 静默 ⇒ ready 永不置位）。
        # executor 侧 pen_authority=cpn_forever 时起飞前置此旗，60s 超时
        # fail-open（allow_degraded 降级/CPN 死不冻结全队）。
        self._models_ok = False
        self._have = {'odom': False, 'depth': False, 'latent': False}
        self._warm = False
        self.pub_warm = rospy.Publisher(self.base + '/planning/cpn_warm',
                                        Bool, queue_size=1, latch=True)

        # 刀一.5：pen_authority=cpn_forever ⇒ 原始令发中继话题，由 nav 套
        # 同源 Boids sep 混合后统一组装发 vel_cmd（smoke#1 机间新形 3 起根因
        # 修复：出生排 sep 保护窗 + 巡航全程 sep 仲裁，案卷 §三.4）。nav 档
        # 直发 vel_cmd = 逐字节旧拓扑。settle/异常零令同走 pub_cmd——中继侧
        # 零令放行不混 sep，保 settle 冻结语义。读配置失败回落 nav 档。
        _pl_v = {}
        try:
            _pl_v = (_zx_cfg.load("sim_settings.yaml") or {}).get(
                "planner", {}) or {}
        except Exception as _e:
            rospy.logerr('[defop_cpn drone_%s] sim_settings read fail (%s) '
                         '-> pen_relay=nav fallback', drone_id, _e)
        _pa = _pl_v.get("pen_authority", "nav") if isinstance(_pl_v, dict) else "nav"
        # 严格全等（审查 wf_ba224dac major：与 nav pen_mode_of 同款「不识
        # 一律 nav」保守哲学——宽松 strip/lower 会让变体值下 CPN 走中继而
        # nav 不订 ⇒ 巡航段 vel_cmd 零写者死寂）。
        self._pen_relay = (str(_pa) == "cpn_forever")
        _cmd_topic = (self.base + '/planning/cpn_vel_raw' if self._pen_relay
                      else self.base + '/vel_cmd')
        self._log('[defop_cpn] pen_relay=%s cmd_topic=%s '
                  '(cpn_forever: raw 令交 nav sep 混合中继 / nav: 直发 vel_cmd)'
                  % (self._pen_relay, _cmd_topic))
        self.pub_cmd = rospy.Publisher(_cmd_topic, Twist, queue_size=1)
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
        # ③ 碰撞镇定：恢复链路（gazebo_collision_monitor）碰撞旗上升沿 ->
        # 本节点 HOLD 零速 collision_settle_s；弹开+冷却窗内不复规划，
        # 治"同树秒级复撞"（G1 法证：d2 2.4s 二撞 / d5-d6 2.6s 同位）
        self._settle_until = None
        self._col_prev = False
        rospy.Subscriber(self.base + '/collision', Bool,
                         self.cb_collision, queue_size=1)

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

    def _shutdown_summary(self):
        """影子档收尾汇总（SIGKILL 吃不到，故 1 Hz 行才是主证据）。"""
        try:
            line = self.planner.shadow_summary()
            if line:
                self._log('[defop_cpn] %s' % line)
        except Exception as e:                       # 收尾不许再抛
            rospy.logwarn('[defop_cpn] shadow summary failed: %r', e)

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
                # WSL/dxg 稳定性（2026-09-30 法证）：TF 默认每进程独占整卡
                # 显存，6 机同时建上下文会把 dxg 打崩（整机重启）。故限幅每进程，
                # 并把实际 cap 打进日志以便复位取证。
                #
                # 2026-10-01 上限重定：原 1024 是过量四倍——单进程实测真实需求
                # 仅 ~190MB（cap=192 照跑且延迟不变），而 nvidia-smi 全卡口径一个
                # 进程吃 ~1125MB。6×1125=6750 加上桌面基线 1252 已达 8002/8188，
                # 「卡已被上限本身占满」正是 09-30「VAE 上 GPU 把 CPN 从 9.6Hz 打到
                # 1.17Hz」的真因（VAE 自身仅需 62MB，不是它抢的）。收到 512 后
                # 6×~615=3690，叠加 6 个 GPU-VAE 仍余三成。cap 过紧会走 TF OOM ->
                # logerr -> allow_degraded:false -> exit(1)，是响的，不会静默降级。
                cap_mb = int(rospy.get_param('~cpn_mem_cap_mb', 512))
                try:
                    _gpus = tf.config.experimental.list_physical_devices('GPU')
                    for _g in _gpus:
                        tf.config.set_logical_device_configuration(
                            _g, [tf.config.LogicalDeviceConfiguration(
                                memory_limit=cap_mb)])
                    rospy.loginfo('[defop_cpn] GPU 显存上限 %dMB/进程 '
                                  '(devices=%d)', cap_mb, len(_gpus))
                except Exception as _ge:
                    rospy.logwarn('[defop_cpn] GPU 限幅失败，退增量分配: %r',
                                  _ge)
                    try:
                        for _g in tf.config.experimental.list_physical_devices(
                                'GPU'):
                            tf.config.experimental.set_memory_growth(_g, True)
                    except Exception:
                        pass
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
                self._models_ok = True
                self._maybe_warm()
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
        self._have['latent'] = True
        self._maybe_warm()

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist.linear
        self.planner.update_odom([p.x, p.y, p.z], q.x, q.y, q.z, q.w,
                                 [t.x, t.y, t.z], rospy.Time.now().to_sec(),
                                 yaw_rate=float(msg.twist.twist.angular.z))
        self.planner.recover_input('odom')
        self._have['odom'] = True
        self._maybe_warm()

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
        self._have['depth'] = True
        self._maybe_warm()

    def _maybe_warm(self):
        """起飞闸门暖机旗（pen_authority_case §三.1）：models + 三输入各一帧
        ⇒ latched 置位一次。models 装载失败（allow_degraded 降级）则永不
        置位 -> executor 侧 60s 超时 fail-open 放行（logerr 留痕）。"""
        if self._warm:
            return
        if self._models_ok and all(self._have.values()):
            self._warm = True
            self.pub_warm.publish(Bool(data=True))
            self._log('[defop_cpn] cpn_warm=True (models+odom+depth+latent)')

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

    def cb_collision(self, msg):
        """③ 恢复链路碰撞旗（latched）：上升沿 = 新碰撞 -> 镇定窗起点。
        窗内 on_timer 只发 HOLD 零速不复规划（弹开/冷却由监控器 100Hz
        覆写，本节点零速兜底保证交笔前后流不断）。"""
        act = bool(msg.data)
        if act and not self._col_prev:
            settle = float(self.planner.cfg['collision_settle_s'])
            self._settle_until = rospy.Time.now().to_sec() + settle
            rospy.logwarn('[defop_cpn] collision -> settle HOLD %.1fs', settle)
        self._col_prev = act

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
        now = rospy.Time.now().to_sec()
        # ③ 碰撞镇定窗：HOLD 零速不复规划（流不断，桥不进 stale-HOLD）；
        # 窗满清除，交笔回来再复规划。
        if self._settle_until is not None:
            if now < self._settle_until:
                self.pub_cmd.publish(Twist())
                return
            self._settle_until = None
        if not self._busy.acquire(False):
            return  # 上个周期推理还没完（CPU 慢），跳过本拍
        try:
            import time as _time
            t0 = _time.time()
            cmd, dbg = self.planner.plan_once(now)
            # 校验 V1-B1/V2-M3 双写窗：nav 终端接管旗可在本拍推理途中翻转
            # （plan_once 0.2-0.5s），发布前复查——陈旧指令丢弃（return 走
            # finally 释放 _busy），双写窗从"一次推理时长"压到零。
            if not self.active or not self._flag_fresh():
                return
            dbg['t_plan_ms'] = int((_time.time() - t0) * 1000)
            # ① 速度保险夹：plan 路径理论上 ≤ hypot(cmd_velocity, vel_y_gain)
            # （基元库焊死 1.3 + 侧移 tanh 上限 0.5 = 1.39），本夹只在回归时
            # 咬合——正常跑应零 WARN；咬合即说明上游包络被绕过。
            vx, vy, vz = float(cmd['vx_w']), float(cmd['vy_w']), \
                float(cmd['vz_w'])
            h = math.hypot(vx, vy)
            h_max = math.hypot(float(self.planner.cfg['cmd_velocity']),
                               float(self.planner.cfg['vel_y_gain']))
            if h > h_max > 0.0:
                s = h_max / h
                vx, vy = vx * s, vy * s
                rospy.logwarn_throttle(
                    5.0, '[defop_cpn] speed clamp bound: h=%.2f>%.2f '
                    '(envelope bypass?)', h, h_max)
            vz_max = float(self.planner.cfg['vz_abs_max'])
            if abs(vz) > vz_max > 0.0:
                vz = math.copysign(vz_max, vz)
            # ② 预测性减速（2026-09-30）：见 DefopCPNPlanner._gov_scale。
            # 只缩水平模长（世界系 vx_w/vy_w），不改向、不动 vz；关时 frac
            # 恒 1.0，本 if 不进 -> 逐字节旧行为。
            if self.planner.cfg['gov_enable']:
                _gf = self.planner._gov_scale()
                if _gf < 1.0:
                    vx, vy = vx * _gf, vy * _gf
                    dbg['gov_frac'] = _gf
                    rospy.loginfo_throttle(
                        3.0, '[defop_cpn] drone_%s pred-slow frac=%.2f',
                        self.planner.cfg['drone_id'], _gf)
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.linear.z = vx, vy, vz
            tw.angular.z = float(cmd['yaw_rate'])
            self.pub_cmd.publish(tw)
            if not self._ready:
                # 族④ 握手：首拍指令已实际发布 = 暖机完成，通知 nav 可以交笔。
                # 只在真实规划路径置位（异常兜底零速不算——planner 还没证明
                # 自己能出拍）。
                self._ready = True
                self.pub_ready.publish(Bool(data=True))
            # ⑤ 监督层可观测性：veto 计数节流上日志（G1 法证教训：veto
            # 一次都看不见 = 没拦还是没打说不清）
            if int(dbg.get('veto_cnt', 0)) > 0:
                rospy.loginfo_throttle(
                    2.0, '[defop_cpn] drone_%s veto=%d safe=%d min_score=%s',
                    self.planner.cfg['drone_id'], int(dbg['veto_cnt']),
                    int(dbg.get('safe_cnt', 0)),
                    ('%.2f' % dbg['col_prob'])
                    if dbg.get('col_prob') is not None else 'NA')
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
