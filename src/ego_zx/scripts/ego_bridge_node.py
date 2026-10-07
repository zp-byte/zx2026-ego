#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ego_bridge_node.py — ego-planner PositionCommand → Twist 桥（ego 当笔）。

镜像 defop_cpn_node 的 pen_relay 契约（案卷 docs/pen_authority_case.md §三.4）：
  - 输出走 /drone_<i>/planning/cpn_vel_raw，nav 套同源 Boids sep 后统一组装
    发 /vel_cmd（nav 是唯一发布者）；桥绝不直发 vel_cmd。
  - 握手三件：cpn_warm（odom+depth 各见 ≥1 帧，latched）/ cpn_ready（首次真实
    出令后，latched）/ defop/debug 心跳（nav stall 看门狗同名消费者，零改动）。
  - 持笔门：nav 广播 /planning/cpn_active（True=cruise 窗口交笔）；旗失新
    （>1.0s）= nav 死/停播 → 桥 yield 静默（过渡拍补一发零速，语义见 defop_cpn
    同款）。
  - 恢复期让出执行器（yield_actuator，默认开）：/collision 上升沿 → pub_raw
    整窗静默（**含零**），vel_cmd 归 gazebo_collision_monitor 独占（弹开→冷却），
    下降沿交还。这是 python 后端 lock_cmd 的 ego 侧镜像——那边是消费端丢弃
    vel_cmd（world_node._on_vel_cmd），这边消费端是 C++ 插件改不得，故改生产端。
    关掉（false）退回旧行为：发 SETTLE_S 零速，该零速经 nav 中继以 50Hz 上
    /vel_cmd，与监控器 100Hz 弹开令逐拍对撞、掐死逃逸脉冲（2026-10-07 冒烟实测）。
  - 帧约定：cmd.velocity 为 world ENU（bspline 世界系导数）→ 与 odom 同帧直套
    Twist.linear；angular.z = cmd.yaw_dot（traj_server 已前视+限幅 π/s）。
    真机换源：sim 恒等；真机改发局部系 /setpoints_cmd（同码换源，见
    docs/EGO_DESIGN.md §四）。
  - 轨迹结束段（traj_server.cpp:200 return 停发 cmd）：0.5s 无 cmd → 补一发
    零速后静默（drone_vel_plugin 0.5s 断链归零悬停兜底，双保险同语义）。

纯 rospy 实现（无 TF/torch 依赖），系统 python3 直跑。
"""

import json
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String

FLAG_STALE_S = 1.0        # cpn_active 旗活性门（defop_cpn 同款）
CMD_TIMEOUT_S = 0.5       # pos_cmd 断流判据（对齐 drone_vel_plugin 断链窗）
SETTLE_S = 1.0            # 碰撞镇定窗（defop_cpn collision_settle_s 同值）
YIELD_MAX_S = 15.0        # 让出执行器的看门狗上限——监控器若死则永远收不到
                          # /collision 下降沿，防永久静默（正常窗 2.5s，留 6x 裕度）


class EgoBridge(object):
    def __init__(self):
        self.drone_id = str(rospy.get_param('~drone_id', '0'))
        self.ns = '/drone_%s' % self.drone_id

        # ---- 状态 ----
        self._lock = threading.Lock()
        self._cmd = None              # 最近 PositionCommand
        self._cmd_t = None            # 到达时刻（ros time, s）
        self._active = False          # nav 持笔旗现值
        self._flag_t = None           # 旗到达时刻（活性门）
        self._was_active = False      # 上一拍在 TRACKING（过渡拍补零用）
        self._settle_until = None     # 碰撞镇定截止时刻
        self._col_prev = False        # /collision 上升沿检测
        self._ready = False           # cpn_ready 已发
        self._have = {'odom': False, 'depth': False}
        self._warm = False            # cpn_warm 已发
        self._yield = False           # 恢复窗让出执行器中（lock_cmd 契约）
        self._yield_t = None          # 让出起始时刻（看门狗计时）

        # ---- 恢复期让出执行器：python 后端 lock_cmd 的 ego 侧镜像 --------------
        # 语义真源 = arena_world/scripts/world_node.py 的 lock_cmd（"恢复期指令
        # 隔离：弹开/冷却/冻结指令独占 cmd_vel"，那边在 _on_vel_cmd 里**丢弃**
        # 执行器令）。Gazebo 后端的消费端是 C++ 插件 drone_vel_plugin（改不得），
        # 故在**生产端**镜像同一语义：恢复窗内桥不发任何令，vel_cmd 整窗归
        # gazebo_collision_monitor（弹开 100Hz → 冷却零速）。
        # 缺此门时桥仍发 SETTLE_S 零速（50Hz，经 nav 的 _on_pen_raw 中继上
        # /vel_cmd），与监控器弹开令逐拍对撞 —— 2026-10-07 仪器化冒烟实测：
        # d2 被令 2.0m/s 持续 1.0s 只位移 0.10m、离撞点峰值 0.25m（<0.60m 判
        # 楔死）；零令串采样间隔 0.017-0.020s=50Hz 正是本桥定时器，而 nav 主
        # 循环是 20Hz（0.050s）—— 写者指纹互为佐证。
        # 读配置失败绝不 raise：桥起不来 = 六机全冻（项目血案），退默认值即可。
        self.yield_actuator = True
        try:
            from zx2026_common import config as _cfg
            _rc = (_cfg.load('sim_settings.yaml')
                   .get('collision', {}).get('recovery', {}))
            _v = _rc.get('yield_actuator', True)
            if isinstance(_v, str):   # YAML1.1 裸 on/off 会被解析成字符串
                _v = _v.strip().lower() in ('1', 'true', 'yes', 'on')
            self.yield_actuator = bool(_v)
        except Exception as _e:
            rospy.logwarn('[ego_bridge drone_%s] collision.recovery 读取失败'
                          '(%s) -> yield_actuator 退默认 True',
                          self.drone_id, _e)

        # ---- 发布（中继契约，见模块 docstring）----
        self.pub_raw = rospy.Publisher(
            self.ns + '/planning/cpn_vel_raw', Twist, queue_size=1)
        self.pub_ready = rospy.Publisher(
            self.ns + '/planning/cpn_ready', Bool, queue_size=1, latch=True)
        self.pub_warm = rospy.Publisher(
            self.ns + '/planning/cpn_warm', Bool, queue_size=1, latch=True)
        self.pub_dbg = rospy.Publisher(
            self.ns + '/defop/debug', String, queue_size=2)

        # ---- 订阅 ----
        rospy.Subscriber(self.ns + '/planning/pos_cmd', PositionCommand,
                         self._on_cmd, queue_size=10)
        rospy.Subscriber(self.ns + '/odom', Odometry, self._on_odom,
                         queue_size=10)
        rospy.Subscriber(self.ns + '/depth/image_raw', Image,
                         self._on_depth, queue_size=1)
        rospy.Subscriber(self.ns + '/planning/cpn_active', Bool,
                         self._on_active, queue_size=1)
        rospy.Subscriber(self.ns + '/collision', Bool, self._on_collision,
                         queue_size=1)

        rospy.Timer(rospy.Duration(0.02), self._tick)   # 50Hz 组装拍
        rospy.loginfo(
            '[ego_bridge drone_%s] ready: raw=%s '
            '(ego 当笔: PositionCommand→Twist 交 nav sep 中继; warm/ready/debug '
            '握手镜像 defop_cpn)',
            self.drone_id, self.pub_raw.resolved_name)

    # ---- 回调 ----
    def _on_cmd(self, msg):
        with self._lock:
            self._cmd = msg
            self._cmd_t = rospy.get_rostime().to_sec()

    def _mark_input(self, kind):
        """输入首帧登记；两输入齐 → cpn_warm latch 一发（锁内判重防双发）。"""
        publish = False
        with self._lock:
            self._have[kind] = True
            if not self._warm and all(self._have.values()):
                self._warm = True
                publish = True
        if publish:
            self.pub_warm.publish(Bool(data=True))
            rospy.loginfo('[ego_bridge drone_%s] cpn_warm=True (odom+depth)',
                          self.drone_id)

    def _on_odom(self, _msg):
        self._mark_input('odom')

    def _on_depth(self, _msg):
        self._mark_input('depth')

    def _on_active(self, msg):
        with self._lock:
            self._active = bool(msg.data)
            self._flag_t = rospy.get_rostime().to_sec()

    def _on_collision(self, msg):
        """恢复窗边沿自同步：上升沿=恢复开始（让出），下降沿=恢复结束（交还）。

        下降沿由监控器在弹开+冷却走完、把执行器交还 nav 的那一拍发出
        （gazebo_collision_monitor.py:257 与 st["t"]=-1.0 同拍）。故窗口长度
        **不需要**在桥里镜像 bounce_dt/cooldown_dt——边沿即真值，改监控器
        缺省也不会让本窗咬早/咬晚（旧方案用 cap_ahead=bounce+cooldown 做窗口
        算术，是外部约束的镜像，改一个忘一个就错位）。
        """
        col = bool(msg.data)
        handed_back = None
        with self._lock:
            rising = col and not self._col_prev
            self._col_prev = col
            now = rospy.get_rostime().to_sec()
            if rising:
                if self.yield_actuator:
                    self._yield = True
                    self._yield_t = now
                else:
                    self._settle_until = now + SETTLE_S
            elif not col and self._yield:
                self._yield = False
                handed_back = now - (self._yield_t if self._yield_t else now)
        if handed_back is not None:
            rospy.loginfo('[ego_bridge drone_%s] collision window closed '
                          '(%.2fs) -> 交还执行器', self.drone_id, handed_back)

    # ---- 组装拍 ----
    def _flag_ok(self, now):
        return (self._active and self._flag_t is not None
                and (now - self._flag_t) <= FLAG_STALE_S)

    def _tick(self, _evt):
        with self._lock:
            cmd = self._cmd
            cmd_t = self._cmd_t
            was_active = self._was_active
            settle_until = self._settle_until
            yielding = self._yield
            yield_t = self._yield_t
            flag_ok = self._flag_ok(rospy.get_rostime().to_sec())
            now = rospy.get_rostime().to_sec()

        # 恢复窗让出执行器：pub_raw 整窗**不发任何令（含零）**—— vel_cmd 归
        # gazebo_collision_monitor 独占（弹开 100Hz → 冷却零速）。
        # 必须是静默而非发零：发零就是与弹开令对撞的那个写者（见 __init__）。
        # 插件 0.5s 断链兜底由监控器弹开/冷却窗内的高频流量满足（≥50Hz 实测）。
        # 但 **dbg 心跳必须继续**：它是 nav stall 看门狗的唯一输入
        # （pen_stall_freeze_t=1.0s，nav_node.py:2698-2712）。心跳一停，nav 主
        # 循环立刻改发 Twist() 零速（PEN_STALL freeze，画面日志同源）—— 等于
        # 换个写者继续掐弹开。本窗实测 2.5s > 1.0s 阈值 ⇒ 不喂心跳必每窗触发。
        # 该看门狗只记到达时刻、零解析（nav_node._on_cpn_dbg），喂活即静默 nav。
        # 看门狗 YIELD_MAX_S：监控器若死则永远收不到下降沿，防永久静默。
        if yielding:
            if now - (yield_t if yield_t is not None else now) <= YIELD_MAX_S:
                self.pub_dbg.publish(json.dumps({
                    't': round(now, 3), 'mode': 'yield',
                    'reason': 'collision_recovery'}))
                return
            with self._lock:
                if self._yield and self._yield_t == yield_t:
                    self._yield = False
            rospy.logwarn_throttle(
                5.0, '[ego_bridge drone_%s] collision window 超 %.0fs 未收下降沿'
                '（监控器已死？）-> 收回执行器', self.drone_id, YIELD_MAX_S)

        # 持笔权失守：补一发零速后静默（nav 收回笔自会发布）
        if not flag_ok:
            if was_active:
                self.pub_raw.publish(Twist())
                with self._lock:
                    self._was_active = False
            return

        # 碰撞镇定：零令走中继（放行不混 sep = settle 冻结）
        if settle_until is not None:
            if now < settle_until:
                self.pub_raw.publish(Twist())
                return
            with self._lock:
                self._settle_until = None

        # pos_cmd 断流（轨迹结束悬停段）：补一发零速后静默（插件断链悬停兜底）
        if cmd is None or cmd_t is None or (now - cmd_t) > CMD_TIMEOUT_S:
            if was_active:
                self.pub_raw.publish(Twist())
                with self._lock:
                    self._was_active = False
            return

        # NaN 护栏（2026-10-05 run4 bag 法证）：pos_cmd 关键字段非有限→丢帧，
        # 让插件断链保护（0.5s）自然接管悬停，优于硬零。traj_server 的
        # yaw_dot 除零 NaN 已在源端根治（VENDORED.md 补丁 3），此为对
        # 其它 NaN 源的终端护栏。
        if not (math.isfinite(cmd.velocity.x) and math.isfinite(cmd.velocity.y)
                and math.isfinite(cmd.velocity.z) and math.isfinite(cmd.yaw_dot)):
            return

        tw = Twist()
        tw.linear.x = cmd.velocity.x
        tw.linear.y = cmd.velocity.y
        tw.linear.z = cmd.velocity.z
        tw.angular.z = cmd.yaw_dot
        self.pub_raw.publish(tw)

        if not self._ready:
            self._ready = True
            self.pub_ready.publish(Bool(data=True))

        self.pub_dbg.publish(json.dumps({
            't': round(now, 3), 'mode': 'track',
            'vx': round(tw.linear.x, 3), 'vy': round(tw.linear.y, 3),
            'vz': round(tw.linear.z, 3), 'yawdot': round(tw.angular.z, 3)}))
        with self._lock:
            self._was_active = True


if __name__ == '__main__':
    rospy.init_node('ego_bridge', anonymous=False)
    EgoBridge()
    rospy.spin()
