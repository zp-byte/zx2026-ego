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
  - 碰撞镇定：/collision 上升沿 → SETTLE_S 零速（零令走中继=中继侧放行不混
    sep，保 settle 冻结语义；镜像 defop_cpn settle 设计）。
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
        col = bool(msg.data)
        with self._lock:
            rising = col and not self._col_prev
            self._col_prev = col
            if rising:
                now = rospy.get_rostime().to_sec()
                self._settle_until = now + SETTLE_S

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
            flag_ok = self._flag_ok(rospy.get_rostime().to_sec())
            now = rospy.get_rostime().to_sec()

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
