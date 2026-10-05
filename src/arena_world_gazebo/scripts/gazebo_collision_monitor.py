#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world_gazebo::gazebo_collision_monitor — Gazebo 后端的碰撞检测与恢复。

python 后端：world_node.py 检测碰撞 → /drone_<i>/collision + bounce/cooldown 恢复，
nav 收到 collision=True 后把碰撞点注入 _collision_obs 强制占用 → A* 绕开
（验证过：R1 13 撞全恢复、无永久冻结）。
Gazebo 后端：物理在 gzserver(ODE) 里，但缺碰撞事件源 —— /drone_<i>/collision
无人发布 → nav 的 _on_collision 收不到 True → 撞树不注入占用、不绕开 → 点云
避障把指令压零 → 无人机永久卡死在树边（2026-08-27 实测 drone 1 卡 9 分钟）。

本节点补齐事件源 + 恢复状态机（对齐 world_node 语义，不动 C++/不重编译）：
  碰撞（球-柱树干 / 球-AABB 灌木 / 互撞）→ 发布 /drone_<i>/collision=True
    → bounce_dt 弹开（100Hz vel_cmd 覆盖 nav 的 20Hz，恢复窗口内占优）
    → cooldown_dt 悬停 → 恢复，nav 接管。
  累计超过 max_collisions 次 → 永久冻结（持续发 0，对齐 python 后端语义）。
"""
import math

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene


class CollisionMonitor:
    def __init__(self):
        rospy.init_node("gazebo_collision_monitor", anonymous=True)
        s = self.scene = Scene()
        self.drone_count = s.drone_count
        self.dr = s.drone_radius
        self.gz = s.venue["ground_z"]
        self.trees = [o for o in s.obstacles if o.kind == "tree"]
        self.bushes = [o for o in s.obstacles if o.kind == "bush"]

        cc = cfg.load("sim_settings.yaml").get("collision", {}).get("recovery", {})
        self.bounce_dt = float(cc.get("bounce_dt", 1.0))
        self.cooldown_dt = float(cc.get("cooldown_dt", 2.0))
        self.bounce_vel = float(cc.get("bounce_vel", 2.0))
        self.max_collisions = int(cc.get("max_collisions", 8))
        # W5：围栏纳入碰撞事件集——西围栏顶缘 1.30 曾把低空穿行的 d5 物理托住
        # （坐围栏顶 z=1.40），原监控器对围栏接触零事件零恢复。旗控 A/B 后定默认。
        self.fence_events = bool(cc.get("fence_events", False))
        self.fences = [o for o in s.obstacles if o.kind == "fence"] \
            if self.fence_events else []

        self.pos = {}
        self.pub_col = {}
        self.pub_vel = {}
        self.st = {}
        for i in range(self.drone_count):
            self.pub_col[i] = rospy.Publisher(
                "/drone_%d/collision" % i, Bool, queue_size=1, latch=True)
            self.pub_vel[i] = rospy.Publisher(
                "/drone_%d/vel_cmd" % i, Twist, queue_size=1)
            self.st[i] = {"t": -1.0, "count": 0, "dir": None, "kind": None,
                          "freeze": False}

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_states)
        rospy.loginfo("gazebo_collision_monitor: %d trees %d bushes dr=%.2f "
                      "bounce=%.1fs cooldown=%.1fs bounce_vel=%.1f max_coll=%d",
                      len(self.trees), len(self.bushes), self.dr,
                      self.bounce_dt, self.cooldown_dt, self.bounce_vel,
                      self.max_collisions)

    # ---- 碰撞几何 ------------------------------------------------------------
    @staticmethod
    def _sphere_aabb(px, py, pz, r, lo, hi):
        nx = min(max(px, lo[0]), hi[0])
        ny = min(max(py, lo[1]), hi[1])
        nz = min(max(pz, lo[2]), hi[2])
        dx, dy, dz = px - nx, py - ny, pz - nz
        return dx * dx + dy * dy + dz * dz < r * r

    def _tree_hit(self, px, py, pz, t):
        # 圆柱树干；z 带 = [ground_z, ground_z+trunk_h]
        if pz + self.dr < self.gz or pz - self.dr > self.gz + t.trunk_h:
            return False
        return (px - t.cx) ** 2 + (py - t.cy) ** 2 < (t.trunk_r + self.dr) ** 2

    @staticmethod
    def _crown_z_range(t):
        """冠包络竖直跨度（世界系）。无冠返回 None。与 scene._crown_z_range 同口径。

        ⚠ 本函数是 `Scene._crown_z_range` 的**第二份实现**（判官不 import Scene 方法，
        只拿它的 obstacles），两处必须逐条同步。表级 meta `crown_collidable=false`
        （杨树档 2026-10-02 晚起为 False「冠物理、视觉双删」；键保留为回滚杠杆）
        时 gz 不再发 `c_crown`，本处若仍按 `crown_r` 判定 → 「物理不挡、判官照报」
        的静默失配。新增门控时**两处都要改**。
        """
        if not getattr(t, "crown_collidable", True):
            return None
        if t.crown_r is None or t.crown_z is None:
            return None
        if t.crown_h:
            return (t.crown_z - t.crown_h / 2.0, t.crown_z + t.crown_h / 2.0)
        return (t.crown_z - t.crown_r, t.crown_z + t.crown_r)

    def _crown_hit(self, px, py, pz, t):
        """冠包络圆柱（球 vs 竖直圆柱）= scene._collides_crown 同几何。

        杨树档 2026-10-02 晚起 `crown_collidable=false`（冠物理、视觉双删）⇒ 本
        子句恒不触发（`_crown_z_range` 返回 None）。机制保留供云杉/回滚档：云杉
        冠底中位 1.22 正压巡航带，冠是真障碍 —— gz 物理/py Scene/判官三口径必须
        一致，否则出现「物理在挡、判官不报」或反过来的静默失配。
        """
        zr = self._crown_z_range(t)
        if zr is None:
            return False
        if pz + self.dr < zr[0] or pz - self.dr > zr[1]:
            return False
        return (px - t.cx) ** 2 + (py - t.cy) ** 2 < (t.crown_r + self.dr) ** 2

    def _branch_hit(self, px, py, pz, t):
        """球-枝胶囊检测（gz 树枝物理缺环补齐）：球(pos,dr) vs 线段圆柱
        (A→B, br)。命中返回弹开方向（枝最近点→无人机，**水平化**——见下），
        未命中返回 None。与 python 后端 scene.collides 的枝子句同几何。"""
        brs = getattr(t, "branches", None)
        if not brs:
            return None
        p = (px, py, pz)
        for (sx, sy, sz, ex, ey, ez, br) in brs:
            a, b = (sx, sy, sz), (ex, ey, ez)
            if geo.point_segment_dist(p, a, b) > br + self.dr:
                continue
            ab = geo.sub(b, a)
            l2 = geo.norm2(ab)
            tt = 0.0 if l2 < 1e-12 else geo.clamp(
                geo.dot(geo.sub(p, a), ab) / l2, 0.0, 1.0)
            q = geo.add(a, geo.scale(ab, tt))   # 枝上最近点
            d = geo.sub(p, q)
            n = geo.norm(d)
            if n < 1e-9:
                return (0.0, 0.0, 1.0)          # 退化：骑轴心，向上弹
            # 枝弹方向水平化（树干弹 (dx,dy,0) 同律）：枝近水平（el -0.5~0.3），
            # 机在枝下时"最近点→机"方向朝下——gz 真物理下 bounce 2m/s 直坠，
            # seed43 d0 顶枝 z=1.65 被砸坐地 (z=0.18) 判 UNAUTHORIZED_LANDING
            # 退休。水平绕开才是枝恢复的正确语义；水平退化兜底向上。
            h = math.hypot(d[0], d[1])
            if h > 1e-6:
                return (d[0] / h, d[1] / h, 0.0)
            return (0.0, 0.0, 1.0)
        return None

    def _collision(self, i, px, py, pz, dpos):
        """返回 (是否碰撞, 弹开方向 vx,vy,vz, 命中类型)。

        类型 ∈ {"trunk","crown","branch","bush","fence","drone"}，供碰撞归类统计
        （换场后须能分辨「撞干」与「撞冠」，否则无法判断 P3 是否做全）。"""
        for t in self.trees:
            if self._tree_hit(px, py, pz, t):
                dx, dy = px - t.cx, py - t.cy
                n = math.hypot(dx, dy)
                if n < 1e-9:
                    return True, (1.0, 0.0, 0.0), "trunk"
                return True, (dx / n, dy / n, 0.0), "trunk"
            if self._crown_hit(px, py, pz, t):
                # 冠弹方向与树干同律：水平远离树轴。机在冠内（含冠底下方顶入）
                # 时垂直弹会把人朝下砸或朝上顶进更深处，横向脱离才是正确语义。
                dx, dy = px - t.cx, py - t.cy
                n = math.hypot(dx, dy)
                if n < 1e-9:
                    return True, (1.0, 0.0, 0.0), "crown"
                return True, (dx / n, dy / n, 0.0), "crown"
            bdir = self._branch_hit(px, py, pz, t)
            if bdir is not None:
                return True, bdir, "branch"
        for b in self.bushes:
            if self._sphere_aabb(px, py, pz, self.dr, b.lo, b.hi):
                nx = min(max(px, b.lo[0]), b.hi[0])
                ny = min(max(py, b.lo[1]), b.hi[1])
                nz = min(max(pz, b.lo[2]), b.hi[2])
                dx, dy, dz = px - nx, py - ny, pz - nz
                n = math.hypot(dx, dy, dz)
                if n < 1e-9:
                    n = 1.0
                    dx, dy, dz = 1.0, 0.0, 0.0
                return True, (dx / n, dy / n, dz / n), "bush"
        for f in self.fences:
            if self._sphere_aabb(px, py, pz, self.dr, f.lo, f.hi):
                nx = min(max(px, f.lo[0]), f.hi[0])
                ny = min(max(py, f.lo[1]), f.hi[1])
                nz = min(max(pz, f.lo[2]), f.hi[2])
                dx, dy, dz = px - nx, py - ny, pz - nz
                n = math.hypot(dx, dy, dz)
                if n < 1e-9:
                    n = 1.0
                    dx, dy, dz = 1.0, 0.0, 0.0
                return True, (dx / n, dy / n, dz / n), "fence"
        for j, (qx, qy, qz) in dpos.items():
            if j == i:
                continue
            dx, dy, dz = px - qx, py - qy, pz - qz
            d = math.hypot(dx, dy, dz)
            if 1e-9 < d < 2.0 * self.dr:
                # 互撞：弹开方向远离对方
                return True, (dx / d, dy / d, dz / d), "drone"
        return False, (0.0, 0.0, 0.0), None

    # ---- 主循环（100Hz：恢复窗口内高频 vel_cmd 覆盖 nav 的 20Hz） -------------
    def _on_states(self, msg):
        for name, p in zip(msg.name, msg.pose):
            if name.startswith("drone_"):
                try:
                    i = int(name.split("_")[1])
                except (ValueError, IndexError):
                    continue
                self.pos[i] = (p.position.x, p.position.y, p.position.z)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            dpos = dict(self.pos)
            now = rospy.get_time()
            for i in range(self.drone_count):
                if i not in dpos:
                    continue
                st = self.st[i]
                px, py, pz = dpos[i]
                if st["freeze"]:
                    self.pub_vel[i].publish(Twist())
                    continue
                hit, bdir, kind = self._collision(i, px, py, pz, dpos)
                dt = now - st["t"]
                if hit and st["t"] < 0:
                    # 新碰撞：发布事件 + 进入 bounce
                    st["t"] = now
                    st["count"] += 1
                    st["dir"] = bdir
                    st["kind"] = kind
                    if st["count"] > self.max_collisions:
                        st["freeze"] = True
                        st["t"] = -1.0
                        rospy.logerr(
                            "gazebo_collision_monitor: drone %d frozen "
                            "(>max_collisions=%d)", i, self.max_collisions)
                        continue
                    self.pub_col[i].publish(Bool(data=True))
                    rospy.logwarn(
                        "gazebo_collision_monitor: drone %d COLLISION #%d "
                        "kind=%s at (%.2f,%.2f,%.2f) dir=(%.2f,%.2f,%.2f)",
                        i, st["count"], kind, px, py, pz,
                        bdir[0], bdir[1], bdir[2])
                    self.pub_vel[i].publish(self._tw(bdir, self.bounce_vel))
                elif st["t"] >= 0:
                    # 恢复状态机：bounce → cooldown → 恢复（nav 接管）
                    if dt < self.bounce_dt:
                        self.pub_vel[i].publish(self._tw(st["dir"], self.bounce_vel))
                    elif dt < self.bounce_dt + self.cooldown_dt:
                        self.pub_vel[i].publish(Twist())
                    else:
                        st["t"] = -1.0
                        self.pub_col[i].publish(Bool(data=False))
            rate.sleep()

    @staticmethod
    def _tw(d, v):
        t = Twist()
        t.linear.x = d[0] * v
        t.linear.y = d[1] * v
        t.linear.z = d[2] * v
        return t


if __name__ == "__main__":
    try:
        CollisionMonitor().run()
    except rospy.ROSInterruptException:
        pass
