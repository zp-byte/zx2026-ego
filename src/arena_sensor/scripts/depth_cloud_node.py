#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::depth_cloud_node — 深度图 → 点云桥（每机一个）。

目的：让 arena_nav / arena_mission 在**零代码改动**下改吃前视深度相机。
输入 /drone_<id>/depth/image_raw（depth_sim_node 产物），输出 /drone_<id>/cloud
——与 lidar_node.py 的输出**逐字段同契约**（PointCloud2、世界系、x/y/z FLOAT32、
point_step=12、height=1、is_dense=true、queue_size=4）。消费者
nav_node._on_cloud 与 mission_executor_node 按字段偏移解析，故本节点是纯替换件。

发布率 = fps/decim = 15/2 = **7.5Hz**（lidar 是 10Hz，同量级）。取整数抽帧而非
时间门限：时间门限在 15Hz 源上会与 0.1s 拍相拍成 7.5Hz 的**混叠**（实测 min
0.116/max 0.152s），宣称 10Hz 而实发 7.5Hz——数值对不上就是隐患；且时间门限在
gz 后端的仿真时钟下行为还会随 RTF 漂。抽帧对两种时钟都给出精确的 0.1333s。

── 反投影（精确闭式，无需单位化）────────────────────────────────────────────
渲染器 ray_dirs 取 d=[u,v,1]、存 1/|d|，命中时 Z = t*(1/|d|)（沿光轴分量，
非斜距）。故 t = Z*|d|，命中点 = o + t*(d/|d|) = o + Z*d：

    P_world = pos + Z*(u*rgt + v*dwn + fwd)
    u=(col-cx)/fx,  v=(row-cy)/fy,  Z=mm/1000
    (rgt,dwn,fwd) = cam_basis(yaw, cam_pitch_deg)

**必须 yaw-only**：渲染时 cam_basis 只吃 yaw，roll/pitch 已被丢弃。若此处改用
完整 odom 四元数，就等于施加一个图像从未经历过的旋转——每点按当时 roll/pitch
系统性偏移，而点云自身仍自洽、看起来完全合理，是查不出来的错。相机在 odom
原点、无杆臂、无 TF（depth_sim 明写不广播 TF），故外参就是 [rgt,dwn,fwd]。

── 哨兵雷（本节点的核心正确性）──────────────────────────────────────────────
depth_sim 把**无命中**像素写 Z=t_max=max_range（=10000 毫米哨兵，不参与噪声），
无效化判据是 `Z > t_max` 故恰等于 t_max **不被清零**。因此毫米域里
**0 和 ≥ max_range*1000 都是「无效」**，只有 0<mm<10000 才是测量。

漏掉上界的后果是结构性的、不是精度问题：pitch=0 的水平前视相机，无命中射线
反投影后落在 z≈0.85 处，**正好砸进 nav 的占用带 [0.70, 2.60]**，形成一座
跟着机头走的 10 m 幻影墙（实测某巡航位姿：带内真点 4440 vs 带内哨兵假点
19740，假点是真点的 4.4 倍）。且 lidar_node 对未命中的处理是 `continue`
（根本不产点），故发哨兵点也是契约违背。**本节点必须双向丢弃。**

── 降采样（stride=12，由实测定标，不是估的）────────────────────────────────
480x270 有效像素可达 9.2 万/帧，而 lidar 只有 72 方位 x 7 俯仰 = 504 射线
（带内 ~290 命中/帧）。nav_node._on_cloud 是纯 Python 逐点循环，且 bh 的
cloud_mem_frames=6 会把 self.cloud 变成 6 帧并集供反应层/否决层扫描。故必须降。

不用「最近邻优先」：相机在 z=1.2、pitch=0，最下方射线 29° 俯角在 2.17m 处
打**地面**，故「最近的 N 个点」几乎全是地面——而地面 z≈0 **不在**占用带
[0.70,2.60] 内，预算会全打废。改用**均匀行列抽样**：角度间隔 = hfov/(W/stride)
x vfov/(H/stride)，stride=12 时约 2.2°x2.6°，比 lidar 的 5°/5.7° 还密，且
点的成分与 lidar 同型（雷达的下环同样吃地面）。**不做任何 z 带滤波**——那是
nav_node 的职责（见其 _bh_z_lo/_bh_z_hi），桥节点若也滤会连带改掉
mission_executor 的 via-slot 输入语义。

定标（2026-10-07，真值 Scene 实测三姿态，与同姿态 lidar 同尺；"带内"=nav 占用带
z∈[0.70,2.60]）：

    姿态               源          总点   带内
    起降坪(开阔,地面89%) lidar      387    107
                         depth s12  410     27      ← 总点对齐 lidar
                         depth s14  311     18      ← 旧值，只有 lidar 的 1/6
    树南4m 朝树         lidar      300     78
                         depth s12  662    419      ← 5.4x
    树南6m 背树         lidar      284     38
                         depth s12  461     81      ← 2.1x

选 **stride=12 = 总点数对齐 lidar**，让"密度"不成为行为差异的混杂因子：换的
是**传感器视场形状**（用户已接受的前向 87°x58° 代价），不是点云密度。代价在
开阔地（带内 27 vs 107，因为 FOV 看不到身后）；收益在近树（1m 处树干横跨
~47px@s1，s12 仍留 ~100 采样点，lidar 同位置只有 ~7 点）。
若建图证明太稀，s5-s8 可用（带内最多 5.4x-11x lidar），代价是反应层方向更常
触 max_vel 归一化（nav_node.py:2097）——那是行为改动，须走 A/B。

角度分辨率（这是"薄结构能不能被采到"的真正尺子，比点数可比）：
    depth s12 方位 2.17° x 俯仰 2.52°   vs   lidar 方位 5.00° x 俯仰 5.73-14.32°
⇒ 深度两轴都比雷达密（2.3x / 2.3-5.7x）。代价：薄于一个采样格的物体
（5m 处 19cm x 22cm）单帧可能整列漏采，靠多帧逼近累积补回——**雷达同病且更粗**。

── 已验证（2026-10-07，真值 Scene 离线对质 + 活栈烟测）──────────────────────
判据 A：每点取 scene.raycast 求径向残差。四姿态 (起降坪/树南4m/树南2m/林内)
        共 1972 点 **未命中 = 0**，中位|Δ| 0.0141-0.0160 m、最大 0.0907 m ——
        与噪声模型 |N(0,0.02)|*|d| (|d|<=1.49) 逐项吻合，即残差**就是**渲染噪声。
        ★scene.raycast 不含冠而 render_crowns=false 下渲染也不含冠 ⇒ 同一条断言
        即**冠泄漏探测器**：若该键被翻回 true，冠点会整片落进自由空间报未命中。
判据 A'（变异，证明 A 有鉴别力）：yaw±5° → 未命中 94-138/662、p95 残差 4.1-5.4 m；
        pitch+5° → 28/662、p95 3.4 m。正品 0/662、p95 0.046 m。
判据 C：全 0 / 全 10000 哨兵 / 全 10001 / 全 65535 帧 → 均 None（零发布）；
        网格上单像素 → 恰 1 点，坐标解析精确（差 0.00e+00）；大端帧与小端逐位同。
实测（活栈）：点数 410（预测 410）、带内 27（预测 27）、哨兵泄漏 0、发布率 7.500 Hz。

自门控（对齐 depth_sim 约定）：sensor.depth 缺失或 enabled=false 时启动即退、
零发布者、INFO 一次并**说明原因**（云源选了 depth 但愿没开是配置错误，
必须响亮）。odom 断流 > odom_timeout 或本帧无可测点则**停发**——绝不发空云：
空云会把 nav 的 cloud_mem_frames 历史挤掉，等于主动销毁信息。
"""
import math
import os
import sys

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField

# 单一源纪律：相机模型/内参/cam_pitch 一律取自 depth_sim_node，不得在此复制
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from depth_sim_node import make_depth_cfg, cam_basis, intrinsics  # noqa: E402

from zx2026_common import config as cfg  # noqa: E402

ODOM_TIMEOUT_DEFAULT = 0.5   # s，与 depth_sim_node / cloud_adapter 同语义


class DepthCloudNode:
    def __init__(self):
        rospy.init_node("depth_cloud_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        self.ns = "/drone_%d" % self.drone_id
        self._active = False

        settings = cfg.load("sim_settings.yaml")
        dcfg_raw = settings.get("sensor", {}).get("depth", None)
        if not dcfg_raw or not bool(dcfg_raw.get("enabled", False)):
            rospy.loginfo("depth_cloud_node[%d]: 退出——sensor.depth 缺失或 "
                          "enabled=false（自门控，与 depth_sim_node 同约定）。"
                          "若云源选了 depth 则这是配置错误：图像源根本没起。",
                          self.drone_id)
            return
        self.cfg = make_depth_cfg(dcfg_raw)
        self.fx, self.fy, self.cx, self.cy = intrinsics(self.cfg)
        self.pitch = math.radians(self.cfg["cam_pitch_deg"])

        # 哨兵上界：无命中像素恰为 max_range（rint(max_range*1000)）
        self.sentinel_mm = int(round(self.cfg["max_range"] * 1000.0))

        self.stride = max(1, int(rospy.get_param("~stride", 12)))
        self.max_points = max(1, int(rospy.get_param("~max_points", 4000)))
        self.decim = max(1, int(rospy.get_param("~decim", 2)))
        self.odom_timeout = float(rospy.get_param("~odom_timeout",
                                                  ODOM_TIMEOUT_DEFAULT))

        self.odom_pos = (0.0, 0.0, 1.0)
        self.odom_yaw = 0.0
        self._odom_t = None
        self._n_img = 0
        self._info_checked = False
        self._n_pub = 0

        self.pub = rospy.Publisher(self.ns + "/cloud", PointCloud2, queue_size=4)
        rospy.Subscriber(self.ns + "/depth/image_raw", Image, self._on_image,
                         queue_size=2, buff_size=2 ** 22)
        rospy.Subscriber(self.ns + "/depth/camera_info", CameraInfo,
                         self._on_camera_info)
        rospy.Subscriber(self.ns + "/odom", Odometry, self._on_odom)
        rospy.loginfo("depth_cloud_node[%d]: %dx%d stride=%d decim=%d → 发 "
                      "%.1fHz  max_points=%d cam_pitch=%.1f° 哨兵上界=%dmm → %s/cloud",
                      self.drone_id, self.cfg["width"], self.cfg["height"],
                      self.stride, self.decim,
                      float(self.cfg["fps"]) / self.decim,
                      self.max_points, self.cfg["cam_pitch_deg"],
                      self.sentinel_mm, self.ns)
        self._active = True

    # ---- 输入 ---------------------------------------------------------------
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        # 与 lidar_node:68 / depth_sim_node:422 逐字同式（yaw-only 是本链的硬约束）
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_t = rospy.Time.now()

    def _on_camera_info(self, msg):
        """只做对质：内参权威源是 config（渲染器同源），K 不一致=配置被改坏。"""
        if self._info_checked or len(msg.K) < 6:
            return
        self._info_checked = True
        d = max(abs(msg.K[0] - self.fx), abs(msg.K[4] - self.fy),
                abs(msg.K[2] - self.cx), abs(msg.K[5] - self.cy))
        if d > 1e-3:
            rospy.logerr("depth_cloud_node[%d]: camera_info.K 与 config 推导内参"
                         "不符（最大差 %.6f）——渲染器与消费者对同一张图用了不同"
                         "相机模型，点云会系统性错位。K=[%s]",
                         self.drone_id, d, ", ".join("%.3f" % v
                                                     for v in msg.K[:6]))
        else:
            # 正面证据：不打印成功行的话，"无 logerr" 既可能是校验通过、也可能是
            # 这个回调根本没跑（2026-10-07 就踩过：depth_sim 的 CameraInfo.D
            # 序列化抛异常使整条 camera_info 连接建不起来，静默至今）。
            rospy.loginfo("depth_cloud_node[%d]: camera_info.K 与 config 推导内参"
                          "一致（fx=%.3f fy=%.3f cx=%.1f cy=%.1f）——两源对质通过",
                          self.drone_id, self.fx, self.fy, self.cx, self.cy)

    # ---- 主路径 -------------------------------------------------------------
    def _on_image(self, msg):
        # 整数抽帧 ⇒ 发布率 = fps/decim 精确无混叠（见模块 docstring）。
        # 计数器在 odom 门**之前**自增：门停发期间相位照走，恢复后不出现长间隔。
        self._n_img += 1
        if self._n_img % self.decim:
            return
        if self._odom_t is None or \
                (rospy.Time.now() - self._odom_t).to_sec() > self.odom_timeout:
            rospy.logwarn_throttle(5.0, "depth_cloud_node[%d]: odom 断流 >%.1fs，"
                                   "停发（绝不发空云/旧位姿云）",
                                   self.drone_id, self.odom_timeout)
            return

        pts = self._unproject(msg)
        if pts is None or pts.shape[0] == 0:
            rospy.logwarn_throttle(5.0, "depth_cloud_node[%d]: 本帧零可测点"
                                   "（全哨兵/全无效），保持上一帧不发",
                                   self.drone_id)
            return
        self._publish(pts)

    def _unproject(self, msg):
        h, w = int(msg.height), int(msg.width)
        if h != self.cfg["height"] or w != self.cfg["width"]:
            rospy.logwarn_throttle(5.0, "depth_cloud_node[%d]: 图像 %dx%d 与 config "
                                   "%dx%d 不符，丢帧", self.drone_id, w, h,
                                   self.cfg["width"], self.cfg["height"])
            return None
        if msg.encoding != "16UC1":
            rospy.logwarn_throttle(5.0, "depth_cloud_node[%d]: encoding=%s 非 "
                                   "16UC1，丢帧", self.drone_id, msg.encoding)
            return None
        dt = ">u2" if msg.is_bigendian else "<u2"
        mm = np.frombuffer(msg.data, dtype=dt).reshape(h, w)

        rr = np.arange(0, h, self.stride)
        cc = np.arange(0, w, self.stride)
        sub = mm[np.ix_(rr, cc)]
        # ★ 双向无效判据：0=渲染器置的无效；>= 哨兵 = 无命中（恰等于也不清）
        valid = (sub >= 1) & (sub < self.sentinel_mm)
        ri, ci = np.nonzero(valid)
        n_raw = int(ri.size)
        if n_raw == 0:
            return None
        rows = rr[ri]
        cols = cc[ci]
        if n_raw > self.max_points:
            # 兜底均匀抽稀（stride 被调小时才会走到），保持无偏
            sel = np.linspace(0, n_raw - 1, self.max_points).astype(np.int64)
            rows, cols = rows[sel], cols[sel]
            n_raw = self.max_points
            rospy.logwarn_throttle(10.0, "depth_cloud_node[%d]: 抽稀到 max_points="
                                   "%d（stride 建议调大）", self.drone_id,
                                   self.max_points)

        Z = mm[rows, cols].astype(np.float64) * 0.001
        u = (cols.astype(np.float64) - self.cx) / self.fx
        v = (rows.astype(np.float64) - self.cy) / self.fy
        rgt, dwn, fwd = cam_basis(self.odom_yaw, self.pitch)
        B = np.stack([rgt, dwn, fwd], axis=1)               # (3,3) 列=光轴三基
        P = np.stack([u * Z, v * Z, Z], axis=0)             # (3,N) 相机系
        W = B @ P
        W[0] += self.odom_pos[0]
        W[1] += self.odom_pos[1]
        W[2] += self.odom_pos[2]
        return W.T

    def _publish(self, pts):
        """逐字段复刻 lidar_node.py:124-141 的契约。"""
        n = pts.shape[0]
        pc = PointCloud2()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = "world"
        pc.height = 1
        pc.width = n
        pc.is_dense = True
        pc.point_step = 12        # x/y/z 三个 FLOAT32，无填充
        pc.row_step = 12 * n
        pc.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc.data = pts.astype("<f4").tobytes()
        self.pub.publish(pc)
        self._n_pub += 1
        rospy.loginfo_throttle(10.0, "depth_cloud_node[%d]: 已发 %d 帧，本帧 %d 点",
                               self.drone_id, self._n_pub, n)

    def run(self):
        if not self._active:
            return
        rospy.spin()


if __name__ == "__main__":
    try:
        node = DepthCloudNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
