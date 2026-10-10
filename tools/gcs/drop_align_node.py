#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/drop_align_node.py — 下视 RGB 视觉伺服对齐投放点（真机包 2026-10-09）。

用户需求：下视 RGB 相机完成与投放点（平台色盘）的对齐后再投放。
现状（侦察定谳）：识别链只出「颜色+置信度」，质心在 color_detect 内算完
即弃，对齐靠 nav 位置环收敛到 scene_topology 平台心——装订误差/漂移直接
变成释放点偏移。本节点补上像素级几何通道：

  观测  期望色（Mission.box_color）HSV 检测（复用 arena_sensor.color_detect
        纯函数，限定色找块）→ 目标色块质心像素误差 (du, dv)。
  换算  米制偏移 = 像素误差 × 相机到盘面距离 z_cam / f（内参 camera_info
        优先，回退解析式 f=(h/2)/tan(vfov/2)）→ 机体轴（前/右）→ ENU
        （odom yaw 旋转）→ 修正目标 = odom_xy + offset。
  发布  /drone_<id>/align/corr（PoseStamped world，随帧率）+ /align/status
        （String JSON，1Hz）。executor 的 _drop_hold 重拉段优先取新鲜修正
        （age≤align_stale_s，executor 侧判），释放门不变（MATCH + nav
        arrived + release_tol）——不新增速度写者、不改 type_match 契约。

轴向定谳（2026-10-09 侦察）：sim camera_render 的 v 轴=机体前向（图像
底边=机头前方）；真机 D435 常规下视安装图像顶边朝前，两者差 180°——
--axes 切换换算符号：
    sim:       fwd = (v - cy) * z / fy,  right = (u - cx) * z / fx
    d435_down: fwd = (cy - v) * z / fy,  right = (u - cx) * z / fx
接错轴向=伺服方向反向（把对准推成推离）——selftest 断言两档符号。

诚实边界：
  * 仅 AT_DROP/DROP 相位活跃（伺服语义）；其余相位静默不发布。
  * 无检测/异色/conf 低/内参缺 → 不发修正（executor 持旧值，stale 门
    衰减回平台心）；expected 色未知（Mission 未到/UNKNOWN）→ 整体待机。
  * z_cam = odom_z - plane_z：plane_z 是色盘顶面高度参数（sim 盘顶
    0.05m；真机平台实际顶面赛前核对装订），下限 0.3m 防近地除爆。
  * 分辨率量纲参数（min_blob_area_px 等）按 sim 320×240@0.8m 标定——真机
    424×240 换分辨率后 px/m 变化，area 地板须随光照/分辨率重标（侦察既知）。
  * 本节点只发目标，从不写 vel_cmd——控制链不经过此处。

用法（机载；real_fleet.launch camera:=true 时自动拉起）：
  python3 drop_align_node.py --id 0 --image-topic /camera_down/image_raw
      [--axes top_fwd] [--plane-z 0.05] [--min-conf 0.8]
  离线：python3 drop_align_node.py --selftest
"""
import argparse
import json
import math
import os
import sys

import numpy as np


def _add_repo_paths():
    """arena_sensor.color_detect 路径注入（机上/WSL 双布局兜底）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(os.path.dirname(os.path.dirname(here)),
                     "src", "arena_sensor", "scripts"),
        os.path.expanduser("~/zx2026_ego_ws/src/arena_sensor/scripts"),
    ]
    for c in cands:
        if os.path.isfile(os.path.join(c, "arena_sensor", "color_detect.py")):
            if c not in sys.path:
                sys.path.insert(0, c)
            return


_add_repo_paths()
from arena_sensor import color_detect as CD  # noqa: E402


# ---------------------------------------------------------------- 纯逻辑 --
DROP_PHASES = ("AT_DROP", "DROP")
COLOR_U8 = {"red": 0, "blue": 1, "yellow": 2}


def pixel_to_body(du, dv, z_cam, fx, fy, axes):
    """像素误差 → 机体水平偏移 (fwd, right)（米）。axes 定谳见模块 docstring。"""
    right = du * z_cam / fx
    fwd = (dv if axes == "sim" else -dv) * z_cam / fy
    return fwd, right


def body_to_enu(fwd, right, yaw):
    """机体 (前, 右) → ENU (e, n)。前向=(cosψ, sinψ)，右向=(sinψ, -cosψ)。"""
    return (fwd * math.cos(yaw) + right * math.sin(yaw),
            fwd * math.sin(yaw) - right * math.cos(yaw))


def align_offset(du, dv, z_cam, fx, fy, yaw, axes):
    """单帧对齐偏移（ENU 米）：质心像素误差 → 机体 → ENU。"""
    fwd, right = pixel_to_body(du, dv, z_cam, fx, fy, axes)
    return body_to_enu(fwd, right, yaw)


def analytic_fx(width, height, vfov_deg):
    """回退内参：方像素 f=(h/2)/tan(vfov/2)（camera_render 同式）。"""
    return (height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)


# 轴档别名（2026-10-10 C100 下视定谳后加正名；内部只认两个符号档）：
#   顶边朝机头 = top_fwd（旧名 d435_down，历史调用兼容保留）
#   底边朝机头 = bottom_fwd（旧名 sim，仿真口径）
AXES_ALIAS = {"top_fwd": "d435_down", "d435_down": "d435_down",
              "bottom_fwd": "sim", "sim": "sim"}


# ---------------------------------------------------------------- ROS 壳 --
class DropAlignNode(object):
    def __init__(self, args):
        import rospy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from sensor_msgs.msg import Image, CameraInfo
        from zx2026_common import config as cfg
        from zx2026_common.msg import Mission
        self._String = String               # 回调期构造用（导入留在 __init__）
        self._PoseStamped = PoseStamped

        rospy.init_node("drop_align_node", anonymous=True)
        self._rospy = rospy
        self.ns = "/drone_%d" % args.id
        self.axes = args.axes
        self.plane_z = args.plane_z
        self.min_conf = args.min_conf
        self.vfov = args.vfov_deg
        self.params = CD.make_detect_params(cfg.load("competition_rules.yaml"))

        self.expected = None       # 期望色 u8（None=待机）
        self.phase = ""
        self.odom = (0.0, 0.0, 0.0)
        self.odom_yaw = 0.0
        self.fx = self.fy = self.cx = self.cy = None
        self.info_src = "waiting"
        self.img_wh = (0, 0)
        self.last = {"det": None, "conf": 0.0, "du": None, "dv": None,
                     "e": None, "n": None, "z": None}

        self.pub_corr = rospy.Publisher(self.ns + "/align/corr", PoseStamped,
                                        queue_size=5)
        self.pub_status = rospy.Publisher(self.ns + "/align/status", String,
                                          queue_size=1, latch=True)
        rospy.Subscriber(args.image_topic, Image, self.on_image, queue_size=1)
        # camera_info：真机 /camera/color/camera_info（realsense 默认出）；
        # 缺省由 image 话题推 /image_raw→/camera_info（sim 同规则）
        info_topic = args.camera_info
        if not info_topic:
            base = args.image_topic
            info_topic = (base.rsplit("/image_raw", 1)[0] + "/camera_info"
                          if base.endswith("/image_raw")
                          else base.rsplit("/", 1)[0] + "/camera_info")
        rospy.Subscriber(info_topic, CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.on_odom,
                         queue_size=10)
        rospy.Subscriber(self.ns + "/mission/phase", String, self.on_phase,
                         queue_size=2)
        rospy.Subscriber("/zx2026/mission/%d" % args.id, Mission,
                         self.on_mission, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), lambda _e: self.publish_status())
        rospy.loginfo("drop_align_node: drone %d topic=%s axes=%s info=%s "
                      "(active only in %s)", args.id, args.image_topic,
                      self.axes, info_topic, "/".join(DROP_PHASES))
        rospy.spin()

    # -- 回调 ----------------------------------------------------------------
    def on_info(self, m):
        k = m.K
        if k and k[0] > 0.0:
            self.fx, self.fy = k[0], k[4]
            self.cx, self.cy = k[2], k[5]
            self.info_src = "camera_info"

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.odom = (p.x, p.y, p.z)
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_phase(self, m):
        self.phase = m.data

    def on_mission(self, m):
        c = (m.box_color or "").lower()
        if c in COLOR_U8:
            self.expected = COLOR_U8[c]
            self._rospy.loginfo("drop_align: expected color %s", c)
        else:
            self.expected = None
            self._rospy.logwarn("drop_align: box_color %r unknown -> idle "
                                "(no correction will be published)", c)

    # -- 主处理 ---------------------------------------------------------------
    def on_image(self, m):
        if self.phase not in DROP_PHASES or self.expected is None:
            return
        h, w = m.height, m.width
        if h <= 0 or w <= 0:
            return
        self.img_wh = (w, h)
        # 编码统一入口（color_detect.to_bgr；不认识的编码拒帧，不硬解——
        # usb_cam 默认 YUYV，旧「当 bgr8」=静默腐蚀，2026-10-10 C100 定谳）
        img = CD.to_bgr(m.data, m.encoding, w, h)
        if img is None:
            self._rospy.logwarn_throttle(
                5.0, "drop_align: encoding %r unsupported -> frame rejected"
                % m.encoding)
            return
        cu8, conf, u, v, _area = CD.detect_with_centroid(
            img, self.params, color_u8=self.expected)
        if cu8 != self.expected or conf < self.min_conf:
            self.last.update(det=cu8, conf=conf, du=None, dv=None,
                             e=None, n=None, z=None)
            return
        fx, fy, cx, cy = self._intrinsics(w, h)
        if fx is None:
            self.last.update(det=cu8, conf=conf)
            return
        z_cam = max(0.3, self.odom[2] - self.plane_z)
        du, dv = u - cx, v - cy
        e, n = align_offset(du, dv, z_cam, fx, fy, self.odom_yaw, self.axes)
        self.last.update(det=cu8, conf=conf, du=du, dv=dv, e=e, n=n, z=z_cam)
        msg = self._PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self._rospy.Time.now()
        msg.pose.position.x = self.odom[0] + e
        msg.pose.position.y = self.odom[1] + n
        msg.pose.orientation.w = 1.0
        self.pub_corr.publish(msg)

    def _intrinsics(self, w, h):
        if self.fx is not None:
            return self.fx, self.fy, self.cx, self.cy
        if w > 0 and h > 0:                # 解析式回退（camera_info 缺失档）
            f = analytic_fx(w, h, self.vfov)
            self.info_src = "analytic"
            return f, f, w / 2.0, h / 2.0
        return None, None, None, None

    def publish_status(self):
        s = {"phase": self.phase, "expected": self.expected,
             "src": self.info_src, "img_wh": list(self.img_wh),
             "last": {k: self.last[k] for k in
                      ("det", "conf", "du", "dv", "e", "n", "z")}}
        self.pub_status.publish(self._rospy.String(data=json.dumps(s)))


def main():
    import rospy
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--image-topic", default="/camera_down/image_raw",
                    help="下视 RGB sensor_msgs/Image（真机 d435 定谳话题）")
    ap.add_argument("--camera-info", default="",
                    help="CameraInfo 话题（缺省由 image 话题推导）")
    ap.add_argument("--axes", default="d435_down",
                    choices=tuple(sorted(AXES_ALIAS)),
                    help="图像 v 轴朝向档（top_fwd/d435_down=顶边朝机头；"
                         "bottom_fwd/sim=底边朝机头）——**装机后必核**，"
                         "接反=伺服反向推离色盘")
    ap.add_argument("--plane-z", type=float, default=0.05,
                    help="色盘顶面高度（z_cam=odom_z-plane_z；真机赛前核对）")
    ap.add_argument("--min-conf", type=float, default=0.8)
    ap.add_argument("--vfov-deg", type=float, default=50.0,
                    help="解析式回退内参的垂直视场角（camera_info 缺失时才用）")
    ns = ap.parse_args(rospy.myargv()[1:])
    ns.axes = AXES_ALIAS[ns.axes]
    DropAlignNode(ns)


def _selftest():
    """纯逻辑离线自检（--selftest）。"""
    # 1) 轴向符号：sim 档 dv>0=目标在前方 → fwd>0；d435_down 档反向
    f, r = pixel_to_body(0.0, 20.0, 1.0, 110.0, 110.0, "sim")
    assert f > 0 and abs(r) < 1e-9, (f, r)
    f, r = pixel_to_body(0.0, 20.0, 1.0, 110.0, 110.0, "d435_down")
    assert f < 0 and abs(r) < 1e-9, (f, r)
    # 2) 机体系→ENU：yaw=0 时前向=东(+e)；yaw=90° 时前向=北(+n)
    e, n = body_to_enu(1.0, 0.0, 0.0)
    assert abs(e - 1.0) < 1e-9 and abs(n) < 1e-9
    e, n = body_to_enu(1.0, 0.0, math.pi / 2)
    assert abs(e) < 1e-9 and abs(n - 1.0) < 1e-9
    # 3) 右向：yaw=0 时 right=南(-n)
    e, n = body_to_enu(0.0, 1.0, 0.0)
    assert abs(n + 1.0) < 1e-9 and abs(e) < 1e-9
    # 4) 端到端：目标在机头前方 0.5m（sim 图像底边方向 dv>0），yaw=90°（朝北）
    #    → 修正点应在正北 0.5m
    e, n = align_offset(0.0, 55.0, 1.0, 110.0, 110.0, math.pi / 2, "sim")
    assert abs(e) < 1e-9 and abs(n - 0.5) < 1e-6, (e, n)
    # 5) 解析内参：vfov=95° h=240 → f=110.06（camera_render 标称 110 对带）
    f = analytic_fx(320, 240, 95.0)
    assert 108.0 < f < 112.0, f
    # 6) 轴档别名（C100 定谳 2026-10-10）：正名与旧名同符号、两档互异
    assert AXES_ALIAS["top_fwd"] == AXES_ALIAS["d435_down"]
    assert AXES_ALIAS["bottom_fwd"] == AXES_ALIAS["sim"]
    assert AXES_ALIAS["top_fwd"] != AXES_ALIAS["bottom_fwd"]
    # 7) 编码入口 to_bgr：bgr8 原样 / rgb8 换序 / yuyv 打包解出灰阶 / 未知拒帧
    import numpy as _np
    _h, _w = 4, 8
    _bgr = _np.arange(_h * _w * 3, dtype=_np.uint8).reshape(_h, _w, 3)
    out = CD.to_bgr(_bgr.tobytes(), "bgr8", _w, _h)
    assert out is not None and _np.array_equal(out, _bgr), "bgr8 原样失败"
    _rgb = _np.ascontiguousarray(_bgr[:, :, ::-1])
    out = CD.to_bgr(_rgb.tobytes(), "rgb8", _w, _h)
    assert out is not None and _np.array_equal(out, _bgr), "rgb8 换序失败"
    _yuv = _np.full((_h, _w, 2), 128, _np.uint8)
    _yuv[:, :, 0] = 235                     # Y=235,U=V=128 → 中性亮灰（BT.601）
    out = CD.to_bgr(_yuv.tobytes(), "yuyv", _w, _h)
    assert (out is not None and out.shape == (_h, _w, 3)
            and _np.allclose(out[..., 0].astype(int), out[..., 1].astype(int))
            and _np.allclose(out[..., 1].astype(int), out[..., 2].astype(int))
            and out.mean() > 200.0), "yuyv 解包失败 %s" % (
        None if out is None else out.reshape(-1, 3).mean(0))
    out = CD.to_bgr(_yuv.tobytes(), "uyvy", _w, _h)
    assert out is not None and out.shape == (_h, _w, 3), "uyvy 失败"
    assert CD.to_bgr(_bgr.tobytes(), "mono8", _w, _h) is None, "未知编码须拒帧"
    assert CD.to_bgr(_bgr.tobytes()[:10], "bgr8", _w, _h) is None, "短帧须拒帧"
    assert CD.to_bgr(_yuv.tobytes(), "yuyv", 7, _h) is None, "奇宽须拒帧"
    print("DROP_ALIGN SELFTEST PASS")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
    else:
        main()
