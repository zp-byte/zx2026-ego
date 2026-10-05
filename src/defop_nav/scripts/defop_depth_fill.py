#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_depth_fill.py — 节点 1/3：深度补洞（DeFoP img_filler 移植，sim/真机同码）。

订阅 /drone_<id>/depth/image_raw（16UC1 毫米；真机可由 in_topic 参数改接
/camera/depth/image_rect_raw）-> fill_invalid_depth_robust 补洞 ->
发 /drone_<id>/depth/image_filled（16UC1 毫米）。

vendored 出处（BSD-3, https://github.com/guglielmo610/DeFoP）：
  ~/DeFoP/img_filler.py 的 fill_invalid_depth_robust（逐字）与 DepthFixNode
  的 16UC1<->米 换算骨架。改动：话题参数化（原版硬编码 /d435/...）、
  "Pixels under 20 cm" 日志改节流（原版每帧 loginfo，15fps 刷屏）。

运行环境：~/defop_env/bin/python（rospy 缺失时自动补 ROS dist-packages 路径，
见 _ensure_ros；cv2/dilate 用 venv 内 opencv）。
"""
import sys


def _ensure_ros():
    """venv python 下补 ROS Noetic dist-packages（双向引导的第二向）。"""
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
import numpy as np
import cv2
from sensor_msgs.msg import Image


def fill_invalid_depth_robust(depth_image, max_distance=3.0, kernel_size=100):
    """DeFoP img_filler.py 逐字移植（BSD-3）。

    无效像素(<=0/非有限) 用 (kernel_size x kernel_size) 膨胀邻域内的最小
    有效深度填充；孤岛（邻域无有效值）不填。
    """
    depth = depth_image.astype(np.float32).copy()
    valid = np.isfinite(depth) & (depth > 0) & (depth < max_distance)
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    dilated_valid = cv2.dilate(valid.astype(np.uint8), kernel, iterations=1)
    large_val = max_distance + 1000.0
    depth_for_min = depth.copy()
    invalid_all = (~np.isfinite(depth)) | (depth <= 0)
    depth_for_min[invalid_all] = large_val
    min_depths = cv2.erode(depth_for_min, kernel, iterations=1)
    fill_mask = invalid_all & (dilated_valid == 1) & (min_depths < large_val)
    depth[fill_mask] = min_depths[fill_mask]

    return depth


def decode_depth_m(msg):
    """sensor_msgs/Image -> float32 米。16UC1 毫米走 DeFoP 真机路径
    （np.ndarray '<H' 直解，不依赖 cv_bridge）；32FC1 米直收。"""
    if msg.encoding == '32FC1':
        buf = bytes(bytearray(msg.data))
        return np.frombuffer(buf, dtype='<f4').reshape(msg.height, msg.width).copy()
    # 16UC1 / mono16 默认按毫米
    buf = bytes(bytearray(msg.data))
    return np.frombuffer(buf, dtype='<u2').reshape(msg.height, msg.width).astype(np.float32) * 0.001


def encode_depth_mm16(depth_m, header):
    """float32 米 -> 16UC1 毫米 Image（手动装填，避开 cv_bridge）。"""
    out = Image()
    out.header = header
    out.height = depth_m.shape[0]
    out.width = depth_m.shape[1]
    out.encoding = '16UC1'
    out.is_bigendian = 0
    out.step = out.width * 2
    out.data = np.clip(depth_m * 1000.0, 0, 65535).astype('<u2').tobytes()
    return out


class DepthFillNode(object):
    def __init__(self):
        drone_id = rospy.get_param('~drone_id', '0')
        in_topic = rospy.get_param('~in_topic', '') or \
            '/drone_%s/depth/image_raw' % drone_id
        out_topic = rospy.get_param('~out_topic', '') or \
            '/drone_%s/depth/image_filled' % drone_id
        self.kernel_size = int(rospy.get_param('~kernel_size', 100))
        self.max_distance = float(rospy.get_param('~max_distance', 3.0))
        self.close_threshold = float(rospy.get_param('~close_threshold', 0.2))
        self.log_every = int(rospy.get_param('~log_every', 15))

        self.frame_cnt = 0
        self.pub = rospy.Publisher(out_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(in_topic, Image, self.callback, queue_size=1)
        rospy.loginfo('[defop_depth_fill drone_%s] %s -> %s (kernel=%d, max_dist=%.1f)',
                      drone_id, in_topic, out_topic, self.kernel_size,
                      self.max_distance)

    def callback(self, msg):
        depth_m = decode_depth_m(msg)
        fixed = fill_invalid_depth_robust(depth_m, max_distance=self.max_distance,
                                          kernel_size=self.kernel_size)
        self.frame_cnt += 1
        if self.log_every > 0 and self.frame_cnt % self.log_every == 0:
            close_pixels = int(np.sum(fixed < self.close_threshold))
            rospy.loginfo('[defop_depth_fill] frame %d, pixels under %.2f m: %d',
                          self.frame_cnt, self.close_threshold, close_pixels)
        self.pub.publish(encode_depth_mm16(fixed, msg.header))


def main():
    rospy.init_node('defop_depth_fill')
    DepthFillNode()
    rospy.spin()


if __name__ == '__main__':
    main()
