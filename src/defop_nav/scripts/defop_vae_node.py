#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_vae_node.py — 节点 2/3：seVAE 深度编码（DeFoP vae_node 适配，sim/真机同码）。

订阅 /drone_<id>/depth/image_filled（16UC1 毫米，来自 defop_depth_fill）->
DeFoP 预处理（毫米->米、>10m 钳 10、<0.2m 置 -1、/10 归一、resize 480x270）->
seVAE encoder（PyTorch, .pth）-> 发 /drone_<id>/latent_space
（std_msgs/Float32MultiArray, 128 维 means）。

vendored 出处（BSD-3, https://github.com/guglielmo610/DeFoP）：
  - src/planning/sevae/sevae/inference/src/vae_node.py 的 image_callback
    预处理段（逐字，含两段 "<0.2 置 -1" 的原始行为——注意 -1 标记随后被
    VAENetworkInterface.forward 的 torch.clamp(0,1) 压成 0，此为原版
    发布链的既有行为，本移植保持一致，见 README 已知局限）；
  - src/planning/sevae/sevae/inference/scripts/VAENetworkInterface.py
    的 forward（逐字：clamp(0,1) + encode，返回 means）；
  - 网络定义在包内 sevae_vae_def.py（vendored VAE 类）。

运行环境：~/defop_env/bin/python（torch 已装 2.4.1+cpu；rospy 走 _ensure_ros）。
device 参数默认 cpu（"auto" 时探测 cuda）。
"""
import sys
import os


def _ensure_ros():
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
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import torch
from sevae_vae_def import VAE


class VAENetworkInterfaceDef(object):
    """VAENetworkInterface.py 逐字移植（权重路径参数化，device 参数化）。"""

    def __init__(self, latent_space_dims=128, device='cpu', weights_path='',
                 img_h=270, img_w=480):
        self.latent_space = latent_space_dims
        self.device = device
        self.img_h = int(img_h)  # verify F3：forward 形状随配置，不再写死
        self.img_w = int(img_w)
        try:
            self.vae = VAE(input_dim=1, latent_dim=self.latent_space)
            state = torch.load(os.path.expanduser(weights_path),
                               map_location=self.device)
            self.vae.load_state_dict(state, strict=True)
            self.vae = self.vae.to(self.device)
            self.vae.eval()
            self.vae.set_inference_mode(True)  # eps=0，确定性 encode
        except Exception:
            rospy.logerr('[defop_vae] Could not load VAE weights: %s',
                         weights_path)
            raise
        rospy.loginfo('[defop_vae] loaded %s -> device %s', weights_path,
                      self.device)

    def forward(self, image_numpy, calculate_reconstruction=False):
        with torch.no_grad():
            torch_image = torch.from_numpy(image_numpy).float().to(self.device)
            torch_image = torch.clamp(torch_image, 0.0, 1.0)
            z_sampled, means, _ = self.vae.encode(
                torch_image.view(1, 1, self.img_h, self.img_w))
            reconstructed_image = 0
            if calculate_reconstruction:
                reconstructed_image = self.vae.decode(means).cpu().numpy()
        return means.cpu().numpy(), reconstructed_image


def decode_depth_m(msg):
    """16UC1 毫米（或 32FC1 米）-> float32 米。DeFoP 真机 '<H' 直解路径。"""
    buf = bytes(bytearray(msg.data))
    if msg.encoding == '32FC1':
        return np.frombuffer(buf, dtype='<f4').reshape(msg.height, msg.width).copy()
    return np.frombuffer(buf, dtype='<u2').reshape(msg.height, msg.width).astype(np.float32) * 0.001


class VaeNode(object):
    def __init__(self):
        drone_id = rospy.get_param('~drone_id', '0')
        in_topic = rospy.get_param('~in_topic', '') or \
            '/drone_%s/depth/image_filled' % drone_id
        out_topic = rospy.get_param('~out_topic', '') or \
            '/drone_%s/latent_space' % drone_id
        self.latent_size = int(rospy.get_param('~latent_size', 128))
        self.img_h = int(rospy.get_param('~image_height', 270))
        self.img_w = int(rospy.get_param('~image_width', 480))
        self.max_depth = float(rospy.get_param('~depth_max_range', 10.0))
        self.min_valid = float(rospy.get_param('~depth_min_valid', 0.2))
        device_param = str(rospy.get_param('~device', 'cpu'))
        if device_param == 'auto':
            device_param = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        if device_param.startswith('cuda') and not torch.cuda.is_available():
            rospy.logwarn('[defop_vae] device=%s 但 CUDA 不可用 -> 退 cpu',
                          device_param)
            device_param = 'cpu'
        if device_param.startswith('cuda'):
            # WSL/dxg 稳定性：6 进程共享单卡，按 CPN 的整机重启教训先限幅。
            try:
                torch.cuda.set_per_process_memory_fraction(0.15, 0)
                rospy.loginfo('[defop_vae] torch 显存份额 15%%（6 进程共享）')
            except Exception as _e:
                rospy.logwarn('[defop_vae] 显存份额设置失败: %r', _e)
        weights = str(rospy.get_param(
            '~vae_weights',
            '~/DeFoP/src/planning/sevae/sevae/weights/seVAE_weights/seVAE_evo_LD_128.pth'))

        self.interface = VAENetworkInterfaceDef(
            latent_space_dims=self.latent_size, device=device_param,
            weights_path=weights, img_h=self.img_h, img_w=self.img_w)

        self.frame_cnt = 0
        self.pub = rospy.Publisher(out_topic, Float32MultiArray, queue_size=2)
        self.sub = rospy.Subscriber(in_topic, Image, self.callback, queue_size=1)
        rospy.loginfo('[defop_vae drone_%s] %s -> %s (latent=%d, device=%s)',
                      drone_id, in_topic, out_topic, self.latent_size,
                      device_param)

    def preprocess(self, msg):
        """vae_node.py image_callback 真机路径逐字（16UC1 毫米）。"""
        cv_image = decode_depth_m(msg)
        cv_image[np.isnan(cv_image)] = self.max_depth

        np_image = np.asarray(cv_image).copy()
        np_image[np_image > self.max_depth] = self.max_depth
        np_image[np_image < 0.20] = -1.0

        np_image.setflags(write=1)
        np_image = np_image / self.max_depth
        np_image[np_image < self.min_valid / self.max_depth] = -1.0

        input_image = np_image.copy()
        if np_image.shape != (self.img_h, self.img_w):
            torch_img = torch.from_numpy(np_image).float()
            rescaled = torch.nn.functional.interpolate(
                torch_img.unsqueeze(0).unsqueeze(0),
                (self.img_h, self.img_w)).squeeze(0).squeeze(0)
            input_image = rescaled.numpy()
        return input_image

    def callback(self, msg):
        try:
            input_image = self.preprocess(msg)
            means, _ = self.interface.forward(input_image,
                                              calculate_reconstruction=False)
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[defop_vae] forward failed: %r', e)
            return
        self.frame_cnt += 1
        out = Float32MultiArray()
        out.data = means.flatten().tolist()
        self.pub.publish(out)
        if self.frame_cnt % 150 == 0:
            rospy.loginfo('[defop_vae] %d frames encoded', self.frame_cnt)


def main():
    rospy.init_node('defop_vae')
    try:
        VaeNode()
    except ImportError as e:
        rospy.logerr('[defop_vae] 缺依赖（应在 ~/defop_env/bin/python 下运行）: %r', e)
        raise
    rospy.loginfo('[defop_vae] node initialized')
    rospy.spin()


if __name__ == '__main__':
    main()
