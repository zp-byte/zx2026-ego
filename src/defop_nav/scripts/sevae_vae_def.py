#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sevae_vae_def.py — seVAE 网络定义（vendored from ~/DeFoP, BSD-3）。

出处（版权与许可归属原仓库，本文件仅做运行适配）：
  https://github.com/guglielmo610/DeFoP  (BSD-3-Clause)
  - src/planning/sevae/sevae/networks/VAE/vae.py
  - src/planning/sevae/sevae/networks/VAE/condensed_encoder.py
  - src/planning/sevae/sevae/networks/VAE/condensed_decoder.py

适配改动（层定义/参数名逐字未动，state_dict 兼容性不受影响）：
  1) 打印改为模块级 VERBOSE 门控（原 encode() 每次 forward 打 2 行，15fps 会刷屏）；
  2) 去掉 torchsummary 的 __main__ 自检段与 sevae 包内路径依赖；
  3) 权重文件路径不再硬编码（由调用方 defop_vae_node.py 的 ROS 参数传入）。

BSD 3-Clause License（原文见仓库根 LICENSE）：
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
"""
import torch
import torch.nn as nn

VERBOSE = False


def _dbg(*a):
    if VERBOSE:
        print(*a)


# ===================== condensed_encoder.py =====================
class Encoder(nn.Module):
    """
    ResNet8 architecture as encoder.
    """

    def __init__(self, input_dim, latent_dim):
        super(Encoder, self).__init__()
        self.input_dim = input_dim
        self.latent_dim = latent_dim
        self.define_encoder()
        self.elu = nn.ELU()
        _dbg("[Encoder] Encoder network initialized.")

    def define_encoder(self):
        # define conv functions
        self.conv0 = nn.Conv2d(self.input_dim, 32, kernel_size=5, stride=2, padding=2)
        self.conv0_1 = nn.Conv2d(32, 32, kernel_size=3, stride=2, padding=2)
        nn.init.xavier_uniform_(self.conv0_1.weight, gain=nn.init.calculate_gain("linear"))
        nn.init.zeros_(self.conv0_1.bias)

        self.conv1_0 = nn.Conv2d(32, 32, kernel_size=5, stride=2, padding=1)
        self.conv1_1 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        nn.init.xavier_uniform_(self.conv1_1.weight, gain=nn.init.calculate_gain("linear"))
        nn.init.zeros_(self.conv1_1.bias)

        self.conv2_0 = nn.Conv2d(64, 64, kernel_size=5, stride=2, padding=2)
        self.conv2_1 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
        nn.init.xavier_uniform_(self.conv2_1.weight, gain=nn.init.calculate_gain("linear"))
        nn.init.zeros_(self.conv2_1.bias)

        self.conv3_0 = nn.Conv2d(128, 128, kernel_size=5, stride=2)

        self.conv0_jump_2 = nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1)
        self.conv1_jump_3 = nn.Conv2d(64, 128, kernel_size=5, stride=4, padding=(2, 1))

        self.dense0 = nn.Linear(3 * 6 * 128, 512)
        self.dense1 = nn.Linear(512, 2 * self.latent_dim)

    def forward(self, img):
        return self.encode(img)

    def encode(self, img):
        # conv0
        x0_0 = self.conv0(img)
        x0_1 = self.conv0_1(x0_0)
        x0_1 = self.elu(x0_1)

        x1_0 = self.conv1_0(x0_1)
        x1_1 = self.conv1_1(x1_0)

        x0_jump_2 = self.conv0_jump_2(x0_1)
        x1_1 = x1_1 + x0_jump_2
        x1_1 = self.elu(x1_1)

        x2_0 = self.conv2_0(x1_1)
        x2_1 = self.conv2_1(x2_0)

        x1_jump3 = self.conv1_jump_3(x1_1)
        x2_1 = x2_1 + x1_jump3
        x2_1 = self.elu(x2_1)

        x3_0 = self.conv3_0(x2_1)
        _dbg("Shape of x3_0", x3_0.shape)

        x = x3_0.view(x3_0.size(0), -1)
        x = self.dense0(x)
        x = self.elu(x)
        x = self.dense1(x)
        return x


# ===================== condensed_decoder.py =====================
class ImgDecoder(nn.Module):
    """Generates an image reconstruction using ConvTranspose2d layers.

    Adapted from https://github.com/microsoft/AirSim-Drone-Racing-VAE-Imitation
    """

    def __init__(self, input_dim=1, latent_dim=64, with_logits=False):
        super(ImgDecoder, self).__init__()
        _dbg("[ImgDecoder] Starting create_model")
        self.with_logits = with_logits
        self.n_channels = input_dim
        self.dense = nn.Linear(latent_dim, 512)
        self.dense1 = nn.Linear(512, 9 * 15 * 128)

        # Pytorch docs: output_padding is only used to find output shape, but does
        # not actually add zero-padding to output
        self.deconv1 = nn.ConvTranspose2d(128, 128, kernel_size=3, stride=1, padding=1)
        self.deconv2 = nn.ConvTranspose2d(128, 64, kernel_size=5, stride=2, padding=(2, 2), output_padding=(0, 1), dilation=1)
        self.deconv4 = nn.ConvTranspose2d(64, 32, kernel_size=6, stride=4, padding=(2, 2), output_padding=(0, 0), dilation=1)
        self.deconv6 = nn.ConvTranspose2d(32, 16, kernel_size=6, stride=2, padding=(0, 0), output_padding=(0, 1))
        self.deconv7 = nn.ConvTranspose2d(16, self.n_channels, kernel_size=4, stride=2, padding=2)  # tanh or sigmoid
        _dbg("[ImgDecoder] Done with create_model")

    def forward(self, z):
        return self.decode(z)

    def decode(self, z):
        x = self.dense(z)
        x = torch.relu(x)
        x = self.dense1(x)
        x = x.view(x.size(0), 128, 9, 15)

        x = self.deconv1(x)
        x = torch.relu(x)

        x = self.deconv2(x)
        x = torch.relu(x)

        x = self.deconv4(x)
        x = torch.relu(x)

        x = self.deconv6(x)
        x = torch.relu(x)

        x = self.deconv7(x)
        if self.with_logits:
            return x
        x = torch.sigmoid(x)
        return x


# ===================== vae.py =====================
class Lambda(nn.Module):
    """Lambda function that accepts tensors as input."""

    def __init__(self, func):
        super(Lambda, self).__init__()
        self.func = func

    def forward(self, x):
        return self.func(x)


class VAE(nn.Module):
    """Variational Autoencoder for reconstruction of depth images."""

    def __init__(self, input_dim=1, latent_dim=64, with_logits=False, inference_mode=False):
        super(VAE, self).__init__()

        self.with_logits = with_logits
        self.input_dim = input_dim
        self.latent_dim = latent_dim
        self.inference_mode = inference_mode
        self.encoder = Encoder(input_dim=self.input_dim, latent_dim=self.latent_dim)
        self.img_decoder = ImgDecoder(input_dim=1, latent_dim=self.latent_dim, with_logits=self.with_logits)

        self.mean_params = Lambda(lambda x: x[:, :self.latent_dim])    # mean parameters
        self.logvar_params = Lambda(lambda x: x[:, self.latent_dim:])  # log variance parameters

    def forward(self, img):
        # encode
        z = self.encoder(img)
        # reparametrization trick
        mean = self.mean_params(z)
        logvar = self.logvar_params(z)
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        if self.inference_mode:
            eps = torch.zeros_like(eps)
        z_sampled = mean + eps * std
        # decode
        img_recon = self.img_decoder(z_sampled)
        return img_recon, mean, logvar, z_sampled

    def encode(self, img):
        z = self.encoder(img)
        means = self.mean_params(z)
        logvars = self.logvar_params(z)
        std = torch.exp(0.5 * logvars)
        eps = torch.randn_like(logvars)
        if self.inference_mode:
            eps = torch.zeros_like(eps)
        z_sampled = means + eps * std
        return z_sampled, means, std

    def decode(self, z):
        img_recon = self.img_decoder(z)
        if self.with_logits:
            return torch.sigmoid(img_recon)
        return img_recon

    def set_inference_mode(self, mode):
        self.inference_mode = mode
