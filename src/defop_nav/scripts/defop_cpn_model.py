#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_cpn_model.py — CPN 网络定义（vendored from ~/DeFoP, BSD-3）。

出处（版权与许可归属原仓库）：
  https://github.com/guglielmo610/DeFoP  (BSD-3-Clause)
  - src/planning/ORACLE/train/training.py:
      NetworkBuilder.action_network / build_predictor_recurrent_net /
      TrainCPNseVAE.build_collision_prediction_network / InferenceCPNseVAE
  - src/planning/ORACLE/inference/network_inference_tensorflow.py:
      seVAENetworkInferenceTensorflow 的建网+load_weights+load_model 装载序

适配改动（层名/形状逐字未动，state_dict/权重兼容性不受影响）：
  1) 训练侧只保留建网必需部分（train_step/test_step/自定义指标/loss builder
     全部裁掉——load_weights 只依赖层名与形状，主会话 smoke 已验证
     TrainCPNseVAE 建网+nett5/nett6 load_weights+前向 PASS，输出头
     [(1,15,1),(1,15,3),(1,15)]）；
  2) 去掉 gflags/common_flags/plot_model/plot 依赖与 __main__ 段；
  3) ACTION_HORIZON/ACTION_SHAPE_EVALUATE/STATE_INPUT_SHAPE 由
     defop_cpn_node.py 的 ROS 参数传入构造函数（DeFoP 原版为 config.py
     模块常量），默认值与 config.py 一致（15/3/6）；
  4) 新增 SeVAECPNEnsemble：把 seVAENetworkInferenceTensorflow 的
     "TrainCPNseVAE 建网->零输入 build->load_weights(hdf5)->InferenceCPNseVAE
     .load_model()" x N_E 装载序封装成一个入口，缺权重时报明确错误。

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
import os

import numpy as np
import tensorflow as tf

from tensorflow import keras
from tensorflow.keras import Model, Input


# ============================ NetworkBuilder（vendored） ============================

class NetworkBuilder(object):
    """仅保留 seVAE-CPN 推理需要的两个子网（Dronet 残差 CNN 不需要——
    PLANNING_TYPE=1 走 latent 输入）。"""

    @staticmethod
    def action_network(action_shape):
        """action (horizon,3) -> 16 维特征。层名 'action/dense0','action/dense1'。"""
        inputs = Input(shape=action_shape)
        x = keras.layers.Dense(16, activation='relu', name='action/dense0')(inputs)
        x = keras.layers.Dense(16, activation=None, name='action/dense1')(x)
        return Model(inputs, x, name='action_model')

    @staticmethod
    def build_predictor_recurrent_net(action_horizon=15, action_shape_evaluate=3):
        """RNN 碰撞预测头：LSTM(64) + Dense(32 relu) + Dense(1 sigmoid)。

        层名 'recurrent_layer','output_dense_1','output_dense_2'（权重装载依赖）。
        输出 (batch, horizon, 1) 每步碰撞概率。
        """
        initial_state_h = tf.keras.Input(shape=(64,), name="initial_state_h")
        initial_state_c = tf.keras.Input(shape=(64,), name="initial_state_c")

        action_submodel = NetworkBuilder.action_network(
            action_shape=(action_horizon, action_shape_evaluate))

        recurrent_layer = tf.keras.layers.LSTM(
            64, name='recurrent_layer', return_sequences=True)(
                inputs=action_submodel.outputs[0],
                initial_state=[initial_state_h, initial_state_c])

        output_layer = tf.keras.layers.Dense(
            units=32, name='output_dense_1', activation='relu')(recurrent_layer)
        output_layer = tf.keras.layers.Dense(
            units=1, name='output_dense_2', activation='sigmoid')(output_layer)

        return tf.keras.Model(
            inputs=[initial_state_h, initial_state_c, action_submodel.inputs[0]],
            outputs=[output_layer], name='predictor_recurrent')


# ============================ TrainCPNseVAE（vendored 裁剪版） ============================

class TrainCPNseVAE(Model):
    """seVAE-CPN 完整模型（仅推理装载用途）。

    输入 [robot_state (6,), latent (128,), action (horizon,3)]，
    输出 [collision (horizon,1), pos (horizon,3), yaw (horizon,1)]。
    推理只用第 1 个输出头；2/3 头是训练辅助（pos/yaw 回归），
    hdf5 权重里同名层必须存在，故网络结构完整保留。
    """

    def __init__(self, depth_latent_size=128, action_horizon=15,
                 action_shape_evaluate=3, state_input_shape=6):
        super(TrainCPNseVAE, self).__init__()
        self.predictor_model = self.build_collision_prediction_network(
            depth_latent_size, action_horizon, action_shape_evaluate,
            state_input_shape)

    def build_collision_prediction_network(self, depth_latent_size,
                                           action_horizon,
                                           action_shape_evaluate,
                                           state_input_shape):
        robot_state_input = tf.keras.Input(shape=(state_input_shape,),
                                           name="robot_state_input")
        robot_state_processed = tf.keras.layers.Dense(
            32, activation='relu', name='robot_state/dense0')(robot_state_input)
        robot_state_processed = tf.keras.layers.Dense(
            32, activation=None, name='robot_state/dense1')(robot_state_processed)

        depth_image_latent_input = tf.keras.Input(
            shape=(depth_latent_size,), name="depth_latent_input")
        depth_dense = tf.keras.layers.Dense(
            128, activation=None, name='depth_latent/dense0')(
                depth_image_latent_input)

        conc1 = tf.keras.layers.concatenate(
            [robot_state_processed, depth_dense], name="concatenate_depth_states")

        obs_lowd = tf.keras.layers.Dense(
            128, activation='relu', name='obs_lowd/dense0')(conc1)
        obs_lowd = tf.keras.layers.Dense(
            128, activation=None, name='obs_lowd/dense1')(obs_lowd)

        initial_state_h, initial_state_c = tf.split(obs_lowd, 2, axis=1)

        action_submodel = NetworkBuilder.action_network(
            action_shape=(action_horizon, action_shape_evaluate))

        recurrent_layer = tf.keras.layers.LSTM(
            64, name='recurrent_layer', return_sequences=True)(
                inputs=action_submodel.outputs[0],
                initial_state=[initial_state_h, initial_state_c])

        output_layer = tf.keras.layers.Dense(
            units=32, name='output_dense_1', activation='relu')(recurrent_layer)
        output_layer = tf.keras.layers.Dense(
            units=1, name='output_dense_2', activation='sigmoid')(output_layer)

        pos_yaw_output_layer = tf.keras.layers.Dense(
            units=32, name='pos_output_dense_1', activation='relu')(recurrent_layer)
        pos_yaw_output_layer = tf.keras.layers.Dense(
            units=4, name='pos_output_dense_2', activation=None)(
                pos_yaw_output_layer)

        return tf.keras.Model(
            inputs=[robot_state_input, depth_image_latent_input,
                    action_submodel.inputs[0]],
            outputs=[output_layer, pos_yaw_output_layer[..., 0:3],
                     pos_yaw_output_layer[..., 3]],
            name='actor_net')

    def call(self, inputs):
        return self.predictor_model(inputs, training=False)

    def summary(self, line_length=None, positions=None, print_fn=None):
        return self.predictor_model.summary(line_length, positions, print_fn)

    def get_model(self):
        return self.predictor_model


# ============================ InferenceCPNseVAE（vendored） ============================

class InferenceCPNseVAE(object):
    """分体推理模型：depth_state_combiner + recurrent_net 分开调用
    （evaluate.py 的 call_depth_state_combiner / call_recurrent_net 契约）。"""

    def __init__(self, depth_latent_size=128, action_horizon=15,
                 action_shape_evaluate=3, state_input_shape=6):
        self.depth_state_combiner = self.build_predictor_depth_robot_state_combiner(
            depth_latent_size, state_input_shape)
        self.recurrent_net = NetworkBuilder.build_predictor_recurrent_net(
            action_horizon, action_shape_evaluate)

        self.depth_state_combiner_outputs = self.depth_state_combiner(
            [self.depth_state_combiner.inputs[0], self.depth_state_combiner.inputs[1]])
        self.initial_state_h, self.initial_state_c = tf.split(
            self.depth_state_combiner_outputs, 2, axis=1)

        self.model = tf.keras.Model(
            inputs=[self.depth_state_combiner.inputs[0],
                    self.depth_state_combiner.inputs[1],
                    self.recurrent_net.inputs[2]],
            outputs=self.recurrent_net(
                [self.initial_state_h, self.initial_state_c,
                 self.recurrent_net.inputs[2]]),
            name='actor_net')

    def build_predictor_depth_robot_state_combiner(self, depth_latent_size,
                                                   state_input_shape):
        robot_state_input = tf.keras.Input(shape=(state_input_shape,),
                                           name="robot_state_input")
        robot_state_processed = tf.keras.layers.Dense(
            32, activation='relu', name='robot_state/dense0')(robot_state_input)
        robot_state_processed = tf.keras.layers.Dense(
            32, activation=None, name='robot_state/dense1')(robot_state_processed)

        depth_image_latent_input = tf.keras.Input(
            shape=(depth_latent_size,), name="depth_latent_input")
        depth_dense = tf.keras.layers.Dense(
            128, activation=None, name='depth_latent/dense0')(
                depth_image_latent_input)

        conc1 = tf.keras.layers.concatenate(
            [robot_state_processed, depth_dense], name="concatenate_depth_states")

        obs_lowd = tf.keras.layers.Dense(
            128, activation='relu', name='obs_lowd/dense0')(conc1)
        obs_lowd = tf.keras.layers.Dense(
            128, activation=None, name='obs_lowd/dense1')(obs_lowd)
        return tf.keras.Model(
            inputs=[robot_state_input, depth_image_latent_input],
            outputs=[obs_lowd], name='depth_robot_state_combiner')

    def load_model(self, model):
        """从完整 TrainCPNseVAE.predictor_model 按层名搬权重（原版逐字）。"""
        trainable_layers = [
            'robot_state/dense0', 'robot_state/dense1', 'depth_latent/dense0',
            'action/dense0', 'action/dense1',
            'obs_lowd/dense0', 'obs_lowd/dense1',
            'recurrent_layer', 'output_dense_1', 'output_dense_2']

        for layer in self.depth_state_combiner.layers:
            if layer.name in trainable_layers:
                layer.set_weights(model.get_layer(layer.name).get_weights())

        for layer in self.recurrent_net.layers:
            if layer.name in trainable_layers:
                layer.set_weights(model.get_layer(layer.name).get_weights())

    def call_depth_state_combiner(self, inputs):
        return self.depth_state_combiner.predict_on_batch(inputs)

    def call_recurrent_net(self, inputs):
        return self.recurrent_net.predict_on_batch(inputs)

    def summary(self):
        return self.model.summary()

    def get_model(self):
        return self.model


# ============================ 集成装载入口（新增封装） ============================

def load_sevae_cpn_ensemble(weight_paths, latent_size=128, action_horizon=15,
                            action_shape_evaluate=3, state_input_shape=6,
                            log=print):
    """按 seVAENetworkInferenceTensorflow 的装载序加载 N_E 个 CPN。

    每个权重：TrainCPNseVAE 建网 -> 零输入 build -> load_weights(hdf5)
    -> InferenceCPNseVAE.load_model()。
    返回 list[InferenceCPNseVAE]；任一权重缺失/损坏抛带路径的明确异常。
    """
    if not weight_paths:
        raise ValueError("load_sevae_cpn_ensemble: weight_paths 为空")
    zero_state = np.zeros((1, state_input_shape), dtype=np.float32)
    zero_latent = np.zeros((1, latent_size), dtype=np.float32)
    zero_action = np.zeros((1, action_horizon, action_shape_evaluate),
                           dtype=np.float32)

    models = []
    for idx, path in enumerate(weight_paths):
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            raise IOError(
                "CPN 权重不存在: %s (集成成员 %d/%d)——检查 defop_nav.yaml "
                "cpn_weight_paths 或 ~/DeFoP 仓库是否就位" % (path, idx + 1,
                                                            len(weight_paths)))
        tmp = TrainCPNseVAE(depth_latent_size=latent_size,
                            action_horizon=action_horizon,
                            action_shape_evaluate=action_shape_evaluate,
                            state_input_shape=state_input_shape)
        tmp([zero_state, zero_latent, zero_action])  # build 变量
        tmp.load_weights(path)
        inf = InferenceCPNseVAE(depth_latent_size=latent_size,
                                action_horizon=action_horizon,
                                action_shape_evaluate=action_shape_evaluate,
                                state_input_shape=state_input_shape)
        inf.load_model(tmp.get_model())
        models.append(inf)
        log("defop_cpn_model: loaded %s (%d/%d)" % (path, idx + 1,
                                                    len(weight_paths)))
    return models
