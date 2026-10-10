# defop_nav — DeFoP 三节点推理链适配（arena 契约，sim/真机同码）

把 DeFoP 论文官方实现（~/DeFoP，BSD-3，只读参考仓）的 seVAE-ORACLE 推理链
移植成本工作区 ROS1 Noetic 契约节点：输入 `/drone_<id>/` 族话题，输出
`/drone_<id>/vel_cmd`（vel_bridge 下游契约）。**模型定义 vendored 进本包
（BSD-3 头保留出处），权重引用 ~/DeFoP 仓库内文件不拷贝。**

## 架构

```
                        ~/DeFoP 权重（引用不拷贝）
                        ├─ seVAE_evo_LD_128.pth (torch)
                        └─ nett5/nett5/nett6 saved-model.hdf5 (TF)
                                    │ load
 /drone_<id>/depth/image_raw        │
   (16UC1 毫米 480x270@15fps)       │
        │                           │
        v                           │
  [1] defop_depth_fill.py           │   fill_invalid_depth_robust 补洞
        │  /drone_<id>/depth/image_filled (16UC1 毫米)
        v                           v
  [2] defop_vae_node.py  -----> /drone_<id>/latent_space (Float32MultiArray 128)
        │                           │
        │   /drone_<id>/odom (世界 ENU)         /drone_<id>/planning/goal (PoseStamped)
        v                                       v (goal_source=topic|waypoint_file)
  [3] defop_cpn_node.py
        ├─ 几何监督器 filter_actions_by_grid_3（扇区网格否决 + margin 扩张）
        ├─ CPN 3 集成 + Unscented Transform -> 256 基元碰撞概率
        ├─ 行为树：deadend 逃逸 / slowdown / 航点推进 / follow_goal 选择
        ├─ 帧手术：机体系基元 -> 世界系速度
        └─ 安全包络：水平 1.2 限幅 + 高度带 [0.3, 4.5] 门控 + 断流 HOLD
        │
        v
  /drone_<id>/vel_cmd (Twist, 世界系 linear.xyz + angular.z=yaw 率)
  /drone_<id>/defop/debug (String JSON: 基元/碰撞概率/否决数/wp/模式)
```

启动：`roslaunch defop_nav defop.launch drone_id:=0`（真机加
`depth_in:=/camera/depth/image_rect_raw`；航点模式加
`goal_source:=waypoint_file waypoint_file:=<yaml>`）。

## 运行环境（~/defop_env，py3.8 venv）

三节点脚本必须用 venv 解释器跑（launch 已用 `launch-prefix="$(arg python_exe)"`
指定，默认 `~/defop_env/bin/python`）：

- TF 2.4.1 + h5py 2.10（CPN）；torch 2.4.1+cpu（VAE，已装）
  `~/defop_env/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu`
- 脚本内置 `_ensure_ros()`：venv 里 import rospy 失败时自动补
  `/opt/ros/noetic/lib/python3/dist-packages`，所以 venv python 直接跑 ROS 节点；
  反向（系统 python3 跑 venv 依赖）不可行，缺 TF/torch 时节点打明确错误并 exit。
- 手动单跑示例：`~/defop_env/bin/python ~/zx2026_ego_ws/src/defop_nav/scripts/defop_cpn_node.py`

## 帧手术（DeFoP 真机 Twist -> arena 世界系 vel_cmd）

**DeFoP 原版指令**（evaluate.py L1487-1490 + realtime_ros_wrapper_infogain_ardupilot
.step()）：机体系 Twist

```
linear.x = vel_x                     # 基元前向速度（CMD_VELOCITY=1.0 恒值）
linear.y = vel_y = 0.5*tanh(yaw_rel) # 侧向速度，y 左正
linear.z = vel_z                     # 基元垂向速度 = cmd_velocity*tan(elevation)
angular.z = yaw_ctrl                 # yaw 速率 = clip(1.0*yaw_rel, ±pi/4)
```

注：wrapper 里 `angular.z = action[0][3] # reference yaw angle HERE!` 的注释
是陈旧的——`calculate_yaw_ctrl` 返回的是**速率**（K*yaw_error 限幅 ±pi/4）；
只有 `yaw_in_spot()`/`direction_service` 传角度语义，本移植已把这两处换算成
限速率旋转（见"差异清单"第 5 条）。

**arena 契约**（vel_bridge.py 已核实）：`/drone_<id>/vel_cmd` = 世界系 ENU
`linear.xyz` + `angular.z = yaw_dot`；yaw 从 odom 四元数提取：
`yaw = atan2(2(wz+xy), 1-2(y²+z²))`，机体系前向 = 世界系 `(cos yaw, sin yaw)`。

正变换（指令侧，每拍）：

```
[vx_w, vy_w] = Rz(yaw)·[vx, vel_y]      vx_w =  vx·cos(yaw) - vel_y·sin(yaw)
                                         vy_w =  vx·sin(yaw) + vel_y·cos(yaw)
vz_w = vel_z（再过高度带门控）
angular.z = yaw_ctrl（原值直传）
```

逆变换（CPN 状态侧）：DeFoP 的状态前三维是 `velocity_W_psi =
R.from_euler('xy',[roll,pitch])·v_body`（去 roll/pitch 留 yaw 的速度系）。由
`R_full = Rz(yaw)·Ry(pitch)·Rx(roll)` 且 `v_world = R_full·v_body` 可得
`velocity_W_psi = Rz(-yaw)·v_world`——**对世界系速度直接去 yaw 即可**，与
原公式逐字等价。sim 侧 odom twist 已核实为世界系
（drone_vel_plugin.cpp `WorldLinearVel`），故 `odom_twist_frame: world` 默认；
真机若上游 odom twist 是机体系，改参数 `odom_twist_frame: body`（节点内先
Rz(yaw) 转世界系再走同一管线）。

**z 钳位语义**：`z_min=0.3 / z_max=4.5` 是**高度带门控**不是速度钳位——当前
高度低于 0.3 m 时禁负 vz、高于 4.5 m（= flight_z_ceiling，距 OVER_HEIGHT 5.0
保 0.5 m）时禁正 vz。另加 `vz_abs_max=1.0` 速度硬限与 `vel_max=1.2` 水平模长
限幅（vel_bridge 还有全 3D max_vel 限幅 + 0.5s 断链 HOLD，双保险不冲突）。

## 与 ~/DeFoP 原版的差异清单

1. **ns 化**：原版全局话题（/latent_space、cmd_velocity、/d435/...）全部改为
   `/drone_<id>/` 前缀绝对名；参数从 config.py 模块常量改为 ROS 私有参数
   （config/defop_nav.yaml）。
2. **三节点拆分**：原版 evaluate.py 单进程死循环（队列+get_new_obs 轮询）拆为
   depth_fill / vae / cpn 三节点；vae 与规划解耦（原版 seVAE 推理就是独立节点
   vae_node.py，本移植保持该形态）。
3. **帧手术**：指令侧机体系->世界系（见上）；状态侧世界系->yaw 对齐系
   （等价证明见上）。真机 angular.z 角度语义的两处（yaw_in_spot、waypoint
   对准）换算为限速率旋转：死端逃逸 `angular.z = ±yaw_rate_spot(30°/s)·dir`，
   航点对准 `angular.z = clip(-1.5·Δyaw, ±yaw_rate_spot)`。
4. **航点源双轨**：`goal_source=topic` 订阅 `/drone_<id>/planning/goal`
   （PoseStamped，arena 任务层发布；到达 `goal_reach_dist=0.8` 后原地悬停等
   新目标）；`goal_source=waypoint_file` 读 yaml 航点列表，复用原版
   reach_goal_action（3.0 m 2D 阈值推进 + ±45° 对准 + 终点 FAIL=done）。
5. **安全包络**（原版无）：vel_max 1.2 / 高度带 [0.3,4.5] / vz_abs_max 1.0 /
   depth+latent 断流 >1.0s HOLD 零速 WARN 一次 / 规划异常兜底零速。
6. **方向服务与超时移除**：`set_goal_dir` 服务（原版外部指挥接口）与
   A-ORACLE timeout 机制（PLANNING_TYPE=1 本来就恒 SUCCESS）不移植。
7. **无模型降级**（默认关，`allow_degraded=false` 时模型装载失败即 exit）：
   TF/权重缺失时可用纯监督器模式（无 CPN 打分，选目标航向差最小的监督器
   安全基元，同档 tie-break 用高度差；yaw 库 32 档无精确 0，最近档
   ±0.022 rad → vy 残差 ≤0.011；监督器全否决仍走死端 HOLD）。单元测试用
   该路径验证监督器。
8. **监督器参数**：取 filter_actions_by_grid_3 函数签名默认档
   min_dist=1.5/min_ratio=0.02/drone_radius=0.32（evaluate.py 实跑传的是
   1.0/0.05/0.30，按场地可调）。
9. **TF 权重装载**：hdf5 为 weights-only 语义——按原版
   seVAENetworkInferenceTensorflow 的装载序（TrainCPNseVAE 建网 -> 零输入
   build -> load_weights -> InferenceCPNseVAE.load_model 按层名搬权重）×3。

## 权重文件清单（引用路径，不拷贝）

| 模型 | 路径 | 框架 |
|---|---|---|
| seVAE encoder | `~/DeFoP/src/planning/sevae/sevae/weights/seVAE_weights/seVAE_evo_LD_128.pth` | torch (LD=128) |
| CPN 集成 x3 | `~/DeFoP/src/planning/ORACLE/model_weights/VAE_EVO_back_to_origin/{nett5,nett5,nett6}/saved-model.hdf5` | TF keras (horizon=15, vel_max=2.0 档) |

CPN 网络定义 vendored 于 `scripts/defop_cpn_model.py`（training.py 裁剪版，
层名逐字未动）；VAE 定义 vendored 于 `scripts/sevae_vae_def.py`
（encoder/decoder 逐字）。两文件头部均保留 BSD-3 声明与出处。

## 测试证据（py3.8，~/defop_env/bin/python）

- VAE 前向：加载 .pth -> 随机 (270,480) 输入 -> latent shape==(1,128)
- CPN 加载：3 集成 hdf5 load -> 零输入 + 随机输入前向，输出 (N,15,1)；
  全链路打分 (7 sigma x 256 基元) 输出 (256,1)。**打分尺度=horizon 时间衰减
  加权和（Σw≈10.9），0-11.5 档非单步概率**——deadend 阈值 6.5/5.0 即此尺度；
  实测空场景打分范围 [0.016, 3.23]，全 0.5m 墙监督器全否决。
- 帧换算：yaw=90° 时机体系前向 1.0 m/s -> 世界系 (0,1)；状态侧往返一致
- 监督器：全 0.5 m 合成深度图 -> 256 基元全否决 -> HOLD 零速（死端停机）

## 已知局限

1. **CPU 推理速度**：原版在 Xavier NX 上跑 TensorRT；本移植 TF CPU 打分一次
   约 0.5-2 s（3 网 x (7x256) x LSTM15）。`plan_rate=10Hz` 定时器遇慢推理
   自动跳拍（不堆积），等效降频运行；上 TRT/GPU 才能回到 15fps 原设计。
2. **-1 无效标记被 clamp 压 0**：原版 vae_node 预处理把 <0.2 m 像素置 -1，
   随后 VAENetworkInterface.forward 的 `torch.clamp(0,1)` 把 -1 压成 0。
   这是原版发布链的既有行为，本移植逐字保留（语义=无效像素喂 0）。
3. **监督器吃的是补洞后深度**（原版同）：fill 的 kernel_size=100 在 270x480
   上每帧约几十 ms（cv2 胶囊形核已测），15fps 可承受但占核。
4. **死端逃逸速率档**：原版 yaw_in_spot 角度语义依赖其下游控制器解释，移植
   改为固定速率旋转（30°/s），旋转 90° 需 3 s，比原版慢但可控。
5. **未实飞**：整链未进 Gazebo 闭环（本阶段只做静态/单元级验证）；vel_cmd
   上游限幅与 vel_bridge 下游限幅的重叠行为需联调日对质。
6. **depth 非 480x270 丢帧**：监督器按 480x270 网格切扇区，尺寸不符直接丢帧
   并 WARN（不静默缩放）；VAE 侧倒是会自动 resize。
