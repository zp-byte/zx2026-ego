# real/orin_nx — Jetson Orin NX 8GB 真机部署包（D435 + PX4）

2026-10-09 组装。目标硬件：**Jetson Orin NX 8GB / 飞控 PX4 / 深度相机 D435 / 机载
Ubuntu 20.04 + ROS Noetic**。算法与 sim 完全同码（三桥一芯），换的只是输入源与
物理后端。

## 0. 拓扑一图

```
                    ┌──────────── 官方飞行栈（Diff-planner sh_files 拉起，非本包管辖）────────────┐
 D435 (USB3 直插)   │  mavros ←→ PX4     faster-lio → /laserMapping/cloud_registered            │
   │ 424×240@15     │                        └→ ekf → /ekf/ekf_odom（LIO 局部系）              │
   ▼                │                        └→ px4ctrl ~cmd=/setpoints_cmd → mavros → PX4      │
 /camera/depth/color/points                                                                      │
   │                                                                                             │
   ▼                    ┌──────────────── 本包（real_d435.launch，每机一份）────────────────┐   │
 d435.launch ───────────►│ odom_tf        /ekf/ekf_odom → /drone_<id>/odom (ENU)            │   │
 (realsense nodelet)     │ cloud_adapter   D435 points --相机系重投影+四道滤--> /drone_<id>/cloud │
                         │ nav_node        sim 同码：栅格+避障+群集 → /drone_<id>/vel_cmd     │   │
                         │ vel_bridge      vel_cmd ENU→局部系逆变换→ /setpoints_cmd (50Hz) ───┼──►│ px4ctrl
                         │ mission/executor/type_match/gcs_agent（sim 同码）                  │   │
                         └───────────────────────────────────────────────────────────────────┘   │
```

**注入点红线（2026-09-27 源码定谳）**：px4ctrl 订的是 **`/setpoints_cmd`**，不是
`/position_cmd`（历史误记）。vel_bridge 在 real_fleet.launch 已显式 `--out
/setpoints_cmd`，勿改。断链 0.5s → px4ctrl HOLD 定点（急停靠这个）。

## 1. 包结构

```
real/orin_nx/
├── README.md                          本文件
├── launch/
│   ├── d435.launch                    D435 降流驱动档（424×240@15、8m 剪远、TF 不发）
│   └── real_d435.launch               整机一键入口 = d435 + real_fleet（D435 默认值覆盖）
├── config/
│   ├── settings_real.yaml             部署时改名 sim_settings.yaml 覆盖到 zx2026_common/config/
│   └── missions.example.yaml          人工指派样例（从 tools/gcs/missions.yaml 拷出）
├── scripts/
│   ├── preflight.sh                   起飞前九检（FAIL 即禁飞）
│   ├── estop.sh                       软件急停 kill（断链→HOLD）/ zero（零速刹停）
│   └── install_deps.sh                apt 依赖 + udev + 自检（飞行栈不在此范围）
└── udev/
    └── 99-realsense-libusb.rules      D435 非 root 权限规则
```

源码桥不复制进包：`real_fleet.launch` 直接引用 `src/arena_bridges/` 的脚本
（runpy→tools/gcs 单一源），机载须完整 `zx2026_ego_ws` + catkin build。

## 2. Bring-up 序（0→6 步，顺序不可乱）

| 步 | 动作 | 判据 |
|---|---|---|
| 0 | 两个工作空间 build：`~/Diff-planner`（faster-lio/ekf/px4ctrl/quadrotor_msgs）与 `~/zx2026_ego_ws`（arena_*） | 无 error；`source devel/setup.bash` 后 rospack find 各包 OK |
| 1 | `sudo nvpmodel -m 0 && sudo jetson_clocks`（MAXN，50Hz 桥对调度敏感） | `nvpmodel -q` = MAXN |
| 2 | 飞行栈拉起：Diff-planner `sh_files`（mavros→LIO→ekf→px4ctrl 一链） | `rosnode list` 见四件套 |
| 3 | 部署三件套（一次性）：①`cp real/orin_nx/config/settings_real.yaml <ego_ws>/src/zx2026_common/config/sim_settings.yaml`（改名覆盖）②missions/profile 拷 tools/gcs/ 改 ids ③catkin build 一次让 settings 生效 | preflight 第 8 检过 |
| 4 | `roslaunch ~/zx2026_ego_ws/real/orin_nx/launch/real_d435.launch id:=<N>`（1 号机另加 `stage_local:=true`；本目录非 catkin 包，须全路径调） | 无 red 字；`rostopic list` 见三桥话题 |
| 5 | `bash real/orin_nx/scripts/preflight.sh <N>` | 输出 `PREFLIGHT PASS`（FAIL=禁飞） |
| 6 | 人工放行（真机无供给方，P1_INIT 会卡死不放）：`rostopic pub -1 /zx2026/fleet_ready std_msgs/Bool "data: true"` → P2 停稳后 `rosservice call /zx2026/start` | stage 机 log 见 P1→P2→P3 |

**现场必测三参（装订进 real_d435.launch 参数，写死）**：
`cam_tilt_deg`（D435 下俯角，下俯为正）/ `cam_dx,dy,dz`（光心在机体系平移）/
`yaw0_deg`（LIO 系 x 轴相对场地东向的安装偏角，三桥同源）。
猜错后果 = 整张障碍图错位。悬停目检法：起飞到 1m 悬停，`rostopic echo -n1
/drone_0/cloud`，核对云中树的方位与肉眼一致再放行。

## 3. 8GB 内存预算（量级账）

| 块 | 预算 |
|---|---|
| 系统+桌面(关闭 GUI 可省) | ~1.2 GB |
| mavros + PX4 相关 | ~0.3 GB |
| faster-lio + ekf | ~0.8 GB |
| px4ctrl | ~0.1 GB |
| D435 nodelet（424×240@15 点云） | ~0.4 GB |
| nav_node + executor + 三桥（纯 py） | ~0.6 GB |
| 余量 | ~4.5 GB |

swap 须存在（≥4GB zram/文件）；preflight 第 7 检看 `MemAvailable ≥ 800MB`。
**不许在机上跑训练/推理重的别的东西**——VEL 50Hz 桥的调度迟滞直接变成
`/setpoints_cmd` 抖动。

## 4. 安全边界（三层，软件层永远不是最后防线）

1. **硬层**：RC 三通——6 通悬停 / 5 通接管 / 7 通锁桨（px4ctrl 断链兜底同源语义）
2. **GCS 层**：panic（六机同帧 ABORT 语义在 sim 已验证，真机首飞前须对 hub 链联调）
3. **机载软件层**：`estop.sh kill`（杀 vel_bridge → /setpoints_cmd 断流 0.5s → HOLD
   定点，对 px4ctrl 语义最干净）；`estop.sh zero`（停 nav 写者后 50Hz 零速，平滑刹停）
   两者都不落桨；kill 后恢复 = 重拉 real_d435.launch。

## 5. 已定谳坑表（首飞检查单浓缩）

| 坑 | 定谳 | 对策 |
|---|---|---|
| px4ctrl 注入点 | **/setpoints_cmd** 非 /position_cmd（09-27 源码实锤） | launch 已纠偏，勿"修"回去 |
| D435 error-71 断流 | USB 接触病，换口解决（09 定谳） | USB3 直插禁 hub；断流签名=preflight 第 4 检掉零 |
| rs_camera overflow | 必带降流参数 | d435.launch 已配 424×240@15；勿回 848×480 |
| exec bit 剥离 | UNC/拷贝掉 +x → spawn 静默失败（二犯） | 拷机后 `chmod +x` 全部脚本+桥；preflight 第 9 检 |
| YAML 裸 off | `mode: off` 解析成 False（二犯） | heading_align.mode 带引号 "strict"/"off" |
| fleet_ready 无供给方 | 真机 P1_INIT 卡死 | bring-up 第 6 步人工发（latched，发一次即可） |
| via_slots 真值泄漏 | source=truth 是泄漏 | 装订审计须过（preflight 第 8 检附带） |
| cloud_adapter NaN | D435 无效像素 NaN/inf 防毒 | 10-09 已在滤内跳过（selftest 15/15） |
| 相机外参猜错 | 图整体错位=伪障碍/真盲区 | 三参现场量+悬停目检 |
| 地面滤 | 真机 D435 前视扫不到地面点时地面滤自然空转 | 误报地面=cam_tilt 大俯角下校准问题，先查外参 |

## 6. 诚实边界（本包 0% 物理验证，首飞按此纪律）

**整链物理验证 0%**——所有 sim 定谳在真机上只是"应携带的先验"，不是证据。首飞纪律：

1. **首飞 max_vel 1.2**（settings_real 已设；sim 定谳 2.0 须现场复验再抬）
2. **heading_align 首飞可一键回 off**（mode: "off" 一行）——sim 判词 strict 是深度臂
   视场痛点的解，真机 D435 视场(87°×58°)同形，但转向与真实动力学的耦合未验；
   建议第一架先 off 飞通，第二架再 strict 对比
3. **rigid_min_pts=36 / 云密度**：D435 点密度远超 sim 合成渲染，36 保守；若见细枝
   误锁存（地图假墙）现场上调
4. **LIO 漂移与退化**：sim 漂移注入=0（真机差异②），真实 LIO 在长直走廊/重复纹理
   的退化行为未测——场地预飞一次纯悬停看 /ekf/ekf_odom 漂移量级
5. **comms.expire_s=2.0** 是 sim 拍的，真机 WiFi 实测延迟联调日重校
6. **风 >0.8 m/s 停飞**（投放到位阈值硬约束，非建议）
7. preflight 九检 + estop 两模式均须**起飞前空跑一遍**（kill 模式在地面通电状态试，
   确认 px4ctrl 语义如断言）

## 7. 与 sim 的 4 处设置差异（settings_real.yaml 头部同文）

① `use_sim_time: false`（墙钟）② `drift_rate/max: 0`（真实定位误差不注入）
③ `comms.enabled: true`（邻居走 odom_from_<s> 契约，comm_model_node 不启动）
④ `collision.recovery.enabled: false`（无 sim 碰撞事件源；兜底=RC+panic）
2026-10-09 refresh：键结构与 sim_settings.yaml 全量对齐（heading_align /
rigid_min_pts 等 8 组新键补齐，除 heading_align 外均为结构对齐键）。
