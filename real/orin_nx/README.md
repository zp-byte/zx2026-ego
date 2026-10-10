# real/orin_nx — Jetson Orin NX 8GB 真机部署包（D435 + PX4，VIO 主档）

2026-10-09 组装；同日换源定谳：**主定位=VINS-Fusion（D435 双目 infra + PX4 IMU）**，
faster-lio/EKF 降为备档（三条参数一条命令回退，见 §2 步骤 2b）。目标硬件：
**Jetson Orin NX 8GB / 飞控 PX4 / D435（深度+双目 infra+彩色）/ 机载
Ubuntu 20.04 + ROS Noetic**。算法与 sim 完全同码（三桥一芯），换的只是输入源与
物理后端。

## 0. 拓扑一图

```
                    ┌──────────── 官方飞行栈（Diff-planner sh_files 拉起，非本包管辖）────────────┐
 D435 (USB3 直插)   │  mavros ←→ PX4     VINS-Fusion（VIO 主档，vins_d435.launch）：             │
   │ infra1/2        │    infra1/2 640×480 + /mavros/imu/data_raw → /vins/imu_propagate          │
   │  640×480@15     │    （重力对齐；world 原点=VINS 初始化点，x 轴=初始化时机头朝向）           │
   │ depth 424×240@15│  备档（默认不起）：faster-lio → /laserMapping/cloud_registered            │
   │ color 424×240@15│                        └→ ekf → /ekf/ekf_odom（LIO 局部系）              │
   ▼                │                        └→ px4ctrl ~cmd=/setpoints_cmd → mavros → PX4      │
 /camera/{depth,color}/...                                                                       │
   │                                                                                             │
   ▼                    ┌──────────────── 本包（real_d435.launch，每机一份）────────────────┐   │
 d435.launch ───────────►│ odom_tf        /vins/imu_propagate → /drone_<id>/odom (ENU)       │   │
 vins_d435.launch ───────┤ cloud_adapter   D435 points --相机系重投影+四道滤--> /drone_<id>/cloud │
 (VINS 节点名=vins)      │ nav_node        sim 同码：栅格+避障+群集 → /drone_<id>/vel_cmd     │   │
                         │ vel_bridge      vel_cmd ENU→局部系逆变换→ /setpoints_cmd (50Hz) ───┼──►│ px4ctrl
                         │ mission/executor/type_match/gcs_agent/gps_home/drop_align（sim 同码）│   │
                         └───────────────────────────────────────────────────────────────────┘   │
```

**下视相机定谳（2026-10-10）**：投放对齐与识别线输入 = **WHEELTEC C100 独立下视**
（USB3.0 200 万像素 UVC 免驱单目，WHEELTEC 套件可选配件），**不是 D435 彩色流**——
一颗刚体相机不能既前视导航又下视对齐，斜装 θ 会把 z·tanθ 系统误差注入对齐换算
（θ=20°、z=1.5m ⇒ 0.55m）。驱动=usb_cam，节点名 `camera_down` ⇒ 话题
`/camera_down/image_raw` + `/camera_down/camera_info`；**禁复用 D435 的
`/camera/color/image_raw`**（两发布者同话题混流 = 前视 15Hz + 下视 30Hz 交错的
静默灾难）。

**注入点红线（2026-09-27 源码定谳）**：px4ctrl 订的是 **`/setpoints_cmd`**，不是
`/position_cmd`（历史误记）。vel_bridge 在 real_fleet.launch 已显式 `--out
/setpoints_cmd`，勿改。断链 0.5s → px4ctrl HOLD 定点（急停靠这个）。

**VINS 话题名定谳（2026-10-09）**：VINS `advertise("imu_propagate")` 是相对名，
前缀随节点名走——本包节点名定为 `vins` ⇒ 全名 `/vins/imu_propagate`，与官方
px4ctrl `run_ctrl_vio.launch` 的 odom remap 逐字同源；odom_tf/三桥/GCS profile
全链同一写法。**任何一端都不要再加 remap**（多端 remap 漂移=静默断链病）。

## 1. 包结构

```
real/orin_nx/
├── README.md                          本文件
├── launch/
│   ├── d435.launch                    D435 降流驱动档（depth/color 424×240@15、8m 剪远、
│   │                                  TF 不发；infra1/2 640×480@15 双目流=VINS 输入，
│   │                                  emitter 保持 on——散斑静态利特征跟踪）
│   ├── vins_d435.launch               VINS-Fusion 节点（cfg=vins_d435_ego.yaml；
│   │                                  机上部署三步见其头注释）
│   ├── real_d435.launch               整机一键入口 = d435 + real_fleet（VIO 主档默认值
│   │                                  odom_src=/vins/imu_propagate、cloud_frame=camera）
│   └── camera_down.launch             C100 下视驱动档（usb_cam；节点名 camera_down ⇒
│                                      /camera_down/image_raw，yuyv 640×480@30）
├── config/
│   ├── vins_d435_ego.yaml             VINS 主参（fd250 模板改：双目 infra+PX4 IMU、
│   │                                  estimate_extrinsic:1 / estimate_td:1 在线优化）
│   ├── vins_left.yaml / vins_right.yaml  infra PINHOLE 内参（出厂值装机；Kalibr 现场
│   │                                  标定后回填，K 顺序 fx/cx/fy/cy）
│   ├── settings_real.yaml             部署时改名 sim_settings.yaml 覆盖到 zx2026_common/config/
│   └── missions.example.yaml          人工指派样例（从 tools/gcs/missions.yaml 拷出）
├── scripts/
│   ├── preflight.sh                   起飞前十一检（FAIL 即禁飞；③ odom 双探=VINS 主档
│   │                                  优先、EKF 备档 warn；④ GPS/⑥ RGB 取 warn 不禁飞）
│   ├── estop.sh                       软件急停 kill（断链→HOLD）/ zero（零速刹停）
│   └── install_deps.sh                apt 依赖 + udev + 自检（飞行栈不在此范围）
└── udev/
    ├── 99-realsense-libusb.rules      D435 非 root 权限规则
    └── 99-camera-down.rules           C100 稳定软链模板（VID:PID 现场装订 → /dev/camera_down）
```

源码桥不复制进包：`real_fleet.launch` 直接引用 `src/arena_bridges/` 的脚本
（runpy→tools/gcs 单一源），机载须完整 `zx2026_ego_ws` + catkin build。

## 2. Bring-up 序（0→6 步，顺序不可乱）

| 步 | 动作 | 判据 |
|---|---|---|
| 0 | 工作空间 build：`~/Diff-planner`（px4ctrl **run_ctrl_vio.launch** 档）+ **VINS-Fusion 拷入机载 ws `catkin build vins`**（部署三步见 vins_d435.launch 头注释；fd250 fork 源码自带 ceres-2.0.0rc1）+ `~/zx2026_ego_ws`（arena_*） | `rospack find vins` 响亮报错即缺件；各包 rospack find OK |
| 1 | `sudo nvpmodel -m 0 && sudo jetson_clocks`（MAXN，50Hz 桥对调度敏感） | `nvpmodel -q` = MAXN |
| 2 | 飞行栈拉起（VIO 主档）：mavros（官方 sh_files 含 mavcmd 511 限流四连）→ **VINS**（`roslaunch real/orin_nx/launch/vins_d435.launch`）→ px4ctrl（**`run_ctrl_vio.launch`**，odom←/vins/imu_propagate） | `/vins/imu_propagate` hz≥50；`rosnode list` 见 mavros/vins/px4ctrl |
| 2b | （备档，可选）LIO 链：Diff-planner sh_files 原链拉起后，整机入口改传 `odom_src:=/ekf/ekf_odom cloud_src:=/laserMapping/cloud_registered cloud_frame:=world` | `/ekf/ekf_odom` hz≥20；preflight ③ 会自动双探给 warn |
| 3 | 部署三件套（一次性）：①`cp real/orin_nx/config/settings_real.yaml <ego_ws>/src/zx2026_common/config/sim_settings.yaml`（改名覆盖）②missions/profile 拷 tools/gcs/ 改 ids ③catkin build 一次让 settings 生效 | preflight 第 10 检过 |
| 4 | `roslaunch ~/zx2026_ego_ws/real/orin_nx/launch/real_d435.launch id:=<N>`（1 号机另加 `stage_local:=true`；本目录非 catkin 包，须全路径调。gps_home/drop_align 默认 true，drop_bridge 默认 false=投放停在安全侧；**C100 已装机则加 `camera_down:=true camera:=true`**——preflight ⑥ 与对齐/识别线全靠它） | 无 red 字；`rostopic list` 见三桥+home/align 状态话题 |
| 5 | `bash real/orin_nx/scripts/preflight.sh <N>` | 输出 `PREFLIGHT PASS`（FAIL=禁飞；WARN=操作员裁决） |
| 6 | 人工放行（真机无供给方，P1_INIT 会卡死不放）：`rostopic pub -1 /zx2026/fleet_ready std_msgs/Bool "data: true"` → P2 停稳后 `rosservice call /zx2026/start` | stage 机 log 见 P1→P2→P3 |

**现场必测三参（装订进 real_fleet.launch 参数，写死）**：
`cam_tilt_deg`（D435 下俯角，下俯为正）/ `cam_dx,dy,dz`（光心在机体系平移）/
`yaw0_deg`（ odom 系 x 轴相对场地东向的安装偏角，三桥同源）。
猜错后果 = 整张障碍图错位。悬停目检法：起飞到 1m 悬停，`rostopic echo -n1
/drone_0/cloud`，核对云中树的方位与肉眼一致再放行。

**C100 下视装订四件（现场一次，2026-10-10 定谳新增）**：
1. **udev 软链**：lsusb 找 C100 的 VID:PID，替换 99-camera-down.rules 模板里
   XXXX/yyyy，拷 /etc/udev/rules.d/ 后 `udevadm control --reload-rules &&
   udevadm trigger` 拔插一次——`ls -l /dev/camera_down` 出现才算成。launch 绑
   软链不绑 /dev/videoN（机上多 USB 设备编号随上电顺序漂移，绑 N 迟早绑错机）
2. **分辨率与 min_blob_area_px 重标**：camera_down.launch 默认 640×480；识别线
   检测参数 min_blob_area_px 按 sim 320×240@0.8m 标定，换分辨率后必须重标
   （drop_align 诚实边界既知）
3. **camera_vfov_deg**：C100 垂直视场角（real_fleet.launch arg，默认 50.0=占位），
   按规格书装订；错 vfov=错米制偏移比例。未标定时走解析式回退内参
   f=(h/2)/tan(vfov/2) 也能飞；做过 camera_info 标定后自动优先接管
4. **align_axes**：top_fwd=顶边朝机头 / bottom_fwd=底边朝机头（real_fleet.launch
   arg）——装机后必核，接反=伺服反向推离色盘

**VINS 标定三件 + 初始化纪律（VIO 主档新增）**：
1. **infra 内参**：vins_left/right.yaml 现装出厂值（fx=fy≈382.580, cx≈321.421,
   cy≈235.155, 畸变全 0）；Kalibr 现场标定后回填（K 顺序 fx/cx/fy/cy）
2. **body_T_cam0/1 外参**：fd250 初值 + `estimate_extrinsic: 1` 在线优化；首飞
   稳定后把优化结果固化回 yaml（每次都在线估=收敛不一致）
3. **td 相机-IMU 时延**：`estimate_td: 1` 在线估，同理固化
初始化纪律：VINS world 原点=初始化点、x 轴=初始化时机头朝向 ⇒ **起飞前机头对准
场地东向、整机停在 ENU 原点桩位再起 VINS**，则 yaw0/ex0/ny0 全 0 免装订；对不准
才填非零（GPS 锚点在差值中相消，坐标口径详见 gps_home_node.py 头注释）。

## 3. 8GB 内存预算（量级账）

| 块 | 预算 |
|---|---|
| 系统+桌面(关闭 GUI 可省) | ~1.2 GB |
| mavros + PX4 相关 | ~0.3 GB |
| VINS-Fusion（VIO 主档；备档 LIO 不起则省回 0.8） | ~1.0 GB（估，未实测） |
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
| D435 error-71 断流 | USB 接触病，换口解决（09 定谳） | USB3 直插禁 hub；断流签名=preflight 第 5/6 检掉零 |
| VINS 未初始化 | infra 流没开/IMU 流没起=头号根因 | d435.launch enable_infra1/2 已配；检 3 掉零先查检 5 与 mavros IMU 流 |
| VINS 相对话题名 | advertise("imu_propagate") 前缀随节点名 | 节点名锁死 `vins`，三端零 remap（见 §0） |
| XML 注释 ASCII `--` | roslaunch 必炸（vins_d435.launch 首验抓出） | 注释禁 `--`，破折号一律中文—— |
| rs_camera overflow | 必带降流参数 | d435.launch 已配 424×240@15；勿回 848×480 |
| exec bit 剥离 | UNC/拷贝掉 +x → spawn 静默失败（二犯） | 拷机后 `chmod +x` 全部脚本+桥；preflight 第 11 检 |
| YAML 裸 off | `mode: off` 解析成 False（二犯） | heading_align.mode 带引号 "strict"/"off" |
| fleet_ready 无供给方 | 真机 P1_INIT 卡死 | bring-up 第 6 步人工发（latched，发一次即可） |
| via_slots 真值泄漏 | source=truth 是泄漏 | 装订审计须过（preflight 第 8 检附带） |
| cloud_adapter NaN | D435 无效像素 NaN/inf 防毒 | 10-09 已在滤内跳过（selftest 15/15） |
| 相机外参猜错 | 图整体错位=伪障碍/真盲区 | 三参现场量+悬停目检 |
| 地面滤 | 真机 D435 前视扫不到地面点时地面滤自然空转 | 误报地面=cam_tilt 大俯角下校准问题，先查外参 |
| YUYV 静默腐蚀 | usb_cam 默认 YUYV，旧消费者「非 rgb8/bgr8 一律当 bgr8」硬解=颜色全错恒 NO_DET 只有 warning（10-10 定谳） | 已根修：to_bgr 白名单+拒帧（color_detect.py）；preflight ⑥ 探 encoding 白名单 |
| /dev/videoN 漂移 | 机上多 USB 设备编号随上电顺序漂移，绑 N 迟早绑错机 | udev 99-camera-down.rules 发 /dev/camera_down 软链，launch 绑软链 |

## 6. 诚实边界（本包 0% 物理验证，首飞按此纪律）

**整链物理验证 0%**——所有 sim 定谳在真机上只是"应携带的先验"，不是证据。首飞纪律：

1. **首飞 max_vel 1.2**（settings_real 已设；sim 定谳 2.0 须现场复验再抬）
2. **heading_align 首飞可一键回 off**（mode: "off" 一行）——sim 判词 strict 是深度臂
   视场痛点的解，真机 D435 视场(87°×58°)同形，但转向与真实动力学的耦合未验；
   建议第一架先 off 飞通，第二架再 strict 对比
3. **rigid_min_pts=36 / 云密度**：D435 点密度远超 sim 合成渲染，36 保守；若见细枝
   误锁存（地图假墙）现场上调
4. **VINS 漂移与退化（主定位新风险）**：sim 漂移注入=0（真机差异②），VINS 在
   长直走廊/重复纹理/弱纹理的退化行为未测——场地预飞一次纯悬停看
   `/vins/imu_propagate` 漂移量级；LIO 备档同险（/ekf/ekf_odom）
5. **VINS 外参/td 在线优化未固化**：estimate_extrinsic/estimate_td 初飞在估，
   飞稳一轮后固化回 yaml（不固化=每次上电收敛结果漂移）
6. **comms.expire_s=2.0** 是 sim 拍的，真机 WiFi 实测延迟联调日重校
7. **风 >0.8 m/s 停飞**（投放到位阈值硬约束，非建议）
8. preflight 十一检 + estop 两模式均须**起飞前空跑一遍**（kill 模式在地面通电状态
   试，确认 px4ctrl 语义如断言）
9. **GPS/RGB 是辅助链（warn 不禁飞）**：GPS 爆值/无 fix 时 executor 返航自动回退
   场景 pad（correct 档钳位 gps_max_corr_m=5m）；RGB 断流对齐回退平台心——
   退化为降级任务非不安全，禁不禁飞操作员裁决
10. **C100 下视链零物理验证**：usb_cam 驱动 / YUYV 拒帧门 / 解析式回退内参 /
    对齐轴档均未上机。装机四件装订（§2）→ 地面通电跑 preflight ⑥（hz≥10 且
    encoding 在白名单）→ 手持悬停看 `/drone_0/align/corr` 方向与手移方向一致
    （不一致=align_axes 接反）

## 7. 与 sim 的 4 处设置差异（settings_real.yaml 头部同文）

① `use_sim_time: false`（墙钟）② `drift_rate/max: 0`（真实定位误差不注入）
③ `comms.enabled: true`（邻居走 odom_from_<s> 契约，comm_model_node 不启动）
④ `collision.recovery.enabled: false`（无 sim 碰撞事件源；兜底=RC+panic）
2026-10-09 refresh：键结构与 sim_settings.yaml 全量对齐（heading_align /
rigid_min_pts 等 8 组新键补齐，除 heading_align 外均为结构对齐键）。

## 8. GPS 记点 / 下视 RGB 对齐 / 投放桥（2026-10-09 三件）

任务主链两条新能力，全在 real_fleet.launch args 控制：

| arg | 默认 | 语义 |
|---|---|---|
| `gps_home` | true | 起飞记 GPS 起飞点 → `/drone_<id>/home/corr` 修正 executor 返航 pad。`gps_home_mode`：replace=GPS 目标接管（场景 pad 未实测装订时的正确档）/ correct=距原 pad 超 `gps_max_corr_m`(5m) 拒用（GPS 爆值/帧失准防毒） |
| `drop_align` | true | 下视 RGB 视觉伺服对齐（仅 `camera:=true` 时拉起）→ `/drone_<id>/align/corr` 投放前修正。`align_axes`：top_fwd=顶边朝机头（旧名 d435_down 同义）/bottom_fwd=底边朝机头（旧名 sim 同义）——**接反=伺服反向（推离色盘）**，C100 装机后必核 |
| `camera` | false | 下视识别线开关（C100 驱动 `camera_down:=true` 起后置 true） |
| `camera_down` | false | C100 驱动开关（real_d435.launch：include camera_down.launch 起 usb_cam 节点；preflight ⑥ 查 /camera_down/image_raw，须 true 才能过检） |
| `camera_topic` | /camera_down/image_raw | 下视图像话题（识别线+drop_align 共用输入；**与 real_d435.launch 同名 arg 必须同值**——pass_all_args 会用上层默认覆盖此处默认，selftest L4 对质） |
| `camera_vfov_deg` | 50.0 | C100 垂直视场角（规格书装订项；未标定时解析式回退内参 f=(h/2)/tan(vfov/2)，camera_info 标定后自动优先接管） |
| `drop_bridge` | false | 投放执行桥（`drop_backend`: auto=地面联调假应答**带弹飞行严禁** / serial=机构串口行协议）。false=释放指令发空，**DROP 自愈**：`drop_wait_timeout_s`（真机默认 15s，0=关）超时走与 payload/done 同一返航出口自动回家；桥开着卡死时仍可人工 `rostopic pub -1 /drone_<id>/payload/done std_msgs/Bool "data: true"` 提前收尾 |

- 数学锚点：GPS 修正目标 = gps_home_enu + (odom_now − gps_now_enu)，**锚点在差值
  中相消**——对 odom 源零假设（VIO/LIO 两档通用），venue_gps.yaml 仅诊断（home
  是否落在起降区带内），已从桌面《比赛坐标》两 docx 抄录入
  zx2026_common/config/venue_gps.yaml。
- GCS 状态透出：profile_real_vio/lio.yaml 已接线 `home_gps: /drone_<id>/home/status`
  与 `align: /drone_<id>/align/status`（String latch），agent/hub 全链透出到面板
  status 文件；phase/match/drop_done/metrics 亦已接线（executor/type_match/
  payload_bridge 真源直发）。
- 自检：`python3 tools/realpack_gps_selftest.py`（五层：三节点 --selftest 子进程 /
  _home_pad_apply 纯函数 / venue 换算烟测 / GCS 透出链断言 / C100 对质+DROP 自愈门
  ——两文件 camera_topic 默认同值 / camera_down.launch 绑 /dev/camera_down /
  preflight ⑥ 指向 /camera_down/image_raw / executor DROP 超时自愈与 launch 下传）。
- 下视 RGB 相机归属**已定谳（2026-10-10）**：WHEELTEC C100 独立下视（几何论据见
  §0）。现场遗留四件装订见 §2「C100 下视装订四件」。
