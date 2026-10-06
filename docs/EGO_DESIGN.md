# zx2026_ego_ws 设计案卷：ego-planner 换笔路线

- **日期**：2026-10-05
- **工作区**：`~/zx2026_ego_ws`（由 `zx2026_defop_ws` 源码基线快照创建，defop 机制源保留默认关）
- **母本**：`~/fd250_ws/Fast-Drone-250`（ZJU-FAST-Lab ego-planner 全栈，28 包编译绿 + 09-28 sim 闭环实飞，git 8ff6427）
- **用户裁决**：① D435 = **真加 Gazebo depth camera 插件**（不复用真值渲染 depth_sim）；② 里程计 = **odom 进规划器**（ego 标准输入三件套：odom+depth+goal）

## 一、总架构：ego 当笔（镜像刀一.5）

```
mission_executor ──/drone_i/planning/goal(PoseStamped, ENU)──┐
/drone_i/odom(Odometry, world ENU)───────────────────────────┤
/drone_i/depth/image_raw(32FC1, Gazebo D435)─────────────────┤
                                                             ▼
                              ego_planner_node( FSM: 目标触发→B样条梯度优化→重规划 )
                                                             │ ~planning/bspline
                                                             ▼
                              traj_server(PositionCommand@100Hz, 轨迹前视 yaw)
                                                             │ /drone_i/planning/pos_cmd
                                                             ▼
                              ego_bridge_node.py（新包 ego_zx）
                                PositionCommand → Twist(linear=cmd.velocity 世界ENU,
                                                        angular.z=cmd.yaw_dot)
                                发 /drone_i/planning/cpn_vel_raw（中继话题，复用 nav 全套）
                                                             ▼
                              nav_node（唯一 /vel_cmd 发布者，白名单 vel_source_of 加 'ego'）
                                + Boids 机间 sep + stall 看门狗 + arrived 闩锁 + 降落收尾
                                                             ▼
                                              /drone_i/vel_cmd → drone_vel_plugin(kp=3.0)
```

**与 defop 档的唯一差异**：规划器换笔（DeFoP→ego）。nav/任务/判分/世界/服务（`/zx2026/start`）全栈不动。
ego 不做 pen_authority=cpn_forever 起飞闸门（无 torch 模型，秒就绪；闸门依赖不配置即不存在）。

## 二、vendor 清单（fd250 → src/，来源 commit 8ff6427，记录 VENDORED.md）

| 包 | 用途 |
|---|---|
| plan_manage（改包名 ego_planner） | ego_planner_node + traj_server + advanced_param.xml |
| plan_env | GridMap（深度→占据，无 ESDF） |
| path_searching | A*/kinodynamic 前端 |
| bspline_opt | B 样条梯度优化后端 |
| traj_utils | Bspline 消息 + 多机共享 |
| utils/quadrotor_msgs、utils/uav_utils | 消息/工具依赖 |

跳过：drone_detect、rosmsg_tcp_bridge、uav_simulator（local_sensing 真值渲染不需要——Gazebo 插件供深度）。

**vendored 补丁（2 处，全部记录进 VENDORED.md）**：
1. `traj_server.cpp:241` `advertise("/position_cmd")` 绝对名 → **相对名 "position_cmd"**。
   原版 remap `from="position_cmd"` 对绝对名不生效，六机会互踩同一全局 `/position_cmd`。
2. `ego_replan_fsm.cpp:220` waypointCallback `end_wp z 写死 1.0`（rviz 2D 惯例）→
   **透传 `msg->pose.position.z`**。不改则 mission 分腿 z（RECON 1.2 / cruise 2.4 /
   投放 0.8 / pad 0.3）全被压平。`:213` 的 z<-0.1 忽略守卫不碍（mission goal z 恒>0）。
   z 安全网：FSM `virtual_ceil_height 4.4` + OVER_HEIGHT 判分兜底。
3. 无其他代码改动。`k_depth_scaling_factor`、内参、virtual_ceil 全走 launch 参数。

**goal 契约（已核实源码）**：flight_type=1 订**绝对名 `/move_base_simple/goal`**
（ego_replan_fsm.cpp:64，rviz 惯例）——launch remap 对绝对名有效，
1 行接到 `/drone_i/planning/goal`。新 goal 到达时 FSM 从当前位置全局重规划
（waypointCallback → planNextWaypoint，EXEC_TRAJ 态转 REPLAN）——**多腿衔接天然支持**。

## 三、Gazebo D435 深度相机（world_builder.py 注入）

`drone_model(i,x,y)` 的 base_link 内加 `<sensor type="depth">` + libgazebo_ros_camera.so（/opt/ros/noetic/lib 已确认存在）：

- **位姿**：机体系原点平视（`0 0 0 0 0 0`）。自遮挡免疫零补丁的依据：own visuals 最大前伸 = 机臂尖 0.12+桨 0.10 = **0.22m < near clip 0.30**；且 cam2body_ 平移零不破（grid_map.cpp:95 硬编码光学→机体旋转，我们不改）。
- **插件选型（已核本机 gazebo_plugins）**：用**新式 `libgazebo_ros_camera.so`**（XTDrone 同款 depth 模式参数；老式 libgazebo_ros_depth_camera.so 参数表不同弃用）。`<depth_camera><output>depths</output>` 关点云省 CPU。SDF 块：
  ```xml
  <sensor name="d435_depth" type="depth">
    <pose>0 0 0 0 0 0</pose><update_rate>15.0</update_rate>
    <always_on>true</always_on><visualize>false</visualize>
    <camera>
      <horizontal_fov>1.5184</horizontal_fov>
      <image><width>424</width><height>240</height><format>R8G8B8</format></image>
      <clip><near>0.30</near><far>10.0</far></clip>
      <depth_camera><output>depths</output></depth_camera>
    </camera>
    <plugin name="d435_depth_ros" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn><updateRate>15.0</updateRate>
      <cameraName>drone_%d/depth</cameraName>
      <imageTopicName>image_color</imageTopicName>          <!-- RGB 弃用不订 -->
      <cameraInfoTopicName>rgb_camera_info</cameraInfoTopicName>
      <depthImageTopicName>image_raw</depthImageTopicName>  <!-- 32FC1 米制 -->
      <depthImageCameraInfoTopicName>camera_info</depthImageCameraInfoTopicName>
      <pointCloudTopicName>points</pointCloudTopicName>
      <frameName>drone_%d/depth_optical</frameName>
      <pointCloudCutoff>0.30</pointCloudCutoff><pointCloudCutoffMax>10.0</pointCloudCutoffMax>
    </plugin>
  </sensor>
  ```
- **分辨率/FOV**：424×240，hfov 1.5184rad(87°) → 内参 **fx=fy=223.4, cx=212, cy=120**（launch 参数喂 grid_map，不订 camera_info）。
- **速率**：update_rate 15Hz，clip 0.30–10.0m。
- **话题契约对齐旧 depth_sim**（消费者零改动）：`cameraName=drone_<i>/depth` + `depthImageTopicName=image_raw` → `/drone_<i>/depth/image_raw`；`depthImageCameraInfoTopicName=camera_info`。RGB 图发 `image_color`（弃用不订）。
- **编码**：libgazebo_ros_camera depth 模式输出 **32FC1 米制** —— grid_map.cpp:994-997 原生 `convertTo(CV_16UC1, k_depth_scaling_factor=1000)` → 与 fd250 sim 链逐字节同语义，**plan_env 零改动**。
- **同步**：grid_map.h:220 是 **ApproximateTime**(queue 100) —— 15Hz depth × 20Hz odom 天然同步。
- **门控**：新 sim_settings 键 `sensor.depth.mode: gazebo_plugin | truth_render`（默认 gazebo_plugin；回退真值渲染的逃生门）。
- **安全阀**：grid_map `depth_filter_mindist 0.2`（<0.2 丢弃）保留原值；min_ray_length 0.1 / max_ray_length 4.5 不动。

## 四、新包 ego_zx

- `launch/ego.launch`：per-drone include（drone_id arg），include vendored `advanced_param.xml` 的 zx 版副本：
  - remap：`~odom_world`/`~grid_map/odom` → `/drone_<i>/odom`；`~grid_map/depth` → `/drone_<i>/depth/image_raw`；`~planning/bspline` → `/drone_<i>/planning/bspline`（arena 斜杠命名，非 fd250 下划线命名）
  - 参数：内参 223.4/223.4/212/120；map_size 70/70/14；max_vel 2.0/max_acc 6.0（对齐 competition_rules）；planning_horizon 7.5；flight_type=1（goal 话题触发）；`virtual_ceil_height` **4.4**（fd250 默认 2.9 会把 cruise_z 2.4 以上当天花板；对齐 flight_z_ceiling 4.5）
  - traj_server：`position_cmd` → `/drone_<i>/planning/pos_cmd`（相对名，配补丁 1）
- `scripts/ego_bridge_node.py`：PositionCommand→Twist + 握手镜像（defop_cpn_node.py:1322-1360 语义）：
  - `/planning/cpn_ready`（首次收到 pos_cmd 后 latch True）
  - `/planning/cpn_warm`（odom+depth 各见 ≥1 帧 latch True——语义复用，ego 档无人消费也不碍事）
  - `/defop/debug` 心跳 String JSON（复用现有观测面）
  - 发 `/planning/cpn_vel_raw`（nav 中继混合 sep 后统一组装 vel_cmd）
  - **FSM**：WAITING（无 bspline/cmd）→ TRACKING（发速度）；0.5s 无 cmd → 停发（drone_vel_plugin 断链归零悬停兜底，traj_server 轨迹结束悬停段本就停发 cmd，:200 return——语义天然闭环）
  - **坐标系钩子**：输入假设 cmd=世界 ENU（sim 恒等）。真机换源：goal 入口做 ENU→LIO 局部系逆变换（PoseTF 逆），traj_server 输出局部系 PositionCommand 直连 `/setpoints_cmd`——同码换源。

## 五、nav 改动（ego_ws 副本，单文件 6 处判据，全部同一语义扩列）

`nav_node.py`（只读实锚，行号以 defop_ws 版为准）：

1. `:179 vel_source_of`：`return src if src in ("defop","ego") else "internal"`（保原值区分，不折叠成布尔）
2. `:660 _pen_forever`：`(_vel_source in ("defop","ego") and pen_mode_of(settings)=="cpn_forever")` —— 中继订阅门/stall 冻结/departure 旁路全由此旗带活
3. `:827` LANDED 补 False 旗判据 → `in ("defop","ego")`
4. `:2636` _cpn_gate 每拍刷新判据 → `in ("defop","ego")`
5. `:2678` 发布门（ready 握手 + PEN_STALL 冻结 + SHADOW 心跳）判据 → `in ("defop","ego")`，内部逻辑零改动
6. `:2839 cpn_active_of` 首参 `self._vel_source in ("defop","ego")`

**executor 侧不动**：mission_executor_node.py:350 `_pen_forever=(source=='defop' and ...)` 对 ego 判 False ⇒ 起飞暖机闸门不启用（ego 无模型秒就绪，闸门依赖不存在）；executor 全文件仅此一处 vel_source 判据。

**ego 档 sim_settings**：`planner.source: ego` + `planner.pen_authority: cpn_forever`（复用中继语义，不含起飞闸门——闸门在 executor 侧，已随上面判 False）。

**launch 拓扑**：zx2026_all.launch → stack_nodes.launch（:139-159 `defop_arm` arg 自门控，×6 include defop.launch）→ ego_ws 版同位加 `ego_arm` arg ×6 include ego_zx/launch/ego.launch（镜像同款约定：缺省 false、跑批器显式开）。

## 六、坐标系/内参总账

| 项 | 值 | 出处 |
|---|---|---|
| odom | world ENU（drone_vel_plugin 发，frame world） | 现状不动 |
| goal | world ENU PoseStamped | mission_executor 现状 |
| depth 帧 | drone_<i>/depth_optical（仅语义标注，无 TF 提供者——与 depth_sim 相同，消费者不查 TF） | 同 depth_sim 契约 |
| 相机安装 | 机体系原点平视 | cam2body_ 平移零成立 |
| 内参 | fx=fy=134.9 cx=128 cy=72 | hfov 87° 256×144 方像素（Phase 0.5 降档） |

## 七、分阶段验证

- **Phase 0**：脚手架（defop_ws 快照 + git init baseline commit）→ 设计文档 → vendor → catkin 编译绿（defop_nav CATKIN_IGNORE 留源不编）。
- **Phase 0.5（硬验收门）✅ 2026-10-05 收官**：单机 D435 渲染 smoke 定谳三病根两修复——
  ①**病根一（插件语义）**：`libgazebo_ros_camera.so` 不读 `depthImage*` 参数（strings 全空，
  depth 参数只在 depth_camera/kinect 两 .so）⇒ 深度话题从未创建，v1-v6 全程零帧；
  修复=换 `libgazebo_ros_openni_kinect.so`（FUEL/ego 官方 sim 同款），换后深度流立活
  （32FC1、frame drone_<i>/depth_optical、话题族齐全）。②**病根二（分辨率）**：424×240
  全栈 2.3Hz < 5Hz 门 → 降 256×144（fx=fy=134.9 cx=128 cy=72 同帧装订）→ 仅 2.6Hz
  （像素 -64% 只 +15% ⇒ 非逐像素瓶颈）。③**病根二真身（v9 隔离定谳）**：60+ 节点全栈
  8 核 CPU 争抢——裸 roscore+gzserver **15.1Hz** 跑满 update_rate、RTF≈1.19、零错误。
  **验收口径（证据定案）**：裸机能力门 ≥5Hz（实测 15.1Hz ✅）；全栈集成率 2.6Hz ≥
  defop 链实际输入率 1.51Hz×1.7 ✅（同构栈当年 1.51Hz 飞完 85/80/90 分）。
  全栈率后备杠杆：栈节点 renice +10、camera_sim/tag_detector 节流（Phase 1/2 按需启用）。
- **Phase 1**：单机 sim smoke——`/zx2026/start` 服务 → rostopic pub 一个 goal → 到达 + nav arrived 闩锁 + 零碰撞 + 判分链跑通。
- **Phase 2**：六机 E2E 全任务（run_verify.sh 同款口径，150s）。
- **Phase 3**：vs defop 基线 A/B（defop 近三批 score 85/80/90、col 8–15 为对照；同 seed 43、≥3 run，禁单判）。

## 八、风险预案

| 风险 | 证据 | 缓解 |
|---|---|---|
| WSL 无头 gzserver 渲染不出深度 | **已排除**（Phase 0.5：裸 15.1Hz/RTF 1.19） | — |
| llvmpipe 帧率不足 | **真身=全栈 CPU 争抢**（裸 15.1 vs 全栈 2.6Hz） | 256×144 已落地；后备：栈节点 renice、感知节点节流 |
| 32FC1 编码不匹配 | 已排除：grid_map 原生转换 | — |
| depth/odom 不同步 | 已排除：ApproximateTime | — |
| 多机 /position_cmd 互踩 | traj_server.cpp:241 绝对名 | vendored 补丁 1（相对名） |
| goal 单发 latch vs ego FSM 触发 | mission 是单次发布 | ego flight_type=1 订 goal 话题即可触发；首次 goal 到达后 FSM 进 WAIT_GOAL 再等下一条 |
| ego 输出速度 2.0 vs 判分口径 | competition_rules max_vel 2.0 匹配 | 无需限幅；若 A/B 出超速证据再夹 |
| 轨迹结束悬停段停发 cmd | traj_server :200 return | 插件 0.5s 断链归零悬停兜底；桥 0.5s 无 cmd 停发 |
| defop_ws 未 commit 变更堆叠 | c1/c2 机制源默认关 | 快照携带（baseline commit 说明） |

## 九、与 defop 路线的关系

- defop_ws **原样冻结**：c1/c2 机制源已落地默认关未 commit，等用户指令；ego 路线不回写 defop_ws 任何东西。
- 两工作区共享场地口径（杨树世界 seed 42 / cruise_z 2.4 / venue 70×70×14 / drone_radius 0.35）→ A/B 同世界同任务同判分，唯一变量=规划器。
- ego 路线的战略位：DeFoP 是「学习型端到端速度基元」（撞树 8–15/局 未解），ego 是「显式优化型 B 样条」（学术主流、真机链成熟 fd250 已实飞验证）。若 ego 在同世界同判分下 col 显著低且 landed 不劣，则换主路线；否则 defop 线继续 c2 pilot。
