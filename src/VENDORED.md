# VENDORED — zx2026_ego_ws 第三方源码账

## 来源

- **母本**：`~/fd250_ws/Fast-Drone-250`（ZJU-FAST-Lab Fast-Drone-250 全栈）
- **快照 commit**：`8ff6427`（2026-09-26 前后 27/27 编译绿 + 09-28 sim 闭环实飞验证）
- **拷贝日期**：2026-10-05，由 ego 路线立项拷入

## vendor 包清单（fd250 src → 本 ws src/）

| 包 | 内容 | 状态 |
|---|---|---|
| plan_manage | ego_planner_node + traj_server + launch 模板 | **2 处补丁**（下） |
| plan_env | GridMap（深度→占据栅格，无 ESDF） | **2 处补丁**（下，2026-10-06 深度链修正） |
| path_searching | A*/kinodynamic 前端 | 无改动 |
| bspline_opt | B 样条梯度优化后端 | 无改动 |
| traj_utils | Bspline 消息/多机共享 | 无改动 |
| utils/quadrotor_msgs | PositionCommand 等消息 | 无改动 |
| utils/uav_utils | 工具依赖 | 无改动 |

**跳过**：drone_detect、rosmsg_tcp_bridge、uav_simulator/local_sensing（真值渲染仿真器——
本路线深度源=Gazebo D435 插件，见 docs/EGO_DESIGN.md §三）。

## vendored 补丁（唯一允许的改动，全部在此登记）

### 补丁 1：`plan_manage/src/traj_server.cpp:241`（2026-10-05 法证修正）

```cpp
// 原：ros::Publisher pos_cmd_pub = nh.advertise<...>("/position_cmd", 50);
pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);
```

**原记载有误，现更正**：原论断「绝对名 `/position_cmd` 对 `<remap from="position_cmd">`
不生效」是错的——母本绝对名+母本 remap 本来匹配（from 相对名按 namespace 解析为
`/position_cmd`，与代码绝对名相等）。真实规则：**roslaunch 的 remap from 相对名解析到
namespace 层（不带节点私有化）；私有 NodeHandle 下的相对名解析到节点私有名**——补丁 1
把代码改成相对名后，advertise 解析为 `/drone_<i>_traj_server/position_cmd`，与
from="position_cmd"（→`/position_cmd`）永不相等 → remap 失效，**六机 PUB 全落私有
死话题**（Phase 1 master.log 定谳：6 条 `+PUB [/drone_<i>_traj_server/position_cmd]`，
`/drone_<i>/planning/pos_cmd` 仅 bridge 的 SUB，PUB 为零；traj_server stdout 0 字节
是 SIGTERM 吃缓冲伪象，进程全程健在）。

**修复（launch-only，零代码改动零重编译）**：`ego.launch` remap from 加 `~`——
`from="~position_cmd"` 解析为节点私有名，与 advertise 相对名的私有解析相等，同
`~planning/bspline` 订阅的已验证生效模式。代码相对名保持（与 bspline 订阅同款
「私有 nh 相对名 + ~remap」模式，语义一致）。

### 补丁 2：`plan_manage/src/ego_replan_fsm.cpp:220`（waypointCallback）

```cpp
// 原：Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
```

**理由**：原版 goal z 写死 1.0（rviz 2D Nav Goal 惯例）。mission 的分腿目标
z 携带任务语义（RECON identify_z 1.2 / cruise 2.4 / 投放 0.8 / 平台 0.3），
压平 = 全任务失效。z 安全网由 FSM `virtual_ceil_height 4.4` + 判分 OVER_HEIGHT 兜底。
`:213` 的 `z < -0.1` 忽略守卫保留（mission goal z 恒 >0，不碍）。

### 补丁 3：`plan_manage/src/traj_server.cpp`（calculate_yaw，2026-10-05 run4 bag 法证）

**根因**：`use_sim_time` 下 Gazebo RTF<1 时 `/clock` 会重复发布同值，ros Timer
随之连续两拍 `time_now==time_last`（dt=0）；恰逢 yaw 收敛段（`yaw_temp==last_yaw_`，
低速直线/悬停段常态）时 `(yaw_temp-last_yaw_)/dt = 0/0 = NaN`。NaN 经 LPF
（`0.5*last_yaw_dot_+0.5*yawdot`）**永久污染** `last_yaw_dot_`，此后每帧
`cmd.yaw_dot=NaN` → ego_bridge 直通 `tw.angular.z` → drone_vel_plugin
`SetAngularVel(NaN)` → **ODE 一步内全身 NaN**（WorldPose 有保护返零、
WorldLinearVel 透传 NaN）→ odom `pos=(0,0,0) vel=(nan,nan,nan)` →
executor 位置恒零误判 hover-lock → `RETIRED (UNAUTHORIZED_LANDING)`。

**法证链**（run4 bag 逐帧）：85.816s 位置正常 (24.6,-1.7,2.47) 悬停 →
85.858s vel_cmd 首帧 `ang.z=NaN`（linear 正常）→ 85.870s 物理体
pose=0/vel=NaN，**66ms 毒杀**。pos_cmd 末三帧 `yaw=0.819 恒定`而
`yaw_dot=NaN`——0/0 签名。run3/run4 共七机同签名退赛（drone_0/1/2/3/4×2 局），
各机独立死于各自轨迹的直线/悬停段（非机间传染）。fd250 母本同款除零但
其 uav_simulator 的 /clock 按固定步长发布不重复同值，竞态窗口未踩中；
arena Gazebo RTF≈0.4-0.6（WSL 软渲染）必然周期性触发。

**修复**（calculate_yaw 内三处）：
1. 函数开头 dt 防护：`dt<=1e-6`（同拍重复）保持上一拍输出、不推进状态；
2. 三处除法与 `max_yaw_change` 复用防过的 `dt`；
3. LPF 后输出护栏（`yaw != yaw` 自比较截断 NaN），防未来其它 NaN 路径出闸。

伴生护栏：`ego_bridge_node.py` `_tick` 对 pos_cmd 四关键字段
（velocity.xyz + yaw_dot）做 `math.isfinite` 检查，非有限丢帧（插件 0.5s
断链保护自然接管悬停），对其它 NaN 源兜底。

**已知残留**（run3 记录，run4 复现框架下未重现）：三架 planner
`exit -11` 段错误（sim 160-174 碰撞风暴期）；drone_3 OVER_HEIGHT
z=5.4 超 flight_z_ceiling——ego 当笔后垂直方向仅 `virtual_ceil_height 4.4`
规划层软约束，无执行层硬限幅。两者独立于本案，待后续立案。

### 补丁 4：`plan_env/src/grid_map.cpp:depthToPointCloud`（off-by-one，2026-10-06）

**母本自带缺陷**（`diff` 对 `~/fd250_ws/Fast-Drone-250` 的 grid_map.cpp 为空 ⇒ 非本
ws 引入）。原码：

```cpp
depth = (*row_ptr) * inv_factor;
row_ptr = row_ptr + mp_.skip_pixel_;   // 先前移
if (*row_ptr == 0) { depth = max_ray_length_ + 0.1; }   // 再用前移后的指针判零
```

`depth` 取自当前像素，判零却读**右邻像素** ⇒ 「当前像素有效、右邻为 0」时把
**真障碍点当空点丢弃**。本 ws 深度图 52% 像素为 0/NaN（插件无回波处写 0）⇒
约半数有效点被吞。修复=先取值判零、再前移（`raw_depth` 临时量）。

**为何要修**：树干在栅格图上本就只有 2-3 格宽（干径 0.15 m / 分辨率 0.1 m），
再被吞掉约一半采样点，地图上的树干会出现空洞 ⇒ 前端 A* 可从"洞"里穿枝。

### 补丁 5：`plan_env/src/grid_map.cpp:depthToPointCloud`（地面滤，2026-10-06）

**新增参数** `grid_map/ground_filter_margin`（米，相对 `ground_height_`），
默认 **0.0 = 关 = 逐字节母本行为**。>0 时丢弃 `pt_world(2) < ground_height_ + margin`
的回波。

**为何要**：D435 水平前视且装在机体原点，巡航高度 2.4 m 时地面在 4.3 m 处进视场
下沿（`z/tan(hfov_v/2)`, hfov_v=±28.1°）。实测地面占 occ 图 **61%**
（z 直方图 12,758/20,983 落在 z∈[-0.2,0.2)）。地面本身不碍避障，**但被膨胀后抬起
一张"地板"**：`inf_step = ceil(inflation/resolution)`，膨胀半径一旦 ≥ 机身半径
（0.35 m ⇒ 4 格 = 0.4 m）后，**起飞/低飞段的机心就落进膨胀地面内部** ⇒
`getInflateOccupancy(机心) != 0` ⇒ `bspline_optimizer.cpp:1243`
"the drone is in obstacle. This should not happen." ⇒ 规划器**全程拒停**（自囚）。
F1/H1 档（inflation 0.45）实测 33k+ 次该报错、位置冻结不动。
守卫=`ego_fleet_monitor.py` 的 `dmin_inf < 膨胀半径` 判据。



## launch 层适配（非代码补丁，不算入上表）

- `ego_zx/config/advanced_param_zx.xml`：advanced_param.xml 的 arena 适配版
  （话题命名斜杠化 / goal remap / 内参 256×144@87° fx=fy=134.9 / virtual_ceil 4.4）
  ——差异逐条注释在文件头。
- `ego_zx/launch/ego.launch`：per-drone 组装（母本 single_run_in_sim.launch 的
  对应物，去掉 map_generator/simulator/rviz——场地由 arena_world_gazebo 提供）。
- **`grid_map/pose_type` 1 → 2（2026-10-06 定谳修：ego 盲飞根因）**。原值 1 配注释
  「1=odom」，但 `plan_env/include/plan_env/grid_map.h:147` 声明
  `enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 }` ——**注释把枚举值
  记反了**。值 1 使 `grid_map.cpp:106-113` 走 POSE_STAMPED 分支，同步器把深度与
  `~grid_map/pose`（remap 到 `/drone_<i>/planning/camera_pose`，**全仓无发布者**）
  配对 ⇒ `depthPoseCallback` 永不触发 ⇒ line 719/1000 的 `occ_need_update_` 永不为真
  ⇒ **栅格图恒空、ego 全程盲飞**（所有"擦边撞树"现象的总根因）。
  上游同源对照可证：`advanced_param.xml:123`=1（该档 `~grid_map/pose` 接
  VINS/真值 pose）、`advanced_param_exp.xml:122`=2（深度实飞档）——本 ws 属深度档，
  应为 2。**注意不要混入「补丁」表**：这是本 ws 自己的配置，非 vendored 代码改动。
  全程法证与验收见 `docs/ego_blind_flight_case.md`。
- **感知修正臂注入点（2026-10-06）**：新增两个 arg，从 `gazebo.launch` →
  `stack_nodes.launch` → `ego.launch` → `advanced_param_zx.xml` 贯穿（与既有
  `inflation` 同链）：
  - `ground_filter`（默认 `0.0` = 关）→ `grid_map/ground_filter_margin`（补丁 5）
  - `ray_length`（默认 `4.5` = 母本）→ `grid_map/max_ray_length`，并以 `$(eval
    float(arg('ray_length')) + 0.5)` / `+1.0` **同步派生** `depth_filter_maxdist`
    与 `local_update_range_x/y`——母本三件套（4.5 / 5.0 / 5.5）必须同动，任缺其一
    视距都不变（射线裁剪 `:380-384` 与半径外复位 `:488-493` 各自独立设限）。
  - 已用 `roslaunch --dump-params` 双向验收：默认档 = 4.5/5.0/5.5/0.099/0.0
    （逐项等于旧口径）；`ray_length:=8.0` → 8.0/8.5/9.0 ✓。
- **`depth_mindist`（默认 `0.45`，**非**母本 0.2）→ `grid_map/depth_filter_mindist`
  （2026-10-06 自桨幻影定谳修）**。母本值是 0.2，注释还写着「与插件 near clip 0.30
  兼容」——**这正是病根**：near clip 只裁掉比它**近**的几何，比它**远**的照渲染。
  本机型桨叶是 `r=0.10` 的圆盘、中心在 `(±0.24, ±0.24, +0.062)`
  （`world_builder.py:356-364`）⇒ 外缘伸到 **0.44 m**，其中落在 87° 半视场内的
  内侧象限正好在 **[0.30, 0.40] m**——**无人机把自己的桨画成了贴身障碍格**。
  `drone_depth_sensor()` 的注释「最大前伸=机臂尖 0.12+桨 0.10=0.22 < near clip
  ⇒ 自遮挡免疫」**两处错**：把机臂**半长** 0.12 当成臂尖（实际桨心 0.24），且误以为
  「近于 near clip ⇒ 不可见」。实测（无任务探针，`~/ego_depth_frame_dump.py`）：
  每帧 **132 px @[0.25,0.32) + 54 px @[0.32,0.40)**，drone_0 与 drone_3 **逐字相同**
  ⇒ 体固定签名（非场景内容）；投影成体素后是 **3 个占据格 @ 距机心 0.303 m、
  z=1.04（机心 z=1.00）**，两机同样逐字相同（`~/ego_occ_near.py` 实测 0.303/0.303）。
  **后果链**：自格距机心 0.303 m ⇒ 膨胀半径 ≥ 0.3 时膨胀体吞掉机心本身 ⇒
  `getInflateOccupancy(机心)!=0` ⇒ `bspline_optimizer.cpp:1243` 全程拒停（自囚）；
  膨胀 0.099 时最近膨胀格 = 0.303−0.1 = **0.203 m**（实测 0.204 ✓），即**机头两侧
  常驻两团幻影障碍**——这才是 H0 那个「`dmin_inf` 恒 0.13–0.26」的真身（原判
  「贴着膨胀边界飞」作废）。
  **修复**：`depth_mindist = 0.45`（自格物理上限 0.40 之上）。修后同探针实测
  `min|occ| = 1.201 m`、1 m 内 0 格（最近的是真实场景物）。**纯配置，零代码改动**。
  语义与真机一致：真 D435 近距有盲锥（插件自己也声明 `pointCloudCutoff 0.30`），
  且实机相机装在机头，桨叶根本不在深度视场内。
  代价：0.45 m 内不设障——但碰撞判据半径就是 0.35 m，`[0.35,0.45]` 只剩 0.1 m
  预警带，而实测深度帧率仅 2.8–3.5 Hz（2 m/s 下一帧走 0.6–0.7 m），该带无信息量。
- **接线复核（2026-10-05 Phase 0.5）**：`ego_planner_node.cpp:12` 与
  `traj_server.cpp:237` 的 main 均为 `NodeHandle nh("~")` 私有句柄——代码里的
  相对名（`planning/bspline`、`odom_world`）实际解析为节点私有名，与
  `~planning/bspline` / `~odom_world` 形式的 remap **精确匹配**，无 patch-1
  同类病；goal 侧 `"/move_base_simple/goal"` 是绝对名，remap 用同形绝对名
  （`advanced_param_zx.xml:37`）匹配。mission_executor_node.py:330 发
  `/drone_<i>/planning/goal`（goal 单一发布者）。

## 参数继承备忘

`k_depth_scaling_factor=1000.0` **保持母本值**：Gazebo depth 插件输出 32FC1 米制，
`grid_map.cpp:994-997` 原生 `convertTo(CV_16UC1, 1000)` 转毫米——与母本 sim 链
语义逐字节一致，不是"毫米输错"。
