# VENDORED — zx2026_ego_ws 第三方源码账

## 来源

- **母本**：`~/fd250_ws/Fast-Drone-250`（ZJU-FAST-Lab Fast-Drone-250 全栈）
- **快照 commit**：`8ff6427`（2026-09-26 前后 27/27 编译绿 + 09-28 sim 闭环实飞验证）
- **拷贝日期**：2026-10-05，由 ego 路线立项拷入

## vendor 包清单（fd250 src → 本 ws src/）

| 包 | 内容 | 状态 |
|---|---|---|
| plan_manage | ego_planner_node + traj_server + launch 模板 | **2 处补丁**（下） |
| plan_env | GridMap（深度→占据栅格，无 ESDF） | 无改动 |
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

## launch 层适配（非代码补丁，不算入上表）

- `ego_zx/config/advanced_param_zx.xml`：advanced_param.xml 的 arena 适配版
  （话题命名斜杠化 / goal remap / 内参 256×144@87° fx=fy=134.9 / virtual_ceil 4.4）
  ——差异逐条注释在文件头。
- `ego_zx/launch/ego.launch`：per-drone 组装（母本 single_run_in_sim.launch 的
  对应物，去掉 map_generator/simulator/rviz——场地由 arena_world_gazebo 提供）。
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
