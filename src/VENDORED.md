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

### 补丁 1：`plan_manage/src/traj_server.cpp:241`

```cpp
// 原：ros::Publisher pos_cmd_pub = nh.advertise<...>("/position_cmd", 50);
pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);
```

**理由**：原版绝对名 `/position_cmd` 对 launch `<remap from="position_cmd">` **不生效**
（remap 只匹配相对名解析）。母本单机演示无碍；本 ws 六机同机 → 全发同一全局话题互踩。
改相对名后 ego.launch remap 生效到 `/drone_<i>/planning/pos_cmd`。

### 补丁 2：`plan_manage/src/ego_replan_fsm.cpp:220`（waypointCallback）

```cpp
// 原：Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
```

**理由**：原版 goal z 写死 1.0（rviz 2D Nav Goal 惯例）。mission 的分腿目标
z 携带任务语义（RECON identify_z 1.2 / cruise 2.4 / 投放 0.8 / 平台 0.3），
压平 = 全任务失效。z 安全网由 FSM `virtual_ceil_height 4.4` + 判分 OVER_HEIGHT 兜底。
`:213` 的 `z < -0.1` 忽略守卫保留（mission goal z 恒 >0，不碍）。

## launch 层适配（非代码补丁，不算入上表）

- `ego_zx/config/advanced_param_zx.xml`：advanced_param.xml 的 arena 适配版
  （话题命名斜杠化 / goal remap / 内参 424×240@87° / virtual_ceil 4.4）——差异逐条
  注释在文件头。
- `ego_zx/launch/ego.launch`：per-drone 组装（母本 single_run_in_sim.launch 的
  对应物，去掉 map_generator/simulator/rviz——场地由 arena_world_gazebo 提供）。

## 参数继承备忘

`k_depth_scaling_factor=1000.0` **保持母本值**：Gazebo depth 插件输出 32FC1 米制，
`grid_map.cpp:994-997` 原生 `convertTo(CV_16UC1, 1000)` 转毫米——与母本 sim 链
语义逐字节一致，不是"毫米输错"。
