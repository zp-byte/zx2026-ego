# P3.5 参照域校验 harness 设计（lmf_sim/rmf_sim 森林世界，双链对质）

状态：设计稿 **v2**（2026-09-30）。v1 经双对抗校验，**9 个 blocker 已全部落稿**（行内以 ⚑B1…B7 标注，另吸收关键 minor）；v1 由纯源码侦察产出。
背景：DeFoP 复现（arXiv 2512.17553）。P3 A/B 已定谳 vendored 链（defop_nav）在 zx2026 arena 输给场地调优基线（57.8 vs 78.9，95% 差=落地完成度 1.0 vs 2.0 m/s）。P3.5 把两套链放回论文自己的 sim 域（上游 rmf_sim 森林世界），对质 **"移植 bug vs 域差"**。

路径约定（下文引用简写）：
- `UP` = `~/DeFoP`（上游原仓，权重/评测原链所在地）
- `AR` = `~/zx2026_arena_ws`（arena 工作区；vendored 链 `AR/src/defop_nav`，本 harness 一切文件 `AR/tools/p35_refdom/`）

---

## 0. 一句话架构

同一个 Gazebo 世界（forest2）+ 同一架 delta 无人机，分别由上游原链（臂 U）和 vendored defop_nav 链（臂 V）驾驶；两臂之间不做任何端到端轨迹硬比，而是用三层对质（帧级 latent → 决策级 CPN → episode 级行为），阈值由**分层校准**预先测定（预处理像素差/resize 取整/跨实现前向差的 P99.9 + episode 层 U×5 非参带，⚑B6）。臂 C 独立做深度口径交叉核（Gazebo 真深度 vs P1 渲染器统计）。

```
                    ┌──────────────────────────────────────────────┐
                    │  roscore :11312（P3.5 专用 master）           │
                    │  use_sim_time=true（gazebo 起来后）           │
                    └──────────────────────────────────────────────┘
 臂 U：rmf_sim_sevae.launch(world_file:=forest2.world)
        + vae_node.py --sim=True          → /latent_space
        + evaluate.py（TF fallback）      → /delta/cmd_velocity
        （上游绝对话题，无 ns，单实例假设）

 臂 V：rmf_sim_sevae.launch(world_file:=forest2.world)   ← 同世界同机
        + shims 五桥：
          odom_bridge   /delta/odometry_sensor1/odometry → /drone_0/odom
                        （⚑B4 twist 系别手术，见 §3#2）
          depth_bridge  /delta/rgbd/camera_depth/depth(32FC1 640x360)
                        → nearest resize 480x270 值透传 → /drone_0/depth/image_raw
          vel_bridge    /drone_0/vel_cmd(世界系) → Rz(-yaw) → /delta/cmd_velocity(机体系)
                        （⚑B7 三态：悬停 / 透传+重发末条 / 急停）
          goal_bridge   waypoint(60,0,1) → /drone_0/planning/goal（episode_start 触发，一次性）
          cpn_flag      /drone_0/planning/cpn_active = True @5Hz（sim 时）
          episode_mgr   set_model_state 重摆 (-2,0,0.5)（⚑B3）+ 100s sim 时窗 + 判分落盘
        + roslaunch defop_nav defop.launch drone_id:=0   ← vendored 原样启动

 臂 C：任一臂运行期间旁路录 /delta/rgbd/camera_depth/depth + camera_info
        → 离线统计（哨兵率/值分布/fx fy）与 P1 渲染器口径对质
```

---

## 1. 三臂定义

### 1.1 臂 U —— 上游原栈原 config

| 项 | 内容 | 证据 |
|---|---|---|
| 启动 | `roslaunch rmf_sim rmf_sim_sevae.launch world_file:=$(find rmf_sim)/worlds/forest2.world` | launch_notes：corridor 默认，forest 域换 world_file（UP `src/simulate/lmf_gazebo_models/rmf_sim/worlds/`，forest2=9803 行=200 tree_N include + 16 棵内联云杉） |
| VAE 节点 | `cd UP/src/planning/sevae/sevae/inference/src && python vae_node.py --sim=True`（解释器 `~/defop_env/bin/python`） | recon topics：vae_interface 订 `/delta/rgbd/camera_depth/depth`（vae_node.py:33-48 sim 分支硬编码）、发 `/latent_space`（vae_node.py:31,82,145-147，queue=2） |
| 评测 | `cd UP/src/planning/ORACLE && ~/defop_env/bin/python evaluate/evaluate.py`（**cwd 必须是包根**，config/common_flags 靠 `sys.path.append('.')`，evaluate.py:4-6,1656-1661；**解释器必须 defop_env**——TF2.4.1+h5py2.10，系统 python3 无 TF） | recon gotchas：cwd 敏感条 |
| 发车 | 等 `START planner` 打印后 `rosservice call /start_planner "{}"`（rotors_wrappers.py:100 → evaluate.py:264-267 置 run_planner=True） | README.md:198-201 |
| config | **主 config.py 为基准**（不是 corridor 配置），经 runner"拷贝-还原"装订成 `UP/src/planning/ORACLE/config.py`，**改动白名单六键见下** | 见下 |
| 权重 | ⚑B1 **主 config 默认 TF 路径指向 net1/net2/net3——与 vendored 的 nett5,5,6 不是同一批权重（h5py 逐张量对质：80309 个元素全不同）**，必须白名单强制覆盖：`COLLISION_USE_TENSORRT=False`（config.py:85）+ `seVAE_CPN_TF_CHECKPOINT_PATH=[nett5,nett5,nett6]/saved-model.hdf5`——与 vendored `defop_nav.yaml cpn_weight_paths` 逐字同一批文件，天然同权对质 | network_inference_tensorrt.py:223-236（TRT engine 与 TRT 版本/GPU 绑定）；vendored params cpn_weight_paths |
| VAE 权重 | `seVAE_evo_LD_128.pth`（VAENetworkInterface.py:14），与 vendored yaml `vae_weights` 同一文件 | recon 双方 topics/params |

主 config（非 corridor）选型理由：CMD_VELOCITY=1.0（config.py:75，action 里 vel_x 唯一取值 np.repeat）= vendored yaml `cmd_velocity: 1.0`；WAYPOINT_DISTANCE_THRESHOLD=3.0 / YAW 45°（config.py:109-112）= vendored `waypoint_distance_threshold: 3.0 / waypoint_yaw_threshold_deg: 45.0`；DEADEND 6.5/5.0、COLLISION 0.35（config.py:103-106）= vendored `deadend_high/low: 6.5/5.0、collision_threshold: 0.35`；N_E=3（config.py:115）+ nett 列表前 3 项 nett5,nett5,nett6（config.py:48-52）= vendored `n_ensemble: 3` + 同名 hdf5。**corridor 配置（N_E=5/net1..net5/阈值 3.5/1.5/0.15/5.0m）与 vendored 全不对齐，禁用**（recon gotchas：两套配置 DEADEND/THRESHOLD 数值不同，配置即结果的一部分，不可混搭）。

**runner 拷贝白名单（六键全强制，拷贝后 shakedown 逐键 grep 复验，缺一即停）**：
1. `ROBOT_DEPTH_TOPIC='/delta/rgbd/camera_depth/depth'`——否则 `evaluate.py:1061 depth=self.depth_image.copy()` 无条件执行、sim 无人发 `/d455/...` → 首个 tick AttributeError 崩（recon gotchas P0 必修坑；sim 分支 callback evaluate.py:329-344 passthrough+NaN→MAX_RANGE）
2. `COLLISION_USE_TENSORRT=False`（config.py:85；本机无匹配 TRT，TF fallback）
3. `seVAE_CPN_TF_CHECKPOINT_PATH` → `[nett5,nett5,nett6]/saved-model.hdf5`（⚑B1：主 config 默认指 net1/2/3，是另一批权重）
4. `EVALUATE_MODE=True`（⚑B2：否则 latent 评估链路**静默失效**、EPISODE_TIMEOUT 退化 30s）
5. `MIN_INITIAL_X=MAX_INITIAL_X=-2`（⚑B3：原生 spawn X∈[-13,0] 均匀未播种、Z=0.5，config.py:229-236；钉死两臂同点 (-2,0,0.5)）
6. `NUM_EPISODES_EVALUATE=5`（一次 launch 连跑 5 局；episode 边界由 harness 监视器按位姿突跳/超时判定）

**VAE 无 GPU 适配**：vae_node 硬编码 cuda:0（vae_node.py 加载处）；本机无 GPU 时 runner 把 `vae_node.py` 拷到 `p35_refdom/configs/vae_node_p35.py` 改 `cuda:0→cpu` 再以同 cwd 运行（原文件不动；sevae 已 pip install -e，包路径定位 VAENetworkInterface.py:9-14 不受拷贝影响）。

### 1.2 臂 V —— 同世界同机喂 vendored 链

同一条 `rmf_sim_sevae.launch world_file:=forest2.world`（Gazebo/delta/odom/深度/接触传感器全部复用），**不启动** evaluate.py / vae_node，改起：

1. `roslaunch defop_nav defop.launch drone_id:=0`（三节点 /drone_0/defop_depth_fill|defop_vae|defop_cpn，venv `~/defop_env/bin/python`，launch-prefix；defop_nav config/defop_nav.yaml 原样，不改 yaml —— 见 §6 配置对齐清单对残余差的登记方式）
2. shims 五桥 + episode_mgr（§3 桥接表、§5 运行手册）

外部契约五件的对账（vendored launch_notes"独立跑闭环外部契约五件"）：
- (1) `/drone_0/odom` ← odom_bridge ✓
- (2) 深度源 `/drone_0/depth/image_raw` ← depth_bridge ✓（16UC1/32FC1 自识别 decode_depth_m L62-70）
- (3) `/drone_0/planning/goal` ← goal_bridge（goal_source=topic）✓
- (4) `/drone_0/planning/cpn_active` ← cpn_flag shim ✓（**生存必需**：默认 False + 旗新鲜度 1.0s 按 `rospy.Time.now()`=sim 时（defop_cpn_node.py L740-747,749-753），论文域无 nav_node，不补 shim 则 cpn 一帧 vel_cmd 都不发、连 HOLD 零速都没有，静默跑成空链）
- (5) `/drone_0/vel_cmd` 消费方 ← vel_bridge（原消费者 gz 插件 cmd_timeout 0.5s 在本域不存在）✓

### 1.3 臂 C —— 深度交叉核（同帧 Gazebo 真深度 vs P1 渲染器口径）

- 采集：任一臂（建议臂 U，成本低）运行期间 recorder 旁路录 `/delta/rgbd/camera_depth/depth` + `/delta/rgbd/camera_depth/depth/camera_info` + `/clock`。
- 离线统计（`p35_depth_stats.py`）：
  - 哨兵率：NaN / d≥10.0m / d<0.2m 像素占比（delta 相机 near/far = 0.2/10.0，delta.gazebo:31-46；对照 P1 渲染 depth_sim_node 同口径：0=无效、无命中=10000mm 哨兵、min_range 0.2/max 10.0，sim_settings.yaml L714-729）。
  - 值分布：有效像素深度直方图 + P5/P50/P95 分位（16UC1 mm 换算 m 后比）。
  - 内参：`camera_info` 的 fx,fy,640x360，hfov=1.5184 rad≈87.0°（delta.gazebo:31-46）→ fx≈640/(2·tan(0.7592))≈337，归一化 fx/W≈0.527；P1 渲染 480x270 hfov 87°（sim_settings sensor.depth）→ fx≈253，fx/W≈0.527。**两口径水平 FOV 一致，fx/W 应逐位吻合**，这是 C 臂最硬的一条判据。
- 判据：哨兵率差 <2 个百分点、分位分布逐档单调吻合、fx/W 一致 → "P1 渲染器深度 ≈ Gazebo 深度"成立，帧级 latent 对质中由分辨率/编码引入的差异可归因于桥的 resize 而非域差。
- 注意：640x360 与 480x270 同为 16:9，纯降采样无纵横比畸变；上游 vae_node 也是 nearest 硬压（vae_node.py:126-129），**不要"顺手修正"成分辨率一致以外的任何再加工**（recon gotchas：训练域 DI_SHAPE=270x480，config.py:194）。

---

## 2. 对质协议（核心）

原则：**端到端轨迹不做主判据**。上游整链时间基准混杂（EPISODE_TIMEOUT 用 rospy sim 时 rotors_wrappers.py:1110-1116、evaluate.py:811；slowdown_action 1s 停机用墙钟 evaluate.py:293,296；观测同步=到达驱动 deque 无 stamp 对齐 rotors_wrappers.py:259-279），RTF~0.5 下逐帧跨链对表不可靠。因此只在"同帧无反馈回路"的层面对质，episode 级只比统计量。

### 2.1 层一：同帧深度 → 双链 latent 余弦相似度

- **主判据=离线确定性模式**（首选，无 ROS 抖动）：臂 U 期间 recorder 把 `/delta/rgbd/camera_depth/depth`（bag 存体）与 `/latent_space`（JSONL 存到达序，无 header 戳）成对落盘；离线 `p35_latent_offline.py`（~/defop_env）对每帧：
  - 上游口径 latent：复刻 vae_node 预处理（NaN→10.0、/10 归一、<0.2m 置 -1、nearest 缩 270x480，vae_node.py:102-117,126-129）→ 同一 `seVAE_evo_LD_128.pth` 前向；
  - vendored 口径 latent：复刻 defop_vae preprocess（preprocess L124-144，逐字上游但 resize 路径走本桥产物 480x270）→ 同一权重前向；
  - 算两 latent 余弦相似度逐帧落 JSON；**同时落双链 latent L2 范数，近零范数（<1e-6）帧剔除并注明剔除数** ⚑B6。
- 辅判据=在线双 VAE 模式（shakedown 用）：同 master 同时跑 vae_node 与 defop 三节点（vel_bridge 不接，cpn_flag=False），p35_latent_probe.py 按"到达序 + ≤0.5s sim 窗"配对 `/latent_space` 与 `/drone_0/latent_space` 算余弦——只验传输/订阅链路活，不进判定。
- 阈值 ⚑B6：**不用"同批帧双跑 3σ"**——同机确定性重跑 σ≈1e-7 量级，测的是确定性不是系统差，只作 sanity（非零即查线程池泄漏）。真正零假设=**跨实现系统差**，按 §2.4 分层校准：(a) 预处理像素差地板 + (c) resize 取整量化 + (b) 前向差（层一双链 VAE 同用 ~/defop_env 同 torch，理论近零，仍实测确认）；判定线 = 校准分布 P99.9 对应的 1−cos。
- 报告：1−cos **全分布 + worst-10 逐帧**（带 stamp 供回放定位）⚑B6。
- 超线归因表（嫌疑按序查）：resize 插值口径（nearest vs 其他）、无效标记（-1 压 0 vs NaN→10）、mm/m 制、(0,65535) vs NaN 编码、队列丢帧（双方 queue=2）。

### 2.2 层二：同 tick (latent, odom, depth, goal) → CPN 决策对质

- 采集（臂 U 运行时 recorder 同步录，全部记到达序）：`/latent_space`、`/delta/odometry_sensor1/odometry`、`/delta/rgbd/camera_depth/depth`、`/probs`（**只有 best col_prob 字符串，无全分布**）、`/delta/cmd_velocity`。**不录 /signs**——上游发布已被注释（evaluate.py:1506），死话题 ⚑B5。
- **配对=tick 重构** ⚑B5（禁"cmd 消息时刻回溯最近 latent"——会错拿旧帧，对质整体错位一拍）：上游 tick 语义 = cmd_k 的输入是 cmd_{k−1} 发出之后到达的**第一帧新 latent**，配的 odom/深度 = 该 latent 到达时刻之前最近的帧。按到达序重构元组（latent, odom, depth, goal=(60,0,1) 全程恒定——evaluate.py:766 2D 判定，单 waypoint 文件无推进）。
- 重放（vendored 侧，`p35_cpn_offline.py`）：对每元组灌 `/drone_0/{latent_space,odom,planning/goal,planning/cpn_active}` + 深度图走桥同款 nearest resize 后灌 **`/drone_0/depth/image_raw`**（不是 /drone_0/depth——cpn 订的是 image_raw 名字空间）⚑B5-minor，录 `/drone_0/vel_cmd` + `/drone_0/defop/debug`（col_prob/primitive/safe_cnt/veto_cnt/t_plan_ms）。
- **主判据=逐 tick min-col_prob 选档对质** ⚑B5：上游当拍真实动作已记录在 /delta/cmd_velocity（vel_z 档、yaw 档序号），vendored 同元组重算选档；col_prob 数值先对齐尺度——CPN 输出=15 步 horizon 时间衰减加权和 **Σw=11.51**（0–11.5 档；v1 写 10.9 系笔误），非单步碰撞概率。
- 动作级一致性**降级为选做** ⚑B5：上游动作含行为树段级状态（deadend_step、slowdown 计时），跨链一致需整段行为树重放，成本高——登记选做，不作主判据。
- 安全集对质（Jaccard，⚑minor）：跨链上游不暴露逐基元安全旗；Jaccard 只在"vendored torch vs 上游 TF 模块离线重算 256 基元安全旗"间算（§2.4(b)）；跨链只比选中基元 + col_prob。
- **场景分箱覆盖** ⚑minor：元组按同帧深度几何分四箱 open/near/blocked/deadend，每箱 ≥K（episode_plan.yaml 定）才计入判定；空箱=该场景未测，报告明示——防均值合格掩盖死角。
- 上游侧不重跑 evaluate.py（单体太重、不可重放），用**录制回放对质**：上游在元组 t 时的"选择"就记录在 /delta/cmd_velocity 里，vendored 侧对同元组重算——这正是"同输入比输出"。
- 阈值 ⚑B6：选档不一致率与 |Δcol_prob| 判定线 = §2.4(b)-CPN 校准分布 P99.9（兜底绝对地板：不一致率 3%、|Δcol_prob| 0.05，取 max）；同元组 vendored 重放 ×2 差必须为 0（确定性 sanity，非零先查线程池）。
- 超线归因表：state 组装序（pos/vel/-acc/quat/ang，rotors_wrappers.py:259-279 vs defop_cpn L700-707+defop_common L223-231）、**odom twist 系**（⚑B4：rotors 插件实发 RelativeLinearVel=**机体系**，gazebo_odometry_plugin.cpp:249-252；上游按机体系吃——evaluate.py:1146-1152 只去 roll/pitch；桥已转世界系喂 vendored `odom_twist_frame: world`——若对质出系统性速度差，先查 §5.1 系别探针定谳与桥转换的 yaw 源，再查此行）、UT 参数（双方 ALPHA=1/BETA=2/L=3/λ=0/P_v*=0.16，config.py:125-133 / yaml ut_*）、监督器参数差（见 §6 表，**这是已知残余差，不是 bug**）。

### 2.3 层三：episode 级行为 + vel_cmd 序列统计

- 每臂 **N=5 run**（同世界同钉死 spawn；**不用"同 seed"措辞**——上游 spawn X 原生 [-13,0] 均匀未播种，白名单键 5 钉死 MIN=MAX 后才可比；实际 spawn 位姿逐局记录作协变量）⚑B3+minor，episode 级指标全部由 **harness 侧统一裁判**从共享真值话题算，不信任何一链的自报：
  - 到达率与用时：判据双口径并列——2D≤3.0m（上游口径 config.py:109-112、evaluate.py:766）与 2D≤0.8m（vendored goal_reach_dist，defop_nav.yaml）；源=`/delta/odometry_sensor1/odometry`（零噪声 GT，mav_generic_odometry_sensor.gazebo:26-55）+ `/clock` sim 时。
  - 路径长 / 离障距离：树位由 `parse_world.py` 实例化解析 forest2.world 的 200 棵 tree_N include 坐标（**必须解析 world 而非 yaml——树位必须实例化，yaml 无显式坐标=假阴性，arena 教训**）；16 棵云杉 mesh 断链=幽灵无碰撞（forest2.world 绝对路径 file:///home/dgguglielmo/...，recon gotchas），从离障统计中排除并注明。min clearance 分位 + 碰撞计数（`/delta/delta_contact` ContactsState，判据 collision1_name=='delta::delta/base_link::...' rotors_wrappers.py:146-149,404-436；bumper 源 multirotor_base.xacro:92-93）。
  - vel_cmd 序列统计（双臂同话题 `/delta/cmd_velocity` 同口径可直比）：|linear| mean/std/max、linear.y 占比（0.5·tanh 横移项最大 0.5 m/s，evaluate.py:1488）、linear.z 分布、angular.z 直方图、指令速率（Hz）、断流时长分布；**V 臂分层统计：纯透传段 vs 桥待机/重发注入段分开算**（pre-bridge `/drone_0/vel_cmd` 已录）⚑B7。
  - V 臂附加：`/drone_0/defop/debug` 的 t_plan_ms/safe_cnt/veto_cnt（有效规划率=排除跳拍；TF CPU 单拍 0.5-2s，rospy.Timer 遇慢推理 _busy 非阻塞跳拍→等效 0.5-2Hz）。
- 判定：层一+层二都在线内、而层三 V vs U 超出 **U×5 局的非参带（逐局指标 min–max；N=3 撑不起 3σ）** ⚑minor → **不是移植 bug，是配置/域差**，逐条对 §6 配置对齐清单归因（首要嫌疑：vendored `vel_max: 1.2` 安全包络硬限 vs 上游 2.0 m/s 论文口径 config.py:170-177——P3 已定谳这是"移植保真"差不是域差，勿误归因）。

### 2.4 阈值校准协议（分层零假设，先于一切对质执行）⚑B6

帧级/决策级是确定性计算，**零假设不是跑次方差，是跨实现系统差**（预处理像素差、resize 取整、TF↔torch 前向差）。三层校准各产出分布，阈值取分布 P99.9：

1. **(a) 预处理像素差地板**：同一批原始深度帧分别走上游 vae_node 预处理与 vendored preprocess（含 NaN 处理/-1 标记/归一化差异）→ 逐像素差分布 → P99.9。
2. **(c) resize 取整量化**：nearest 640x360→480x270 的实现差（cv2 vs PIL vs numpy 索引）单独量化，并入 (a)。
3. **(b) 跨实现前向差**：层二为主——同一批合成元组（随机 latent/odom/depth，覆盖 §2.2 四场景箱）分别喂上游 TF CPN 模块（network_inference_tensorflow.py 离线 import，~/defop_env）与 vendored torch CPN（若节点核可离线 import；不可则跨链阈值退化为取兜底绝对地板并在报告标注"校准不完整"）→ |Δcol_prob| 与选档不一致率分布 → P99.9。层一双链 VAE 同用 ~/defop_env 同 torch，前向差实测确认近零即可。
4. **episode 层（层三）仍用跑次带**：臂 U 同配置连跑 5 局得逐局指标 min–max 非参带（arena 跨日方差定谳教训：ROS 调度毫秒抖动→轨迹分叉，A/B 禁单判）。
5. 全部落盘 `results/thresholds.json`（先测后用，禁止先验拍数）；校准与离线对质统一在 ~/defop_env、OMP/OPENBLAS/MKL=1 钉死。

---

## 3. roslaunch/remap 对齐表（上游绝对话题 ↔ vendored ns 话题）

前提事实：上游 config.py 话题全是绝对字符串无 ns（config.py L63-159）、同 master 单实例；vendored 三节点话题是 `self.base+'/...'` 拼的绝对名 `/drone_0/...`，`<remap>` 无效（vendored gotchas）。**故对接全走 relay shim，两侧源码零改动。**

| # | 方向 | 上游话题（绝对名） | vendored 话题（/drone_0 ns） | shim | 语义手术 | 证据 |
|---|---|---|---|---|---|---|
| 1 | sim→两臂 | `/delta/rgbd/camera_depth/depth` 32FC1 米制 640x360@15Hz | `/drone_0/depth/image_raw` | depth_bridge | **值透传（不改值）+ nearest resize 640x360→480x270**（16:9 同比，复刻 vae_node.py:126-129 插值口径；v2 把 v1 的"桥内 NaN→10.0"移出——哨兵/NaN 处理留给下游预处理，与上游 vae_node 同口径，桥不加工、少一层注入差；不 resize 则 cpn 硬丢非 (270,480) 帧，cb_depth L715-720） | delta.gazebo:31-46；defop depth_in arg（defop.launch L16-17,35）也可直指上游话题，但 cpn 尺寸硬校验使 resize 桥不可省 |
| 2 | sim→两臂 | `/delta/odometry_sensor1/odometry` nav_msgs/Odometry 零噪声 | `/drone_0/odom` | odom_bridge | pose 原样（世界 ENU）；**twist 必须系别手术** ⚑B4：rotors odometry 插件发的是 RelativeLinearVel=**机体系**（gazebo_odometry_plugin.cpp:249-252；上游 evaluate.py:1146-1152 只去 roll/pitch 按机体系吃），vendored `odom_twist_frame: world` 要世界系 → 桥默认做 Rz(yaw)·v_body→world（参数 `twist_in: body`，可切 `world` 直通）；**§5.1 硬探针（twist vs 位置有限差分）定谳后锁死** | gazebo_odometry_plugin.cpp:249-252；vendored yaml odom_twist_frame |
| 3 | V→sim | `/drone_0/vel_cmd` Twist 世界系 linear + angular.z | `/delta/cmd_velocity` Twist 机体系 | vel_bridge | **linear: Rz(-yaw)·v_world→机体系**（逆 body_vel_to_world，defop_common.py L66/_cmd_from_primitive L366-389——上游 converter 按机体系速度吃，acc_command_converter.cpp:235-259 + use_vehicle_frame 只作用于 /delta/goal）；**angular.z 默认直通**（两链公式同构：上游 yaw_ctrl=1.0·rel_yaw 限 ±π/4，evaluate.py:848-861,1487,1490；vendored yaw_ctrl=clip(K·err,±π/4)，K=1.0/45°；下游 converter swap_yaw_rate=true 再叠 K_yaw=2.4 对两者一视同仁，rmf_sim_sevae.launch:58 + pid_position_delta.yaml）；**三态语义见 #7** ⚑B7 | yaw 源=桥自己订 odom 四元数 |
| 4 | harness→V | （无上游对应） | `/drone_0/planning/goal` PoseStamped | goal_bridge | waypoint(60,0,1) 一次性发布（只读 position，header 忽略 cb_goal L724-727；新 goal 置 done=False L248-252）；**不周期重发**——cpn 2D<0.8m 进 goal_hold（_reach_goal_action L308-317）即到达语义，重发会复活导致永不收官；由 `/p35/episode_start` 触发 | config.py:194 waypoint 格式 np.loadtxt ndmin=2 |
| 5 | harness→V | （无上游对应） | `/drone_0/planning/cpn_active` Bool | cpn_flag | True @ **5Hz 按 sim 时**（rospy.Timer 走 /clock；旗新鲜度 1.0s 按 rospy.Time.now()=sim 时计，RTF~0.5 时墙钟 1Hz 会误判 stale——1Hz sim 恰贴线故取 5Hz） | defop_cpn_node.py L626-627,740-753 |
| 6 | harness→sim | `/gazebo/set_model_state`、`/gazebo/pause_physics` | — | episode_mgr | 臂 V 每局重摆 delta 到 **(-2,0,0.5) yaw0 零速**（⚑B3：config 原生 spawn X∈[-13,0] 均匀未播种 Z=0.5，config.py:229-236；白名单键 5 钉死 MIN=MAX_INITIAL_X=-2 两臂同点；**实际 spawn 位姿逐局记入 episode JSON 协变量**；上游侧由 rotors_wrappers.py:850-956 spawn/reset 完成） | config.py:229-236 |
| 7 | V→sim（待机/断流） | `/delta/goal` Pose + `/delta/cmd_velocity` | — | vel_bridge 状态机 | **三态** ⚑B7：**PRE-START**（未收到过 vel_cmd：发零位姿 goal 悬停——use_vehicle_frame=true 下机体系零偏置=原位悬停；orientation 语义 shakedown 探针复验，若有转向副作用改发当前 yaw 对应四元数）；**ACTIVE**（转发 vel_cmd；**短断流 <5s sim 重发上一条已转换 cmd**——上游 TF tick 间保持末条指令，若桥转而发零=单方面注入"急减速"假行为污染层三）；**STALL**（断流 ≥5s sim：零 cmd + 零位姿 goal **每拍重发**——converter 对 vel_cmd/goal last-message-wins 互斥，早先的 goal 已被后来的 vel_cmd 盖掉，不重发不悬停）。断流阈值 5s sim ≫ TF 单拍 0.5-2s。协调话题 `/p35/episode_start|end` 切态。上游 slowdown 1s 是**墙钟**（evaluate.py:293,296）、vendored 1s 是 sim 时——已知口径差，§6 登记 | evaluate.py:269-272,293,296；rmf_sim_sevae.launch use_vehicle_frame；vendored gz 插件 cmd_timeout 0.5s HOLD 语义的域内等价物 |
| 8 | 两臂→harness | `/delta/delta_contact` ContactsState | （V 臂同样生效） | recorder 旁路 | 碰撞判据同串（rotors_wrappers.py:146-149） | multirotor_base.xacro:92-93 |
| 9 | 上游独有 | `/latent_space` Float32MultiArray(128)、`/probs`（仅 best col_prob 串）、~~/signs~~（**发布已注释 evaluate.py:1506，死话题不录** ⚑B5）、`/start_planner`(srv)、`/stop_planner`、`/set_goal_dir` | — | recorder / runner | 层一/层二采集与发车控制 | rotors_wrappers.py:73-74,100-102；evaluate.py:1506,1523 |

上游**不订**的东西（别误接）：PLANNING_TYPE=1 时 wrapper 用 `/latent_space` 当观测源不订深度（rotors_wrappers.py:73-76）；`/delta/command/rate_thrust`（rotors_wrappers.py:84,175-186）只在 len(action)==3 走，evaluate 动作 4 元不用。

---

## 4. 目录约定

全部文件在 `~/zx2026_arena_ws/tools/p35_refdom/`（**不进 defop_ws**——那边在 catkin build，drop-in 会污染构建；本 harness 无编译件，纯 python/launch/sh）。launch 内包引用用 `$(find rmf_sim)`，在 `source defop_ws/devel/setup.bash` 后可解析（runner preflight 先 `rospack find rmf_sim` 与 `rospack find lmf_planner_srvs_msgs` 验一下，缺了再补 source）。

```
~/zx2026_arena_ws/tools/p35_refdom/
├── HARNESS_DESIGN.md            # 本文档
├── shims/
│   ├── p35_odom_bridge.py       # 表#2（twist_in: body|world 参数化）
│   ├── p35_depth_bridge.py      # 表#1（32FC1/16UC1 自识别 → nearest 480x270 值透传 → /drone_0/depth/image_raw）
│   ├── p35_vel_bridge.py        # 表#3/#7（三态状态机 + ang_mode: passthrough|rate_to_angle）
│   ├── p35_goal_bridge.py       # 表#4（读 configs/waypoint.yaml，episode_start 触发一次性发布）
│   ├── p35_cpn_flag.py          # 表#5
│   └── p35_episode_mgr.py       # 表#6 + 100s sim 时窗 + 判分落盘 + 局间循环
├── probes/
│   ├── p35_recorder.py          # 双轨录制：rosbag（depth/camera_info/odom/cmd_velocity/contact，带 header sim 戳）
│   │                            #   + JSONL（/latent_space /probs /drone_0/vel_cmd /drone_0/defop/debug 等**无戳话题的到达序**，/clock sim 时自记 arrival_t）
│   ├── p35_latent_offline.py    # 层一主判据（~/defop_env）
│   ├── p35_latent_probe.py      # 层一在线辅判（shakedown）
│   ├── p35_cpn_offline.py       # 层二重放对质（要求 defop 栈已起）
│   └── p35_depth_stats.py       # C 臂
├── launch/
│   ├── p35_upstream.launch      # 臂 U：仅 rmf_sim(forest2) + recorder；evaluate/vae_node 由 run_arm.sh 以正确 cwd/解释器拉起（<node> 起不了 cwd）
│   └── p35_vendored.launch      # 臂 V：rmf_sim(forest2) + 五桥 + episode_mgr + include defop.launch
├── configs/
│   ├── config_p35_main.py       # 主 config.py 拷贝 + 白名单六键（§1.1），由 runner 拷成 ORACLE/config.py，run 后还原
│   ├── vae_node_p35.py          # vae_node 无 GPU 拷贝改 cpu 档（有 GPU 则不用）
│   ├── waypoint.yaml            # 单一真源 {x:60,y:0,z:1}
│   └── episode_plan.yaml        # 局数(5)/超时(100s sim)/分箱 K/RTF 表
├── scripts/
│   ├── run_arm.sh               # 每臂 runner：env source、config 拷贝-还原、launch/进程编排、发车 srv、超时、清场（rosmaster 点名 pkill，防孤儿毒批）
│   ├── make_waypoints.py        # waypoint.yaml → 上游 txt（"60.0 0.0 1.0" 单行）+ goal_bridge yaml，双臂同源
│   ├── parse_world.py           # forest2.world → tree_N 树位实例化清单（离障用；云杉剔除注记）
│   └── report_p35.py            # 汇总三层对质 → verdicts.json
└── results/                     # 每局 bag/jsonl/json + thresholds.json（先测后用）
```

make_waypoints.py 规范：上游格式=每行世界系 `x y z`（np.loadtxt ndmin=2，evaluate.py:194；默认 waypoint_front.txt 已是 '60.0 0.0 1.0'，P3.5 生成同值文件 `waypoints/waypoint_front_p35.txt` 并在 config_p35_main.py 的 WAYPOINT_FILE 指过去，**不覆盖上游原文件**）；vendored 侧由 goal_bridge 直接读 waypoint.yaml 发 PoseStamped（goal_source=topic）。目标 (60,0,1) 1.0 m/s 主 config 口径直线 62m≈62s+绕障，EPISODE_TIMEOUT=100s sim（config.py:227-236）**偏紧——shakedown 实测臂 U 到达用时后再定是否放宽（episode_plan.yaml 一处改，双臂同步）**。

### 4.5 数据契约（实现各文件必须逐字遵守，report_p35 依赖这些键名）

- `results/ep_{U|V}{r}.json`：`{arm, run, spawn:[x,y,z,yaw], t_start_sim, t_end_sim, outcome: "reached_080"|"reached_300"|"timeout"|"collision"|"other", reach_t_sim_080, reach_t_sim_300, path_len, minclr_p05, minclr_p50, collisions:[{t_sim}], vel_stats:{mean,max,y_ratio,z_hist,ang_hist,rate_hz,stall_max_s}}`
- `results/frames_{U|V}{r}.jsonl`：每行 `{t_arr_sim, topic, ...payload}`（/latent_space: `vec:[128]`；/probs: `text`；/drone_0/defop/debug: 原串；/drone_0/vel_cmd: `{lx,ly,lz,az}`）
- `results/latent_cos_U{r}.json`：每帧 `{i, t_arr_sim, cos, l2_up, l2_vd, dropped}` + 顶层 `{n, n_dropped, threshold}`
- `results/cpn_U{r}.jsonl`：每元组 `{tick, t_arr_sim, latent_arr, up_vz, up_yaw, vd_primitive, vd_col_prob, agree, bin}`（bin ∈ open|near|blocked|deadend）
- `results/thresholds.json`：`{prep_pixel_p999, resize_p999, fwd_cos_p999, cpn_absdiff_p999, cpn_choice_disagree_p999, band_U:{metric:[5 局逐局值]}, meta:{env, omp1, date}}`

---

## 5. 运行手册

### 5.0 公共环境（每条 runner 脚本开头）

```bash
source /opt/ros/noetic/setup.bash
source ~/defop_ws/devel/setup.bash          # $(find rmf_sim) 可解析（preflight rospack 验）
export ROS_MASTER_URI=http://localhost:11312  # P3.5 专用 master，与 arena 11411 无关
# 全部 harness 节点 rosparam set /use_sim_time true（gazebo 起来后自动为 true，shim 启动前显式再设一道）
```

观测窗纪律：**一切计时/超时/对齐按 `/clock` sim 时**（rospy.Timer、rospy.Time.now() 在 use_sim_time 下自动走 sim 时）；墙钟只用于 runner 的看门狗换算：`wall ≈ sim / RTF`，RTF 实测（shakedown 量 60s）后写入 episode_plan.yaml，预算按 RTF=0.4 保守取。

### 5.1 Shakedown（阶段 0，一次性）

1. `roscore -p 11312` → 起臂 U 的 rmf_sim(forest2)：`rostopic hz /delta/rgbd/camera_depth/depth`（15Hz 640x360 32FC1）、`/delta/odometry_sensor1/odometry`、`/delta/delta_contact` 在列。
2. **白名单六键逐键 grep 复验** ⚑B1/B2/B3：ROBOT_DEPTH_TOPIC / COLLISION_USE_TENSORRT=False / TF_CHECKPOINT_PATH→nett5,5,6 / EVALUATE_MODE=True / MIN=MAX_INITIAL_X=-2 / NUM_EPISODES_EVALUATE=5；**`rosnode info` 验 evaluate 节点订阅列表含 `/latent_space`**（EVALUATE_MODE 生效断言——缺失=latent 链路静默失效）。
3. 臂 U 干跑 1 局：evaluate 起来后首个 tick 无 AttributeError（P0 修否则必崩，evaluate.py:1061）；`rosservice call /start_planner "{}"` 后 `/delta/cmd_velocity` 出流；实测 RTF。
4. vel 语义探针：录 60s 上游 angular.z vs odom yaw 差分，确认"K·err 角"假设（|值|≤π/4 且与 yaw 误差相关）；若呈 rad/s 行为，vel_bridge 切 `rate_to_angle`（桥内维护 yaw_ref+=cmd·dt_sim，发 (yaw_ref−yaw) 作 error-angle）。
5. **odom twist 系别硬探针** ⚑B4：直线巡航段 twist.linear vs 位置有限差分（世界系）——差分≈twist 原值=世界系（桥 `twist_in:=world` 直通）；差分≈Rz(yaw)⁻¹·twist=机体系（桥默认 `twist_in:=body` 转换）。定谳后锁进 episode_plan.yaml。
6. 臂 V 单链活体：五桥+defop.launch，`rostopic echo /drone_0/planning/cpn_active` 5Hz、`/drone_0/latent_space` 出流、`/drone_0/defop/debug` t_plan_ms 有值、vel_bridge 出 `/delta/cmd_velocity`。**cpn_flag_dead（ratio<0.05）探针是唯一活体检出**（两条联锁键缺一都伪装成"跑成基线"：planner.source 缺=cpn 恒静默、sensor.depth.enabled 缺=hold_stale_depth——本域不经 sim_settings，此两键风险由"五桥全在列"检查替代）。
7. **vel_bridge 三态注入测试** ⚑B7：人为停 cpn 5s sim（kill cpn_flag）→ 桥进 STALL 悬停不坠机、零位姿 goal 被每拍重发（rostopic echo 可见）；恢复后桥回 ACTIVE 重发末条→正常续飞。
8. 层一在线辅判跑通（cos 输出非 NaN）。

### 5.2 臂 U × 5 局（噪声基线 + 校准帧源 + 主数据）

```bash
scripts/run_arm.sh U   # 内部：拷 config_p35_main.py→ORACLE/config.py（还原留底 hash）
                       #      起 p35_upstream.launch → 等 'START planner'
                       #      rosservice call /start_planner "{}"
                       #      NUM_EPISODES_EVALUATE=5 一次连跑；episode_mgr 监视器只旁观判分落盘
                       #      （边界=位姿突跳>5m 或 100s 无到达，双判取先）
                       #      finally 还原 config.py 并 filecmp 校验（defop_ab_run 同款惯例）
```
- TRT 失败时按 §1.1 落 TF fallback（预期路径——本机无匹配 TRT）；vae_node 按 §1.1 GPU 档。
- recorder 全程录（§4 双轨）。C 臂统计随这些 bag 免费产出。
- 单实例红线：11312 上同一时刻只有一条链；双 master（11311/11411 并存）不冲突（绝对名互不干扰）但 CPU 争抢会劣化 RTF——**串行跑臂**。

### 5.3 臂 V × 5 局

```bash
scripts/run_arm.sh V   # 起 p35_vendored.launch（rmf_sim forest2 + 五桥 + defop.launch drone_id:=0）
                       # episode_mgr 内循环 5 局：set_model_state 重摆 (-2,0,0.5) yaw0 → pause/unpause →
                       #   /p35/episode_start → goal_bridge 发 (60,0,1) → 100s sim 窗内判 2D≤0.8（到达）/≤3.0（U 口径参考线）
                       #   /超时 /碰撞 → ep_V{r}.json → 下一局
```

### 5.4 离线校准、对质与报告

先校准后对质：`p35_latent_offline.py`（含 (a)+(c) 像素地板）→ (b) 跨实现前向校准（合成元组扫 TF↔torch CPN）→ `p35_cpn_offline.py`（复用 11312，双臂全收工后单实例跑）→ `report_p35.py`：读 results/ 与 thresholds.json，产出三层判定 + §6 归因表勾选 + verdicts.json。

### 5.5 每臂墙钟预算

sim 100s/局 × 5 局，RTF~0.5 → 1000s/臂纯 sim；加 TF CPU 0.5-2s/拍跳拍（两臂同为 TF 同速，公平）、launch/重摆开销 → **每臂 CELL 预算 2400s，runner 看门狗 2700s 强杀清场**（rosmaster 点名 + 格尾点名清场，防孤儿毒批；SIGTERM 显式 trap）。run-to-run 方差教训：结论只从 5 局聚合出，禁单局判。

---

## 6. 配置对齐清单（U/V 残余差登记，层三归因用）

| 参数 | 上游（主 config） | vendored（defop_nav.yaml） | 归类 |
|---|---|---|---|
| CMD_VELOCITY / cmd_velocity | 1.0（config.py:75） | 1.0 | ✅ 已对齐 |
| WAYPOINT 阈值 / yaw | 3.0m / 45°（config.py:109-112） | 3.0 / 45.0 | ✅ |
| DEADEND / COLLISION | 6.5/5.0 / 0.35（config.py:103-106） | 6.5/5.0 / 0.35 | ✅ |
| N_E / UT / 权重 | 3 / P_v*=0.16（config.py:115,125-133） / nett5,5,6 hdf5（**主 config 默认 TF 路径指 net1/2/3≠vendored——白名单键 2/3 强制覆盖** ⚑B1） | 3 / 同 / 同三文件 | ✅ 同权同档（覆盖后） |
| EVALUATE_MODE | True（白名单键 4；缺失=latent 链路静默失效+EPISODE_TIMEOUT 退 30s） ⚑B2 | n/a（vendored 无此键） | ✅ |
| action 维数 | vel_x 1 档、vel_z 8、yaw 32=256 条（config.py:78-81） | 1/8/32 | ✅ |
| VEL_MAX（打分用速度尺度） | 2.0（config.py:170-177，DEPTH_TS/SKIP/HORIZON 推导） | horizon 15 + cpn_weight 同档（yaml 注明 vel_max=2.0 档） | ✅ |
| **安全包络 vel_max** | 无（论文口径 2.0） | **1.2 水平模长硬限**（defop_common.py L95） | ⚠ V 独有，P3 已定谳"移植保真"差 |
| **监督器参数** | min_dist=1.0, min_ratio=0.05, radius=0.30（evaluate.py:1085,586） | 1.5 / 0.02 / 0.32（yaml sup_*） | ⚠ 残余差：defop.launch 无法透传覆盖（yaml 整包灌+节点启动即读），**登记不修改**；层二对质若呈系统性否决率差，先查此行 |
| goal 终点语义 | 到点=stop_robot+episode 结束（FAIL→'reach final WP'，evaluate.py:766-770，2D 3.0m） | 2D<0.8 进 goal_hold 悬停（_reach_goal_action L308-317），判"到达"用 0.8 线 | ⚠ 已知口径差（紧 6 倍），双口径并列报（§2.3） |
| angular.z 语义 | K·rel_yaw 角（evaluate.py:848-861），converter 再 P（K_yaw=2.4） | clip(K·err,±π/4) 同构 | ✅（探针复验） |
| linear.y | 0.5·tanh(rel_yaw) 横移（evaluate.py:1488） | vel_y_gain 0.5 同构 | ✅ |
| 深度分辨率路径 | 640x360 →vane nearest→270x480（vae_node.py:126-129） | 640x360 →桥 nearest→480x270→cpn 硬校验 (270,480)（cb_depth L715-720） | ✅（桥复刻插值口径） |
| slowdown 计时 | 1s **墙钟**（evaluate.py:293,296） ⚑B7 | 1s sim 时 | ⚠ 已知口径差，层三归因登记 |
| EPISODE_TIMEOUT | 100s sim（config.py:227-236） | episode_mgr 同 100s sim | ✅ |
| 起点位姿 | 原生 X∈[-13,0] 均匀未播种、Z=0.5（config.py:229-236）；白名单键 5 钉死 MIN=MAX_INITIAL_X=-2 → (-2,0,0.5) YAW=0 ⚑B3 | episode_mgr set_model_state 同值 | ✅（钉死后；实际 spawn 逐局记协变量） |

---

## 7. gotchas 吸收清单（逐条落位）

| 教训 | 落位 |
|---|---|
| 上游 ROBOT_DEPTH_TOPIC stale → 首 tick 崩（evaluate.py:1061） | §1.1 白名单键 1，shakedown §5.1-2 复验 |
| **权重暗换：主 config TF 默认 net1/2/3 ≠ vendored nett5,5,6（80309 元素全异）** ⚑B1 | 白名单键 3 + §5.1-2 grep |
| **EVALUATE_MODE 缺失 = latent 链路静默失效** ⚑B2 | 白名单键 4 + rosnode info 订阅断言（§5.1-2） |
| **spawn 契约：原生 X∈[-13,0] 未播种 Z=0.5，非 (-2,0,1)** ⚑B3 | 白名单键 5 钉死 + spawn 协变量记录（§3#6/§6） |
| **odom twist 系别：插件实发机体系 RelativeLinearVel** ⚑B4 | odom_bridge twist_in 参数 + 有限差分探针定谳（§3#2/§5.1-5） |
| **对质配对错拍：按 cmd 时刻回溯最近 latent 会错位一拍** ⚑B5 | tick 重构配对（§2.2）；/signs 不录（发布已注释 evaluate.py:1506） |
| **阈值测错零假设：跑次 σ≈1e-7 是确定性不是系统差** ⚑B6 | §2.4 分层校准 P99.9 |
| **零指令注入污染：桥断流转发零 ≠ 上游保持末条** ⚑B7 | vel_bridge 三态 + 断流 5s sim + 重发末条（§3#7）；pre-bridge 分层统计 |
| PLANNING_TYPE=1 观测源=/latent_space，vae_node 缺席=静默空转（rotors_wrappers.py:73-74、evaluate.py:918-928 只 print exception） | 臂 U 发车前 `rostopic hz /latent_space` 活体检查（§5.1-3） |
| 时间基准混杂（sim 时+墙钟混用、到达驱动 deque） | §2 原则：帧级只在无反馈回路层面对质，episode 级比统计；一切 harness 计时按 /clock |
| goal 契约 0.8 停机悬停 vs 上游 3.0m+episode 终结 | goal_bridge 一次性发布不重发（表#4）；双口径裁判（§2.3/§6） |
| cpn_active 旗默认 False+1.0s sim 时新鲜度=完全静默 | cpn_flag 5Hz sim 时 shim（表#5），shakedown 活体探针 |
| 上游无 ns 绝对话题 / vendored 拼接绝对名、remap 无效 | §3 全 relay shim 方案；11312 专用 master 单实例 |
| RTF~0.5（100s sim≈200s 墙钟） | §5.0 换算纪律 + §5.5 预算 2400s/臂 + 看门狗强杀清场 |
| cpn 硬丢非 (270,480) 帧 | depth_bridge nearest resize（表#1） |
| 上游 640x360→270x480 nearest 拉伸是"上游原样"勿修正 | 桥只做同比例降采样，插值口径对齐 vae_node（§1.3） |
| TRT engine 版本绑定 / nett 列表前 N_E 项 | §1.1 TF fallback + 主 config（corridor 配置禁用） |
| cwd 敏感（evaluate 于 ORACLE 包根、vae_node 包路径）+ 解释器敏感（TF2.4.1 在 defop_env） | §1.1/§4 runner 内固定 cwd 与 `~/defop_env/bin/python` |
| 世界网格断链：forest2=200 细杆树+16 幽灵云杉、forest4=空场 | 世界选 forest2；云杉剔除离障统计（§2.3）；**禁用 forest4** |
| 树位必须实例化解析（yaml 无坐标=假阴性） | parse_world.py 解析 forest2.world（§2.3） |
| 单实例假设 | §5.2 红线：11312 同刻单链、双臂串行；层二重放亦排他 |
| sim_settings 联锁两键（planner.source/sensor.depth.enabled） | 本域不经 sim_settings，风险由 §5.1-6 五桥在列检查替代 |
| 配置即结果、不可混搭 | §6 全表登记，层三归因照单勾选 |

---

## 8. 判定矩阵

| 层一（latent 同帧） | 层二（CPN 同元组） | 层三（episode） | 结论 |
|---|---|---|---|
| 超线 | — | — | **移植 bug（VAE 预处理段）**，按 §2.1 归因表修 |
| 线内 | 超线 | — | **移植 bug（CPN 决策段）**，按 §2.2 归因表修（state 组装/UT/监督器差先排除） |
| 线内 | 线内 | V 显著差于 U 非参带 | **域/配置差**，§6 表逐行归因（首查 vel_max 1.2 包络与监督器参数） |
| 线内 | 线内 | U 非参带内 | **移植保真成立**：P3 arena 的差距定谳为域差+落地完成度差，P3.5 结案 |

---

## 9. 验收清单（harness 本身的 DONE 线）

- [ ] thresholds.json 先于一切对质产出：含 (a)/(c)/(b) 三校准分布 + U×5 非参带（⚑B6）
- [ ] 臂 U 5 局零 AttributeError、/latent_space 全程有流、config.py 还原 filecmp 一致
- [ ] 臂 V 5 局 cpn_flag_dead ratio<0.05（旗活）、t_plan_ms 有值、vel_bridge 三态经注入测试（人为停 cpn 5s：STALL 悬停不坠机；恢复后重发末条续飞）⚑B7
- [ ] odom twist 系别探针已定谳并锁进桥参数（⚑B4）
- [ ] 白名单六键 grep 复验通过 + rosnode info 订阅含 /latent_space（⚑B1/B2/B3）
- [ ] 层一 cos 全分布+worst-10、层二分箱覆盖率+一致率、层三双口径指标三表齐
- [ ] report_p35.py 产出 verdicts.json（§8 矩阵命中行）
- [ ] 11312 上无残留进程（rosmaster 点名清场）
