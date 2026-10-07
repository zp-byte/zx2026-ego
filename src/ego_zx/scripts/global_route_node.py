#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""global_route_node.py — ego 臂**全局路由器**（2026-10-06 用户裁决「ego 引入
全局规划器」）。

## 为什么需要它（病灶，可复算）

mission 只发**单点 goal**（mission_executor_node.py:722-733 `_goto`），ego 的
"全局参考"是一条**直线** min-snap 多项式（planner_manager.cpp:457-519），
`getLocalTarget`（ego_replan_fsm.cpp:899-954）把它裁到 `planning_horizon` 7.5 m。
⇒ 任何长于 7.5 m 的横墙 = 局部极小值 = 死锁。实测：网（AABB x[-22.85,-22.75]
y[4,12] z[0,2.50]）把 3 台 y=+3.75 的机（机身半径 0.35 ⇒ y 跨 [3.40,4.10]）
顶死在 x≈-23.1。

## 本节点的做法（非侵入）

    读 mission 单点 goal ──A*（arena_nav.astar）──> 绕障路线
                                                  │  LOS 简化 + 按 hop 细分
                                                  v
                          逐点重发 /planning/route_goal ──remap──> ego

- **不动 ego 任何 C++**：ego 仍是 flight_type=1 单点 goal，只是 goal 话题被
  remap 到本节点的输出。每次重发 = `waypointCallback`→`planNextWaypoint`，
  FSM 在 EXEC_TRAJ 时立刻转 REPLAN_TRAJ 从当前位置续播（ego_replan_fsm.cpp:187-192），
  不减速、无累积状态。
- **不动 nav**：nav 仍订原始 `/planning/goal`（旁路影子管线），本节点是只读旁听。
- **OFF 臂逐字节旧行为**：launch arg `global_route` 同时控制「起不起本节点」与
  「ego remap 到哪」，一个开关原子生效，不存在半开。

## 占用源（诚实性是本节点的设计轴）

- `net_only`（**默认、主臂**）：只把 `kind=net` 当障碍——scene_topology.yaml:86-90
  明写「此段坐标=装订禁行源，到场踏勘复核」，因为真机 Mid360 对遮阳网
  **结构性失明**。不读树表 ⇒ 不含 oracle 知识 ⇒ 可迁移真机。
- `scene_truth`（**诊断臂，须标注 oracle**）：直接用 `Scene.collides_xy`（含
  203 棵树）。等同"机群读到了没飞到的林子"，与 via_slots source=cloud 的
  诚实化裁决相冲突，**只作 sim 路线层上限的量尺，不可迁移**。

## 已知边界（诚实记录）

- 只治**路线层**（选哪条线）。~~不治**脱困放大器**~~ ⇒ **2026-10-06 起治**，
  见下节。
- `net_only` 只绕网，**首撞集合要靠 `learn_obs` 才动**（见下节）。
- 网仍然**感知不到**：路线只是让它别靠近，不是让它看见。

## 碰撞 / 越网冻结同源治理（2026-10-06 用户裁决，§9.14）

**为什么同源**：`gazebo_collision_monitor.py:235` —— 累计第 **9** 撞 ⇒ 本局
**永久零速**；而一次 10-15s 的物理楔死会被 bounce→cooldown 重入状态机放大成
**4-6 撞**。六局实测：**扣掉楔死重入簇后每台冻结机只剩 1-6 撞，全部低于 9**
⇒ **治住楔死放大器即救回全部冻结**。冻结 ≡ 第 9 撞，不是两个病。

**总闸 `govern`**（OFF 臂与 2026-10-06 之前逐字节同），三件套：

1. `learn_obs` —— 订 `/grid_map/occupancy`（**ego 自己深度图的产物**，
   world 系点云，`grid_map.cpp:859 publishMap` 按需发布：没人订就**不序列化**）
   累积成世界系障碍格。**只认本机看过的世界**，树的真值表从不参与
   ⇒ 与 `via_slots source=cloud` 的诚实化裁决同源。让 ego 的全局参考
   不再**首程**就穿过它已经看见的树列。
2. `hazard_learn` —— 碰撞上升沿挂**动态禁行盘**（激活门：等监视器的 bounce
   把机推到盘外才入盘，否则 A* 起点被占 ⇒ 退成 pass-through ⇒ 反而强化病灶）
   \+ **撞后撤退航点**（反向接近向量，比监视器"树心→机心"的径向 bounce 安全：
   后者可能指向邻树）。撤退窗 = 监视器占有执行器的 2.5s + 后退飞行时间。
3. `stuck_skip_hazard` —— STUCK-SKIP 从"盲目跳点"改成"标危险盘 + 重规划"，
   把到不了的航点变成**可复用路线知识**（原行为单局链式 12-16 次、吃任务时间）。

危险盘有界（合并去重 / 上限淘汰 / **mission goal 豁免**）——项目内血案先例：
`sim_settings.yaml:425` 记永久 `_collision_obs` 曾把投放点变成不可达 ⇒ 进近死循环。

## z 支持（2026-10-06 补齐）

原先本节点**只规划 xy**：z 只是一个用来判障碍的固定带（`z_lo/z_hi`），出口
z 从 mission goal 原样透传。四处缺口，一处硬化：

1. **学习格带竖直区间** `[(ix,iy)] -> [zmin,zmax]`。这是**放宽观测带的前置
   条件**——不带 z 时一个只在 z=3.4 看到的格会堵死 z=2.4 的巡航走廊（假堵）。
   区间化后"放宽观测带"从有害变成纯收益。
2. **观测带放宽**：`obs_z_lo/hi` 2.0/2.9 → **1.5/3.5**（实测撞点 z 域
   1.0–4.41，原窗与真实撞高错配：G 臂 63/79 撞点在带内、B 臂只有 36/77）。
   上限取 **3.5 而不是 4.5** 是硬约束：`grid_map.cpp:887` 的
   `pos(2) > visualization_truncate_height_(3.5)` 把 3.5 以上的体素**在发布
   路径上直接丢掉** ⇒ 写 4.5 是个**永不生效的死旋钮**。想看见 3.5–4.41 那
   一段，真正的杠杆是抬 `advanced_param_zx.xml:128` 的
   `visualization_truncate_height`（本轮不动：属 ego 栈参数，与本项混在一批
   会污染 A/B 归因）。
3. **出口 z 封顶**：`min(z, heights.flight_z_ceiling)`。封顶值**不新增本地
   键**，直接读 `competition_rules.yaml`（nav_node.py:222 同源）⇒ 不会漂移。
   钳位必留 warn 痕（诚实化纪律）。
4. **撤退航点 z 取实测高度**（封顶后），不再取 mission goal z——撤退是纯水平
   动作（垂直弹向有砸地血案，见 zx2026-gz-branch-physics）。

**危险盘仍保持 2D 竖直柱**，刻意为之：172/173 起碰撞是树干（竖直柱），柱就是
忠实模型；而 `/collision` 是裸 `std_msgs/Bool`（无 kind/位置/z）——给它编个
z 区间等于凭空造数据，违反本节点「只认看过的世界」的诚实轴。

4 项全部挂在 `govern` 总闸与 `clamp_z` 子键下 ⇒ OFF 臂仍**逐字节**旧行为。

纯 rospy（无 TF/torch），系统 python3 直跑。tunables 全在 sim_settings.yaml
`planner.global_route`。
"""

import json
import math
import time
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Bool, String

from arena_nav.astar import AStar
from zx2026_common import config as cfg
from zx2026_common.scene import Scene


class _BoundObs(object):
    """占用适配器：**装订禁行源（net）∪ 本机已见障碍格 ∪ 动态碰撞危险盘**。

    (a) `nets`：只把 `kind=net` 当障碍（scene_topology.yaml:86-90 的「装订禁行
        源」——真机 Mid360 对遮阳网结构性失明）。几何与 scene.py:607-611 的
        通用 AABB 分支逐字一致，**不含 oracle 知识**。
    (b) `learned`：把 **ego 自己的占用图**（/grid_map/occupancy，world 系点云，
        由 grid_map.cpp:859 `publishMap` 按需发布）累积成世界系整数格。**只认
        本机看过（=本机深度图拍到过）的世界** —— 与 `via_slots source=cloud`
        的诚实化裁决同源，`mission_executor_node.py:735-751` 的 `_vs_obs_cells`
        是同款做法。树的坐标表**从不参与**。
    (c) `hazards`：碰撞点动态禁行盘，事件真值（本机 `/collision` 上升沿 + 全队
        latched `/zx2026/hazard_cells`）。语义与 `nav_node.py:905-929` 同。

    **z 语义（2026-10-06 补齐）**：
    - (b) 学习格**带竖直区间**（`[(ix,iy)] -> [zmin,zmax]`）。(a)(c) 不带：
      - **网**的 AABB 自带 `lo[2]/hi[2]`，本来就按 z 判（scene.py:607-611）。
      - **危险盘保持 2D 竖直柱**，是**刻意的**：172/173 起碰撞是树干（竖直
        柱），柱就是它的忠实模型；而 `/collision` 是**裸 `std_msgs/Bool`**
        （无 kind、无位置、无 z）——给它编一个 z 区间等于**凭空造数据**，
        违反本节点「只认看过的世界」的诚实轴。
    - 学习格若不带 z，观测带一放宽就会**主动有害**：一个只在 z=3.4 被看到的
      格会挡住 z=2.4 的巡航走廊（假堵）⇒ 区间化是放宽观测带的**前置条件**。

    OFF 臂（`govern: false`）下 (b)(c) 恒空 ⇒ 本类退化为原 `_NetOnly`，行为与
    2026-10-06 之前逐字节同。
    """

    def __init__(self, scene, res):
        self.nets = [o for o in scene.obstacles if o.kind == "net"]
        self.res = float(res)
        self.learned = {}         # {(ix, iy): [zmin, zmax]} 世界系整数格 + 该格
                                  # 被看到的竖直区间（单调只增，见 add_cell）
        self.hazards = []         # [(x, y, r)] 2D 竖直柱（理由见类 docstring）

    # ---- 写入：返回 True = 状态**确有变化**（调用方据此失效 A* 的按格缓存）----
    def add_cell(self, ix, iy, z):
        """记一个已见格，并把它被看到的 z **并进**该格的竖直区间。

        **拓宽也算变化**（返回 True）：`astar._occ_cache` 是按 (ix,iy) 缓存的，
        格从「区间不相交 ⇒ 通」变成「相交 ⇒ 挡」必须失效缓存，否则那条过期的
        「通」会让 A* 继续穿过去。
        """
        old = self.learned.get((ix, iy))
        if old is None:
            self.learned[(ix, iy)] = [float(z), float(z)]
            return True
        if z < old[0]:
            old[0] = float(z)
            return True
        if z > old[1]:
            old[1] = float(z)
            return True
        return False

    def add_hazard(self, x, y, r, merge_m=None):
        m = float(merge_m) if merge_m else 0.5 * r
        for (hx, hy, _hr) in self.hazards:
            # 合并半径：一次物理顶死会被 bounce→cooldown→重入放大成 4-6 次
            # 碰撞，同点重复建盘只会徒增判据成本并吃掉可选走廊。
            if (hx - x) ** 2 + (hy - y) ** 2 < m * m:
                return False
        self.hazards.append((x, y, r))
        return True

    # ---- 查询 ----
    def _learned_hit(self, x, y, z_lo, z_hi, pad):
        """已见格按 pad 膨胀后是否罩住 (x,y)，**且**该格的竖直区间与 [z_lo,z_hi]
        相交。

        水平判定**精确**：算查询点到格矩形的欧氏距离，而不是拿格心近似。
        k = ceil(pad/res) 已足够——索引相距 k+1 的格，最近距离必 ≥ k*res ≥ pad。
        竖直判定用半开区间相交（`hi < z_lo or lo > z_hi`），与 net 分支
        （`collides_xy` 第一段）同口径。"""
        k = int(math.ceil(pad / self.res - 1e-9))
        if k < 0:
            k = 0
        cx = int(math.floor(x / self.res))
        cy = int(math.floor(y / self.res))
        for di in range(-k, k + 1):
            ix = cx + di
            x0 = ix * self.res
            dx = max(x0 - x, 0.0, x - (x0 + self.res))
            for dj in range(-k, k + 1):
                iy = cy + dj
                zz = self.learned.get((ix, iy))
                if zz is None:
                    continue
                if zz[1] < z_lo or zz[0] > z_hi:
                    continue            # 竖直不相交 ⇒ 在本次飞行的带外，不挡
                y0 = iy * self.res
                dy = max(y0 - y, 0.0, y - (y0 + self.res))
                if dx * dx + dy * dy <= pad * pad:
                    return True
        return False

    def collides_xy(self, x, y, z_lo, z_hi, pad=0.0):
        for ob in self.nets:
            if ob.hi[2] < z_lo or ob.lo[2] > z_hi:
                continue
            if x + pad >= ob.lo[0] and x - pad <= ob.hi[0] and \
               y + pad >= ob.lo[1] and y - pad <= ob.hi[1]:
                return True
        if self.learned and self._learned_hit(x, y, z_lo, z_hi, pad):
            return True
        for (hx, hy, hr) in self.hazards:
            if (hx - x) ** 2 + (hy - y) ** 2 <= (hr + pad) ** 2:
                return True
        return False


class GlobalRoute(object):
    def __init__(self):
        self.drone_id = str(rospy.get_param('~drone_id', '0'))
        self.ns = '/drone_%s' % self.drone_id

        r = cfg.load('sim_settings.yaml').get('planner', {}).get('global_route', {})
        self.source = str(r.get('source', 'net_only'))
        self.res = float(r.get('resolution', 0.5))
        self.infl = float(r.get('inflation', 0.45))
        self.los_pad = float(r.get('los_pad', 0.45))
        self.z_lo = float(r.get('z_lo', 2.0))
        self.z_hi = float(r.get('z_hi', 2.9))
        self.hop_max = float(r.get('hop_max', 5.0))
        self.min_route_len = float(r.get('min_route_len', 3.0))
        self.advance_dist = float(r.get('advance_dist', 1.5))
        self.min_pub_dt = float(r.get('min_pub_dt', 0.3))
        self.stuck_skip_s = float(r.get('stuck_skip_s', 8.0))
        self.los_step = float(r.get('los_step', 0.25))
        self.los_span = int(r.get('los_span', 60))
        self.los_step_fine = float(r.get('los_step_fine', 0.05))

        # ---- 2026-10-06 碰撞/越网冻结同源治理（§9.14）----
        # `govern` 是**总闸**：关时下面四组全部不生效 ⇒ 退化为原 `_NetOnly`
        # 行为。冻结机理见 gazebo_collision_monitor.py:235（第 9 撞 ⇒ 永久零速），
        # 而一次物理楔死会被 bounce→cooldown 重入放大成 4-6 撞 ⇒ 治楔死即可救回。
        self.govern = bool(r.get('govern', False))
        # (1) 学障碍：只认 ego 自己深度图拍到的世界
        self.learn_obs = self.govern and bool(r.get('learn_obs', True))
        # 占用话题的**全名**。注意命名空间是 ego 规划器节点名
        # `drone_<i>_ego_planner_node`（advanced_param_zx.xml:46），**不是**
        # `/drone_<i>`——`publishMap` 发的是相对名 `grid_map/occupancy`
        # （grid_map.cpp:859），且**无 remap**（该文件 remap 段落只覆盖
        # odom/cloud/pose/depth/goal），所以实际落在
        # `/drone_<i>_ego_planner_node/grid_map/occupancy`。
        # 2026-10-06 冒烟实锤：订 `/drone_<i>/grid_map/occupancy` 时
        # `Publishers: None`（该话题只因我们误订而存在）。
        self.obs_topic = str(r.get('obs_topic',
                                   '/drone_{i}_ego_planner_node/'
                                   'grid_map/occupancy')).format(i=self.drone_id)
        self.obs_z_lo = float(r.get('obs_z_lo', self.z_lo))
        self.obs_z_hi = float(r.get('obs_z_hi', self.z_hi))
        self.obs_cell = float(r.get('obs_cell', self.res))
        self.obs_hz = float(r.get('obs_hz', 2.0))
        self.obs_self_r = float(r.get('obs_self_r', 0.6))
        # (2) 反楔死：碰撞点动态禁行盘 + 撞后撤退航点
        self.hazard_learn = self.govern and bool(r.get('hazard_learn', True))
        self.hazard_r = float(r.get('hazard_r', 0.8))
        self.hazard_ttl = float(r.get('hazard_ttl', 0.0))    # 0 = 本局持久
        # **激活门**：碰撞瞬间机心**就在**危险盘里，此刻入盘会让 A* 起点被占
        # （astar.py:64-66 返回 [start]）⇒ _plan_route 退成 pass-through ⇒ ego
        # 又被直线拉回树里 = 反而强化病灶。所以先挂 pending，等监视器的 bounce
        # 把机推到盘外（径向弹开 ~2m），或在 hazard_activate_max_s 后强制激活。
        self.hazard_clear_margin = float(r.get('hazard_clear_margin', 0.5))
        self.hazard_activate_max_s = float(r.get('hazard_activate_max_s', 2.0))
        self.hazard_merge_m = float(r.get('hazard_merge_m', 0.75))
        # 上限 + 合并：**项目内已有血案**——sim_settings.yaml:425 记永久
        # `_collision_obs` 占格曾把投放点变成不可达 ⇒ 进近死循环。危险盘必须
        # 有界，且**不得压在 mission goal 上**（见 goal_exempt）。
        self.hazard_max = int(r.get('hazard_max', 40))
        self.hazard_goal_exempt = float(r.get('hazard_goal_exempt', 1.5))
        # 全队 haz 广播：默认**关**——本机/他机盘可能掐掉别的机的走廊，先当
        # 独立臂验证（批 2），不进批 1 的决定性单变量。
        self.hazard_fleet = self.govern and bool(r.get('hazard_fleet', False))
        self.replan_min_s = float(r.get('replan_min_s', 1.0))
        self.escape_back = float(r.get('escape_back', 2.5))
        # 撤退窗须盖住 **监视器占有执行器的整段时间**（bounce_dt 1.0 +
        # cooldown_dt 1.5 = 2.5s，其间 /vel_cmd 被监视器 100Hz 接管，我们发什么
        # 都不被执行）+ 真正后退 2.5m 所需的飞行时间（nav max_vel 1.8 ⇒ ~1.5s）。
        # 取 2.5 会在监视器刚交还执行器的同一拍就取消撤退 —— 那等于没撤。
        self.escape_hold_s = float(r.get('escape_hold_s', 4.0))
        self.escape_clear_m = float(r.get('escape_clear_m', 0.6))
        # (3) STUCK-SKIP 改造：把「跳点」变成「标危险盘 + 重规划」
        self.stuck_skip_hazard = self.govern and bool(r.get('stuck_skip_hazard', True))
        # (4) 出口 z 封顶（2026-10-06 补齐 z 支持）
        # **封顶值不新增本地键**，直接读规则真值源 `competition_rules.yaml`
        # `heights.flight_z_ceiling`——nav_node.py:222 与
        # mission_executor_node.py:192 读的是同一个键，这样不会出现「规则改了
        # 3 处、路由器这处忘了改」的漂移。0 = 不钳（nav 的既有约定）。
        self.clamp_z = self.govern and bool(r.get('clamp_z', True))
        try:
            self.z_ceiling = float(
                cfg.load('competition_rules.yaml')
                .get('heights', {}).get('flight_z_ceiling', 0.0))
        except Exception as e:
            self.z_ceiling = 0.0
            rospy.logwarn('[global_route d%s] flight_z_ceiling 读取失败(%s) ⇒ '
                          '不钳 z', self.drone_id, e)

        # ---- 状态 ----
        self.odom = None
        self.goal = None              # 最近一次 mission goal (x,y,z)
        self.routed_goal = None       # 已规划的 mission goal（去重）
        self.route = []
        self.ridx = 0
        self.last_pub_t = 0.0
        self.last_adv_t = 0.0
        self.skipped = set()
        self.ready = False            # astar 未就绪前不得规划

        # ---- 治理态（§9.14）----
        self.occ = None               # _BoundObs（scene_truth 档为 None=直用 Scene）
        self.traj = deque(maxlen=200)  # [(t,x,y)] 近 1s 速度方向用
        self.last_obs_t = 0.0
        self.obs_dirty = False        # 有新学习格 ⇒ 待失效 A* 缓存
        self.n_learned = 0
        self.col_prev = None          # /collision 边沿检测（latched，首帧不触发）
        self.pending_haz = None       # 待激活危险盘 {x,y,r,t}（激活门见 _tick）
        self.esc_pt = None            # 撤退航点 (x,y,z)
        self.esc_until = 0.0
        self.n_escape = 0
        self.n_hazard = 0
        self.last_replan_t = 0.0

        # **先订阅、再预热**：mission 的 goal 是**一次性**发布（_goto 每腿发一次）。
        # 若订在预热之后，预热那几十秒里到达的 goal 直接蒸发 —— 本节点永远拿不到
        # goal ⇒ 不转发 ⇒ ego 拿不到 goal ⇒ 六机全冻。rospy 订阅队列会缓冲。
        self.pub = rospy.Publisher(self.ns + '/planning/route_goal',
                                   PoseStamped, queue_size=1)
        rospy.Subscriber(self.ns + '/planning/goal', PoseStamped, self._on_goal,
                         queue_size=10)
        rospy.Subscriber(self.ns + '/odom', Odometry, self._on_odom,
                         queue_size=10)
        self.pub_dbg = rospy.Publisher(self.ns + '/planning/route_debug',
                                       String, queue_size=10)

        # **治理订阅**（§9.14）。三条都要在预热前挂上，理由同上（预热几十秒
        # 里的消息不能丢）。`learn_obs` 关时不订阅 —— 那正是我们要的：
        # grid_map.cpp:862 的发布门是 `getNumSubscribers() <= 0`，**没人订就
        # 不序列化** ⇒ OFF 臂零成本、逐字节旧行为。
        if self.learn_obs:
            rospy.Subscriber(self.obs_topic,
                             PointCloud2, self._on_occ_cloud, queue_size=2)
            rospy.loginfo('[global_route d%s] govern: learn_obs ON <- %s',
                          self.drone_id, self.obs_topic)
        if self.hazard_learn:
            rospy.Subscriber(self.ns + '/collision', Bool, self._on_collision,
                             queue_size=10)
            rospy.loginfo('[global_route d%s] govern: hazard_learn ON r=%.2f '
                          'esc=%.1fm/%.1fs fleet=%s',
                          self.drone_id, self.hazard_r, self.escape_back,
                          self.escape_hold_s, self.hazard_fleet)
        if self.hazard_fleet:
            rospy.Subscriber('/zx2026/hazard_cells', String,
                             self._on_fleet_hazard, queue_size=10)

        self.scene = Scene()
        if self.source == 'scene_truth':
            occ = self.scene
        else:
            self.occ = _BoundObs(self.scene, self.obs_cell)
            occ = self.occ
        sz = self.scene.venue['size']
        half_x, half_y = sz[0] / 2.0, sz[1] / 2.0
        self.astar = AStar(occ, resolution=self.res, inflation=self.infl,
                           x_range=(-half_x, half_x), y_range=(-half_y, half_y),
                           z_lo=self.z_lo, z_hi=self.z_hi)
        t0 = time.time()
        for ix in range(self.astar.nx):
            for iy in range(self.astar.ny):
                self.astar._occ(ix, iy)
        rospy.loginfo('[global_route d%s] prewarm %dx%d %.2fs source=%s',
                      self.drone_id, self.astar.nx, self.astar.ny,
                      time.time() - t0, self.source)
        self.ready = True
        if self.goal is not None and self.odom is not None:
            self._plan_route()        # 预热期间缓冲到的 goal 补规划
        rospy.Timer(rospy.Duration(0.05), self._tick)   # 20Hz 推进拍
        rospy.loginfo('[global_route d%s] ready: %s -> %s (hop<=%.1f adv<=%.1f)',
                      self.drone_id, self.ns + '/planning/goal',
                      self.pub.resolved_name, self.hop_max, self.advance_dist)

    # ---- 回调 ----
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, p.z)
        if self.govern:
            self.traj.append((rospy.get_rostime().to_sec(), p.x, p.y))

    def _on_occ_cloud(self, msg):
        """学障碍：ego 自己的占用图（world 系点云）→ 世界系整数格。

        诚实性：这是**本机深度图**的产物（`grid_map.cpp` 的 `occupancy_buffer_`，
        publishMap:885 发的**未膨胀**原始占用格 ⇒ 膨胀由本节点自己套 pad，
        不与 ego 的 0.36 叠加），不是真值表。FOV 外/背后的世界学不到——
        那正是它的诚实之处，也正是本方案的已知边界。
        """
        if not self.learn_obs:
            return
        now = rospy.get_rostime().to_sec()
        if (now - self.last_obs_t) < (1.0 / self.obs_hz):
            return                      # 发布端固定 9Hz，解析成本由我们限频
        self.last_obs_t = now
        ox, oy = (self.odom[0], self.odom[1]) if self.odom else (None, None)
        new = 0
        try:
            for pt in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                      skip_nans=True):
                x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
                if z < self.obs_z_lo or z > self.obs_z_hi:
                    continue
                if ox is not None and \
                        (x - ox) ** 2 + (y - oy) ** 2 < self.obs_self_r ** 2:
                    continue            # ego 已清自足迹，双保险
                # z **一并入格**（不是丢弃）：格子的竖直区间就是"本机在这个
                # xy 上看到过的高度"，查询时与规划带相交判定（见 _learned_hit）。
                if self.occ.add_cell(int(math.floor(x / self.obs_cell)),
                                     int(math.floor(y / self.obs_cell)), z):
                    new += 1
        except Exception as e:          # 点云字段异常不该毒死整条链
            rospy.logwarn_throttle(10.0, '[global_route d%s] occ parse: %s',
                                   self.drone_id, e)
            return
        if new:
            self.n_learned = len(self.occ.learned)
            self.obs_dirty = True

    def _on_collision(self, msg):
        """`/collision` **上升沿** ⇒ 挂待激活危险盘 + 记撤退方向。

        此刻**不动盘**：碰撞瞬间机心就在树旁，立刻入盘会让 A* 起点被占
        （`astar.py:64-66` 返回 `[start]`）⇒ `_plan_route` 退成 pass-through
        ⇒ ego 又被直线拉回树里 = **反而强化病灶**。激活门在 `_govern_tick`。
        """
        col = bool(msg.data)
        if self.col_prev is None:
            self.col_prev = col          # latched 话题：首帧当基准，不造幻影边沿
            return
        rising = col and not self.col_prev
        self.col_prev = col
        if not rising or not self.hazard_learn or self.odom is None:
            return
        x, y = self.odom[0], self.odom[1]
        now = rospy.get_rostime().to_sec()
        self.pending_haz = {'x': x, 'y': y, 'r': self.hazard_r, 't': now}
        d = self._approach_dir()
        # 撤退是**纯水平**动作：z 取当前**实测**高度（封顶后），不取 mission
        # goal z。理由：撞后机可能正悬在平台/低空，拉到 goal z（巡航高度）是
        # 一次竖直 excursion；且项目内已有「垂直弹向」血案
        # （zx2026-gz-branch-physics：向下弹=砸地 UNAUTHORIZED_LANDING）。
        z = self._clamp_z(self.odom[2])
        if d is None:
            rospy.logwarn('[global_route d%s] COLLISION @(%.2f,%.2f) 无接近方向'
                          ' ⇒ 只挂盘不撤退', self.drone_id, x, y)
            return
        # 撤退方向 = **反向接近向量**（回它来的那条净空路），比监视器的径向
        # bounce 更安全：bounce 是"树心→机心"，可能指向邻树（nav_node.py:
        # 882-887 记过 bounce 与逃逸同向叠加、把机推进 3m 外另一棵树的血案）。
        for k in (1.0, 0.6, 0.3):       # 逐步缩短，直到撤退点在界内且不落网
            ex = x - d[0] * self.escape_back * k
            ey = y - d[1] * self.escape_back * k
            if self._esc_ok(ex, ey):
                self.esc_pt = (ex, ey, z)
                self.esc_until = now + self.escape_hold_s
                self.n_escape += 1
                rospy.logwarn('[global_route d%s] ESCAPE armed #%d -> '
                              '(%.2f,%.2f) dir=(%.2f,%.2f) back=%.1fm hold=%.1fs',
                              self.drone_id, self.n_escape, ex, ey,
                              d[0], d[1], self.escape_back * k,
                              self.escape_hold_s)
                return
        rospy.logwarn('[global_route d%s] COLLISION @(%.2f,%.2f) 撤退点全不可用'
                      ' ⇒ 只挂盘不撤退', self.drone_id, x, y)

    def _on_fleet_hazard(self, msg):
        """全队危险格广播（nav_node.py:871-877 发、`hazard_share` 默认开）。

        他机盘**直接激活**——那是他机**已经脱离**的点，不存在"起点被占"问题。
        """
        if not self.hazard_fleet or self.occ is None:
            return
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if int(d.get('id', -1)) == int(self.drone_id):
            return
        try:
            x, y = float(d['x']), float(d['y'])
        except (KeyError, TypeError, ValueError):
            return
        if self._add_hazard(x, y, self.hazard_r):
            rospy.loginfo('[global_route d%s] HAZARD-SHARE <- drone %s '
                          '(%.2f,%.2f) n=%d', self.drone_id, d.get('id'),
                          x, y, self.n_hazard)


    def _on_goal(self, msg):
        p = msg.pose.position
        if p.z < -0.1:            # 镜像 ego_replan_fsm.cpp:213 的守卫
            return
        g = (p.x, p.y, p.z)
        if self.routed_goal is not None:
            d = ((g[0] - self.routed_goal[0]) ** 2
                 + (g[1] - self.routed_goal[1]) ** 2) ** 0.5
            if d < 0.25 and abs(g[2] - self.routed_goal[2]) < 0.05:
                # 同一点重发（状态机重入）：**不重规划路线**——那会清掉推进进度
                # 并把机重新拉向直线；但仍把**当前待飞航点**再推一遍，复刻基线
                # 下 ego 收到重发 goal 会重规划的行为（pass-through 时 route 只有
                # 一点，重推=原样转发，与基线逐字节同）。
                self._publish_next(force=True)
                return
        self.goal = g
        self._plan_route()

    # ---- 治理：危险盘 / 撤退 / 重规划 ----
    def _approach_dir(self):
        """近 1 s 平均速度方向（单位向量）；机被顶死无位移时用「机心→当前
        航点」反向兜底。都取不到返回 None。"""
        now = rospy.get_rostime().to_sec()
        pts = [p for p in self.traj if (now - p[0]) <= 1.0]
        if len(pts) >= 2:
            dx = pts[-1][1] - pts[0][1]
            dy = pts[-1][2] - pts[0][2]
            n = math.hypot(dx, dy)
            if n > 0.05:
                return (dx / n, dy / n)
        if self.route and self.odom is not None and self.ridx < len(self.route):
            tx, ty, _ = self.route[self.ridx]
            dx, dy = tx - self.odom[0], ty - self.odom[1]
            n = math.hypot(dx, dy)
            if n > 1e-6:
                return (dx / n, dy / n)
        return None

    def _esc_ok(self, x, y):
        """撤退点可用性：界内 + 不落网。**故意不看学习格/危险盘**——树就在
        撤退路径旁，拿它们当判据会把唯一的退路自己堵死。"""
        sz = self.scene.venue['size']
        if abs(x) > sz[0] / 2.0 - 1.0 or abs(y) > sz[1] / 2.0 - 1.0:
            return False
        if self.occ is not None:
            for ob in self.occ.nets:
                if ob.hi[2] < self.z_lo or ob.lo[2] > self.z_hi:
                    continue
                if ob.lo[0] <= x <= ob.hi[0] and ob.lo[1] <= y <= ob.hi[1]:
                    return False
        return True

    def _add_hazard(self, x, y, r):
        """入盘（合并去重 + goal 豁免 + 上限淘汰）。True = 确有变化。"""
        if self.occ is None:
            return False
        # **goal 豁免**：项目内血案先例——sim_settings.yaml:425 记永久
        # `_collision_obs` 占格曾把投放点变成不可达 ⇒ 进近死循环。危险盘
        # 绝不能压住 mission goal。
        if self.goal is not None and \
                (x - self.goal[0]) ** 2 + (y - self.goal[1]) ** 2 < \
                self.hazard_goal_exempt ** 2:
            rospy.logwarn('[global_route d%s] hazard (%.2f,%.2f) 落在 goal 邻域'
                          '(%.1fm) 内 ⇒ 豁免不入盘', self.drone_id, x, y,
                          self.hazard_goal_exempt)
            return False
        if not self.occ.add_hazard(x, y, r, self.hazard_merge_m):
            return False
        hs = self.occ.hazards
        if len(hs) > self.hazard_max:            # 有界：防累积吃掉全部走廊
            n_ev = len(hs) - self.hazard_max
            del hs[:n_ev]
            rospy.logwarn('[global_route d%s] hazard 超上限 %d ⇒ 淘汰最旧 %d 个',
                          self.drone_id, self.hazard_max, n_ev)
        self.n_hazard = len(hs)
        self.astar._occ_cache.clear()            # 占用一变，按格缓存必须失效
        return True

    def _clamp_z(self, z):
        """出口 z 封顶（`heights.flight_z_ceiling`，0=不钳）。

        **为什么路由器也要钳**：nav 的 `_on_goal` 钳的是**它自己那份** goal
        （nav_node.py:791-796，`self.goal` 局部变量，不回写 mission，也不
        影响 ego）——而 ego 的 goal 已被 remap 到本节点，**本节点是 ego 唯一
        的 goal 来源**。所以只要本节点不钳，ego 就会照着超限 z 飞。
        实测（§9.14 G_1 d4）：mission 的 hover-lock 目标 z=5.20（= 机**当时
        实测高度**）经本节点原样转发，rule_monitor 的 OVER_HEIGHT 判据是
        **实测** z>5.0 持续 2s ⇒ 机停在超限高度上。钳到 4.50 会让 ego **主动
        下降**脱离违规区，比"原样转发"严格更好。

        夹持必留痕（诚实化纪律：静默改写指令=新的不可见真值泄漏面，
        与 nav_node.py:792 同口径）。"""
        if self.clamp_z and self.z_ceiling > 0 and z > self.z_ceiling:
            rospy.logwarn_throttle(5.0,
                                   '[global_route d%s] goal z %.2f clamped to '
                                   'flight_z_ceiling %.2f',
                                   self.drone_id, z, self.z_ceiling)
            return self.z_ceiling
        return z

    def _pub_pt(self, pt, tag='WP'):
        m = PoseStamped()
        m.header.frame_id = 'world'
        m.header.stamp = rospy.get_rostime()
        m.pose.position.x, m.pose.position.y = pt[0], pt[1]
        m.pose.position.z = self._clamp_z(pt[2])
        m.pose.orientation.w = 1.0
        self.pub.publish(m)
        self.last_pub_t = rospy.get_rostime().to_sec()

    def _route_blocked(self):
        """剩余路线是否已与新学习格/危险盘相交（细采样口径同 _audit）。"""
        if not self.route or self.ridx >= len(self.route):
            return False
        r = self.route
        for k in range(self.ridx, len(r) - 1):
            if not self._clear_ex(r[k][0], r[k][1], r[k + 1][0], r[k + 1][1],
                                  self.los_pad, self.los_step):
                return True
        x, y, _ = r[self.ridx]
        return self.astar.scene.collides_xy(x, y, self.z_lo, self.z_hi,
                                            pad=self.los_pad)

    def _replan_now(self, why):
        """从**当前 odom** 重规划到原 mission goal。

        退化保护：重规划若退成单点 pass-through（直线）而原来本是一条绕行
        路线，则**保留原路线**——治理臂绝不能因为一次瞬时 no_path 而比对照臂
        更差（那会把机直接拉回树里）。
        """
        if self.goal is None or self.odom is None:
            return
        keep = (self.route, self.ridx, set(self.skipped))
        self._plan_route()
        if len(self.route) == 1 and len(keep[0]) > 1:
            self.route, self.ridx, self.skipped = keep[0], keep[1], keep[2]
            rospy.logwarn('[global_route d%s] 重规划(%s)退化为单点 ⇒ 保留原 '
                          '%d 点路线', self.drone_id, why, len(self.route))
            self._publish_next(force=True)
        else:
            rospy.loginfo('[global_route d%s] 重规划(%s) -> %d 点',
                          self.drone_id, why, len(self.route))

    def _govern_tick(self, now):
        """每拍治理：激活待定盘（激活门）→ 重规划；学习格切到路线 → 重规划。"""
        # 遥测（15s 一拍）：smoke/法证全靠它读「学障碍链是否真有料」。
        rospy.loginfo_throttle(
            15.0, '[global_route d%s] govern: learned=%d hazards=%d escapes=%d '
                  'pend=%s esc=%s obs_dirty=%s',
            self.drone_id, len(self.occ.learned) if self.occ is not None else -1,
            self.n_hazard, self.n_escape, self.pending_haz is not None,
            self.esc_pt is not None, self.obs_dirty)
        p = self.pending_haz
        if p is not None:
            d = math.hypot(self.odom[0] - p['x'], self.odom[1] - p['y'])
            clear = d > (p['r'] + self.hazard_clear_margin)
            if clear or (now - p['t']) >= self.hazard_activate_max_s:
                self.pending_haz = None
                if self._add_hazard(p['x'], p['y'], p['r']):
                    rospy.logwarn('[global_route d%s] HAZARD #%d active '
                                  '@(%.2f,%.2f) r=%.2f d=%.2f%s', self.drone_id,
                                  self.n_hazard, p['x'], p['y'], p['r'], d,
                                  '' if clear else ' (FORCED: 机仍被顶死)')
                    if (now - self.last_replan_t) >= self.replan_min_s:
                        self.last_replan_t = now
                        self._replan_now('hazard')
        if self.obs_dirty:
            self.obs_dirty = False
            self.astar._occ_cache.clear()
            if self._route_blocked() and \
                    (now - self.last_replan_t) >= self.replan_min_s:
                self.last_replan_t = now
                self._replan_now('obs')

    # ---- 规划 ----
    def _pass_through(self, why, warn=False):
        """降级：原样转发 mission goal（= 基线行为）。

        两条路走这里：(a) 直线本来就通/腿太短——**病灶不在此，不动它**，
        让未受阻的机与基线逐字节同行为；(b) 规划失败——宁可退回基线，也
        绝不能不发令（ego 的 goal 已 remap 到本节点，不发 = 六机全冻）。"""
        self.route = [self.goal]
        self.ridx = 0
        self.routed_goal = self.goal
        self.skipped = set()
        self._publish_next(force=True)
        log = rospy.logwarn if warn else rospy.loginfo
        log('[global_route d%s] PASS-THROUGH (%s) goal=(%.2f,%.2f,%.2f)',
            self.drone_id, why, self.goal[0], self.goal[1], self.goal[2])

    def _plan_route(self):
        if not self.ready or self.odom is None or self.goal is None:
            return
        start = (self.odom[0], self.odom[1])
        if self.occ is not None:
            # A* 起点被自己的学习格/危险盘罩住 ⇒ plan() 退成 [start]
            # （astar.py:64-66）⇒ pass-through ⇒ 直线拉回树里。先螺旋找
            # 最近自由格当起点。
            ix, iy = self.astar._to_index(start[0], start[1])
            if 0 <= ix < self.astar.nx and 0 <= iy < self.astar.ny \
                    and self.astar._occ(ix, iy):
                nf = self.astar._nearest_free(ix, iy)
                if nf is not None:
                    start = self.astar._to_xy(nf[0], nf[1])
                    rospy.logwarn('[global_route d%s] A* 起点被占 ⇒ 就近自由格 '
                                  '(%.2f,%.2f)（机在 %.2f,%.2f）',
                                  self.drone_id, start[0], start[1],
                                  self.odom[0], self.odom[1])
        goal2 = (self.goal[0], self.goal[1])
        straight = ((goal2[0] - start[0]) ** 2 + (goal2[1] - start[1]) ** 2) ** 0.5
        # 短腿（末段降落/识别/悬停）不路由：z 带是巡航带，对低空腿无意义
        if straight < self.min_route_len:
            self._pass_through('short_leg')
            return
        # **手术刀**：直线本来就通 ⇒ 原样单点转发，与基线逐字节同行为。
        # 只有真正被挡住的腿才改道（net_only 下即 3 台 y=+3.75 的机）。
        if self._clear(start[0], start[1], goal2[0], goal2[1], self.los_pad):
            self._pass_through('line_clear')
            return
        raw = self.astar.plan(start, goal2)
        if raw is None or len(raw) < 2:
            self._pass_through('no_path', warn=True)
            return
        simp = self._los_simplify(raw)
        hop = self._subdivide(simp)
        if len(hop) < 2:
            self._pass_through('degenerate', warn=True)
            return
        # 首点若已在身后则丢弃；末点**必须是 mission goal 本身**（mission 的
        # 到位判据是 ||goal-odom||，见 mission_executor_node.py:1118-1123）
        z = self.goal[2]
        pts = [p for p in hop if (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2 > 0.25]
        route = [(x, y, z) for (x, y) in pts]
        route.append(self.goal)
        nbad = self._audit(route)
        self.route = route
        self.ridx = 0
        self.routed_goal = self.goal
        self.skipped = set()
        self._publish_next(force=True)
        rospy.loginfo('[global_route d%s] ROUTE raw=%d simp=%d hop=%d 违例=%d '
                      'straight=%.1fm route=%.1fm goal=(%.2f,%.2f,%.2f)',
                      self.drone_id, len(raw), len(simp), len(pts), nbad, straight,
                      sum(((route[i + 1][0] - route[i][0]) ** 2
                           + (route[i + 1][1] - route[i][1]) ** 2) ** 0.5
                          for i in range(len(route) - 1)),
                      self.goal[0], self.goal[1], self.goal[2])

    def _clear_ex(self, x0, y0, x1, y1, pad, step):
        d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        n = max(2, int(d / step) + 1)
        for i in range(n + 1):
            t = i / float(n)
            if self.astar.scene.collides_xy(x0 + (x1 - x0) * t,
                                            y0 + (y1 - y0) * t,
                                            self.z_lo, self.z_hi, pad=pad):
                return False
        return True

    def _clear(self, x0, y0, x1, y1, pad):
        return self._clear_ex(x0, y0, x1, y1, pad, self.los_step)

    def _audit(self, route):
        """组装完成后的**只读**复核（细采样）。

        起手过滤与 goal 追加会造出 `_los_simplify` 没验过的段（A* 把目标夹到
        最近自由格时尤其）。这里把结果**量出来**而不是去修——「插中点二分」修
        不了切角：中点仍在那条线上，线不动，违例原样留着（实测：插了 11 个点，
        最近点纹丝不动还是 0.4596）。真修法在 `_los_simplify` 的两级采样里。
        """
        bad = [k for k in range(len(route) - 1)
               if not self._clear_ex(route[k][0], route[k][1],
                                     route[k + 1][0], route[k + 1][1],
                                     self.los_pad, self.los_step_fine)]
        if bad:
            rospy.logwarn('[global_route d%s] 组装后 %d 段细采样不净空 %s'
                          '（A* 格心仍净空，残余毫米级）',
                          self.drone_id, len(bad), bad[:5])
        return len(bad)

    def _los_simplify(self, raw):
        """贪心视线简化——**两级采样 + 每段必验**。

        **为什么两级**：A* 的对角线一步会**切障碍角**——两个格心各自净空
        >pad，连线的最近点却能掉到 pad 以下。实测 d5 net_only：
        raw 里 (-23.5,4.0)→(-23.0,3.5) 两格心距网 0.65/0.52，连线最近点
        只有 **0.4596 < pad 0.45**；而粗采样（los_step=0.25）的采样点落在
        t=1/3、2/3，违例带在 t=0.648 ⇒ 一步跨过去，漏。所以：粗采样快速
        下探，再对**候选**做 los_step_fine 细采样复核。

        **为什么 `j > i`（原版是 `j > i+1`）**：原版在 j 退化到 i+1 时直接
        放行，**配对从未被验过**。改成 `j > i` 后每段必有验证记录；连相邻
        格心都不净空时硬走一格并 logwarn（A* 保证格心净空，残余是毫米级）。"""
        out = [raw[0]]
        i = 0
        ncorner = 0
        n = len(raw)
        while i < n - 1:
            j = min(n - 1, i + self.los_span)
            while j > i and not self._clear_ex(raw[i][0], raw[i][1],
                                               raw[j][0], raw[j][1],
                                               self.los_pad, self.los_step):
                j -= 1
            while j > i and not self._clear_ex(raw[i][0], raw[i][1],
                                               raw[j][0], raw[j][1],
                                               self.los_pad, self.los_step_fine):
                j -= 1
            if j == i:
                j = i + 1
                ncorner += 1
            out.append(raw[j])
            i = j
        if ncorner:
            rospy.logwarn('[global_route d%s] %d 处相邻格心连接仍不净空'
                          '（A* 对角线切角），已硬走一格',
                          self.drone_id, ncorner)
        return out

    def _subdivide(self, simp):
        """把简化折线按 hop_max 细分（hop <= planning_horizon 7.5 保证
        getLocalTarget 的局部目标恰好落在航点上，见 ego_replan_fsm.cpp:940-944）。"""
        out = []
        for k in range(len(simp) - 1):
            (x0, y0), (x1, y1) = simp[k], simp[k + 1]
            d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            m = max(1, int(d / self.hop_max) + (1 if d % self.hop_max > 1e-6 else 0))
            for i in range(m):
                t = i / float(m)
                out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        if simp:
            out.append(simp[-1])
        return out

    # ---- 推进 ----
    def _publish_next(self, force=False):
        if not self.route:
            return
        now = rospy.get_rostime().to_sec()
        if not force and (now - self.last_pub_t) < self.min_pub_dt:
            return
        x, y, z = self.route[self.ridx]
        m = PoseStamped()
        m.header.frame_id = 'world'
        m.header.stamp = rospy.get_rostime()
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.position.z = self._clamp_z(z)   # 出口钳（见 _clamp_z；route 里存原值）
        m.pose.orientation.w = 1.0
        self.pub.publish(m)
        self.last_pub_t = now
        self.last_adv_t = now
        rospy.loginfo('[global_route d%s] WP %d/%d (%.2f,%.2f,%.2f)',
                      self.drone_id, self.ridx + 1, len(self.route), x, y, z)

    def _tick(self, _evt):
        # 兜底补规划：goal/odom 早于 astar 就绪到达（含预热期订阅队列缓冲）时，
        # _on_goal 里的那次 _plan_route 会空转 —— 这里以 20Hz 重试。
        if not self.route:
            if self.ready and self.goal is not None and self.odom is not None:
                self._plan_route()
            return
        if self.odom is None:
            return
        now = rospy.get_rostime().to_sec()

        if self.govern:
            self._govern_tick(now)

        # 撤退窗：只推撤退航点，冻结常规推进与 STUCK-SKIP（此刻 ego 的全局
        # 参考必须**指向身后**，否则监视器一交还执行器它又朝树里飞）。
        if self.esc_pt is not None and now < self.esc_until:
            if (now - self.last_pub_t) >= self.min_pub_dt:
                self._pub_pt(self.esc_pt, tag='ESCAPE')
            return
        if self.esc_pt is not None:
            self.esc_pt = None
            self.last_adv_t = now
            rospy.loginfo('[global_route d%s] ESCAPE done -> 自当前位置重规划',
                          self.drone_id)
            self._replan_now('escape')
            return

        x, y, _ = self.route[self.ridx]
        d = ((self.odom[0] - x) ** 2 + (self.odom[1] - y) ** 2) ** 0.5
        if d < self.advance_dist:
            if self.ridx < len(self.route) - 1:
                self.ridx += 1
                self._publish_next()
            return
        # 卡死放行：同一航点久不推进 ⇒ 跳下一个（对治 bounce/cooldown 楔死）。
        # 只在"自身位移也小"时判卡（复刻 mission GOTO_HI 看门狗口径，避免把
        # 正常绕行误判成卡死）。每个航点最多跳一次。
        if self.ridx in self.skipped:
            return
        if (now - self.last_adv_t) > self.stuck_skip_s and self.ridx < len(self.route) - 1:
            if self.stuck_skip_hazard:
                # §9.14 改造：把这个**到不了**的航点变成可复用的路线知识再重
                # 规划，而不是盲目跳点——原行为单局链式 12-16 次、吃光任务
                # 时间 ⇒ landed 低（§9.13 归因）。去重/豁免命中时回退旧行为。
                wx, wy, _wz = self.route[self.ridx]
                if self._add_hazard(wx, wy, self.hazard_r):
                    rospy.logwarn('[global_route d%s] STUCK-SKIP -> 标危险盘 '
                                  'wp%d (%.2f,%.2f) d=%.2f %.1fs + 重规划',
                                  self.drone_id, self.ridx, wx, wy, d,
                                  now - self.last_adv_t)
                    self.last_replan_t = now
                    self._replan_now('stuck')
                    return
            self.skipped.add(self.ridx)
            self.ridx += 1
            rospy.logwarn('[global_route d%s] STUCK-SKIP wp%d (d=%.2f, %.1fs) -> wp%d',
                          self.drone_id, self.ridx, d, now - self.last_adv_t,
                          self.ridx + 1)
            self._publish_next(force=True)


if __name__ == '__main__':
    rospy.init_node('global_route', anonymous=False)
    GlobalRoute()
    rospy.spin()
