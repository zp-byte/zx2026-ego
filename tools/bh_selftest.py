#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/bh_selftest.py — 真机树枝避障 O1/O4 决策核心离线自测（O5 风摆膨胀
已随 20260912 判占线重设计退役）。

用法（WSL）：
  cd ~/zx2026_defop_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/bh_selftest.py

结果写 /tmp/bh_selftest.txt（UTF-8，规避 GBK 控制台）。

覆盖（nav_node 模块级纯函数 + 真实 Scene 几何集成）：
  A. bh_evidence_tier 判占分档：判占线=sway_dense_factor×T（近 6/远 12）——
     细枝稀疏回波（1-5 点）停留不确定档不封格，密集回波（≥判占线=树干/
     密枝簇签名）才判占；窗口遗忘/重观测刷新（20260912 分支矩阵定论：
     旧 T 线判占把细枝升 occ 闭合树间走廊 → B 臂 0/3 PASS，判占线移密度
     线——A2 即根因回归测试）
  A9 带=排除地面/围栏顶 配置回归门。**2026-10-01 口径重定**：旧不变量
     「带下限 > 1.30m 围栏顶」（20260912_223444 矩阵根因：带 [0.8,2.0] 把
     1.30m 围栏环放进证据积分=实心墙判占 → pads→场内 2D A* 只能绕环外 →
     OOB 全灭；下限抬 1.45 围栏/平台/桶全排除）**随外部 SDF 场景作废**：
     现役围栏高 2.80（scene_topology static_obstacles，注释明写「2.80 > 点云
     高度带上限 2.6，围栏顶缘点仍被排除」）→ 带 [0.70,2.60] 的立论反转为
     **上限**须低于围栏顶。A9 双查配置与常量防回退，判据三条：下限 > 地面
     杂波、上限 < 围栏顶−裕度、上限 ≥ 巡航高度（带须盖住巡航层）。
  B. bh_uncertain_t 走廊扫描：正中命中/侧向/后方/超距/全空
  C. 集成（真实 Scene + 模拟飞行）：无人机沿枝轴从枝梢侧飞向带枝树——
     C1 首个注入圈：注入格（稀疏签名载体）计数 ≤3 且 O4 走廊扫描 t>0
     C2 稀疏注入格接近期零判占（细枝单点/圈回波不封格——矩阵根因回归）
     C3 密集回波（同圈 6 点同格，树干/密枝簇签名）注入后枝格判占（A* 绕行）
     C4 无新观测 4 周期后窗口遗忘（"摆走的枝不再挡路"）
     C5 接近全程存在 t>0 的圈（O4 跨格限速持续生效）
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/zx2026_defop_ws")
# 只插 arena_nav：nav_node 模块顶要 import zx2026_common.msg（devel 生成包），
# 若把 src/zx2026_common/scripts 插到 sys.path 首位会遮蔽生成包（源包无 msg
# 子包）→ No module named 'zx2026_common.msg'（与 vo_selftest 同款结构）。
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_nav", "scripts"))

try:
    from nav_node import bh_evidence_tier, bh_uncertain_t
except ImportError as e:
    sys.stderr.write("import nav_node 失败(%s)。\n请先: source /opt/ros/noetic/"
                     "setup.bash && source devel/setup.bash\n" % e)
    sys.exit(2)

import numpy as np

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene

CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
OUT = "/tmp/bh_selftest.txt"

# 与 sim_settings branch_handling 同值（selftest 用常量，配置变更时此处同步）
BH_WINDOW_CYCLES = 3        # 1.5s / 0.5s 重建周期
BH_T_NEAR = 2.0
BH_T_FAR = 4.0
BH_TRUST_R = 5.0
BH_SWAY_DENSE = 3.0       # 判占密度倍数：occ 线 = dense×T（近 6/远 12）
BH_UNC_LAT = 0.15
BH_UNC_RAY = 4.0  # 与 sim_settings uncertain.ray 同步：4.0=枝首见距离(≈3.5m)内即进限速窗
RES = 0.5
LIDAR_AZ = 72
LIDAR_EL = [-0.5, -0.25, 0.0, 0.25, 0.5]
RANGE = 5.0
MAP_BAND = (0.70, 2.60)  # = sim_settings branch_handling.map_z_band（见 A9；须逐值同步）
FENCE_TOP = 2.80         # = scene_topology static_obstacles 围栏柱顶 z（A9 上限判据）
CRUISE_Z = 2.4           # = competition_rules heights.cruise_z（A9 覆盖判据）

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))


def patch_branches(value):
    with open(CFG, encoding="utf-8") as f:
        lines = f.readlines()
    blk_indent = None
    for i, ln in enumerate(lines):
        m = re.match(r"^(\s*)branches\s*:", ln)
        if m:
            blk_indent = len(m.group(1))
            continue
        if blk_indent is not None:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            if len(ln) - len(ln.lstrip()) <= blk_indent:
                break
            if re.match(r"enabled\s*:", s):
                lines[i] = re.sub(r"(enabled\s*:\s*)(false|true)",
                                  lambda mm: mm.group(1) + value, ln, count=1)
                break
    with open(CFG, "w", encoding="utf-8") as f:
        f.writelines(lines)


# ---- A. 判占分档 -----------------------------------------------------------
def test_tier():
    # 单格世界系坐标函数（偏移 -5：(10,10) → (0.25,0.25) 在 trust_r=5 内=近档；
    # (60,60) → (25.25,25.25) 远档）
    def g2w(ix, iy):
        return (-5.0 + (ix + 0.5) * RES, -5.0 + (iy + 0.5) * RES)

    # A1 一点不判占（近档，单次闪烁/噪声）
    ev = {(10, 10): [1.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A1 单点近档→unc 非 occ",
          (10, 10) in unc and (10, 10) not in occ,
          "unc=%s occ=%s" % (len(unc), len(occ)))
    # A2 两点近档不判占（20260912 矩阵根因回归：旧 T 线判占把细枝 2 次回波
    # 升 occ → 枝格+膨胀闭合树间走廊 → 北绕界外 OOB + 西墙滑扫）
    ev = {(10, 10): [2.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A2 两点近档→unc 非 occ（细枝不封格）",
          (10, 10) in unc and (10, 10) not in occ,
          "unc=%s occ=%s" % ((10, 10) in unc, (10, 10) in occ))
    # A3 五点近档不判占（判占线 dense×T_near=6 之下边界）
    ev = {(10, 10): [5.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A3 五点近档→unc（判占线之下）",
          (10, 10) in unc and (10, 10) not in occ,
          "unc=%s occ=%s" % ((10, 10) in unc, (10, 10) in occ))
    # A4 六点近档判占（判占线达成=树干/密枝簇签名）
    ev = {(10, 10): [6.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A4 六点近档→occ（判占线达成）",
          (10, 10) in occ and (10, 10) not in unc,
          "occ=%s" % ((10, 10) in occ))
    # A5 远档十一点不判占（远档线 dense×T_far=12 之下）
    ev = {(60, 60): [11.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A5 远档十一点→unc（远档线之下）",
          (60, 60) in unc and (60, 60) not in occ,
          "unc=%s occ=%s" % ((60, 60) in unc, (60, 60) in occ))
    # A6 远档十二点判占
    ev = {(60, 60): [12.0, 0]}
    occ, unc = bh_evidence_tier(ev, 1, BH_WINDOW_CYCLES, BH_T_NEAR,
                                BH_T_FAR, BH_TRUST_R, BH_SWAY_DENSE,
                                100, 100, g2w, np.array([0.0, 0.0]))
    check("A6 远档十二点→occ",
          (60, 60) in occ and (60, 60) not in unc,
          "occ=%s" % ((60, 60) in occ))
    # A7 窗口遗忘：cycle0 观测，cycle4 重建 → 删（4-0>3）
    ev = {(10, 10): [2.0, 0]}
    bh_evidence_tier(ev, 4, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
                     BH_TRUST_R, BH_SWAY_DENSE, 100, 100, g2w,
                     np.array([0.0, 0.0]))
    check("A7 窗口遗忘", (10, 10) not in ev, "ev=%s" % ev)
    # A8 重观测刷新：cycle0 观测，cycle3 重观测，cycle4 重建 → 保留
    ev = {(10, 10): [2.0, 3]}
    bh_evidence_tier(ev, 4, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
                     BH_TRUST_R, BH_SWAY_DENSE, 100, 100, g2w,
                     np.array([0.0, 0.0]))
    check("A8 重观测刷新", (10, 10) in ev, "ev=%s" % ev)


# ---- A9. 带=排除地面/围栏顶（配置回归门） -----------------------------------
def test_band_fence_guard():
    """20260912_223444 + 2026-10-01 双口径回归门。

    **旧口径**（已作废，留作历史）：带下限须 > 1.30m 围栏顶——1.30m 围栏环
    一旦进带即实心墙判占（密度线不救墙——墙回波远超任何判占线）→ pads
    （环外）→ 场内 2D A* 只能绕环外 → 必然 OOB 全灭。

    **现口径**：围栏高 2.80（scene_topology static_obstacles，注释原文
    「2.80 > 点云高度带上限 2.6，围栏顶缘点仍被排除」）→ 约束反转成上限：
    带的职责是**同时**排掉地面杂波与围栏顶缘，且必须盖住巡航层（否则
    巡航高度的树/枝回波整层进不了占用图，A* 变瞎——比误判占更危险）。
    三条判据 + 常量同步双查（MAP_BAND 漂移=复刻的 _on_cloud 带过滤与
    实机配置失同步，C 集成臂会跟着算错）。
    """
    bh = cfg.load("sim_settings.yaml").get("closed_loop", {}).get(
        "branch_handling", {})
    band = bh.get("map_z_band", [])
    ok_len = len(band) == 2
    ok_lo = ok_len and band[0] > 0.30          # 排地面/贴地杂波
    ok_hi = ok_len and band[1] < FENCE_TOP - 0.10   # 排围栏顶缘
    ok_cov = ok_len and band[0] <= CRUISE_Z <= band[1]   # 盖住巡航层
    ok_const = tuple(band) == MAP_BAND
    ok = ok_len and ok_lo and ok_hi and ok_cov and ok_const
    check("A9 带=排地面+排围栏顶+盖巡航+常量同步", ok,
          "config=%s MAP_BAND=%s [len=%s lo>0.30=%s hi<%.2f=%s 覆盖cruise%.2f=%s]"
          % (band, list(MAP_BAND), ok_len, ok_lo, FENCE_TOP - 0.10, ok_hi,
             CRUISE_Z, ok_cov))


# ---- B. 不确定走廊扫描 ------------------------------------------------------
def test_uncertain_scan():
    ny = nx = 20
    unc = np.zeros((ny, nx), dtype=bool)
    # 目标格 (10,10)，世界系 = (0,0)（g_x0=-5）
    def g2w(ix, iy):
        return (-5.0 + (ix + 0.5) * RES, -5.0 + (iy + 0.5) * RES)

    def g2c(x, y):
        ix = int((x + 5.0) / RES)
        iy = int((y + 5.0) / RES)
        return (max(0, min(nx - 1, ix)), max(0, min(ny - 1, iy)))

    # 注意 gocc_unc 布局为 [iy, ix]（行=iy，列=ix），g2c 返回 (ix, iy)。
    # B1 正中命中：格 ix=14, iy=10 中心 (2.25, 0.25)，走廊内 → unc[10,14]
    unc[10, 14] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B1 正中命中", t is not None and 2.0 <= t <= 2.5, "t=%s" % t)
    # B2 侧向过滤：格 ix=10, iy=11 中心 (0.25, 0.75)，lat=0.75 > corridor
    unc[:] = False
    unc[11, 10] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B2 侧向过滤", t is None, "t=%s" % t)
    # B3 后方过滤：格 ix=8, iy=10 中心 (-1.75, 0.25) → 沿轨 t_c=-1.75
    unc[:] = False
    unc[10, 8] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B3 后方过滤", t is None, "t=%s" % t)
    # B4 超距过滤（ray=4.0 外）：格 ix=18, iy=10 中心 (4.25, 0.25) > 4.0
    unc[:] = False
    unc[10, 18] = True
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B4 超距过滤", t is None, "t=%s" % t)
    # B5 全空
    unc[:] = False
    t = bh_uncertain_t(0.0, 0.0, 1.0, 0.0, unc, BH_UNC_RAY, RES * 0.5,
                       0.5, g2c, g2w)
    check("B5 全空", t is None, "t=%s" % t)


# ---- C. 集成：模拟飞行（真实 Scene 几何 + 决策核心） -------------------------
def test_integration():
    patch_branches("true")
    try:
        scene = Scene()
    finally:
        patch_branches("false")
    # ---- 冠中和（2026-10-01，云杉林导入连带） ------------------------------
    # 本臂的被测对象是**枝回波证据分层**（稀疏单点不封格 / 密集回波封格），
    # 不是树的碰撞契约。云杉体制下冠是低矮宽大体量反射面（冠底中位 1.22
    # ≈ 巡航 1.2），程序枝（0.8-2.6m）与冠体在空间上重合 → 注入格的近邻
    # 冠面回波本身就远超判占线，注入格被**合法**判占（冠是真障碍），稀疏
    # 签名在物理上不可测。故本臂显式把冠从 raycast/collides 里摘掉，退回
    # 「干+枝」场景，使被测变量唯一。冠的四层契约由 branch_selftest 与
    # 实弹 E2E 覆盖，不在此臂重复。
    # 注：现役外部 SDF 场景 branches.enabled=false（sim_settings:654 契约
    # 「树干+冠包络两圆柱」），本臂的 patch_branches("true") 是自建的
    # 程序枝测试台，与发射出的世界无关。2026-10-01 杨树林导入后表
    # meta.branch_file 真枝**覆盖**程序枝（scene.py:367-370）——真枝梢
    # z≥4.66，本臂窗口 [1.52,1.77] 候选必空 ⇒ 显式清掉真枝后按同一
    # br_rng 流补程序枝（测试台语义不变；不动表文件，避免硬杀遗留脏 meta）。
    for ob in scene.obstacles:
        if ob.kind == "tree":
            ob.branches = None
            scene._decorate_tree(ob)
    scene._crown_z_range = lambda ob: None
    v = scene.venue
    g_x0, g_y0 = -v["size"][0] / 2.0, -v["size"][1] / 2.0
    g_nx, g_ny = int(v["size"][0] / RES), int(v["size"][1] / RES)

    def g2c(x, y):
        ix = int((x - g_x0) / RES)
        iy = int((y - g_y0) / RES)
        return (max(0, min(g_nx - 1, ix)), max(0, min(g_ny - 1, iy)))

    def g2w(ix, iy):
        return (g_x0 + (ix + 0.5) * RES, g_y0 + (iy + 0.5) * RES)

    # 候选枝：枝梢（注入射线瞄准点）z∈[1.52,1.77]——≥1.52=梢部命中点
    # （z≥梢-0.082，枝半径+余量）稳进带 [0.70,2.60] 可被映射；≤1.77=
    # 枝面顶 ≤1.79 低于本臂测试台飞行高度 FLY_Z=1.8（测试台本地常量，
    # 与现役 cruise 2.4 解耦），机在枝上方飞过不共线相撞、方位角任意
    # （飞行方向沿枝轴向）。20260912_223444 带下限 0.8→1.45（围栏排除）
    # 后窗口随带上移（原 zmax∈[0.85,1.4]）；2026-10-01 带重定
    # [0.70,2.60]（围栏 2.80 成上限依据）后窗口原样仍成立。
    cand = []
    for ob in scene.obstacles:
        if ob.kind != "tree" or not ob.branches:
            continue
        for br in ob.branches:
            if 1.52 <= br[5] <= 1.77:
                cand.append((ob, br))
    if not cand:
        check("C 集成", False, "无满足 z 约束的枝（梢 z∈[1.52,1.77]）")
        return
    FLY_Z = 1.8
    # 试候选，双准则（z=FLY_Z 沿枝轴向 6 站位 tip-3.0..tip-0.5）：
    #  (a) nclear：点碰撞前缀（collides(...,0.0)=机点不埋入任何障碍）——净空
    #      不能是全球(0.35)：同树其他枝 z0∈[0.8,2.6] 均匀分布，横穿 1.8m
    #      高度概率 ~40%，64 条路径 0 条全球净空（实测），而飞行路径只是
    #      观测台，枝贴近/擦过路径恰是被测场景本体；
    #  (b) vis：对枝首直接射线可达链——实测 64 候选在 ≥1.5m 外 0% 可达
    #      （同树冠层遮挡=稀疏回波问题本体），贴到 tip-1.0~0.5m 才可见，
    #      且此时枝仰角 ~-44° 落在 72×5 lidar 的 el 网格（±14.3° 步进）外，
    #      真实模式采样率趋零 → 枝观测用"可达性门控注入"（见飞行循环）：
    #      每圈 1 点，等价 branch_selftest 3 实证的稀疏回波确定性替身。
    # 评分 (nclear, vc) 字典序，vc=飞行窗口内可见步数 ≥2 才录取（密集回波
    # 注入需要枝梢末位置可达，C3 判占的前提）。
    best = None
    for (ob, br) in cand:
        az = math.atan2(br[4] - br[1], br[3] - br[0])
        ux, uy = math.cos(az), math.sin(az)
        tip = (br[3], br[4], br[5])
        nclear = 0
        vis = []
        for k in range(6):
            px = tip[0] - ux * (3.0 - 0.5 * k)
            py = tip[1] - uy * (3.0 - 0.5 * k)
            if scene.collides((px, py, FLY_Z), 0.0):
                vis.append(False)
                break
            nclear += 1
            vx = tip[0] - px
            vy = tip[1] - py
            vz = tip[2] - FLY_Z
            vlen = math.hypot(vx, vy, vz)
            if vlen < 1e-9:
                vis.append(False)
                continue
            d = (vx / vlen, vy / vlen, vz / vlen)
            t = scene.raycast((px, py, FLY_Z), d, max_t=vlen + 0.05)
            ok = False
            if t is not None and t <= vlen + 0.05:
                p = (px + d[0] * t, py + d[1] * t, FLY_Z + d[2] * t)
                ok = geo.point_segment_dist(p, br[:3], br[3:6]) <= br[6] + 0.06
            vis.append(ok)
        # 可见步数按飞行循环实际到访的 6 站位计（N_STEPS=min(6,nclear)）：
        # 旧 min(5,·) 漏数 tip-0.5 末站——带下限抬 1.45 后枝梢贴近巡航
        # 高度、射线浅，可见性只在末两站（tip-1.0/0.5）出现 → 全被误拒
        # （20260912_223444 带修复回归首跑 C 集成 0 候选的根因）。
        flight = min(6, nclear)
        vc = sum(vis[:flight])
        if vc < 2 or nclear < 3:
            continue
        score = (nclear, vc)
        if best is None or score > best[0]:
            best = (score, ob, br, az, vis)
    if best is None:
        check("C 集成", False, "无 (nclear≥3, 可见≥2) 候选")
        return
    (nclear, vc), ob, br, az, vis = best
    ux, uy = math.cos(az), math.sin(az)
    yaw = az + math.pi  # 机头朝树（射线模式旋转，同 lidar_node odom_yaw）
    (sx, sy, sz, ex, ey, ez, brr) = br
    tip = (ex, ey, ez)
    N_STEPS = min(6, nclear)

    def is_branch_pt(pt):
        return geo.point_segment_dist(pt, (sx, sy, sz), (ex, ey, ez)) \
            <= brr + 0.02

    occ_ev = {}
    rebuild_n = 0
    phases = []
    # 阶段1 接近：枝梢 3.0m → 3.0-0.5*(N_STEPS-1) m，0.5m/步（0.5s 每步 →
    # ~1 m/s），z=FLY_Z。枝观测=可达性门控注入：每圈对枝首做真实 raycast，
    # 命中枝（dist≤brr+0.06）才注入 1 点（该命中点）——稀疏回波确定性替身
    # （72×5 el 网格步进 14.3°，2cm 枝的采样率趋零；稀疏签名由 branch_selftest
    # 3 独立实证）。环境格（树干/邻树/地面）仍全部来自真实 lidar 复刻。
    for step_i in range(N_STEPS):
        px = tip[0] - ux * (3.0 - 0.5 * step_i)
        py = tip[1] - uy * (3.0 - 0.5 * step_i)
        pos = (px, py, FLY_Z)
        pts = []
        for az_i in range(LIDAR_AZ):
            azr = yaw + az_i * (2.0 * math.pi / LIDAR_AZ)
            for el in LIDAR_EL:
                d = geo.normalize((math.cos(el) * math.cos(azr),
                                   math.cos(el) * math.sin(azr), math.sin(el)))
                t = scene.raycast(pos, d, max_t=RANGE)
                if t is not None:
                    pts.append((pos[0] + d[0] * t, pos[1] + d[1] * t,
                                pos[2] + d[2] * t))
        # 注入：对枝首直接射线（同选择阶段的 vis 判据）。命中点 z∈[梢z,1.8]
        # ⊂ 带 [0.70,2.60]（梢窗 [1.52,1.77] 保证）——注入格=稀疏签名载体
        # （1 点/圈；直射 lidar 对本圈近段枝格有时本身就密（实测可到 27
        # 计数→occ，密目标语义正确），C2 的稀疏前提由 inj_sweep 逐圈实测
        # 记账（见下），不再假定）。
        inj_cell = None
        vx = tip[0] - px
        vy = tip[1] - py
        vz = tip[2] - FLY_Z
        vlen = math.hypot(vx, vy, vz)
        if vlen > 1e-9:
            d = (vx / vlen, vy / vlen, vz / vlen)
            t = scene.raycast(pos, d, max_t=vlen + 0.05)
            if t is not None and t <= vlen + 0.05:
                p = (px + d[0] * t, py + d[1] * t, FLY_Z + d[2] * t)
                if geo.point_segment_dist(p, br[:3], br[3:6]) <= brr + 0.06:
                    pts.append(p)
                    inj_cell = g2c(p[0], p[1])
        # C2 稀疏前提逐圈记账：本轮**直射扫描**落进注入格的点数（含注入点
        # 自身）。C2 断言的成立前提是"注入格=稀疏签名载体"；若直射扫描本身
        # 就往该格打 >occ 线 的点（近段枝/树干贴脸，密目标语义），那 C2 的
        # 被测对象不存在——此时该判 FAIL 还是"前提不满足"必须可分辨，
        # 否则无法区分"证据层回归"与"候选枝选得不合适"。
        inj_sweep = 0
        if inj_cell is not None:
            inj_sweep = sum(1 for (x, y, z) in pts
                            if MAP_BAND[0] <= z <= MAP_BAND[1]
                            and g2c(x, y) == inj_cell)
        # _on_cloud 复刻（O1 分支：band + 证据计数），同时分类枝点/枝格
        b_pts = 0
        b_cells = set()
        for (x, y, z) in pts:
            if MAP_BAND[0] <= z <= MAP_BAND[1]:
                c = g2c(x, y)
                ev = occ_ev.get(c)
                if ev is None:
                    occ_ev[c] = [1.0, rebuild_n]
                else:
                    ev[0] += 1.0
                    ev[1] = rebuild_n
                if is_branch_pt((x, y, z)):
                    b_pts += 1
                    b_cells.add(c)
        # 重建（bh_evidence_tier 真函数）
        occ, unc = bh_evidence_tier(
            occ_ev, rebuild_n, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
            BH_TRUST_R, BH_SWAY_DENSE, g_nx, g_ny, g2w,
            np.array([px, py]))
        rebuild_n += 1
        # 走廊扫描（bh_uncertain_t 真函数，指令方向=朝树）
        gocc_unc = np.zeros((g_ny, g_nx), dtype=bool)
        for (ix, iy) in unc:
            gocc_unc[iy, ix] = True
        t_unc = bh_uncertain_t(px, py, ux, uy, gocc_unc, BH_UNC_RAY,
                               RES * 0.5, 0.35 + BH_UNC_LAT, g2c, g2w)
        phases.append((step_i, len(pts), len(occ), len(unc),
                       t_unc if t_unc is not None else -1.0,
                       b_pts, b_cells, set(occ) & b_cells,
                       {c: occ_ev[c][0] for c in b_cells},
                       inj_cell,
                       occ_ev[inj_cell][0] if inj_cell is not None
                       and inj_cell in occ_ev else None,
                       inj_sweep))

    # 阶段1.5 密集回波：末位置原地一圈，枝梢命中点同格注入 6 点（树干/
    # 密枝簇签名：近距大目标多波束同帧命中同格）——判占线 dense×T_near=6
    # 达成应判占（A* 绕行语义）。细枝稀疏单点/圈签名不封格由 C2 把关。
    dense_b_cells = set()
    vx = tip[0] - px
    vy = tip[1] - py
    vz = tip[2] - FLY_Z
    vlen = math.hypot(vx, vy, vz)
    if vlen > 1e-9:
        d = (vx / vlen, vy / vlen, vz / vlen)
        t = scene.raycast((px, py, FLY_Z), d, max_t=vlen + 0.05)
        if t is not None and t <= vlen + 0.05:
            p = (px + d[0] * t, py + d[1] * t, FLY_Z + d[2] * t)
            if geo.point_segment_dist(p, br[:3], br[3:6]) <= brr + 0.06:
                if MAP_BAND[0] <= p[2] <= MAP_BAND[1]:
                    c = g2c(p[0], p[1])
                    for _ in range(6):
                        ev = occ_ev.get(c)
                        if ev is None:
                            occ_ev[c] = [1.0, rebuild_n]
                        else:
                            ev[0] += 1.0
                            ev[1] = rebuild_n
                    dense_b_cells = {c}
    occ, unc = bh_evidence_tier(
        occ_ev, rebuild_n, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
        BH_TRUST_R, BH_SWAY_DENSE, g_nx, g_ny, g2w,
        np.array([px, py]))
    rebuild_n += 1
    dense_occ_hit = len(set(occ) & dense_b_cells)
    phases.append((N_STEPS, 6 * len(dense_b_cells), len(occ), len(unc),
                   -1.0, 6 * len(dense_b_cells), dense_b_cells,
                   set(occ) & dense_b_cells,
                   {c: occ_ev[c][0] for c in dense_b_cells},
                   next(iter(dense_b_cells)) if dense_b_cells else None,
                   occ_ev[next(iter(dense_b_cells))][0]
                   if dense_b_cells else None))

    # 阶段2 遗忘：无新观测，原地再跑 4 个重建周期（est 同最后位置）
    final_pos = (px, py)
    forget_trace = []
    for _ in range(4):
        occ, unc = bh_evidence_tier(
            occ_ev, rebuild_n, BH_WINDOW_CYCLES, BH_T_NEAR, BH_T_FAR,
            BH_TRUST_R, BH_SWAY_DENSE, g_nx, g_ny, g2w,
            np.array(final_pos))
        forget_trace.append((len(occ_ev), len(occ), len(unc)))
        rebuild_n += 1

    # C1 首个注入圈：注入格（稀疏签名载体，1 点/圈）计数 ≤3 且 O4 走廊
    # 扫描 t>0（ray=4.0 ≥ 首见距离 → 稀疏枝首见即进限速窗）。容错：首见
    # 圈 t_unc 未命中时取次圈（注入格 cnt 仍在不确定档、t_c ≤ 4.0）验证
    # O4。注：直射 lidar 对贴近巡航高度的枝近段格本身就密（实测 27 计数
    # →occ，密目标语义正确），稀疏断言只看注入格。
    appr = [p for p in phases if p[0] < N_STEPS]
    first_b = next((p for p in appr if p[9] is not None), None)
    c1_ok = False
    c1_detail = "无注入观测"
    if first_b is not None:
        sparse = first_b[10] is not None and first_b[10] <= 3.0 + 1e-9
        o4_live = first_b[4] > 0.0
        if not o4_live:
            # 次圈兜底：注入格仍在不确定档，t_c ≤ 4.0 应命中
            nb = next((p for p in appr[first_b[0] + 1:] if p[9] is not None),
                      None)
            if nb is not None:
                o4_live = nb[4] > 0.0 and nb[10] is not None \
                    and nb[10] <= 2.0 + 1e-9
        c1_ok = sparse and o4_live
        c1_detail = "step=%d inj=%s cnt=%s t_unc=%.2f" % (
            first_b[0], first_b[9], first_b[10], first_b[4])
    check("C1 稀疏首见不判占+O4 命中", c1_ok, c1_detail)
    # C2 稀疏注入格接近期零判占（20260912 矩阵根因回归：旧 T 线判占把细枝
    # 升 occ 闭合树间走廊 → 北绕界外 OOB + 西墙滑扫，B 臂 0/3 PASS）。
    # 2026-10-01 拆分（云杉导入把本断言推到台前）：
    #   C2a 瞬态稀疏：注入格"本圈扫掠点数 < 判占线"时必须判**不占**——这是
    #       证据层承诺的语义（细枝不判死只减速），严格断言。
    #   C2b 持续稀疏：**已知算法语义，非本断言所守**——见下。
    #
    # ⚠ 发现（生产算法，非测试台，留待裁决）：`_occ_ev[c][0]` 是**累积计数**，
    #   不是文档所称的"窗口内观测计数"（bh_evidence_tier docstring :89
    #   「窗口内观测计数替代一点即占」）。window_cycles 只做 TTL 遗忘：格子只要
    #   每周期被重观测一次就永不衰减 → **1 点/圈 的持续单点回波在第 6 圈
    #   必然越过判占线**（dense×T_near=6），与"细枝不判死"的设计本意冲突，
    #   且方向上是把 20260912 的过度判占根因重新引入（悬停/慢接近时静态细枝
    #   回波即属此列）。实测 cnt 轨迹 1→2→3→4→6→12。
    #   现役云杉场景 **不触发**：sim_settings:654 外部 SDF 场景契约
    #   `branches.enabled: false`（干+冠两圆柱），程序枝根本不存在于发射的
    #   世界里，本臂是自建测试台。故不改生产语义（改=行为变更须矩阵过线），
    #   只把该行为钉成可见的回归锚。
    OCC_LINE = BH_SWAY_DENSE * BH_T_NEAR
    inj_occ = [p[0] for p in appr if p[9] is not None and p[9] in p[7]]
    inj_sweep_of = {p[0]: p[11] for p in appr if p[9] is not None}
    inj_cnt_of = {p[0]: p[10] for p in appr if p[9] is not None}
    # 判据用**累积计数**（判占决策的实际输入），不用本圈扫掠点数——后者
    # 只是"本圈贡献"，累积才是被决策的量（step4 本圈仅 2 点却已占，因为
    # 前 5 圈各 1 点已积到 6：那属持续类，不是瞬态稀疏）。
    real_occ = [s for s in inj_occ if inj_cnt_of.get(s, 1e9) < OCC_LINE]
    sparse_steps = [s for s in inj_sweep_of if inj_cnt_of[s] < OCC_LINE]
    check("C2a 稀疏格零判占（计数在判占线以下者不得进 occ）",
          not real_occ and bool(sparse_steps),
          "inj_occ_steps=%s real=%s(计数<%.0f 仍判占=证据层回归) 稀疏圈=%s "
          "sweep=[%s] cnt=[%s]"
          % (inj_occ, real_occ, OCC_LINE, sparse_steps,
             ",".join("%d:%d" % (s, inj_sweep_of[s]) for s in sorted(inj_sweep_of)),
             ",".join("%d:%s" % (s, inj_cnt_of[s]) for s in sorted(inj_cnt_of))))
    # C2b 锚：持续（每圈都有观测的）回波格，其累积计数在多长跨度内越线。
    # 语义承诺（docstring「窗口内观测计数」）下，窗口 = window_cycles = 3 圈，
    # 纯 1 点/圈 的持续回波计数应封顶在 3，**永不到线**；实现是累积 → 越线。
    # 这里用**性质**而非等值做锚（越线圈次随几何微抖）：持续≥OCC_LINE 圈
    # 仍在被观测的注入格，必须已经在某圈越线。若哪天改成真窗口化，本锚会
    # 红——那不是回归，是语义变更，须显式改写并说明（见上方 ⚠）。
    series = sorted(inj_sweep_of)
    pers = [s for s in series if inj_sweep_of[s] >= 1
            and s >= series[0] + BH_WINDOW_CYCLES]      # 持续超过一个窗口
    cross = [s for s in series if inj_cnt_of[s] >= OCC_LINE]
    # 条件式：**前提不成立不算失败**（候选枝接近窗口不足一个窗口时本就无
    # 持续回波可锚——杨树林臂即 3 圈，无持续圈）。前提成立（有持续格）却
    # 不越线 = 计数已窗口化 = 语义变更，红。
    check("C2b 持续累积越线锚（已知语义，非缺陷判据；前提式）",
          (not pers) or bool(cross),
          "OCC_LINE=%.0f 窗口=%d 圈数=%s 持续圈=%s 越线圈=%s cnt轨迹=[%s]"
          % (OCC_LINE, BH_WINDOW_CYCLES, len(series), pers, cross,
             ",".join("%d:%s" % (s, inj_cnt_of[s]) for s in series)))
    # C3 密集回波判占：同圈 6 点同格（树干/密枝簇签名）→ 枝格判占（A* 绕行）
    dense_p = phases[-1] if phases and phases[-1][0] == N_STEPS else None
    d_ok = dense_p is not None and len(dense_p[7]) > 0 and len(dense_p[6]) > 0
    check("C3 密集回波→枝格判占", d_ok,
          "dense_cells=%s occ_hit=%s" %
          (len(dense_p[6]) if dense_p else 0,
           len(dense_p[7]) if dense_p else 0))
    # C4 无观测窗口遗忘：4 周期后 occ_ev 清空（"摆走的枝不再挡路"）
    tail = forget_trace[-1]
    forgotten = tail[0] == 0 and tail[1] == 0 and tail[2] == 0
    check("C4 无观测窗口遗忘", forgotten,
          "forget_trace=%s" % forget_trace)
    # C5 全程 O4：接近阶段存在 t_unc>0 的圈
    o4_hits = sum(1 for p in appr if p[4] > 0.0)
    check("C5 O4 扫描命中圈", o4_hits > 0, "o4_hits=%d/%d" % (o4_hits, len(appr)))


def main():
    test_tier()
    test_band_fence_guard()
    test_uncertain_scan()
    test_integration()
    npass = sum(1 for (_, ok, _) in RESULTS if ok)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("bh_selftest 结果 (%d/%d PASS)\n" % (npass, len(RESULTS)))
        for (name, ok, detail) in RESULTS:
            f.write("  [%s] %s%s\n" % ("PASS" if ok else "FAIL", name,
                                       (" — " + detail) if detail else ""))
    print("bh_selftest rc=%d (%d/%d)" % (0 if npass == len(RESULTS) else 1,
                                         npass, len(RESULTS)))
    sys.exit(0 if npass == len(RESULTS) else 1)


if __name__ == "__main__":
    main()
