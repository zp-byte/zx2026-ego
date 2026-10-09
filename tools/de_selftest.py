#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/de_selftest.py — _de_pick_dir / _de_tick 独立自测（不依赖 ROS 运行时，只 import）。

用合成占用栅格验证死端逃逸方向评分的 7 个场景：
  S0 空地：应选朝 goal 方向
  S1 碰撞标记壳包住自机：应穿壳逃出（pierce 预算内）
  S2 壳 + goal 侧堵墙：应绕开被堵方向
  S3 四面全堵（ray 内不可行）：应返回 None（悬停重试）
  S4 靠场地边界：不应逃向边界外
  S5 (P2) goal 对角沿线上有点云真树：该方向被走廊检查淘汰
  S6 (P2) 点云在走廊外：对角方向不受罚
  S7 (P2) 四周沿线上全是真树点云：全淘汰 → None 悬停
起 2 撞点淘汰（contact_block_r）：
  S8 壳内 0.35m 处有撞点（=真树）在 goal 对角上：对角方向被淘汰，改选背离/切向
  S9 同场景 contact_block_r=0（默认关）：逐位复现旧行为（与 S1 同解）
  S10 同 S8 但撞点 t 已过 TTL：不淘汰，回到 S1 解
  S11 撞点在本机后方（t<=0）：不淘汰
起 2 撞点推断 contact_point_of（纯函数，两路）：
  L1 点云确认（clr ≤ block_r）：接触点 == est + u*clr（逐位等于那个点云点）
  L2 盲撞（clr > block_r）且水平速度够：接触点 == est + dr*v̂
  L3 盲撞但水平速度 < blind_v：None（低速贴树挤入方向无意义）
  L4 blind_v=0（本路关）：None，即使速度够
  L5 block_r=0（总关）：None
  L6 无点云（clr_u=None）+ 速度够：走盲撞路
状态机（P5 退出迟滞，显式时刻驱动 _de_tick）：
  H1 正常迟滞退出：规划恢复后仍保持逃逸 ≥exit_hyst_s 才交还
  H2 迟滞窗口内规划再失败：计时清零重计
  H3 规划快速翻转（成功窗口 <confirm_s）：应根本不触发
"""
import importlib.util
import math
import os
import types

import numpy as np

# 本仓约定：按 __file__ 定位，绝不写死工作区绝对路径（旧版写死
# ~/zx2026_arena_ws ⇒ 该副本自 2026-09-29 起已是陈旧拷贝，本自测此前一直
# 在测另一个工作区的 nav_node，对本仓改动全盲）。
_HERE = os.path.dirname(os.path.abspath(__file__))
_NV = os.path.join(_HERE, "..", "src", "arena_nav", "scripts", "nav_node.py")
spec = importlib.util.spec_from_file_location("nav_node", _NV)
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)
H = nav.NavNode
# 无 ROS 运行时：撞点 TTL 剪枝读 rospy.get_time()，打桩为固定钟
nav.rospy.get_time = lambda: 0.0


def make_self(occ, est, goal, res=0.5, cloud=None, contacts=None, cblock_r=0.0):
    s = types.SimpleNamespace()
    ny, nx = occ.shape
    s._gocc_infl = occ
    s.est_pos = np.array(est)
    s.goal = goal
    s.res = res
    s.g_x0 = -nx * res / 2.0
    s.g_y0 = -ny * res / 2.0
    s.g_nx = nx
    s.g_ny = ny
    s._de_ndirs = 16
    s._de_pierce = 8
    s._de_free_need = 3
    s._de_goal_bias = 1.5
    s._de_ray = 3.0
    s._de_cloud_r = 1.0
    # 起 2 撞点淘汰桩：contacts=[(x,y,t)]，cblock_r=0 即关（默认，逐位原行为）
    s._de_contacts = list(contacts) if contacts else []
    s._de_cblock_r = cblock_r
    s._de_cblock_ttl = 12.0
    s._de_cblock_n = 0
    # M2 bounce_corridor 桩补（落地时未同步 selftest，存量破损修复）
    s._bc_enabled = False
    s._bc_w = 1.0
    # W1 rescue_quiet 桩补（de 退出爬坡窗钩子读旗；关=不置窗）
    s._swq_enabled = False
    s._swq_ramp = 2.5
    s._swq_until = -1e9
    s.scene = types.SimpleNamespace(drone_radius=0.25)
    s.odom = np.array([est[0], est[1], 2.5])
    s.cloud = cloud if cloud is not None else []
    s._g_to_cell = lambda x, y, _s=s: H._g_to_cell(_s, x, y)
    return s


def ang_of(d):
    return math.atan2(d[1], d[0])


# S0 空地
occ0 = np.zeros((20, 20), dtype=bool)
d0 = H._de_pick_dir(make_self(occ0, (0.0, 0.0), (8.0, 8.0)))
print("S0 dir:", d0, "ang=%.3f" % ang_of(d0))
assert d0 is not None and abs(ang_of(d0) - math.pi / 4) < 0.5, "S0 fail"

# S1 5x5 碰撞标记壳（世界系 [-1.0,1.5]^2），自机在中心，goal 对角
occ1 = np.zeros((20, 20), dtype=bool)
occ1[8:13, 8:13] = True
d1 = H._de_pick_dir(make_self(occ1, (0.0, 0.0), (8.0, 8.0)))
print("S1 dir:", d1, "ang=%.3f" % ang_of(d1))
assert d1 is not None and abs(ang_of(d1) - math.pi / 4) < 0.5, "S1 fail"

# S2 壳 + +x 侧整排堵死（模拟树墙），goal 在 +x
occ2 = np.zeros((20, 20), dtype=bool)
occ2[8:13, 8:13] = True
occ2[8:13, 13:20] = True
d2 = H._de_pick_dir(make_self(occ2, (0.0, 0.0), (9.0, 0.0)))
print("S2 dir:", d2, "ang=%.3f" % ang_of(d2))
assert d2 is not None and not (-0.5 < ang_of(d2) < 0.5), "S2 fail"

# S3 10x10 大堵死区（ray 内不可行）
occ3 = np.zeros((20, 20), dtype=bool)
occ3[5:15, 5:15] = True
d3 = H._de_pick_dir(make_self(occ3, (0.0, 0.0), (9.0, 9.0)))
print("S3 dir:", d3)
assert d3 is None, "S3 fail"

# S4 自机靠 +x 场地边界（hx=5.0），goal 在远处 -x
occ4 = np.zeros((20, 20), dtype=bool)
occ4[8:13, 8:13] = True
d4 = H._de_pick_dir(make_self(occ4, (4.4, 0.0), (-9.0, 0.0)))
print("S4 dir:", d4, "ang=%.3f" % ang_of(d4))
assert d4 is not None and abs(ang_of(d4)) > 1.0, "S4 fail"

# S5 (P2 修复) 壳场景 + goal 对角沿线上 0.7m 有点云真树：
# 对角方向（±22.5° 邻域同被走廊扫到）被点云淘汰，应改选其他可行方向
occ5 = np.zeros((20, 20), dtype=bool)
occ5[8:13, 8:13] = True
d5 = H._de_pick_dir(make_self(
    occ5, (0.0, 0.0), (8.0, 8.0),
    cloud=[(0.5, 0.5, 2.5)]))
print("S5 dir:", d5, "ang=%.3f" % ang_of(d5))
assert d5 is not None and abs(ang_of(d5) - math.pi / 4) > 0.3, "S5 fail"

# S6 (P2 修复) 同壳但点云点在走廊外（垂轨 0.53 > 0.35）：
# 对角方向不受罚，仍应选 goal 对角
d6 = H._de_pick_dir(make_self(
    occ5, (0.0, 0.0), (8.0, 8.0),
    cloud=[(0.05, 0.8, 2.5)]))
print("S6 dir:", d6, "ang=%.3f" % ang_of(d6))
assert d6 is not None and abs(ang_of(d6) - math.pi / 4) < 0.3, "S6 fail"

# S7 (P2 修复) 四周沿线上 1.0m 全是真树点云：全方向被点云淘汰 → None 悬停
# （12 点/30° 间隔：任意 16 向方向上最大垂轨 1.0·sin(15°)=0.26 < 走廊 0.35）
d7 = H._de_pick_dir(make_self(
    occ5, (0.0, 0.0), (8.0, 8.0),
    cloud=[(math.cos(a), math.sin(a), 2.5) for a in
           [2 * math.pi * k / 12 for k in range(12)]]))
print("S7 dir:", d7)
assert d7 is None, "S7 fail"

# ---- 起 2 撞点淘汰（contact_block_r）----------------------------------------
# 复刻现场形态：自机在对称接触壳中心，真树在壳内部 ~0.35m 处、恰在 goal 对角上。
_HIT = (0.247, 0.247)   # ≈0.35m，方向 = goal 对角（45°），在壳 [-1.0,1.5]^2 内部

# S8 开：对角被撞点走廊淘汰 ⇒ 不得再选对角（治"推回同一棵树"）
s8 = make_self(occ1, (0.0, 0.0), (8.0, 8.0),
               contacts=[(_HIT[0], _HIT[1], 0.0)], cblock_r=1.0)
d8 = H._de_pick_dir(s8)
print("S8 dir:", d8, "ang=%.3f" % ang_of(d8), "cblock=%d" % s8._de_cblock_n)
assert d8 is not None, "S8 fail: 应仍有可行方向"
assert abs(ang_of(d8) - math.pi / 4) > 0.6, "S8 fail: 仍朝刚撞上的障碍选向"
assert s8._de_cblock_n > 0, "S8 fail: 撞点淘汰未生效（计数为 0）"

# S9 关（默认）：同一撞点在表内但 contact_block_r=0 ⇒ 逐位复现 S1
s9 = make_self(occ1, (0.0, 0.0), (8.0, 8.0),
               contacts=[(_HIT[0], _HIT[1], 0.0)], cblock_r=0.0)
d9 = H._de_pick_dir(s9)
print("S9 dir:", d9, "ang=%.3f" % ang_of(d9), "cblock=%d" % s9._de_cblock_n)
assert d9 == d1, "S9 fail: 关位未逐位复现旧行为"
assert s9._de_cblock_n == 0, "S9 fail: 关位不应有淘汰计数"

# S10 TTL 过期：撞点记录 t=-100（> TTL 12）⇒ 不淘汰，同 S1
s10 = make_self(occ1, (0.0, 0.0), (8.0, 8.0),
                contacts=[(_HIT[0], _HIT[1], -100.0)], cblock_r=1.0)
d10 = H._de_pick_dir(s10)
print("S10 dir:", d10, "ang=%.3f" % ang_of(d10), "cblock=%d" % s10._de_cblock_n)
assert abs(ang_of(d10) - math.pi / 4) < 0.5, "S10 fail: TTL 过期仍被淘汰"
assert s10._de_cblock_n == 0, "S10 fail: TTL 过期不应淘汰"

# S11 撞点在本机后方：对 t<=0 的方向不构成淘汰（前向不受影响，仍选 goal 对角）
s11 = make_self(occ1, (0.0, 0.0), (8.0, 8.0),
                contacts=[(-0.247, -0.247, 0.0)], cblock_r=1.0)
d11 = H._de_pick_dir(s11)
print("S11 dir:", d11, "ang=%.3f" % ang_of(d11), "cblock=%d" % s11._de_cblock_n)
assert abs(ang_of(d11) - math.pi / 4) < 0.5, "S11 fail: 后方撞点应不影响前向选向"


# ---- 起 2 撞点推断 contact_point_of（模块级纯函数）---------------------------
CP = nav.contact_point_of
DR = 0.25


def near(a, b, tol=1e-9):
    return a is not None and abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


# L1 点云确认：clr=0.32 ≤ block_r=1.0 ⇒ 接触点 = est + u*clr
p, why = CP(0.0, 0.0, 0.32, (0.6, 0.8, -0.1), 0.0, 0.0, 1.0, 0.15, DR)
print("L1", p, why)
assert near(p, (0.192, 0.256)) and why == "cloud", "L1 fail"

# L2 盲撞 + 速度够：clr=2.38 > 1.0 ⇒ 接触点 = est + dr*v̂（正东 1.0）
p, why = CP(0.0, 0.0, 2.38, (1.0, 0.0, 0.0), 1.0, 0.0, 1.0, 0.15, DR)
print("L2", p, why)
assert near(p, (DR, 0.0)) and why == "blind", "L2 fail"

# L2b 斜行进：v=(3,4) → |v|=5 → 接触点 = est + dr*(0.6,0.8)
p, why = CP(0.0, 0.0, 9.9, None, 3.0, 4.0, 1.0, 0.15, DR)
print("L2b", p, why)
assert near(p, (DR * 0.6, DR * 0.8)) and why == "blind", "L2b fail"

# L3 盲撞但速度 < blind_v：None（方向无意义）
p, why = CP(0.0, 0.0, 2.38, (1.0, 0.0, 0.0), 0.05, 0.0, 1.0, 0.15, DR)
print("L3", p, why)
assert p is None, "L3 fail"

# L4 blind_v=0（本路关）：None，即使速度够
p, why = CP(0.0, 0.0, 2.38, (1.0, 0.0, 0.0), 1.0, 0.0, 1.0, 0.0, DR)
print("L4", p, why)
assert p is None, "L4 fail"

# L5 block_r=0（总关）：None
p, why = CP(0.0, 0.0, 0.32, (0.6, 0.8, 0.0), 1.0, 0.0, 0.0, 0.15, DR)
print("L5", p, why)
assert p is None, "L5 fail"

# L6 无点云（clr_u=None）+ 速度够：走盲撞路
p, why = CP(1.0, 2.0, 1e9, None, 0.0, 1.0, 1.0, 0.15, DR)
print("L6", p, why)
assert near(p, (1.0, 2.0 + DR)) and why == "blind", "L6 fail"

# S12 端到端（=drone5 现场形态）：goal 正东、机正东行进、障碍在正东但**点云看漏**。
# 盲撞路给出接触点 → 逃逸不得再选正东（否则就是"推回同一棵树"）。
_c, _w = CP(0.0, 0.0, 2.38, (1.0, 0.0, 0.0), 1.0, 0.0, 1.0, 0.15, 0.25)
print("S12 推断出的撞点:", _c, _w)
s12 = make_self(occ1, (0.0, 0.0), (8.0, 0.0),
                contacts=[(_c[0], _c[1], 0.0)], cblock_r=1.0)
d12 = H._de_pick_dir(s12)
print("S12 dir:", d12, "ang=%.3f" % ang_of(d12), "cblock=%d" % s12._de_cblock_n)
assert d12 is not None, "S12 fail: 应仍有可行方向"
assert abs(ang_of(d12)) > 0.6, "S12 fail: 仍朝盲撞方向（正东）选向"
assert s12._de_cblock_n > 0, "S12 fail: 淘汰未生效"


# ---- P5 退出迟滞状态机（_de_tick，显式时刻驱动；rospy 日志打桩） ------------
_logs = []
nav.rospy.loginfo = lambda fmt, *a: _logs.append(str(fmt) % a if a else str(fmt))
nav.rospy.loginfo_throttle = lambda p, fmt, *a: _logs.append(str(fmt) % a if a else str(fmt))


def make_tick_self():
    """_de_tick 所需最小状态（数值与 sim_settings 默认一致）。"""
    s = types.SimpleNamespace()
    s.drone_id = 0
    s._de_active = False
    s._de_dir = None
    s._de_t0 = -1e9
    s._de_total_t0 = -1e9
    s._de_fail_t0 = None
    s._de_ok_t0 = None
    s._de_confirm = 0.5
    s._de_exit_hyst = 2.5
    s._de_dwell = 1.2
    s._plan_fail_since = None
    # W7-C1 撞后镇定窗桩补（_de_tick 逃逸分支读该值；-1e9=窗未开）
    s._ph_hold_until = -1e9
    # 起 2 撞点淘汰桩补（_de_tick engage/重选日志读这两项）
    s._de_contacts = []
    s._de_cblock_n = 0
    # W1 rescue_quiet 桩补（de 退出爬坡窗钩子读旗；关=不置窗）
    s._swq_enabled = False
    s._swq_ramp = 2.5
    s._swq_until = -1e9
    s._de_pick_dir = lambda now=None: (1.0, 0.0)
    s._de_dir_str = types.MethodType(H._de_dir_str, s)
    return s


def H1_normal_exit():
    """规划恢复后保持逃逸满 2.5s 才交还；窗口内不提前退出。"""
    s = make_tick_self()
    s._plan_fail_since = 100.0
    H._de_tick(s, 100.0)                      # 失败计时起点
    assert not s._de_active, "H1: 提前触发"
    H._de_tick(s, 100.6)                      # 0.6s >= confirm → 触发
    assert s._de_active, "H1: 应触发"
    s._plan_fail_since = None                 # 规划恢复（单次成功窗口）
    H._de_tick(s, 101.0)
    assert s._de_active, "H1: 单次成功不应立即退出"
    H._de_tick(s, 103.4)                      # 2.4s < 2.5s
    assert s._de_active, "H1: 未满迟滞不应退出"
    H._de_tick(s, 103.6)                      # 2.6s >= 2.5s
    assert not s._de_active, "H1: 满迟滞应退出"
    assert any("escape done" in m for m in _logs), "H1: 缺 done 日志"
    assert s._de_ok_t0 is None, "H1: 退出后应清计时器"


def H2_fail_reset():
    """迟滞窗口内规划再失败 → 计时清零重计，退出时刻顺延。"""
    s = make_tick_self()
    s._plan_fail_since = 200.0
    H._de_tick(s, 200.0)
    H._de_tick(s, 200.6)
    assert s._de_active, "H2: 应触发"
    s._plan_fail_since = None
    H._de_tick(s, 201.0)                      # ok_t0=201.0
    s._plan_fail_since = 202.0                # 窗口内再失败
    H._de_tick(s, 202.0)
    assert s._de_active, "H2: 再失败应保持逃逸"
    assert s._de_ok_t0 is None, "H2: 再失败应清成功计时"
    s._plan_fail_since = None
    H._de_tick(s, 204.0)                      # 重计起点 204.0
    H._de_tick(s, 206.4)                      # 2.4s < 2.5s → 未满
    assert s._de_active, "H2: 重计未满不应退出"
    H._de_tick(s, 206.6)                      # 2.6s >= 2.5s
    assert not s._de_active, "H2: 重计满应退出"


def H3_rapid_flip_never_engages():
    """规划成败快速翻转（成功窗口 <confirm_s）：de 根本不触发。"""
    s = make_tick_self()
    s._plan_fail_since = 300.0
    H._de_tick(s, 300.0)
    s._plan_fail_since = None
    H._de_tick(s, 300.3)                      # 失败 0.3s < confirm
    assert not s._de_active, "H3: 短失败不应触发"
    s._plan_fail_since = 300.4
    H._de_tick(s, 300.4)
    s._plan_fail_since = None
    H._de_tick(s, 300.7)                      # 又只失败 0.3s
    assert not s._de_active, "H3: 快速翻转不应累积触发"
    assert s._de_fail_t0 is None, "H3: 成功期应清失败计时"


H1_normal_exit()
print("H1 PASS")
H2_fail_reset()
print("H2 PASS")
H3_rapid_flip_never_engages()
print("H3 PASS")

print("ALL SELFTEST PASS")
