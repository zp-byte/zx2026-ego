#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_w1_selftest.py — W1 进度感知腿看门狗干测（离线，无 ROS master）。

用 object.__new__ 绕开 MissionExecutor.__init__ 的 ROS 构造，只装配
_recon_tick 真实代码路径所需属性，然后按可控时钟逐拍驱动。**驱动的是真品
方法，不是逻辑副本**（副本测等于没测）。

覆盖：
  T1 旧行为对齐   stall_s=0  -> 恰在 45s 触发 leg_timeout，全程无 stall/cap
  T2 停滞早退     stall_s=10 + 机体冻住 -> ~10s 触发 leg_stall（旧版要 45s）
  T3 硬上限放行   stall_s=10 + 机体慢爬（>eps/窗）-> 越过 45s，90s 触发 leg_cap
                 ← 这是主病灶的修复断言：慢但在动的腿不再被 45s 砍断
  T4 正常到达     推进到 d<drop_tol -> 交棒 GOTO_LO，任何看门狗都不触发
  T5 GOTO_LO 不动  mode=GOTO_LO + 冻住 -> 仍走旧 45s 路（stall 不作用于它）
"""
import os
import sys

WS = os.path.expanduser("~/zx2026_defop_ws")
sys.path.insert(0, os.path.join(WS, "src", "arena_mission", "scripts"))

import rospy  # noqa: E402
import mission_executor_node as ME  # noqa: E402

CLOCK = [0.0]
rospy.get_time = lambda: CLOCK[0]          # 可控时钟（模块内全部 get_time 走它）

ADVANCES = []


class DP(object):
    id = 7
    color = "red"
    xyz = (27.0, -9.0, 0.0)


def make(mode, stall_s, xy_start, frozen=False, speed=0.0, drop_tol=0.6):
    """装配一个只够 _recon_tick 跑的伪执行器。"""
    o = object.__new__(ME.MissionExecutor)
    o.drone_id = 0
    o._scan_order = [DP()]
    o._scan_idx = 0
    o._scan_mode = mode
    o._cid_leg_timeout = 45.0
    o._cid_leg_stall_s = stall_s
    o._cid_leg_stall_eps = 0.30
    o._cid_leg_cap_s = 90.0
    o.drop_tol = drop_tol
    o.identify_z = 1.2
    o.cruise_z = 1.2
    o.drop_mid_z = 0.90
    o.drop_hover_z = 0.80
    o._lo_stage = 0
    o.odom = [xy_start[0], xy_start[1], 1.2]
    o._frozen = frozen
    o._speed = speed
    CLOCK[0] = 0.0          # 必须归零：_scan_leg_reset 取的就是此刻的时钟，
    o._scan_leg_reset()     # 否则腿起点带上上一测的残留 -> elapsed 假偏移
    o._goto = lambda xyz: None
    o._low_occupant = lambda dp: None
    o._enter_wait = lambda *a: None
    o._scan_advance = lambda reason: ADVANCES.append((round(CLOCK[0], 2), reason))
    return o


def drive(o, goal, horizon=120.0, dt=0.5):
    """按 dt 逐拍驱动；返回 (触发时刻, 原因) 或 ("MODE->X", None) 或 None。"""
    ADVANCES[:] = []
    m0 = o._scan_mode
    t = 0.0
    # 模拟器：机体每拍按 speed 朝 goal 水平走（frozen 则不动）。
    # 不复现 z 下降——T4 只断言"健康 GOTO_HI 腿不被看门狗砍、交棒给 GOTO_LO"，
    # 之后的 GOTO_LO 中继层卡住是我这个不下降的伪机的固有产物，不在测程内。
    while t <= horizon:
        CLOCK[0] = t
        if not o._frozen:
            gx, gy = goal[0], goal[1]
            dx, dy = gx - o.odom[0], gy - o.odom[1]
            n = (dx * dx + dy * dy) ** 0.5
            if n > 1e-9:
                step = min(o._speed * dt, n)
                o.odom[0] += dx / n * step
                o.odom[1] += dy / n * step
        d = ((goal[0] - o.odom[0]) ** 2 + (goal[1] - o.odom[1]) ** 2
             + (goal[2] - o.odom[2]) ** 2) ** 0.5
        o._recon_tick(d)
        if ADVANCES:
            return ADVANCES[0]
        if o._scan_mode != m0:          # 任何模式变更都交棒（含 GOTO_HI->GOTO_LO）
            return ("MODE->%s" % o._scan_mode, None)
        t += dt
    return None


GOAL = (27.0, -9.0, 1.2)
FAILS = []


def check(tag, got, want):
    ok = got == want or (callable(want) and want(got))
    print("  %-46s got=%-28s %s" % (tag, got, "OK" if ok else "FAIL want=%s" % (want,)))
    if not ok:
        FAILS.append(tag)


def main():
    print("=== W1 干测：驱动真实 _recon_tick ===")

    print("\nT1 旧行为对齐（stall_s=0，机体冻住，GOTO_HI）")
    # 判据是 elapsed > 45.0，逐拍 0.5s -> 首个越界拍 45.5
    r = drive(make("GOTO_HI", 0.0, (0.0, 0.0), frozen=True), GOAL)
    check("T1 触发", r, (45.5, "leg_timeout"))

    print("\nT2 停滞早退（stall_s=10，机体冻住，GOTO_HI）")
    r = drive(make("GOTO_HI", 10.0, (0.0, 0.0), frozen=True), GOAL)
    check("T2 触发", r, (10.5, "leg_stall"))

    print("\nT3 硬上限放行（stall_s=10，慢爬 0.05m/s，GOTO_HI）")
    # 0.05m/s * 0.5s 拍 = 0.025m/拍；0.30m eps 需 12 拍 = 6s < 10s 窗 -> 不算停滞
    r = drive(make("GOTO_HI", 10.0, (0.0, 0.0), speed=0.05), GOAL)
    check("T3 越过 45s 才触发", r, (90.5, "leg_cap"))

    print("\nT4 正常到达（快机 1.3m/s，GOTO_HI -> GOTO_LO 交棒）")
    r = drive(make("GOTO_HI", 10.0, (0.0, 0.0), speed=1.3), GOAL)
    check("T4 交棒且无看门狗触发", r, ("MODE->GOTO_LO", None))

    print("\nT5 GOTO_LO 不受 stall 影响（stall_s=10，机体冻住，GOTO_LO）")
    r = drive(make("GOTO_LO", 10.0, (0.0, 0.0), frozen=True), GOAL)
    check("T5 仍走旧 45s", r, (45.5, "leg_timeout"))

    print("\n%s  失败 %d 项" % ("=== 全过 ===" if not FAILS else "=== 有失败 ===",
                                len(FAILS)))
    return 1 if FAILS else 0


sys.exit(main())
