#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机头对齐 heading_align —— 离线纯函数自测（不跑仿真，只需可 import）。

用法（WSL）：
  cd ~/zx2026_ego_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/heading_selftest.py

病灶回顾：/drone_<id>/vel_cmd 的 angular.z 契约是**偏航速率**，nav_node 是
唯一写者却从不赋值 ⇒ yaw 恒等于 reset 的 0.0（正东）；深度臂云源=前视相机
按 odom_yaw 渲染 ⇒ 导航感知只剩东向 87°×58° 楔形。案卷 docs/depth_chain_case.md §4.9。

覆盖：
  A. 角度工具：wrap_pi 值域 / 2πk 不变性 / ±180° 取短程
  B. heading_mode_of 保守回退四连（键缺失/段 null/段非 dict/值不识）
  C. off 恒等（含零指令、大偏差、纯垂直）
  D. track 只转不门控：速度逐位不变、engaged 恒 False
  E. 速率钳位 + 左右对称（镜像场景速率精确取反）
  F. strict 进门：本拍即缩放、target 锁存 = 当前去向
  G. **锁存/重锁**：偏差<enter 保留锁存目标（防抖）；>enter 重锁（防陈旧——
     c15028c4 撞拍 err=+154.8° 仍在追 4s 前旧目标即此病）
  H. 滞回：同一 15° 偏差，未进门=不门控、已进门=仍门控（防抖）
  I. 退出与复原 + 超时释放（防楔死兜底）
  J. vh_min / 纯垂直 / 零指令 ⇒ 不转不门控
  K. 垂直分量透传（源级：_tick 内不得出现对 cmd[2] 的赋值）
  L. heading_compose：**只缩放航路分量**，反应层增量原样保留 —— d3/d5 机间
     互撞修复（门控曾把分离层逃逸推一起乘 0.15，在撞前最后一刻关掉防线）
  M. **aim/path 解耦**（10-08 修复）：瞄准=最终去向 cmd、缩放=航路分量；vh_min
     判据在 aim（「航路≈0 但增量在漂」必须看向漂移方向）
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NSRC = os.path.join(HERE, "..", "src", "arena_nav", "scripts")
sys.path.insert(0, NSRC)

try:
    from nav_node import (heading_compose, heading_gate, heading_mode_of,
                          wrap_pi)
except ImportError as e:  # 最常见：没 source ROS/devel 环境
    sys.stderr.write("import nav_node 失败(%s)。\n请先: source /opt/ros/noetic/"
                     "setup.bash && source devel/setup.bash\n" % e)
    sys.exit(2)

# ---- 与 sim_settings.yaml heading_align 段同源的测试参数 ---------------------
RATE_MAX = 1.5
GAIN = 2.5
ENTER = math.radians(20.0)
EXIT_ = math.radians(8.0)
VH_MIN = 0.15
SCALE = 0.15
MAX_S = 4.0
EPS = 1e-9

FAILS = []
NPASS = 0


def check(name, ok, detail=""):
    global NPASS
    if ok:
        NPASS += 1
        print("  [PASS] %s" % name)
    else:
        FAILS.append("%s  %s" % (name, detail))
        print("  [FAIL] %s  %s" % (name, detail))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def G(yaw, vx, vy, mode="strict", engaged=False, target=None, t0=0.0,
      now=0.0, vz=0.0):
    """按配置默认参调用闸门（aim 与 path 同向量），返回 7 元组。"""
    return heading_gate(yaw, vx, vy, vx, vy, vz, engaged, target, t0, now,
                        mode, RATE_MAX, GAIN, ENTER, EXIT_, VH_MIN, SCALE,
                        MAX_S)


def G2(yaw, aim, path, mode="strict", engaged=False, target=None, t0=0.0,
       now=0.0, vz=0.0):
    """aim/path 分离调用（M 组：瞄准=最终去向、缩放=航路分量）。"""
    return heading_gate(yaw, aim[0], aim[1], path[0], path[1], vz, engaged,
                        target, t0, now, mode, RATE_MAX, GAIN, ENTER, EXIT_,
                        VH_MIN, SCALE, MAX_S)


print("=" * 74)
print("A. 角度工具")
print("=" * 74)
xs = [-7.0, -4.0, -3.14159265, -1.0, 0.0, 1.0, 3.14159265, 4.0, 7.0, 1e6]
check("wrap_pi 值域 ⊂ [-pi, pi]",
      all(abs(wrap_pi(x)) <= math.pi + EPS for x in xs))
check("wrap_pi 2πk 不变性",
      all(close(wrap_pi(x + 2 * math.pi * k), wrap_pi(x))
          for x in xs for k in (-3, -1, 1, 3)))
# ±180° 必须取短程：yaw≈+π、去向≈-π 的真实差是 ~0.2rad，不是 ~2π
yaw_e = math.pi - 0.1
psi_e = -math.pi + 0.1
err_e = wrap_pi(psi_e - yaw_e)
check("±180° 取短程（|err|≈0.2 而非 6.08）", close(err_e, 0.2, 1e-6),
      "err=%.6f" % err_e)
check("±180° 短程 ⇒ 不误触发门控（<enter）", abs(err_e) <= ENTER,
      "|err|=%.4f > enter=%.4f" % (abs(err_e), ENTER))

print()
print("=" * 74)
print("B. heading_mode_of 保守回退")
print("=" * 74)
check("键缺失 -> off", heading_mode_of({}) == "off")
check("段为 null -> off", heading_mode_of({"heading_align": None}) == "off")
check("段非 dict（list）-> off",
      heading_mode_of({"heading_align": [1, 2]}) == "off")
check("段非 dict（标量）-> off",
      heading_mode_of({"heading_align": 3}) == "off")
check("值不识 -> off",
      heading_mode_of({"heading_align": {"mode": "bogus"}}) == "off")
check("YAML1.1 裸 off 被解析成 False -> off",
      heading_mode_of({"heading_align": {"mode": False}}) == "off")
check("track 放行", heading_mode_of({"heading_align": {"mode": "track"}}) == "track")
check("strict 放行 + 大小写/空白容忍",
      heading_mode_of({"heading_align": {"mode": " STRICT "}}) == "strict")

print()
print("=" * 74)
print("C. off 恒等（零行为变更基线）")
print("=" * 74)
for (yaw, vx, vy, vz) in [(0.0, 1.0, 0.0, 0.0),          # 对准
                          (math.pi, -2.0, 0.0, 0.0),     # 正北却朝西（最大偏差）
                          (0.0, 0.0, 0.0, -0.5),         # 零水平 + 下降
                          (0.0, 0.3, 0.4, 1.0)]:         # 任意
    r = G(yaw, vx, vy, mode="off", vz=vz)
    check("off 恒等 yaw=%.2f v=(%.1f,%.1f)" % (yaw, vx, vy),
          close(r[0], 0.0) and close(r[1], vx) and close(r[2], vy)
          and r[3] is False and r[4] is None and r[6] is False,
          "got %s" % (r,))

print()
print("=" * 74)
print("D. track 只转不门控（对照臂：隔离「转」的代价，不含「停」的收益）")
print("=" * 74)
r = G(0.0, 0.0, 2.0, mode="track")          # 朝东却往北飞，偏差 +90°
check("track 速率 = clip(gain*err)", close(r[0], RATE_MAX), "rate=%.4f" % r[0])
check("track 速度逐位不变", close(r[1], 0.0) and close(r[2], 2.0),
      "v=(%.4f,%.4f)" % (r[1], r[2]))
check("track engaged 恒 False / gated 恒 False / target 不残留",
      r[3] is False and r[4] is None and r[6] is False)
r = G(0.0, 0.0, 2.0, mode="track", engaged=True, target=0.0, t0=1.0, now=2.0)
check("track 忽略传入的 engaged（不继承 strict 状态）",
      r[3] is False and r[6] is False and close(r[2], 2.0),
      "got %s" % (r,))

print()
print("=" * 74)
print("E. 速率钳位 + 左右对称")
print("=" * 74)
r = G(0.0, 1.0, 0.0, mode="strict")         # err=0
check("err=0 -> rate=0", close(r[0], 0.0) and r[6] is False)
r = G(0.1, 1.0, 0.0, mode="track")          # err=-0.1 -> 线性段
check("线性段 rate=gain*err", close(r[0], GAIN * -0.1, 1e-9), "rate=%.6f" % r[0])
r = G(0.0, 0.0, 1.0, mode="track")          # err=+pi/2 -> 正饱和
check("正饱和 = +yaw_rate_max", close(r[0], RATE_MAX), "rate=%.6f" % r[0])
r = G(0.0, 0.0, -1.0, mode="track")         # err=-pi/2 -> 负饱和
check("负饱和 = -yaw_rate_max", close(r[0], -RATE_MAX), "rate=%.6f" % r[0])
rl = G(-math.pi / 2.0, 2.0, 0.0, mode="track")   # 朝南、往东
rr = G(+math.pi / 2.0, 2.0, 0.0, mode="track")   # 朝北、往东
check("镜像场景速率精确取反", close(rl[0], -rr[0], 1e-12)
      and abs(rl[0]) > 0.0, "%.6f vs %.6f" % (rl[0], rr[0]))

print()
print("=" * 74)
print("F. strict 进门：本拍即缩放 + target 锁存")
print("=" * 74)
r = G(0.0, 0.0, 2.0, mode="strict", now=7.0)     # 朝东往北飞，偏差 90°
check("进门 gated=True", r[6] is True)
check("进门 engaged=True", r[3] is True)
check("进门 target 锁存 = 当前去向 (pi/2)", close(r[4], math.pi / 2.0, 1e-12),
      "target=%r" % (r[4],))
check("进门 t0 = now", close(r[5], 7.0))
check("进门本拍即压水平速度（scale）",
      close(r[1], 0.0) and close(r[2], 2.0 * SCALE), "v=(%.4f,%.4f)" % (r[1], r[2]))
r = G(0.0, 2.0, 0.0, mode="strict")              # 偏差 0
check("偏差 <= enter 不进门（只跟踪）", r[6] is False and r[3] is False
      and close(r[1], 2.0))

print()
print("=" * 74)
print("G. 锁存/重锁：偏差<enter 保留锁存目标；偏差>enter 重锁（防陈旧目标）")
print("=" * 74)
# G1 防抖：锁存目标=北(pi/2)，aim 移到北偏 10°(< enter 20°) ⇒ 不重锁
a80 = math.radians(80.0)
r = G(0.0, math.cos(a80), math.sin(a80), mode="strict", engaged=True,
      target=math.pi / 2.0, t0=1.0, now=2.0)
check("偏差 < enter：target 不重取（仍 pi/2）", close(r[4], math.pi / 2.0),
      "target=%r" % (r[4],))
check("仍门控、速率朝锁存目标", r[6] is True and r[0] > 0.0)
# G2 重锁+当拍解除：aim 翻到正东（与锁存目标差 90°>enter）⇒ target 重锁=0；
#    yaw=0 ⇒ 重锁后 err=0 < exit ⇒ 解除（c15028c4 撞拍 err=+154.8° 即旧锁存态）
r = G(0.0, 2.0, 0.0, mode="strict", engaged=True, target=math.pi / 2.0,
      t0=1.0, now=2.0)
check("偏差 > enter：不再追旧目标（rate 对准新去向=0）", close(r[0], 0.0),
      "rate=%.6f（旧锁存语义应为 +max）" % r[0])
check("重锁后 err=0 < exit ⇒ 当拍解除门控", r[6] is False and r[3] is False)
check("解除后航路恢复全速", close(r[1], 2.0))
# G3 重锁但仍在门控：aim=60°（与旧目标 90° 差 30°>enter），yaw=0 ⇒ err=60°>exit
a60 = math.radians(60.0)
r = G(0.0, math.cos(a60), math.sin(a60), mode="strict", engaged=True,
      target=math.pi / 2.0, t0=1.0, now=2.0)
check("重锁后仍门控（err=60°>exit），target=60°",
      r[6] is True and close(r[4], a60))
check("速率朝新目标（gain*1.047=2.62 → 饱和 1.5）", close(r[0], RATE_MAX))

print()
print("=" * 74)
print("H. 滞回：同一 15° 偏差，未进门不门控 / 已进门仍门控（防抖）")
print("=" * 74)
d15 = math.radians(15.0)      # exit(8°) < 15° < enter(20°)
r_new = G(0.0, math.cos(d15), math.sin(d15), mode="strict", engaged=False)
r_eng = G(0.0, math.cos(d15), math.sin(d15), mode="strict", engaged=True,
          target=d15, t0=1.0, now=2.0)
check("15° 未进门 -> 不门控", r_new[6] is False and r_new[3] is False)
check("15° 已进门 -> 仍门控（不抖）", r_eng[6] is True and r_eng[3] is True)

print()
print("=" * 74)
print("I. 退出复原 + 超时释放（防楔死兜底）")
print("=" * 74)
d5 = math.radians(5.0)        # < exit(8°)
r = G(0.0, math.cos(d5), math.sin(d5), mode="strict", engaged=True,
      target=d5, t0=1.0, now=2.0)
check("|err| < exit -> 解除门控", r[6] is False and r[3] is False)
check("解除当拍即恢复全速", close(r[1], math.cos(d5)) and close(r[2], math.sin(d5)),
      "v=(%.4f,%.4f)" % (r[1], r[2]))
r = G(0.0, 0.0, 2.0, mode="strict", engaged=True, target=math.pi / 2.0,
      t0=0.0, now=MAX_S + 0.01)
check("超时 (%ss) 无条件释放" % MAX_S,
      r[6] is False and r[3] is False and close(r[2], 2.0), "got %s" % (r,))
r = G(0.0, 0.0, 2.0, mode="strict", engaged=True, target=math.pi / 2.0,
      t0=0.0, now=MAX_S - 0.01)
check("未超时仍门控（边界另一侧）", r[6] is True)

print()
print("=" * 74)
print("J. vh_min / 纯垂直 / 零指令 ⇒ 不转不门控")
print("=" * 74)
r = G(math.pi, 0.0, 0.0, vz=1.0, mode="strict")
check("零水平指令（悬停）不转不门控",
      close(r[0], 0.0) and r[6] is False and r[3] is False)
r = G(math.pi, 0.0, 0.0, vz=1.0, mode="track")
check("纯垂直爬升不转（即使机头朝西、将向北飞）", close(r[0], 0.0))
r = G(0.0, 0.10, 0.0, mode="strict")     # 0.10 < vh_min 0.15
check("水平速率 < vh_min -> 不转不门控",
      close(r[0], 0.0) and r[6] is False and r[3] is False)
r = G(0.0, 0.0, 0.20, mode="strict")     # 0.20 >= vh_min
check("水平速率 >= vh_min -> 参与", r[6] is True and abs(r[0]) > 0.0)
r = G(math.pi, 0.0, 0.0, vz=-1.0, mode="strict", engaged=True,
      target=0.0, t0=1.0, now=2.0)
check("已门控时悬停 -> 释放且不动（不留楔死态）",
      r[6] is False and r[3] is False and close(r[0], 0.0))

print()
print("=" * 74)
print("K. 垂直分量透传（源级守卫）")
print("=" * 74)
# 注：cmd[2] 在文件别处**确有**合法写者（垂直规划器、分离/群集的 vz 推力），
# 故不能全文件禁 cmd[2]。真正的不变量是「对齐注入**只**动水平两轴」。
src = open(os.path.join(NSRC, "nav_node.py"), encoding="utf-8").read()
check("注入点只回写 cmd[0]/cmd[1]（经 heading_compose 合成）",
      "cmd[0], cmd[1] = heading_compose((ha_vx, ha_vy), (ha_px, ha_py), cmd)"
      in src)
check("注入点无垂直分量（无 ha_vz / cmd[2] 回写）",
      "ha_vz" not in src and "cmd[2] = ha_" not in src)
check("tw.linear.z 仍取自 cmd[2]",
      "tw.linear.x, tw.linear.y, tw.linear.z = cmd[0], cmd[1], cmd[2]" in src)
check("tw.angular.z 已接线（本改动前该赋值不存在）",
      "tw.angular.z = ha_rate" in src)
check("闸门返回 7 元组（不含任何 z 通道）", len(G(0.0, 1.0, 0.0, mode="track")) == 7)

print()
print("=" * 74)
print("L. heading_compose：只缩放航路分量，反应层增量原样保留")
print("=" * 74)
# L1 恒等：未缩放时逐位不变（off/track 与非门控拍必须零侵入）
check("恒等：gated==path 时结果 == cmd",
      heading_compose((1.0, 2.0), (1.0, 2.0), [1.0, 2.0, 0.5]) == (1.0, 2.0))
# L2 只缩航路：反应增量原样
#   path=(1,0) cmd=(1,0)+(0,3)  门控把 path 压到 0.15 ⇒ 期望 (0.15, 3.0)
r = heading_compose((0.15, 0.0), (1.0, 0.0), [1.0, 3.0, 0.0])
check("只缩航路：结果 = 0.15·path + 全强度反应增量",
      close(r[0], 0.15) and close(r[1], 3.0), "got %s" % (r,))
# L3 航路被压到 0，逃逸推仍满额（这是 d3/d5 案的底线）
r = heading_compose((0.0, 0.0), (1.0, 0.0), [1.0, 3.0, 0.0])
check("航路压到零后逃逸推仍为满额 3.0", close(r[1], 3.0), "got %s" % (r,))
# L4 复刻 d3/d5 案：航路=向南 1.46，反应=向北 3.0 的逃逸推（指令被翻向）
path = (0.01, -1.46)
cmd = [0.01, -1.46 + 3.0, 0.0]        # 分离层把 cmd_y 翻成 +1.54
r = heading_compose((path[0] * SCALE, path[1] * SCALE), path, cmd)
old = (cmd[0] * SCALE, cmd[1] * SCALE)  # 旧设计：连逃逸推一起乘 0.15
# 期望 y = 0.15·path_y + 逃逸增量 3.0 = −0.219 + 3.0
check("d3/d5 案：逃逸推为旧设计的 %.1f 倍（新 %.2f / 旧 %.2f）"
      % (r[1] / max(old[1], 1e-9), r[1], old[1]),
      close(r[1], path[1] * SCALE + 3.0) and old[1] < 0.25,
      "新 %s / 旧 %s" % (r, old))
bad = (1.0, 6.0)
heading_compose((0.15, 0.0), (1.0, 0.0), bad)
check("不副作用：不修改入参 cmd", bad == (1.0, 6.0))
# 源级守卫：缩放输入必须是航路分量、瞄准输入是解耦后的 (aim_x, aim_y)，
# 且快照在反应层**之前**
check("缩放输入 (ha_px, ha_py)、瞄准输入 (aim_x, aim_y)",
      "self.odom_yaw, aim_x, aim_y, ha_px, ha_py, cmd[2]" in src)
check("瞄准源 = 最终 cmd（heading_align.aim 默认 final）",
      "aim_x, aim_y = float(cmd[0]), float(cmd[1])" in src)
check("门控拍二次瞄准真实去向(scale·path+inc)",
      "ha_px * self._ha_scale + (aim_x - ha_px)" in src)
check("航路分量快照存在",
      "ha_px, ha_py = float(cmd[0]), float(cmd[1])" in src)
SNAP = "ha_px, ha_py = float(cmd[0]), float(cmd[1])"
check("快照先于反应层（紧跟指令生产，早于 _apply_swarm）",
      0 < src.index(SNAP) < src.index("cmd = self._apply_swarm(cmd)"))
check("两个分支各取一次航路快照（closed_loop / god_mode）",
      src.count(SNAP) == 2)
check("叠回后有末闸（发布模 ≤ max_vel 原保证不破）",
      "vn2 = float(np.linalg.norm(cmd))" in src)

print()
print("=" * 74)
print("M. aim/path 解耦：瞄准=最终去向，缩放=航路分量（2026-10-08 修复）")
print("=" * 74)
# M1 c15028c4 撞拍形态：航路朝东 (2,0)、反应增量把最终去向翻到北 (0,1)、
#    机头朝东。旧代码（aim=path）err=0 不转不门控=盲漂撞树；新代码必须瞄北。
r = G2(0.0, (0.0, 1.0), (2.0, 0.0), mode="strict")
check("M1 瞄准最终去向：err=90°>enter ⇒ 进门", r[6] is True)
check("M1 锁存目标 = 最终去向 pi/2", close(r[4], math.pi / 2.0))
check("M1 缩放只作用航路：(2,0)→(0.3,0) 而非按 aim 缩成 (0,0.15)",
      close(r[1], 2.0 * SCALE) and close(r[2], 0.0),
      "path_out=(%.4f,%.4f)" % (r[1], r[2]))
# M2 vh_min 判据在 aim 不在 path：航路≈0、增量漂移 1.0 ⇒ 必须看向漂移方向
r = G2(0.0, (0.0, 1.0), (0.02, 0.0), mode="strict")
check("M2 航路≈0 但增量在漂 ⇒ 仍瞄准/仍门控（旧代码此处恒不转）",
      r[6] is True and close(r[4], math.pi / 2.0))
# M3 真悬停：增量恰好抵消航路（aim≈0）⇒ 不转不门控、航路不缩
r = G2(0.0, (0.0, 0.0), (2.0, 0.0), mode="strict")
check("M3 aim≈0（机体不动）⇒ 不转不门控、航路不缩",
      close(r[0], 0.0) and r[6] is False and close(r[1], 2.0))
# M4 track 臂同样瞄 aim（对照臂不缩放但方向必须对）
r = G2(0.0, (0.0, 1.0), (2.0, 0.0), mode="track")
check("M4 track 瞄准 aim（rate=+max 朝北）、航路不缩",
      close(r[0], RATE_MAX) and close(r[1], 2.0) and close(r[2], 0.0))

print()
print("=" * 74)
if FAILS:
    print("HEADING SELFTEST FAIL  —— %d 项未过 / 共 %d 项"
          % (len(FAILS), len(FAILS) + NPASS))
    for f in FAILS:
        print("   ! " + f)
    sys.exit(1)
print("HEADING SELFTEST PASS  —— %d/%d 项" % (NPASS, NPASS))
