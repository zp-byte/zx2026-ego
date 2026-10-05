#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_sup_geo_selftest.py — DeFoP 监督层「几何口径」守卫（2026-10-01 立案）。

背景：filter_actions_by_grid_3 把"障碍扇区"映射到"基元"时用的是基元的**命令角
dyaw**，而基元的**真实飞行航向**是
    β = atan2(vel_y_gain·tanh(dyaw), cmd_velocity) = atan2(0.5·tanh(dyaw), 1.0)
dyaw 覆盖 ±39.15°，β 只有 ±16.53° —— 2.37 倍口径错配。后果是监督器**漏判**：
被判"安全"的基元里，相当一部分按真实航迹仍会撞上树干。

本测把该缺陷与修复固化为**可失败的定量断言**（退出码非 0 = 有人改了口径）：

  [A] off 档（上游逐字口径）必须 **HOLE > 0** —— 反向注入：若有人"顺手修好"
      却没走 prim_bearing 通道，本断言当场 FAIL（防止缺陷被静默掩盖）。
  [B] on  档（扇区真方位轴 + 净空锥 asin）必须 **HOLE == 0**（正确性主断言），
      且 DEAD >= off 档。
  [C] on 档的**误锁(FDEAD)不作零断言**，只断言"只出现在安全航向本就稀少的
      边缘格"（n_safe <= 256/8）。理由：监督器用 ±margin 扇区做保守扩张，且是
      2D 矩形放大（y_mask & z_mask），在仅剩几条安全航向的几何里必然吞掉它们
      —— 这是换来 HOLE==0 的**对价**，不是缺陷。真正的回归信号是：它开始在
      安全集充裕的格里误锁（→ margin 被放大过界）。实测 3/45 格，n_safe 均 ≤32。
  [C] 相机模型一致性：sim_settings.yaml 的 hfov_deg 必须 == defop_nav.yaml 的
      fov_h_deg。扇区真方位轴由 fx = (w/2)/tan(fov_h/2) 推出，两键一旦分叉，
      方位与 Δθ 同时算错，而**任何一侧单独看都正常**。

判据（三个都取自生产代码/判分器，非重写）：
  · 真航迹安全与否 = gazebo_collision_monitor._tree_hit：
      树轴到航迹射线的垂距 < trunk_r + drone_radius，且最近点在航向前方
  · 监督器输出     = defop_common.filter_actions_by_grid_3 的 safe_mask（真函数）
  · 净空锥         = defop_common.compute_margin_cone（真函数，on 档传入）

合成深度用**针孔投影**（col = cx − fx·tan(B)），与函数内部假设同一相机模型。
（早期 /tmp 探针用"列↔角度线性"近似，与 fx 口径不自洽，会给 HOLE 掺入伪漏判。）

用法（WSL）：
  cd ~/zx2026_defop_ws && python3 tools/defop_sup_geo_selftest.py
"""
import io
import math
import re
import sys

import numpy as np

WS = "/home/ubuntu/zx2026_defop_ws"
YAML = WS + "/src/defop_nav/config/defop_nav.yaml"
SIM = WS + "/src/zx2026_common/config/sim_settings.yaml"
COMMON = WS + "/src/defop_nav/scripts/defop_common.py"

sys.path.insert(0, WS + "/src/defop_nav/scripts")
import defop_common as dc  # noqa: E402

TRUNK_R = 0.15          # 与 gazebo_collision_monitor._tree_hit 同源
H, W = 270, 480
FREE = 8.0              # > min_dist，视为空

fails = 0


def chk(label, got, want):
    global fails
    ok = (got == want)
    if not ok:
        fails += 1
    print("  %-52s %-4s got=%s want=%s" % (label, "OK" if ok else "FAIL", got, want))
    return ok


def load_cfg():
    txt = io.open(YAML, encoding="utf-8").read()
    out = {}
    for k in ("cmd_velocity", "vel_y_gain", "sup_min_dist", "sup_min_ratio",
              "sup_drone_radius", "cam_pitch_deg", "fov_h_deg", "fov_v_deg",
              "num_yaw", "num_vel_z", "num_vel_x", "action_horizon"):
        m = re.search(r"^%s\s*:\s*([\d.]+)" % k, txt, re.M)
        if not m:
            print("  FAIL yaml 无键 %s（解析锚点失效）" % k)
            sys.exit(1)
        out[k] = float(m.group(1))
    return out


CFG = load_cfg()
FOV_H = CFG["fov_h_deg"]
FOV_V = CFG["fov_v_deg"]
FOV_H_RAD = math.radians(FOV_H)
VEL = CFG["cmd_velocity"]
DR = CFG["sup_drone_radius"]
NY, NZ = int(CFG["num_yaw"]), int(CFG["num_vel_z"])
MIN_DIST, MIN_RATIO = CFG["sup_min_dist"], CFG["sup_min_ratio"]
HIT_R = TRUNK_R + DR                    # 判据半径

action_seq, yaw_lib, z_lib = dc.build_action_library(
    int(CFG["num_vel_x"]), NZ, NY, int(CFG["action_horizon"]), VEL,
    CFG["cam_pitch_deg"], FOV_H, FOV_V)

DYAW = action_seq[:, 0, 2].astype(np.float64)
VZ = action_seq[:, 0, 1].astype(np.float64)
# 权威式在 _cmd_from_primitive：vel_y = vel_y_gain * tanh(dyaw)
BETA = np.arctan2(CFG["vel_y_gain"] * np.tanh(DYAW), VEL)

CX = W / 2.0
FX = CX / math.tan(FOV_H_RAD / 2.0)

print("=" * 78)
print("DeFoP 监督层几何口径守卫")
print("=" * 78)
print("  dyaw ±%.2f°   ->   β ±%.2f°   （%.2f× 口径差）"
      % (np.degrees(np.abs(DYAW).max()), np.degrees(np.abs(BETA).max()),
         np.degrees(np.abs(DYAW).max()) / np.degrees(np.abs(BETA).max())))
print("  相机 fx=%.1fpx  cx=%.1f  扇区 %.0f×%.0f   Δθ=%.3f°"
      % (FX, CX, NY, NZ, FOV_H / NY))
print("  判据半径 trunk_r %.2f + drone_radius %.2f = %.2f m"
      % (TRUNK_R, DR, HIT_R))

# ---------------- [C] 相机模型一致性 ----------------
print("\n[C] 相机模型：渲染 hfov == 监督层 fov_h_deg")
sim_txt = io.open(SIM, encoding="utf-8").read()
m = re.search(r"^\s*hfov_deg\s*:\s*([\d.]+)", sim_txt, re.M)
if not m:
    print("  FAIL sim_settings 无 hfov_deg 键（解析锚点失效）")
    sys.exit(1)
chk("sim_settings.hfov_deg == defop_nav.fov_h_deg",
    float(m.group(1)), FOV_H)
# 扇区真方位轴（与生产代码同式），供人眼核对单调性/量程
_c = (NY - 1) - np.arange(NY)
_col_ends = np.arange(0, W + 1, W // NY)[:NY + 1]
_col_c = 0.5 * (_col_ends[_c] + _col_ends[_c + 1])
YAXIS = -np.arctan((_col_c - CX) / FX)
print("  扇区真方位轴 [%+.2f°, %+.2f°]  递增=%s"
      % (np.degrees(YAXIS.min()), np.degrees(YAXIS.max()),
         bool(np.all(np.diff(YAXIS) > 0))))


# ---------------- 合成深度（针孔） ----------------
def build_depth(bearing_deg, dist):
    """一棵树干：方位 B（左正）、距离 D，占满像高（本测只考 yaw 轴）。"""
    d = np.full((H, W), FREE, np.float32)
    B = math.radians(bearing_deg)
    half = math.atan(TRUNK_R / dist)
    x0 = int(math.floor(CX - FX * math.tan(B + half)))
    x1 = int(math.ceil(CX - FX * math.tan(B - half))) + 1
    d[:, max(0, x0):min(W, x1)] = dist
    return d


def collide(bearing_deg, dist):
    """判分器同款：每个基元按真实航迹 β 判是否擦上该树。"""
    B = math.radians(bearing_deg)
    return ((dist * np.abs(np.sin(B - BETA)) < HIT_R)
            & (dist * np.cos(B - BETA) > 0))


def masked(depth, use_beta):
    """真源码输出：off = 旧口径；on = prim_bearing + 净空锥。"""
    kw = {}
    if use_beta:
        kw = {"margin_fn": dc.compute_margin_cone, "prim_bearing": BETA}
    _, _, sm, _, _, _ = dc.filter_actions_by_grid_3(
        depth, FOV_H_RAD, math.radians(FOV_V), DYAW, VZ, yaw_lib, z_lib,
        num_yaw=NY, num_vel_z=NZ, min_dist=MIN_DIST, min_ratio=MIN_RATIO,
        drone_radius=DR, **kw)
    return np.asarray(sm).ravel()


def tally(bearings, dists):
    """返回 (tot, fdead_cells)。

    tot[tag] = [存活, HOLE, DEAD, FDEAD]
    fdead_cells = on 档的误锁格明细 [(方位, 距离, 该格物理安全航向数)]
    """
    tot = {"off": [0, 0, 0, 0], "on": [0, 0, 0, 0]}
    fdead_cells = []
    for b in bearings:
        for dd in dists:
            depth = build_depth(b, dd)
            bad = collide(b, dd)
            n_safe = int((~bad).sum())
            for tag, use_beta in (("off", False), ("on", True)):
                s = masked(depth, use_beta)
                ns = int(s.sum())
                nh = int((s & bad).sum())
                t = tot[tag]
                t[0] += ns
                t[1] += nh
                if ns == 0:
                    t[2] += 1
                    if n_safe > 0:
                        t[3] += 1      # 误锁：明明有安全航向却判死端
                        if tag == "on":
                            fdead_cells.append((b, dd, n_safe))
    return tot, fdead_cells


def report(tag, bearings, dists):
    tot, fdead_cells = tally(bearings, dists)
    n = len(bearings) * len(dists)
    print("\n[%s] %d 格（方位 %s × 距离 %s）"
          % (tag, n, list(bearings), list(dists)))
    print("  %-6s %8s %8s %10s %12s" % ("档", "存活", "HOLE", "DEAD", "FDEAD"))
    for k in ("off", "on"):
        s, h, ddead, fdead = tot[k]
        print("  %-6s %8d %8d %6d/%-3d %6d/%-3d" % (k, s, h, ddead, n, fdead, n))
    if fdead_cells:
        print("  on 误锁格明细（方位, 距离, 该格物理安全航向数 / %d）:" % BETA.size)
        for b, dd, nsf in fdead_cells:
            print("    B=%4.1f° D=%.2f  n_safe=%d (%.1f%%)"
                  % (b, dd, nsf, 100.0 * nsf / BETA.size))
    return tot, fdead_cells


# ---------------- [A][B] 单树集：断言 ----------------
print("\n" + "=" * 78)
print("[A][B] 单孤立树集 —— 断言 HOLE 与 FDEAD")
print("=" * 78)
BEARINGS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0)
DISTS = (0.5, 0.6, 0.8, 0.9, 0.95)
tot, FDEAD_CELLS = report("单树", BEARINGS, DISTS)

chk("[A] off 档 HOLE > 0（缺陷存在，反向注入必失败）",
    tot["off"][1] > 0, True)
chk("[B] on  档 HOLE == 0（漏判清零 —— 本测的**正确性主断言**）",
    tot["on"][1], 0)
chk("[B] on  档 DEAD >= off 档（收紧的必然方向）",
    tot["on"][2] >= tot["off"][2], True)
# FDEAD（误锁）**不作为零断言**：监督器靠 ±margin 扇区做保守扩张，而那是个
# 2D 矩形放大（y_mask & z_mask），在"安全航向本就只剩几条"的边缘几何里必然
# 会吞掉它们。真正的判据不是"零误锁"，而是**只在近乎无路的格里出现**：
# 若某天它开始在安全集充裕的格里误锁，说明 margin 被放大过界 —— 那才是回归。
_NSAFE_TOL = BETA.size // 8          # 32/256 = 12.5%，安全集"充裕"的下界
_bad = [(b, d, k) for (b, d, k) in FDEAD_CELLS if k > _NSAFE_TOL]
chk("[C] on 误锁只出现在 n_safe<=%d/%d 的边缘格（无充裕安全集被锁）"
    % (_NSAFE_TOL, BETA.size), _bad, [])

# ---------------- 多障碍集：只报告，不断言 ----------------
print("\n" + "=" * 78)
print("[报告] 多障碍集 —— 合成保真度不足，**只读数不断言**（供风险判读）")
print("=" * 78)
print("  说明：真实场景是多树/枝干/宽障叠加，本测的单树集是最干净的下界。")
print("  下面用 2 树叠加 + 15° 宽障（模拟枝冠/网）看口径修正是否仍不产生误锁。")
for name, fn in (("双树", None), ("宽障15°", None)):
    base = np.full((H, W), FREE, np.float32)
    if name == "双树":
        base = np.minimum(base, build_depth(0.0, 0.8))
        base = np.minimum(base, build_depth(20.0, 0.9))
    else:
        x0 = int(math.floor(CX - FX * math.tan(math.radians(7.5))))
        x1 = int(math.ceil(CX - FX * math.tan(math.radians(-7.5)))) + 1
        base[:, max(0, x0):min(W, x1)] = 0.9
    bad = collide(0.0, 0.8) | collide(20.0, 0.9) if name == "双树" else \
        (0.9 * np.abs(np.sin(BETA)) < HIT_R) & (0.9 * np.cos(BETA) > 0)
    line = "  %-8s" % name
    for tag, use_beta in (("off", False), ("on", True)):
        s = masked(base, use_beta)
        line += "  %s: 存活%3d HOLE%3d %s" % (
            tag, int(s.sum()), int((s & bad).sum()),
            "DEAD" if s.sum() == 0 else "    ")
    print(line)

print("\n" + "=" * 78)
if fails:
    print("FAIL %d 项 —— 监督层几何口径被改动，见上" % fails)
    print("回滚：defop_nav.yaml 的 sup_bearing_mode 置 'off'，"
          "并确认 filter_actions_by_grid_3 的 prim_bearing 缺省仍为 None")
else:
    print("ALL PASS —— off 档漏判可复现；on 档漏判清零，误锁只落在近乎无路的边缘格")
    print("注：合成集是最干净的下界（孤立树、无枝无冠、无旋转扫掠）。"
          "真实场景的 Δ死端率由 sup_bearing_mode=shadow 跑次实测，非本测可代。")
print("=" * 78)
sys.exit(1 if fails else 0)
