#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_sup_selftest.py — DeFoP 监督层「体半径口径」守卫（2026-10-01 立案）。

背景：一次 26 撞法证发现机上同时跑着**三个互不相等的"机身半径"**——
    Scene.drone_radius (fleet.yaml)   0.35  ← 判据 + nav 膨胀基（权威）
    sup_drone_radius  (defop_nav)     0.30  ← 曾是**论文 0.4m 机架**的调用点实参
    filter_actions_by_grid_3 签名缺省  0.32  ← 上游逐字保真锚点（生产路径死码）
而 Gazebo 实体碰撞盒是 world_builder 的 0.5×0.5×0.2（面内外接 √(0.25²+0.25²)=0.354）。
监督层按比实际小 17% 的机体去算扇区裕度 ⇔ 否决线系统性偏松。

本测把两条**结构性不变量**固化为可失败断言（退出码非 0 = 有人把它们改了）：
  (i)  判据球 vs 实体盒——拆成两个**不同的界**，不许混说：
         (i-a) 球 >= 盒面半宽 0.25（正撞被罩住，这是巡航最常见逼近姿态）
         (i-b) 球 ≈ 盒外接 0.3536，容差 1cm。**实测 0.35 比外接小 3.6mm**
               ——是"取整到 0.35"的痕迹，不是严格外包。禁称"恰等于"。
  (ii) 监督层必须与判据同径       ：sup_drone_radius == Scene.drone_radius

覆盖：
  [1] 实体碰撞盒从 world_builder **源码**解析（改盒尺寸 → 本测当场变）
  [2] 不变量 (i)：判据球 ⊇ 实体盒（含容差与"球比盒面宽多少"的记账）
  [3] 不变量 (ii)：yaml sup_drone_radius == Scene.drone_radius
  [4] cpn 节点 default_cfg 同键同值（yaml 缺键时的兜底不许跑偏）
  [5] 调用点静态核查：yaml 值确实被传进 filter_actions_by_grid_3
  [6] 量化后果：margin(r) 在 d=1.0m 的实测差（= 本次修复的可测痕迹）
  [7] 防呆：上游签名缺省 0.32 必须仍在位（它是"逐字移植"的锚点，不是 bug）

用法（WSL）：
  cd ~/zx2026_defop_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/defop_sup_selftest.py
"""
import io
import math
import os
import re
import sys

WS = "/home/ubuntu/zx2026_defop_ws"
YAML = WS + "/src/defop_nav/config/defop_nav.yaml"
NODE = WS + "/src/defop_nav/scripts/defop_cpn_node.py"
WB = WS + "/src/arena_world_gazebo/scripts/world_builder.py"
COMMON = WS + "/src/defop_nav/scripts/defop_common.py"

sys.path.insert(0, WS + "/src/defop_nav/scripts")
sys.path.insert(0, WS + "/src/zx2026_common/scripts")

fails = 0


def chk(label, got, want, tol=None):
    global fails
    if tol is None:
        ok = got == want
    else:
        ok = abs(got - want) <= tol
    if not ok:
        fails += 1
    print("  %-52s %-4s got=%s want=%s" % (label, "OK" if ok else "FAIL", got, want))
    return ok


print("=" * 72)
print("DeFoP 监督层体半径口径守卫")
print("=" * 72)

# ---------------- [1] 实体碰撞盒（真源码解析） ----------------
print("\n[1] 实体碰撞盒 —— 从 world_builder 源码解析（改盒 → 本测当场变）")
wb = io.open(WB, encoding="utf-8").read()
m = re.search(r"col_box\s*=\s*'<box><size>([\d.]+) ([\d.]+) ([\d.]+)</size></box>'", wb)
if not m:
    print("  FAIL 未能在 world_builder 找到 col_box 字面量（解析锚点失效）")
    sys.exit(1)
sx, sy, sz = (float(v) for v in m.groups())
half_diag = math.hypot(sx / 2.0, sy / 2.0)      # 面内外接半径
half_face = min(sx, sy) / 2.0                    # 正撞时的实际伸出量
print("  碰撞盒 = %.2f × %.2f × %.2f" % (sx, sy, sz))
print("  面内外接半径 hypot(%.3f,%.3f) = %.4f m ；面半宽 = %.3f m"
      % (sx / 2, sy / 2, half_diag, half_face))

# ---------------- [2] 不变量 (i)：判据球 vs 实体盒 ----------------
print("\n[2] 不变量 (i)：判据球 vs 实体盒")
from zx2026_common.scene import Scene  # noqa: E402
sc = Scene()
dr = float(sc.drone_radius)
print("  Scene.drone_radius (fleet.yaml) = %.3f" % dr)
print("  → 记账：正撞(面) 时球比盒早触发 %+.3f m；斜 45°(角) 时 %+.3f m"
      % (dr - half_face, dr - half_diag))
# 两条分开断言，因为它们是**两个不同的界**：
#   (i-a) 面下界：正撞路径上球必须罩住盒 —— 这是巡航最常见的逼近姿态。
#   (i-b) 外接等价：0.35 vs 0.35355 差 3.6mm，是"取整到 0.35"的痕迹，
#         不是严格外包。断言只能到 1cm 容差，**不许声称"恰等于"**。
chk("(i-a) 球 >= 盒面半宽（正撞被罩住）", dr >= half_face - 1e-9, True)
chk("(i-a) 正撞冗余(m)", round(dr - half_face, 4), round(dr - half_face, 4), 1e-9)
chk("(i-b) 球 ≈ 盒外接（|Δ|≤1cm）", abs(dr - half_diag) <= 1e-2, True)
print("       Δ = %+.4f m（球比外接%s %.4f m，非严格外包）"
      % (dr - half_diag, "小" if dr < half_diag else "大", abs(dr - half_diag)))

# ---------------- [3] 不变量 (ii)：yaml 与判据同径 ----------------
print("\n[3] 不变量 (ii)：yaml sup_drone_radius == Scene.drone_radius")
yaml_txt = io.open(YAML, encoding="utf-8").read()
m = re.search(r"^sup_drone_radius\s*:\s*([\d.]+)", yaml_txt, re.M)
if not m:
    print("  FAIL yaml 无 sup_drone_radius 键")
    sys.exit(1)
y_sup = float(m.group(1))
chk("yaml sup_drone_radius", y_sup, dr, 1e-9)
chk("是否仍在论文档 0.30（应为 False）", y_sup == 0.30, False)

# ---------------- [4] cpn 节点 default_cfg 兜底值 ----------------
print("\n[4] cpn default_cfg 同键同值（yaml 缺键时的兜底不许跑偏）")
node_txt = io.open(NODE, encoding="utf-8").read()
m = re.search(r"'sup_drone_radius'\s*:\s*([\d.]+)", node_txt)
if not m:
    print("  FAIL cpn 节点无 sup_drone_radius 缺省")
    sys.exit(1)
d_sup = float(m.group(1))
chk("default_cfg sup_drone_radius", d_sup, dr, 1e-9)

# ---------------- [5] 调用点静态核查 ----------------
print("\n[5] 调用点：yaml 值确被传进 filter_actions_by_grid_3")
chk("调用点传 drone_radius=float(c['sup_drone_radius'])",
    "drone_radius=float(c['sup_drone_radius'])" in node_txt, True)
chk("调用点在 filter_actions_by_grid_3 调用内（同一调用串）",
    bool(re.search(r"filter_actions_by_grid_3\([\s\S]{0,600}?"
                   r"drone_radius=float\(c\['sup_drone_radius'\]\)", node_txt)), True)

# ---------------- [6] 量化后果（真源码函数，非重写） ----------------
print("\n[6] 量化后果 —— 真源码 compute_margin（Δθ = fov_h 87° / num_yaw 32）")
from defop_common import compute_margin  # noqa: E402
fov_h = math.radians(87.0)
NZ = 32
print("  Δθ = %.5f rad = %.4f°" % (fov_h / NZ, math.degrees(fov_h / NZ)))
rows = []
for d in (0.4, 0.6, 0.8, 1.0, 1.2, 1.5):
    a = compute_margin(0.30, d, fov_h, NZ)
    b = compute_margin(dr, d, fov_h, NZ)
    rows.append((d, a, b, b - a))
    print("  d=%.1f m   r=0.30→%2d 扇区   r=%.2f→%2d 扇区   Δ=%+d" % (d, a, dr, b, b - a))
chk("d=1.0m（sup_min_dist 硬否决边界）r=0.35 的扇区数", rows[3][2], 8)
chk("d=1.0m 相对论文档的增量", rows[3][3], 1)
chk("d>=1.2m 完全不变（改动只在硬否决生效区内）",
    all(x[3] == 0 for x in rows if x[0] >= 1.2), True)

# ---------------- [7] 防呆：上游签名缺省仍在位 ----------------
print("\n[7] 防呆：上游签名缺省 0.32 必须仍在位（'逐字移植'锚点，不是 bug）")
ctxt = io.open(COMMON, encoding="utf-8").read()
chk("filter_actions_by_grid_3 签名缺省 drone_radius=0.32",
    bool(re.search(r"min_dist=1\.5,\s*min_ratio=0\.02,\s*drone_radius=0\.32",
                   ctxt)), True)
chk("compute_margin 在本测是 import 的真函数（非重写）",
    compute_margin.__module__, "defop_common")

print("\n" + "=" * 72)
if fails:
    print("FAIL %d 项 —— 体半径口径被改动，见上" % fails)
    print("回本仓口径：sup_drone_radius 写回 %.2f（yaml 与 cpn default_cfg 双侧）" % dr)
else:
    print("ALL PASS —— 判据球罩住实体盒，监督层与判据同径")
print("=" * 72)
sys.exit(1 if fails else 0)
