#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""geo_selftest.py — geo_enu.py + venue_gps.yaml 离线自检（零 ROS 依赖）。

跑法（WSL）：python3 tools/geo_selftest.py
判据：全过打 GEO SELFTEST PASS，任一断言炸即 traceback。
覆盖：
  1. dms2deg 已知答案（对 docx 原串手核的十进制值）
  2. m_per_deg 与独立已知值对带（33.87°N 处 1° lat≈110.9km、1° lon≈92.5km）
  3. ENU↔GPS 往返（±400m，误差<1mm）
  4. 尺度：0.001° lat ≈ 110.9m
  5. venue_gps.yaml 全量解析：表长、抄录奇点抽查、补给区中心 ENU 带、
     补给区在飞行区内、起降区半跨 <30m、面积带
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_bridges", "scripts"))
import geo_enu as G  # noqa: E402

import yaml  # noqa: E402

YAML_P = os.path.join(HERE, "..", "src", "zx2026_common", "config",
                      "venue_gps.yaml")
ok = lambda s: print("  [PASS] " + s)


def close(a, b, tol):
    return abs(a - b) <= tol


# 1) dms2deg 已知答案
v = G.dms2deg([113, 42, 26.71])
assert close(v, 113.7074194444, 1e-9), v
v = G.dms2deg([33, 52, 3.53])
assert close(v, 33.8676472222, 1e-9), v
ok("dms2deg 已知答案")

# 2) 弧长独立对带
mx = G.m_per_deg_lon(33.8656)
my = G.m_per_deg_lat(33.8656)
assert 92.3e3 <= mx <= 92.7e3, mx
assert 110.8e3 <= my <= 111.05e3, my
ok("m_per_deg 带 (lon=%.0f lat=%.0f m/deg)" % (mx, my))

# 3) 往返
fr = G.EnuFrame(113.70794515, 33.86555905)
for e, n in [(400, 0), (-400, 0), (0, 400), (0, -400), (300, -250)]:
    lo, la = fr.to_gps(e, n)
    e2, n2 = fr.to_enu(lo, la)
    assert math.hypot(e2 - e, n2 - n) < 1e-3, (e, n, e2, n2)
ok("ENU<->GPS 往返 <1mm")

# 4) 尺度
e, n = fr.to_enu(113.70794515, 33.86555905 + 0.001)
assert close(n, my * 0.001, 0.5) and close(e, 0.0, 1e-6), (e, n)
ok("0.001 deg lat = %.1f m" % n)

# 5) venue_gps.yaml
venue = yaml.safe_load(open(YAML_P))
k3 = venue["venue"]["keyun3"]
k12 = venue["venue"]["keyun12"]
assert len(k3["perimeter"]) == 8
assert len(k3["supply_zone"]) == 4
assert len(k12["perimeter"]) == 25
ok("表长 8/4/25")

fr, m = G.frame_from_venue(venue)
# 抄录奇点抽查（对 docx 原串）
lo, la = fr.to_gps(*m["perimeter"][0])
assert close(lo, 113.70770556, 1e-8) and close(la, 33.86573056, 1e-8)
lo, la = fr.to_gps(*m["supply"][1])
assert close(lo, 113.70960000, 1e-8) and close(la, 33.86233056, 1e-8)
ok("度分秒抄录抽查 (k3#1, supply#2)")

# 补给区中心 ENU 带（手算期望 ≈ E+139 / N-369）
ce = sum(p[0] for p in m["supply"]) / 4.0
cn = sum(p[1] for p in m["supply"]) / 4.0
assert 120 <= ce <= 160, ce
assert -390 <= cn <= -350, cn
ok("补给区中心 ENU=(%.1f, %.1f) m" % (ce, cn))

# 补给区必须整体在飞行区内；起降点在原点近旁
for e, n in m["supply"]:
    assert G.point_in_poly(e, n, m["perimeter"])
for e, n in m["takeoff"]:
    assert max(abs(e), abs(n)) < 30.0, (e, n)
ok("补给区⊂飞行区；起降 4 点半跨 <30m")

# 面积（鞋带公式）带。定谳：补给区=旋转四边形（非正置矩形！），实测 ~287 m²；
# 外接盒 34.4×19.0m 是 bbox 不是形状——读场地数据勿用 bbox 冒充形状。
poly = m["supply"]
A = 0.5 * abs(sum(poly[i][0] * poly[(i + 1) % 4][1]
                  - poly[(i + 1) % 4][0] * poly[i][1] for i in range(4)))
assert 200 <= A <= 400, A
d = G.dist_to_poly(ce, cn, poly)
assert 3 <= d <= 7, d
ok("补给区面积 %.0f m²（旋转四边形）；中心距边 %.1f m" % (A, d))

# point_in_poly 反例
assert not G.point_in_poly(0.0, 0.0, m["supply"])
assert G.point_in_poly(ce, cn, m["supply"])
ok("point_in_poly 内/外判")

print("GEO SELFTEST PASS")
