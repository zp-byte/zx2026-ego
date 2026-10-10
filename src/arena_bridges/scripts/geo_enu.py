#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""geo_enu.py — GPS(经纬度) ↔ 场地 ENU 纯函数换算（真机 GPS 返航的地基）。

口径定谳（2026-10-09）：
  * ENU 原点 = 科目三起降区中心（perimeter 点 1/2/3/8 的经纬度均值，运行时算；
    venue_gps.yaml 只存 docx 原始度分秒，仓库不出现手算派生数）。
  * x=东 y=北 z=上。与 odom_tf 的场地 ENU 同名同向——yaw0/ex0/ny0 现场标定后
    LIO-ENU 与 GPS-ENU 才是同一张图；对质手段 = tools/geo_selftest + 现场
    「同一点两种来源读数比对」。
  * 平面切拍近似：场地 ≤1.1km，曲率误差 ~2cm，比 GPS 单点噪声(2-5m)小两个量级；
    子午/卯酉圈弧长用 WGS84 级数（相对误差 <1e-6）。
  * 本模块零 ROS 依赖（纯函数），可离线自检：tools/geo_selftest.py。

消费方：gps_home_node.py（起飞点记录/返航目标）、drop 对齐节点（可选）、
        现场标定对质工具。
"""
import math

# WGS84 子午/卯酉圈弧长（m/deg），入口：纬度 deg
# 依据：标准级数（愈近极点卯酉圈愈小），<1e-6 相对误差，远优于常数 111320。


def m_per_deg_lat(lat_deg):
    p = math.radians(lat_deg)
    return (111132.92 - 559.82 * math.cos(2 * p)
            + 1.175 * math.cos(4 * p) - 0.0023 * math.cos(6 * p))


def m_per_deg_lon(lat_deg):
    p = math.radians(lat_deg)
    return (111412.84 * math.cos(p) - 93.5 * math.cos(3 * p)
            + 0.118 * math.cos(5 * p))


def dms2deg(dms):
    """[度, 分, 秒] → 十进制度。容负度值（不预期用到，纯防御）。"""
    d, m, s = float(dms[0]), float(dms[1]), float(dms[2])
    sign = -1.0 if d < 0 else 1.0
    return sign * (abs(d) + m / 60.0 + s / 3600.0)


class EnuFrame(object):
    """以锚点经纬度为原点的平面 ENU 框架。"""

    def __init__(self, anchor_lon_deg, anchor_lat_deg):
        self.lon0 = float(anchor_lon_deg)
        self.lat0 = float(anchor_lat_deg)
        self._mx = m_per_deg_lon(self.lat0)
        self._my = m_per_deg_lat(self.lat0)

    def to_enu(self, lon_deg, lat_deg):
        return ((lon_deg - self.lon0) * self._mx,
                (lat_deg - self.lat0) * self._my)

    def to_gps(self, e, n):
        return (self.lon0 + e / self._mx, self.lat0 + n / self._my)


def mean_anchor(pts_deg):
    """锚点 = 点集经纬度均值。pts_deg: [(lon_deg, lat_deg), ...]"""
    n = len(pts_deg)
    return (sum(p[0] for p in pts_deg) / float(n),
            sum(p[1] for p in pts_deg) / float(n))


def frame_from_venue(venue, key="keyun3"):
    """从 venue_gps.yaml 的 dict 建框架（原点=takeoff_zone_pts 均值）。
    返回 (EnuFrame, {'perimeter': [...], 'supply': [...], 'takeoff': [...]})
    每个 '*' 为 [(e, n), ...]（ENU 米）。"""
    kv = venue["venue"][key]
    per = [(dms2deg(p["lon"]), dms2deg(p["lat"])) for p in kv["perimeter"]]
    sup = [(dms2deg(p["lon"]), dms2deg(p["lat"]))
           for p in kv.get("supply_zone", [])]
    idx = [i - 1 for i in kv.get("takeoff_zone_pts", [1, 2, 3, 8])]
    take = [per[i] for i in idx if 0 <= i < len(per)]
    lon0, lat0 = mean_anchor(take)
    fr = EnuFrame(lon0, lat0)
    conv = lambda seq: [fr.to_enu(lo, la) for (lo, la) in seq]
    return fr, {"perimeter": conv(per), "supply": conv(sup),
                "takeoff": conv(take)}


def point_in_poly(e, n, poly):
    """射线法。poly: [(e, n), ...] 顶点序（顺逆时针均可）。"""
    inside = False
    k = len(poly)
    j = k - 1
    for i in range(k):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > n) != (yj > n):
            xc = (xj - xi) * (n - yi) / (yj - yi) + xi
            if e < xc:
                inside = not inside
        j = i
    return inside


def dist_to_poly(e, n, poly):
    """点到多边形边界的最小距离（m）。内部点也返回到边距离（geofence 裕度用）。"""
    best = float("inf")
    k = len(poly)
    j = k - 1
    for i in range(k):
        xi, yi = poly[i]
        xj, yj = poly[j]
        dx, dy = xj - xi, yj - yi
        L2 = dx * dx + dy * dy
        if L2 <= 1e-12:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((e - xi) * dx + (n - yi) * dy) / L2))
        ex, ey = xi + t * dx - e, yi + t * dy - n
        d = math.hypot(ex, ey)
        if d < best:
            best = d
        j = i
    return best
