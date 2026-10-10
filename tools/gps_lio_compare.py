#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gps_lio_compare.py — LIO-ENU 与 GPS-ENU 现场对质仪（2026-10-09）。

用途：真机悬停/摆放静止时，同点对比两种定位源，现场标定 odom_tf 的
yaw0/ex0/ny0，并实测本机 GPS 噪声带（决定 GPS 修正返航目标通道的步长上限）。

用法（真机上，飞行栈+real_fleet.launch 已起）：
  python3 tools/gps_lio_compare.py --id 0            # 1Hz 持续打印，Ctrl-C 结束
  python3 tools/gps_lio_compare.py --id 0 --n 30     # 采 30 拍后出均值建议并退出
输出：
  GPS-ENU(e,n) / LIO-ENU(x,y) / 偏移(dx,dy) / 偏移模长 / GPS 状态与协方差
  结束时给出 N 拍均值偏移 → 现场若把它加进 ex0/ny0（或改 missions 装订）由人工裁决。
口径：
  * ENU 原点 = venue_gps.yaml 的 keyun3 起降区中心（--anchor-lat/--anchor-lon 可覆盖）。
  * yaw0 不可由静点对质标出（静点只有平移自由度）——航向对质须走两点/航线法，
    本工具只管平移与噪声带；头两行会提示。
"""
import argparse
import math
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_bridges", "scripts"))
import geo_enu as G  # noqa: E402

import rospy  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402


def load_frame(args):
    if args.anchor_lat is not None and args.anchor_lon is not None:
        return G.EnuFrame(args.anchor_lon, args.anchor_lat)
    import yaml
    p = os.path.join(HERE, "..", "src", "zx2026_common", "config",
                     "venue_gps.yaml")
    venue = yaml.safe_load(open(p))
    fr, _ = G.frame_from_venue(venue)
    return fr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--n", type=int, default=0, help="采样拍数，0=无限")
    ap.add_argument("--anchor-lon", type=float, default=None)
    ap.add_argument("--anchor-lat", type=float, default=None)
    args = ap.parse_args()

    rospy.init_node("gps_lio_compare", anonymous=True, disable_signals=True)
    fr = load_frame(args)
    print("[gps_lio_compare] 静点对质只标平移(ex0/ny0 方向)；yaw0 须走两点/航线法")

    st = {"gps": None, "odo": None}

    def on_gps(m):
        st["gps"] = m

    def on_odo(m):
        st["odo"] = m

    rospy.Subscriber("/mavros/global_position/global", NavSatFix, on_gps)
    rospy.Subscriber("/drone_%d/odom" % args.id, Odometry, on_odo)

    offs = []
    rate = rospy.Rate(1.0)

    def report():
        g, o = st["gps"], st["odo"]
        if g is None or o is None:
            print("  等待消息 gps=%s odom=%s"
                  % (g is not None, o is not None))
            return
        ge, gn = fr.to_enu(g.latitude, g.longitude)   # 注意顺序：to_enu(lon,lat)
        ge, gn = ge, gn
        px, py = o.pose.pose.position.x, o.pose.pose.position.y
        dx, dy = ge - px, gn - py
        cov = math.sqrt(max(g.position_covariance[0],
                            g.position_covariance[4]))
        status = g.status.status
        print("  GPS-ENU=(%7.2f,%7.2f) LIO=(%7.2f,%7.2f) "
              "off=(%6.2f,%6.2f) |off|=%5.2f gps_status=%d cov~%.2fm"
              % (ge, gn, px, py, dx, dy, math.hypot(dx, dy), status, cov))
        return (dx, dy)

    while not rospy.is_shutdown():
        r = report()
        if r is not None:
            offs.append(r)
            if args.n and len(offs) >= args.n:
                break
        rate.sleep()

    if offs:
        mx = sum(o[0] for o in offs) / len(offs)
        my = sum(o[1] for o in offs) / len(offs)
        sd = math.sqrt(sum((o[0] - mx) ** 2 + (o[1] - my) ** 2
                           for o in offs) / max(1, len(offs) - 1))
        print("[建议] %d 拍均值偏移 GPS-LIO=(%.2f, %.2f) m，散布 σ=%.2f m"
              % (len(offs), mx, my, sd))
        print("       现场处置（人工裁决）：把均值反向计入 odom_tf 的 ex0/ny0，"
              "或据此修订 missions 装订；σ>3m 时 GPS 修正通道应降级为只报不修。")


if __name__ == "__main__":
    main()
