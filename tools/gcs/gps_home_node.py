#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/gps_home_node.py — GPS 锚定返航：起飞记点 + 返航目标修正（真机包 2026-10-09）。

用户需求：真机用 GPS 定位——起飞时记录起飞点 GPS 坐标，完成投放后返回
记录的起飞点。本节点是「目标修正通道」，不夺 nav 笔：

  记点  phase 首次进入 TAKEOFF_PENDING/TAKEOFF（还在坪上）时开窗，
        窗内 status>=0 的 NavSatFix 均值 = home GPS；同窗 odom 均值 =
        home 的 LIO-ENU 读数（对质存档）。
  修正  phase 进入 AT_DROP/DROP（投放悬停，任务末端 GPS 观测最稳时刻）
        开第二窗：offset = mean(odom_xy) - mean(gps_enu)，修正目标 =
        home_gps_enu + offset 发布 /drone_<id>/home/enu（PoseStamped
        world，latched）。executor 在 RETURN 入场换 pad（clamp/告警），
        判完成（LANDING 水平距 pad）随 pad 自动跟随。

数学关键：锚点（venue 原点）在 offset 差值中相消——修正目标只要求
GPS-ENU 与 LIO-ENU 轴向一致（yaw0 现场标定），不要求全局原点对齐；
venue_gps.yaml 仅作诊断（home 是否落在起降区内）。换算用
src/arena_bridges/scripts/geo_enu.py（WGS84 弧长级数，tools/geo_selftest 覆盖）。

质量门：home 窗 fix 数 < min-fix 或散布（各 fix 距均值最大距）> max-spread
→ 拒绝记点（回退 scene pad 原行为），不给爆值目标——replace 模式下
executor 无条件信任本目标，质量门必须在本侧。

诚实边界：
  * sim 无 /mavros/global_position → 恒 NO_FIX 待机，无目标发布，
    executor 收不到修正 = sim 零行为变更（跑批/回归不受影响）。
  * offset 假设「投放时刻 est-gps 偏差 ≈ 返航到达时刻偏差」；LIO 漂移在
    返航腿（分钟级）内累积是固有误差——漂移率用 tools/gps_lio_compare.py
    现场实测后评估（刚性锁存案同源课题）。
  * 本节点只发目标，从不写 vel_cmd/position_cmd——急停/控制链不经过此处。

用法（机载，source 后；real_fleet.launch 自动拉起）：
  python3 gps_home_node.py --id 0 [--home-record-s 3.0 --min-fix 5
                            --max-spread 3.0 --corr-record-s 2.0]
  离线：python3 gps_home_node.py --selftest
"""
import argparse
import json
import math
import os
import sys


def _add_repo_paths():
    """geo_enu.py（src/arena_bridges/scripts）路径注入——机上/WSL 双布局兜底
    （同 arena_bridges shim 的相对回退纪律）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(os.path.dirname(os.path.dirname(here)),
                     "src", "arena_bridges", "scripts"),
        os.path.expanduser("~/zx2026_ego_ws/src/arena_bridges/scripts"),
    ]
    for c in cands:
        if os.path.isfile(os.path.join(c, "geo_enu.py")):
            if c not in sys.path:
                sys.path.insert(0, c)
            return


_add_repo_paths()
import geo_enu  # noqa: E402  (路径注入后导入；纯函数无重依赖)

# ---------------------------------------------------------------- 纯逻辑 --
PAD_PHASES = ("TAKEOFF_PENDING", "TAKEOFF")     # 记点窗：还在坪上/刚离坪
DROP_PHASES = ("AT_DROP", "DROP")               # 修正窗：投放悬停


class GpsHomeLogic(object):
    """记点/修正双窗状态机（纯逻辑，无 ROS，离线可测）。

    enu_of 注入 GPS→ENU 换算闭包（venue 框架或首 fix 锚点兜底，锚点在
    offset 中相消故框架选择不影响修正目标，只影响诊断读数）。
    """

    def __init__(self, enu_of, record_s=3.0, corr_s=2.0, min_fix=5,
                 max_spread=3.0):
        self.enu_of = enu_of
        self.record_s = float(record_s)
        self.corr_s = float(corr_s)
        self.min_fix = int(min_fix)
        self.max_spread = float(max_spread)
        self.phase = ""
        self.home = None       # dict(lon,lat,e,n,odom_xy,n_fix,spread_m)
        self.target = None     # (x, y) LIO-ENU 修正目标
        self.offset = None     # (dx, dy) = odo_mean - gps_mean
        self.corr_done = False
        self.events = []       # (level, text) 待 ROS 侧 drain
        self._win = None

    # -- 输入 --------------------------------------------------------------
    def on_phase(self, ph, now):
        self.phase = ph
        if self._win is None and self.home is None and ph in PAD_PHASES:
            self._win = {"kind": "home", "t0": now, "dur": self.record_s,
                         "gps": [], "odo": []}
            self.events.append(("info", "home GPS recording %.1fs (on pad)"
                                % self.record_s))
        elif (self._win is None and not self.corr_done
              and self.home is not None and ph in DROP_PHASES):
            self._win = {"kind": "corr", "t0": now, "dur": self.corr_s,
                         "gps": [], "odo": []}
            self.events.append(("info", "offset measuring %.1fs (at drop)"
                                % self.corr_s))

    def on_gps(self, lon, lat, status, now):
        if self._win is not None and int(status) >= 0:
            self._win["gps"].append((float(lon), float(lat)))

    def on_odom(self, x, y, now):
        if self._win is not None:
            self._win["odo"].append((float(x), float(y)))

    # -- 窗口收口 ------------------------------------------------------------
    def tick(self, now):
        if self._win is None or now - self._win["t0"] < self._win["dur"]:
            return False
        win = self._win
        self._win = None
        if win["kind"] == "home":
            self._close_home(win)
        else:
            self._close_corr(win)
        return True

    @staticmethod
    def _mean(seq):
        n = len(seq)
        return (sum(p[0] for p in seq) / n, sum(p[1] for p in seq) / n)

    def _close_home(self, win):
        if len(win["gps"]) < self.min_fix:
            self.events.append((
                "warn", "home rejected: %d fix < min_fix %d -> scene pad fallback"
                % (len(win["gps"]), self.min_fix)))
            return
        mlon, mlat = self._mean(win["gps"])
        # 散布：各 fix 距均值的最远距离（m，以均值为局部锚点，框架无关）
        spread = 0.0
        for lo, la in win["gps"]:
            de = (lo - mlon) * self._mlon(mlat)
            dn = (la - mlat) * self._mlat(mlat)
            spread = max(spread, math.hypot(de, dn))
        if spread > self.max_spread:
            self.events.append((
                "warn", "home rejected: spread %.2fm > max_spread %.2fm "
                "(bad fix on pad) -> scene pad fallback"
                % (spread, self.max_spread)))
            return
        e, n = self.enu_of(mlon, mlat)
        odom_xy = self._mean(win["odo"]) if win["odo"] else (0.0, 0.0)
        self.home = {"lon": mlon, "lat": mlat, "e": e, "n": n,
                     "odom_xy": odom_xy, "n_fix": len(win["gps"]),
                     "spread_m": spread}
        self.events.append((
            "info", "home GPS %.7f,%.7f enu=(%.2f,%.2f) n=%d spread=%.2fm"
            % (mlon, mlat, e, n, len(win["gps"]), spread)))

    def _close_corr(self, win):
        if len(win["gps"]) < self.min_fix:
            self.events.append((
                "warn", "offset rejected: %d fix < min_fix %d -> no correction"
                % (len(win["gps"]), self.min_fix)))
            return
        gps_enu = [self.enu_of(lo, la) for lo, la in win["gps"]]
        g = self._mean(gps_enu)
        o = self._mean(win["odo"])
        self.offset = (o[0] - g[0], o[1] - g[1])
        self.target = (self.home["e"] + self.offset[0],
                       self.home["n"] + self.offset[1])
        self.corr_done = True
        self.events.append((
            "info", "home target (%.2f,%.2f) = home_enu + offset(%.2f,%.2f) "
            "n=%d" % (self.target[0], self.target[1],
                      self.offset[0], self.offset[1], len(win["gps"]))))

    @staticmethod
    def _mlat(lat_deg):
        return geo_enu.m_per_deg_lat(lat_deg)

    @staticmethod
    def _mlon(lat_deg):
        return geo_enu.m_per_deg_lon(lat_deg)

    def status(self):
        return {
            "phase": self.phase,
            "home_gps": ([self.home["lon"], self.home["lat"]]
                         if self.home else None),
            "home_enu": ([self.home["e"], self.home["n"]]
                         if self.home else None),
            "home_odom": (list(self.home["odom_xy"]) if self.home else None),
            "n_fix": self.home["n_fix"] if self.home else 0,
            "spread_m": self.home["spread_m"] if self.home else None,
            "offset": (list(self.offset) if self.offset else None),
            "target": (list(self.target) if self.target else None),
            "corr_done": self.corr_done,
            "recording": self._win["kind"] if self._win else None,
        }


# ---------------------------------------------------------------- ROS 壳 --
def _load_venue_frame(path):
    """venue ENU 框架（诊断用：home 落点是否在起降区带内）。失败返 None。"""
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import geo_enu as G
        import yaml
        venue = yaml.safe_load(open(path))
        fr, m = G.frame_from_venue(venue)
        return fr, m
    except Exception:
        return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--gps-topic", default="/mavros/global_position/global")
    ap.add_argument("--record-s", type=float, default=3.0)
    ap.add_argument("--corr-s", type=float, default=2.0)
    ap.add_argument("--min-fix", type=int, default=5)
    ap.add_argument("--max-spread", type=float, default=3.0)
    ap.add_argument("--venue", default="")
    args = ap.parse_args(rospy.myargv()[1:])

    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from sensor_msgs.msg import NavSatFix

    rospy.init_node("gps_home_node", anonymous=True)
    ns = "/drone_%d" % args.id

    # venue 框架：优先包内 venue_gps.yaml；缺失则首 fix 锚点兜底
    # （锚点在 offset 中相消，兜底不影响修正目标，只少诊断读数）
    ws_cfg = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))),
        "src", "zx2026_common", "config", "venue_gps.yaml")
    venue_path = args.venue or ws_cfg
    if not os.path.isfile(venue_path):
        venue_path = os.path.expanduser(
            "~/zx2026_ego_ws/src/zx2026_common/config/venue_gps.yaml")
    fr, venue_poly = _load_venue_frame(venue_path) if os.path.isfile(
        venue_path) else (None, None)

    holder = {"frame": fr}

    def enu_of(lon, lat):
        if holder["frame"] is None:
            import geo_enu
            holder["frame"] = geo_enu.EnuFrame(lon, lat)   # 首 fix 锚点
        return holder["frame"].to_enu(lon, lat)

    logic = GpsHomeLogic(enu_of, record_s=args.record_s, corr_s=args.corr_s,
                         min_fix=args.min_fix, max_spread=args.max_spread)

    pub_home = rospy.Publisher(ns + "/home/enu", PoseStamped,
                               queue_size=1, latch=True)
    pub_status = rospy.Publisher(ns + "/home/status", String,
                                 queue_size=1, latch=True)

    def on_gps(m):
        logic.on_gps(m.longitude, m.latitude, m.status.status,
                     rospy.get_time())

    def on_odom(m):
        p = m.pose.pose.position
        logic.on_odom(p.x, p.y, rospy.get_time())

    def on_phase(m):
        logic.on_phase(m.data, rospy.get_time())

    rospy.Subscriber(args.gps_topic, NavSatFix, on_gps, queue_size=10)
    rospy.Subscriber(ns + "/odom", Odometry, on_odom, queue_size=10)
    rospy.Subscriber(ns + "/mission/phase", String, on_phase, queue_size=2)

    def drain_events():
        for lvl, text in logic.events:
            (rospy.logwarn if lvl == "warn" else rospy.loginfo)(
                "gps_home: %s", text)
        del logic.events[:]

    def publish_status():
        drain_events()
        pub_status.publish(String(data=json.dumps(logic.status())))

    def tick(_evt):
        if logic.tick(rospy.get_time()) and logic.target is not None:
            g = PoseStamped()
            g.header.frame_id = "world"
            g.header.stamp = rospy.Time.now()
            g.pose.position.x = logic.target[0]
            g.pose.position.y = logic.target[1]
            g.pose.orientation.w = 1.0
            pub_home.publish(g)

    rospy.Timer(rospy.Duration(0.2), tick)
    rospy.Timer(rospy.Duration(1.0), lambda _e: publish_status())

    # 起降区带诊断（venue 可得时）：home 应距起降区中心 <30m（zone 半跨）
    def venue_check():
        if fr is None or venue_poly is None or logic.home is None:
            return
        e, n = logic.home["e"], logic.home["n"]
        take = venue_poly.get("takeoff") or []
        if take:
            ce = sum(p[0] for p in take) / len(take)
            cn = sum(p[1] for p in take) / len(take)
            d = math.hypot(e - ce, n - cn)
            if d > 30.0:
                rospy.logwarn("gps_home: home %.1fm off takeoff-zone center "
                              "(>30m)——GPS-ENU 轴向/锚点疑失准，核查 yaw0",
                              d)

    rospy.Timer(rospy.Duration(5.0), lambda _e: venue_check())
    rospy.loginfo("gps_home_node: drone %d ready (gps=%s)", args.id,
                  args.gps_topic)
    rospy.spin()


def _selftest():
    """纯逻辑离线自检（--selftest）。"""
    # 锚点相消性：不同 ENU 锚点下修正目标一致（锚点在 offset 差值中相消）
    fr1 = geo_enu.EnuFrame(113.70, 33.86)
    fr2 = geo_enu.EnuFrame(113.71, 33.87)   # 平移锚点（米制尺度同纬度量级）
    lg1 = GpsHomeLogic(fr1.to_enu)
    lg2 = GpsHomeLogic(fr2.to_enu)
    now = [0.0]

    def feed(lg):
        lg.on_phase("TAKEOFF_PENDING", 0.0)
        home_gps = (113.700010, 33.860015)
        for k in range(5):
            now[0] = 0.1 + k * 0.2
            lg.on_gps(home_gps[0], home_gps[1], 0, now[0])
            lg.on_odom(11.0, 7.0, now[0])
        assert lg.tick(0.2 + 3.0), "home window must close at record_s"
        assert lg.home is not None
        lg.on_phase("DROP", 5.0)
        # 投放点：GPS 再东移 0.001°，odom 读数 = 该点 ENU + (0.5, 0)
        # （est 在 GPS 东侧 0.5m 的刚性偏移）→ 修正目标 = home_enu + (0.5, 0)
        glon, glat = 113.701010, 33.860015
        ge, gn = fr1.to_enu(glon, glat)
        for k in range(5):
            now[0] = 5.2 + k * 0.2
            lg.on_gps(glon, glat, 0, now[0])
            lg.on_odom(ge + 0.5, gn, now[0])
        assert lg.tick(5.4 + 2.0)
        assert lg.corr_done and lg.target is not None
        return lg

    feed(lg1)
    feed(lg2)
    # 目标 = home_enu + offset(0.5, 0)（自洽推导，不依赖弧长常数）
    he, hn = fr1.to_enu(113.700010, 33.860015)
    t1, t2 = lg1.target, lg2.target
    assert abs(t1[0] - (he + 0.5)) < 1e-3, (t1, he)
    assert abs(t1[1] - hn) < 1e-3, (t1, hn)
    # 锚点相消：两框架目标差 <5cm（尺度差引入的极小项）
    assert abs(t1[0] - t2[0]) < 0.05 and abs(t1[1] - t2[1]) < 0.05, (t1, t2)

    # 质量门：fix 不足 → 拒绝
    lg3 = GpsHomeLogic(fr1.to_enu, min_fix=5)
    lg3.on_phase("TAKEOFF", 0.0)
    for k in range(3):
        lg3.on_gps(113.70, 33.86, 0, 0.1 + k * 0.1)
        lg3.on_odom(0, 0, 0.1 + k * 0.1)
    lg3.tick(3.5)
    assert lg3.home is None, "too few fixes must reject home"
    # 质量门：散布爆表 → 拒绝
    lg4 = GpsHomeLogic(fr1.to_enu, max_spread=3.0)
    lg4.on_phase("TAKEOFF", 0.0)
    for k, (lo, la) in enumerate([(113.70, 33.86), (113.7003, 33.86),
                                  (113.70, 33.8603), (113.7003, 33.8603),
                                  (113.70015, 33.86015)]):
        lg4.on_gps(lo, la, 0, 0.1 + k * 0.1)
        lg4.on_odom(0, 0, 0.1 + k * 0.1)
    lg4.tick(3.5)
    assert lg4.home is None, "blown spread must reject home (~40m apart)"
    # 状态 JSON 可序列化
    json.dumps(lg1.status())
    lg5 = GpsHomeLogic(fr1.to_enu)
    json.dumps(lg5.status())
    print("GPS_HOME SELFTEST PASS")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
    else:
        main()
