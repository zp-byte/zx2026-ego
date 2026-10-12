#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/cam_tilt_calib.py — D435 安装下俯角 θ 现场标定（2026-10-12）。

【第一风险】real_fleet.launch cam_tilt_deg 现值 0=水平假设（real_d435.launch
标【现场必测】）。装订误差 Δ 在距离 d 处造成 d·sinΔ 的系统性 z 偏移，直接把
前方树推出/推进 nav 证据带（巡航 2.4m 时树干可见段只有 world 1.8-2.4 的
0.6m）：Δ=1° @4m→0.07m 可接受（装订目标 ±1°）；3°→0.21m 吃掉可见段 1/3；
10°→0.69m 可见段全灭=对该树全盲。防线阈 2°（0.14m@4m≈可见段 1/4）。

原理——地面本身就是标定板。相机光学系（x右 y下 z前）里水平地面满足
  y_c = h/cosθ − z_c·tanθ          （h = 光心离地高）
对带内（默认 1.5-4.0m）地面回波做 y-z 稳健直线拟合（两遍 3σ 剔离群）：
  斜率 = −tanθ → θ = atan(−slope)；截距 a = h/cosθ → h = a·cosθ（卷尺互证）。
θ 是相机↔机体安装角：机体贴地水平时=相对重力角 → 不需要 VINS/起飞/odom，
笔记本起 rs_camera 就够。投影复用 cloud_adapter.cam_to_world——标定值即
四道滤适配器所消费的值，同一套投影数学，无第二套真相。

现场序列（清理区，约 10 分钟）：
  1. 对向两拍：朝向 A --live --save cap_a.txt，转身 180° cap_b.txt——
     对向平均消机体俯仰/地面坡度残差（两拍符号相反）；半差 >0.5° =
     机架或地面不平，先换地或修机械
  2. 机械交叉：卷尺量安装座 θ_tape=atan(Δh/斜边)。与本值差 >1° = 机械
     没靠规，先修机械再标——本模型只有 pitch 一个自由度，roll/yaw 歪了
     数据修不了
  3. 装订：real_fleet.launch 与 real_d435.launch 两处 cam_tilt_deg 同值
     （同名 arg 纪律）；cam_dz = 光心离地 h − 机体原点离地（后者卷尺量）
  4. 闭环确认：--bound <装订值> 重跑 → Δ≈0 且投影残余倾角≈0/带宽<6cm；
     再摆 1.5m 高杆于 2m/4m 处，rosbag play 后 --live 验杆顶 world
     z≈1.5±0.2 且存活 z 带（树不被推出带的要害方向）

用法：
  python3 cam_tilt_calib.py --live --save cap_a.txt      # 采拍存档
  python3 cam_tilt_calib.py cap_a.txt cap_b.txt          # 对向标定→装订建议
  python3 cam_tilt_calib.py cap_a.txt --bound 9.9        # 防线复检（>2° 报警）
  python3 cam_tilt_calib.py --selftest                   # 无 ROS 干测

bag 回放=另终端 rosbag play 后 --live，不设专门 bag 档。存档只写带内点
（D435 整帧 40 万行不可存）。部署自动随 tools/gcs 整目录上机——地面标定
工具不进 launch，无需 arena_bridges shim。报警退出码 1（可接脚本闸）。
"""
import argparse
import math
import os
import sys

from cloud_adapter import cam_to_world, parse_xyz

# ------------------------------------------------------------- 默认参 --
BAND_LO, BAND_HI = 1.5, 4.0   # 拟合带（相机系 z 前向，m）——贴地静态时该带
                              # 恒是地面：D435 竖直 FoV ±29°、光心 0.2-0.5m
                              # 高 → 地面从 ~0.4-0.9m 起可见（θ 下俯更早）
MIN_N = 500         # 拟合最少带内点
SIGMA_MAX = 0.10    # 地面拟合残差 σ 上限（超=场景不像平地）
Z_SPAN_MIN = 1.0    # 带内 z 展开下限（对墙/遮挡=无展开）
TILT_ABS_MAX = 60.0  # θ 合理范围（超出=拟合没收敛/场景不对）
PAIR_TOL_DEG = 0.5  # 对向半差告警阈
ALARM_DEG = 2.0     # 防线阈：observed vs bound
SPREAD_MAX = 0.06   # 装订后投影残差带宽告警（m，≈1.4° 等效）


class FitError(Exception):
    """场景不对/数据不足——信息直接面向现场操作者。"""


# ------------------------------------------------------------- 纯逻辑 --
def band_points(pts, z_lo=BAND_LO, z_hi=BAND_HI):
    """带内有效点（z 前向距离窗；NaN/inf 在此防毒跳过）。"""
    out = []
    for (x, y, z) in pts:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if z_lo <= z <= z_hi:
            out.append((x, y, z))
    return out


def fit_line_yz(pts):
    """稳健拟合 y = a + b·z（两遍 3σ 剔离群）→ (a, b, n0, n_keep, sigma)。"""
    n0 = len(pts)
    cur = list(pts)
    a, b, sigma = 0.0, 0.0, 0.0
    for _ in range(2):
        n = len(cur)
        mz = sum(p[2] for p in cur) / n
        my = sum(p[1] for p in cur) / n
        szz = sum((p[2] - mz) ** 2 for p in cur)
        if szz < 1e-9:
            raise FitError("z 无展开——镜头对墙/遮挡，不是开阔地面")
        b = sum((p[2] - mz) * (p[1] - my) for p in cur) / szz
        a = my - b * mz
        sq = [(p[1] - (a + b * p[2])) ** 2 for p in cur]
        m2 = sum(sq) / n
        sigma = math.sqrt(m2)
        keep = [p for p, s in zip(cur, sq) if s <= 9.0 * m2]
        if len(keep) == n or len(keep) < MIN_N:
            break
        cur = keep
    return a, b, n0, len(cur), sigma


def tilt_from_fit(a, b):
    """拟合 → (θ°, h)：y_c = h/cosθ − z_c·tanθ 的逆。"""
    theta = math.degrees(math.atan(-b))
    return theta, a * math.cos(math.radians(theta))


def calibrate(pts, z_lo=BAND_LO, z_hi=BAND_HI):
    """一拍点云 → 标定结果 dict；场景不对抛 FitError。"""
    band = band_points(pts, z_lo, z_hi)
    if len(band) < MIN_N:
        raise FitError("带内点 %d < %d——镜头朝向/曝光/深度档不对"
                       % (len(band), MIN_N))
    zs = [p[2] for p in band]
    span = max(zs) - min(zs)
    if span < Z_SPAN_MIN:
        raise FitError("带内 z 展开 %.2fm < %.1fm——镜头对墙/遮挡"
                       % (span, Z_SPAN_MIN))
    a, b, n0, nk, sigma = fit_line_yz(band)
    if sigma > SIGMA_MAX:
        raise FitError("地面拟合 σ=%.3fm > %.2f——场景不像平地（人/物占主导）"
                       % (sigma, SIGMA_MAX))
    if nk < 0.5 * n0:
        raise FitError("离群剔除吃掉 >50%% 点（%d/%d）——场景不像平地"
                       % (nk, n0))
    theta, h = tilt_from_fit(a, b)
    if abs(theta) > TILT_ABS_MAX:
        raise FitError("θ=%+.1f° 超出 ±%.0f° 合理安装范围——拟合没收敛"
                       % (theta, TILT_ABS_MAX))
    return {"theta": theta, "h": h, "sigma": sigma,
            "n0": n0, "nk": nk, "slope": b}


def project_spread(pts, tilt_deg, z_lo=BAND_LO, z_hi=BAND_HI):
    """装订 θ 后的闭环确认：地面点过 cam_to_world（恒等 odom）应落在
    水平面（world z 常数 −h）。返回 (残余倾角°, 残差带宽 p5-p95 m)。
    mount 平移只造成常量偏移不改变斜率——此检查只对质 tilt。"""
    band = band_points(pts, z_lo, z_hi)
    body = cam_to_world(band, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                        tilt_deg, (0.0, 0.0, 0.0))
    zs = sorted(p[2] for p in body)
    n = len(zs)
    p5, p95 = zs[int(0.05 * (n - 1))], zs[int(0.95 * (n - 1))]
    _, slope, _, _, _ = fit_line_yz([(0.0, p[2], p[0]) for p in body])
    return math.degrees(math.atan(slope)), p95 - p5


def synth_points(theta_deg, h, n=20000, noise=0.01, seed=7,
                 poles=500, nans=200):
    """干测地景：已知 θ/h 的水平地面 + 杆状离群 + NaN 毒点 + 带外点。"""
    import random
    rnd = random.Random(seed)
    t = math.radians(theta_deg)
    pts = []
    for _ in range(n):
        z = rnd.uniform(BAND_LO, BAND_HI)
        y = h / math.cos(t) - z * math.tan(t) + rnd.gauss(0.0, noise)
        pts.append((rnd.uniform(-2.0, 2.0), y, z))
    for _ in range(poles):   # 竖杆：z 邻域固定、y 大幅散开
        pts.append((rnd.uniform(0.4, 0.6), rnd.uniform(-0.3, 0.5),
                    rnd.uniform(2.8, 3.2)))
    pts.append((0.0, 0.0, 1.0))                 # 带外
    for _ in range(nans):
        pts.append((float("nan"), 0.0, 2.0))    # 毒点
    return pts


def load_capture(path):
    """存档 txt（x y z 每行）→ 点列；坏行跳过（band 滤再兜一道）。"""
    pts = []
    with open(path) as f:
        for ln in f:
            w = ln.split()
            if len(w) < 3:
                continue
            try:
                pts.append((float(w[0]), float(w[1]), float(w[2])))
            except ValueError:
                continue
    return pts


def selftest():
    n = [0]

    def ck(cond, name):
        assert cond, name
        n[0] += 1

    # 带滤：带外 + NaN 防毒
    pts = synth_points(8.0, 0.35)
    ck(len(band_points(pts)) == 20000 + 500, "band 计数（剔带外+NaN）")
    # 主案例：θ=8° h=0.35 还原
    r = calibrate(pts)
    ck(abs(r["theta"] - 8.0) < 0.15, "θ 还原 8°±0.15（got %.3f）" % r["theta"])
    ck(abs(r["h"] - 0.35) < 0.02, "h 还原 0.35±0.02（got %.3f）" % r["h"])
    ck(r["sigma"] < 0.02, "σ≈噪声 0.01（got %.4f）" % r["sigma"])
    ck(r["nk"] < r["n0"], "杆状离群被剔")
    # 零角
    r0 = calibrate(synth_points(0.0, 0.40, seed=11))
    ck(abs(r0["theta"]) < 0.15, "θ=0 还原（got %.3f）" % r0["theta"])
    # 闭环投影：装订=真值 → 残余倾角≈0、带宽≈噪声
    rt, sp = project_spread(pts, 8.0)
    ck(abs(rt) < 0.15, "投影残余倾角≈0（got %.3f）" % rt)
    ck(sp < 0.045, "投影带宽≈2×1.645σ≈3.3cm（got %.4f）" % sp)
    # 防线数学：真值 10.5 vs bound 8 → Δ=2.5 触警；投影残余倾角同号同量
    r2 = calibrate(synth_points(10.5, 0.35, seed=13))
    ck(abs((r2["theta"] - 8.0) - 2.5) < 0.15, "Δ 还原 2.5±0.15")
    ck(abs(r2["theta"] - 8.0) > ALARM_DEG, "Δ=2.5 > 阈 2.0 触警")
    rt2, _ = project_spread(synth_points(10.5, 0.35, seed=13), 8.0)
    ck(abs(rt2 - 2.5) < 0.3, "投影残余倾角≈Δ=2.5（got %.3f）" % rt2)
    # 对向对：半差/均值
    ra = calibrate(synth_points(9.0, 0.35, seed=3, n=6000, poles=200))
    rb = calibrate(synth_points(9.4, 0.35, seed=5, n=6000, poles=200))
    half = abs(ra["theta"] - rb["theta"]) / 2.0
    mean = (ra["theta"] + rb["theta"]) / 2.0
    ck(half <= PAIR_TOL_DEG, "对向半差 0.2≤0.5（got %.3f）" % half)
    ck(abs(mean - 9.2) < 0.15, "对向均值 9.2±0.15（got %.3f）" % mean)
    # 存档往返：写→读→同拍标定一致
    import tempfile
    fd, path = tempfile.mkstemp(suffix=".txt")
    try:
        with os.fdopen(fd, "w") as f:
            f.write("".join("%.6f %.6f %.6f\n" % p
                            for p in band_points(synth_points(
                                9.0, 0.32, seed=17, n=3000,
                                poles=0, nans=0))))
        got = load_capture(path)
        ck(len(got) == 3000, "存档行数=带内点数（got %d）" % len(got))
        rc = calibrate(got)
        ck(abs(rc["theta"] - 9.0) < 0.2,
           "存档往返 θ 还原（got %.3f）" % rc["theta"])
        ck(abs(rc["h"] - 0.32) < 0.03, "存档往返 h 还原")
    finally:
        os.unlink(path)
    # FitError 三路：点太少 / 对墙无展开 / σ 爆
    for bad, name in [
            ([(0.0, 0.0, 2.0)] * 10, "点太少"),
            ([(0.0, 0.0, 2.0)] * 1000, "对墙无展开")]:
        try:
            calibrate(bad)
            ck(False, name + " 应抛 FitError")
        except FitError:
            n[0] += 1
    try:
        calibrate(synth_points(8.0, 0.35, noise=0.3, n=3000,
                               poles=0, nans=0))
        ck(False, "σ 爆 应抛 FitError")
    except FitError:
        n[0] += 1
    print("selftest OK (%d checks)" % n[0])


# ------------------------------------------------------------------ ROS --
def capture_live(topic, frames, seconds, save, z_lo, z_hi):
    """订 topic 聚合 frames 帧（或 seconds 超时）→ 带内点列。"""
    import rospy
    from sensor_msgs.msg import PointCloud2
    rospy.init_node("cam_tilt_calib", anonymous=True)
    got = {"n": 0, "pts": []}

    def cb(m):
        got["pts"].extend(band_points(parse_xyz(m), z_lo, z_hi))
        got["n"] += 1

    rospy.Subscriber(topic, PointCloud2, cb, queue_size=2)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and got["n"] < frames \
            and rospy.get_time() - t0 < seconds:
        rospy.sleep(0.1)
    if got["n"] == 0:
        raise FitError("topic %s %.0fs 内零帧——rs_camera 没起/话题名不对"
                       "（rostopic hz %s 核）" % (topic, seconds, topic))
    print("== 采集: %d 帧 / %d 带内点（topic %s）"
          % (got["n"], len(got["pts"]), topic))
    if save:
        with open(save, "w") as f:
            f.write("".join("%.6f %.6f %.6f\n" % p for p in got["pts"]))
        print("== 存档: %s（%d 行，只存带内点）" % (save, len(got["pts"])))
    return got["pts"]


def main():
    ap = argparse.ArgumentParser(
        description="D435 安装下俯角 θ 现场标定（地面拟合，无需起飞/VINS）")
    ap.add_argument("captures", nargs="*", metavar="存档",
                    help="存档 txt（x y z 每行，--live --save 产出）")
    ap.add_argument("--live", action="store_true", help="订一拍点云")
    ap.add_argument("--topic", default="/camera/depth/color/points",
                    help="深度点云话题（=real_fleet cloud_src 默认）")
    ap.add_argument("--frames", type=int, default=5,
                    help="--live 聚合帧数")
    ap.add_argument("--seconds", type=float, default=15.0,
                    help="--live 采齐超时（s）")
    ap.add_argument("--save", default="", help="--live 存档路径")
    ap.add_argument("--bound", type=float, default=None,
                    help="防线复检：已装订 θ（°），|Δ|>%.0f 报警" % ALARM_DEG)
    ap.add_argument("--zlo", type=float, default=BAND_LO)
    ap.add_argument("--zhi", type=float, default=BAND_HI)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return 0

    caps = []
    if args.live:
        caps.append(("live", capture_live(args.topic, args.frames,
                                          args.seconds, args.save,
                                          args.zlo, args.zhi)))
    for path in args.captures:
        caps.append((os.path.basename(path), load_capture(path)))
    if not caps:
        ap.error("给 --live 或存档文件（干测看 --selftest）")

    alarm = False
    results = []   # (r, pts)
    for tag, pts in caps:
        try:
            r = calibrate(pts, args.zlo, args.zhi)
        except FitError as e:
            print("== 拍 %s: 不可用——%s" % (tag, e))
            alarm = True
            continue
        r["tag"] = tag
        results.append((r, pts))
        print("== 拍 %s: 带内 %d 点 剔 %d 离群 | slope=%+.4f θ=%+.2f° "
              "h=%.3fm σ=%.3fm"
              % (tag, r["n0"], r["n0"] - r["nk"], r["slope"],
                 r["theta"], r["h"], r["sigma"]))
    if not results:
        return 1

    thetas = [r["theta"] for r, _ in results]
    hmean = sum(r["h"] for r, _ in results) / len(results)
    mean = sum(thetas) / len(thetas)

    if len(results) >= 2:
        half = (max(thetas) - min(thetas)) / 2.0
        bad = half > PAIR_TOL_DEG
        alarm = alarm or bad
        print("== 对向 %d 拍: 均值 θ=%+.2f° 半差 %.2f°（阈 %.1f°）%s"
              % (len(results), mean, half, PAIR_TOL_DEG,
                 ("⚠ 超——机架/地面不平，先换地或修机械" if bad else "OK")))

    if args.bound is not None:
        for r, pts in results:
            d = r["theta"] - args.bound
            bad = abs(d) > ALARM_DEG
            rt, sp = project_spread(pts, args.bound, args.zlo, args.zhi)
            bad2 = sp > SPREAD_MAX
            alarm = alarm or bad or bad2
            print("== 防线复检 bound=%+.1f°: 拍 %s Δ=%+.2f° %s | "
                  "投影残余倾角 %+.2f° 带宽 %.1fcm %s"
                  % (args.bound, r["tag"], d,
                     ("⚠ 超防线阈 %.0f°" % ALARM_DEG if bad else "OK"),
                     rt, sp * 100,
                     ("⚠ 带宽超 %.0fcm" % (SPREAD_MAX * 100)
                      if bad2 else "OK")))
    else:
        print("== 装订建议: cam_tilt_deg = %.1f" % mean)
        print("   （real_fleet.launch 与 real_d435.launch 两处同值——"
              "同名 arg 纪律）")
        print("   h=%.2fm=光心离地 → cam_dz = %.2f − 机体原点离地"
              "（卷尺量后者）" % (hmean, hmean))
        print("   机械交叉：卷尺 θ_tape 与本值差 >1° = 没靠规，先修机械再标")
        print("   装订后闭环：--bound %.1f 重跑本命令 + 1.5m 高杆 @2m/4m 验证"
              % mean)

    return 1 if alarm else 0


if __name__ == "__main__":
    sys.exit(main())
