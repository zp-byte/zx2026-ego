#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""realpack_gps_selftest.py — 真机包 GPS/RGB/投放链 总装自检（2026-10-09）。

四层，任一失败打印 FAIL 行并 exit 1：
  L1  三节点 --selftest 子进程实跑（gps_home/drop_align/payload_bridge，
      tools/gcs 单一源；src/arena_bridges 同名文件是 roslaunch shim，
      单一源纪律见 shim 头注释）。
  L2  mission_executor._home_pad_apply 纯函数直测（None 恒等/replace 接管/
      correct 限额内采用/爆值拒用/replace 不设上限——executor RETURN 入场
      pad 修正的五种语义）。
  L3  venue_gps.yaml + geo_enu 换算烟测：框架可建、起降 4 点贴原点、周界
      包络 ≈221×423m（docx 官方数）、补给区在南方、point_in_poly 自洽。
      （venue 数来自桌面《比赛坐标》两 docx 抄录，本层防抄录/换算回归。）
  L4  GCS 透出链断言：agent._blank/snapshot 与 hub status_writer 含
      home_gps/align 键（文本级，防后续改版漏键）；两个 real profile
      YAML 解析 + topics 接线齐全；sim profile 不含两键（sim 栈无发布者）。
  L5  C100 下视链对质（2026-10-10 定谳，纯文件级零依赖）：real_fleet 与
      real_d435 两文件 camera_topic/camera_vfov_deg 同值（pass_all_args
      覆盖链地雷，2026-10-09 死话题定谳的回归闸）；camera_down.launch
      XML 可解析 + usb_cam_node + /dev/camera_down 软链；preflight ⑥
      指向 /camera_down/image_raw 且 encoding 探针与消费端白名单同源；
      消费端 to_bgr 拒帧门 / AXES_ALIAS 正名在位。
  L6  docx 黄金 24 数 + 需求②③装订（2026-10-11 GPS 换血）：keyun3 周界
      8 点+补给区 4 点×(lon,lat) 硬编码逐位比对 venue_gps.yaml（L3 只验
      换算自洽，本层验「抄进库的数=官方原文」）；launch gps_home 默认开/
      gps_home_mode 默认 replace/节点块在位；preflight ④；executor 读参/
      订阅 /home/enu/RETURN 消费 _home_pad_apply。

用法（须 ROS 环境）：
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 tools/realpack_gps_selftest.py
"""
import math
import os
import subprocess
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
REAL = os.path.join(WS, "real", "orin_nx")
sys.path.insert(0, os.path.join(WS, "devel", "lib", "python3", "dist-packages"))

FAILS = []
SKIPS = []
# 需 catkin 构建才可得的模块（生成 msg / 已安装包）：在**未构建**的树里
# 显式 SKIP 并打印原因，绝不当 PASS；在已构建 ws 里零 SKIP（权威读数）。
CATKIN_ONLY = ("zx2026_common", "rospy", "arena_mission", "arena_nav",
               "arena_sensor", "arena_bridges", "arena_world")


def check(name, cond, detail=""):
    if cond:
        print("  ok  %s" % name)
    else:
        FAILS.append(name)
        print("  FAIL %s %s" % (name, detail))


def run_layer(fn, label):
    """跑一层；仅当缺的是 catkin 构建产物时 SKIP（打印缺哪个模块），
    其余 ModuleNotFoundError 照旧炸出去（真断链不许被吞）。"""
    try:
        fn()
    except ModuleNotFoundError as e:
        miss = getattr(e, "name", None) or str(e)
        if miss.split(".")[0] in CATKIN_ONLY:
            SKIPS.append("%s 需 catkin 构建(no module %s)" % (label, miss))
            print("  skip %s 需 catkin 构建：no module %s" % (label, miss))
        else:
            raise


# ---- L1 三节点 --selftest ------------------------------------------------
def l1_nodes():
    print("[L1] 三节点 --selftest（子进程实跑）")
    cases = [("gps_home_node.py", "GPS_HOME SELFTEST PASS"),
             ("drop_align_node.py", "DROP_ALIGN SELFTEST PASS"),
             ("payload_bridge_node.py", "PAYLOAD_BRIDGE SELFTEST PASS")]
    for fn, marker in cases:
        p = os.path.join(TOOLS, "gcs", fn)
        r = subprocess.run([sys.executable, p, "--selftest"],
                           capture_output=True, text=True, timeout=60)
        out = (r.stdout or "") + (r.stderr or "")
        check(fn, r.returncode == 0 and marker in out,
              "rc=%d marker%s" % (r.returncode,
                                  "" if marker in out else " 缺"))


# ---- L2 _home_pad_apply --------------------------------------------------
def l2_home_pad():
    print("[L2] mission_executor._home_pad_apply 纯函数")
    import importlib.util
    src = os.path.join(WS, "src", "arena_mission", "scripts",
                       "mission_executor_node.py")
    spec = importlib.util.spec_from_file_location("mx_node_st", src)
    mx = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mx)
    f = mx._home_pad_apply
    pad = (1.0, 2.0)
    r = f(pad, None, "replace", 5.0)
    check("corr=None 恒等", r == (pad, False, 0.0), str(r))
    r = f((0.0, 0.0), (1.0, 2.0), "replace", 5.0)
    check("replace 接管", r[0] == (1.0, 2.0) and r[1] is True
          and abs(r[2] - math.sqrt(5.0)) < 1e-9, str(r))
    r = f((0.0, 0.0), (1.0, 1.0), "correct", 5.0)
    check("correct 限额内采用", r[0] == (1.0, 1.0) and r[1] is True, str(r))
    r = f((0.0, 0.0), (3.0, 4.1), "correct", 5.0)
    check("correct 爆值拒用", r[0] == (0.0, 0.0) and r[1] is False
          and abs(r[2] - math.hypot(3.0, 4.1)) < 1e-9, str(r))
    r = f((0.0, 0.0), (30.0, 0.0), "replace", 5.0)
    check("replace 不设上限", r[1] is True and r[0] == (30.0, 0.0), str(r))


# ---- L3 venue + geo_enu --------------------------------------------------
def l3_venue():
    print("[L3] venue_gps.yaml + geo_enu 换算")
    import yaml
    sys.path.insert(0, os.path.join(WS, "src", "arena_bridges", "scripts"))
    import geo_enu as geo
    vp = os.path.join(WS, "src", "zx2026_common", "config", "venue_gps.yaml")
    venue = yaml.safe_load(open(vp, encoding="utf-8"))
    fr, poly = geo.frame_from_venue(venue)
    take = poly["takeoff"]
    check("起降 4 点", len(take) == 4, str(len(take)))
    rad = max(math.hypot(e, n) for (e, n) in take)
    check("起降 4 点贴原点(<35m)", rad < 35.0, "%.1fm" % rad)
    per = poly["perimeter"]
    we = max(e for e, _ in per) - min(e for e, _ in per)
    sn = max(n for _, n in per) - min(n for _, n in per)
    check("周界东西 ≈221m", 200.0 < we < 245.0, "%.1fm" % we)
    check("周界南北 ≈423m", 400.0 < sn < 445.0, "%.1fm" % sn)
    sup = poly["supply"]
    sup_c = (sum(e for e, _ in sup) / len(sup),
             sum(n for _, n in sup) / len(sup))
    d_sup = math.hypot(*sup_c)
    check("补给区在南方 300-450m", 300.0 < d_sup < 450.0
          and sup_c[1] < 0, "%.1fm n=%.1f" % (d_sup, sup_c[1]))
    check("原点在周界内", geo.point_in_poly(0.0, 0.0, per))
    check("补给区心在周界内", geo.point_in_poly(sup_c[0], sup_c[1], per))
    # 往返换算闭合（docx 原始数 ↔ ENU 双向一致，弧长级数线性化自洽）
    lo, la = fr.to_gps(12.0, -34.0)
    e2, n2 = fr.to_enu(lo, la)
    check("to_gps/to_enu 往返闭合", abs(e2 - 12.0) < 0.01
          and abs(n2 + 34.0) < 0.01, "(%.3f,%.3f)" % (e2, n2))


# ---- L4 GCS 透出链 -------------------------------------------------------
def l4_gcs_chain():
    print("[L4] GCS 透出链（agent/hub/profile）")
    ag = open(os.path.join(TOOLS, "gcs", "gcs_agent.py"),
              encoding="utf-8").read()
    check("agent._blank 两键", '"home_gps": None' in ag
          and '"align": None' in ag)
    check("agent 订阅", 'tp.get("home_gps")' in ag
          and 'tp.get("align")' in ag)
    check("agent.snapshot 透出", '"home_gps": st["home_gps"]' in ag)
    hb = open(os.path.join(TOOLS, "gcs", "gcs_hub.py"),
              encoding="utf-8").read()
    check("hub status_writer 透出", '"home_gps": d.get("home_gps")' in hb
          and '"align": d.get("align")' in hb)
    import yaml
    for name in ("profile_real_vio.yaml", "profile_real_lio.yaml"):
        pr = yaml.safe_load(open(os.path.join(TOOLS, "gcs", name),
                                 encoding="utf-8"))
        tp = pr["topics"]
        miss = [k for k in ("home_gps", "align", "phase", "mission",
                            "task", "metrics") if not tp.get(k)]
        check("%s 接线齐全" % name, not miss, "缺 %s" % miss)
    ps = yaml.safe_load(open(os.path.join(TOOLS, "gcs", "profile_sim.yaml"),
                             encoding="utf-8"))
    check("sim profile 不含两键",
          not ps["topics"].get("home_gps") and not ps["topics"].get("align"))


# ---- L5 C100 下视链对质（2026-10-10 定谳）--------------------------------
def l5_c100_chain():
    print("[L5] C100 下视链对质（launch/preflight/udev/消费端，文件级）")
    import re
    import xml.etree.ElementTree as ET
    rfl = open(os.path.join(WS, "src", "arena_bridges", "launch",
                            "real_fleet.launch"), encoding="utf-8").read()
    rd4 = open(os.path.join(REAL, "launch", "real_d435.launch"),
               encoding="utf-8").read()

    def arg_default(txt, name):
        m = re.search(r'<arg name="%s" default="([^"]*)"' % name, txt)
        return m.group(1) if m else None

    # pass_all_args 覆盖链地雷（2026-10-09 死话题 /drone_N/camera/image_raw
    # 定谳）：同名 arg 默认两文件必须同值——此断言是那案的永久回归闸。
    for an in ("camera_topic", "camera_vfov_deg"):
        a, b = arg_default(rfl, an), arg_default(rd4, an)
        check("%s 两文件默认同值" % an, a is not None and a == b,
              "real_fleet=%r real_d435=%r" % (a, b))
    check("camera_topic 指向 C100", arg_default(rfl, "camera_topic")
          == "/camera_down/image_raw", arg_default(rfl, "camera_topic"))
    check("align_axes 正名 top_fwd", arg_default(rfl, "align_axes")
          == "top_fwd", arg_default(rfl, "align_axes"))
    check("drop_align 挂 --vfov-deg", "--vfov-deg $(arg camera_vfov_deg)"
          in rfl)

    # camera_down.launch：usb_cam 档 + 绑 udev 软链 + 注释无 ASCII -- 雷
    cdl = open(os.path.join(REAL, "launch", "camera_down.launch"),
               encoding="utf-8").read()
    ET.fromstring(cdl)
    check("camera_down.launch XML 可解析", True)
    check("camera_down.launch usb_cam_node + /dev/camera_down",
          "usb_cam_node" in cdl and "/dev/camera_down" in cdl)
    bad = [c for c in re.findall(r"<!--(.*?)-->", cdl, re.S) if "--" in c]
    check("camera_down.launch 注释无 ASCII --", not bad,
          "%d 处" % len(bad))

    # udev 模板：软链名与 launch 绑定一致
    udev = open(os.path.join(REAL, "udev", "99-camera-down.rules"),
                encoding="utf-8").read()
    check("udev 模板 SYMLINK camera_down", 'SYMLINK+="camera_down"' in udev)

    # preflight ⑥：话题指向 + encoding 探针白名单与消费端同源
    pf = open(os.path.join(REAL, "scripts", "preflight.sh"),
              encoding="utf-8").read()
    check("preflight ⑥ 指向 /camera_down/image_raw",
          "[6/11]" in pf and "/camera_down/image_raw" in pf)
    cd_src = open(os.path.join(WS, "src", "arena_sensor", "scripts",
                               "arena_sensor", "color_detect.py"),
                  encoding="utf-8").read()
    for enc in ("bgr8", "rgb8", "yuyv", "uyvy"):
        check("preflight ⑥ 白名单含 %s（与消费端同源）" % enc,
              enc in pf and enc in cd_src)

    # 消费端：拒帧门 + 正名别名在位（行为细节由各自 --selftest 覆盖，L1）
    check("color_detect.to_bgr 拒帧门在位", "def to_bgr(" in cd_src)
    coln = open(os.path.join(WS, "src", "arena_sensor", "scripts",
                             "color_detector_node.py"),
                encoding="utf-8").read()
    check("color_detector 走 to_bgr 统一入口",
          "to_bgr(" in coln and "def _on_image" in coln)
    da = open(os.path.join(TOOLS, "gcs", "drop_align_node.py"),
              encoding="utf-8").read()
    check("drop_align AXES_ALIAS 正名在位",
          '"top_fwd"' in da and '"d435_down"' in da
          and "AXES_ALIAS" in da)
    check("drop_align 默认 image-topic 已指 C100",
          '"/camera_down/image_raw"' in da
          and "/camera/color/image_raw" not in da)

    # DROP 自愈门（真机档 15s；sim 缺省 0=关零行为变更）：桥不开/桥死时
    # 超时走与 payload/done 同一 _schedule_return 出口，不原位悬停等人工
    mx = open(os.path.join(WS, "src", "arena_mission", "scripts",
                           "mission_executor_node.py"), encoding="utf-8").read()
    check("executor DROP 自愈门在位",
          "drop_wait_timeout_s" in mx and "_schedule_return" in mx
          and "drop-wait-timeout" in mx)
    check("real_fleet 下传 drop_wait_timeout_s", "drop_wait_timeout_s" in rfl)


# ---- L6 docx 黄金 24 数 + 需求②③装订（2026-10-11 GPS 换血）---------------
# 黄金数硬编码自桌面《比赛坐标》两 docx（0924 版共同表）。L3 验换算自洽，
# 本层验「抄进库的数=官方原文」——venue_gps.yaml 转录回归的最后一道闸。
# keyun3 12 点×(lon,lat)=24 三元组；keyun12 无消费方不入黄金锚。
_GOLDEN_KEYUN3 = {
    "perimeter": [
        ((113, 42, 27.74), (33, 51, 56.63)),   # 1 ←起降区
        ((113, 42, 28.09), (33, 51, 55.53)),   # 2 ←起降区
        ((113, 42, 29.29), (33, 51, 55.36)),   # 3 ←起降区
        ((113, 42, 28.36), (33, 51, 45.94)),   # 4
        ((113, 42, 30.85), (33, 51, 42.99)),   # 5
        ((113, 42, 35.32), (33, 51, 42.86)),   # 6
        ((113, 42, 36.36), (33, 51, 56.12)),   # 7
        ((113, 42, 29.29), (33, 51, 56.53)),   # 8 ←起降区
    ],
    "supply": [
        ((113, 42, 33.32), (33, 51, 43.96)),   # S1
        ((113, 42, 34.56), (33, 51, 44.39)),   # S2
        ((113, 42, 34.66), (33, 51, 44.06)),   # S3
        ((113, 42, 33.51), (33, 51, 43.77)),   # S4
    ],
}


def l6_gps_golden():
    print("[L6] docx 黄金 24 数 + 需求②③装订")
    import yaml
    vp = os.path.join(WS, "src", "zx2026_common", "config", "venue_gps.yaml")
    v = yaml.safe_load(open(vp, encoding="utf-8"))["venue"]["keyun3"]
    n = 0
    for sec, raw in (("perimeter", "perimeter"), ("supply", "supply_zone")):
        pts, gold = v[raw], _GOLDEN_KEYUN3[sec]
        check("%s 点数=%d" % (sec, len(gold)), len(pts) == len(gold),
              str(len(pts)))
        for k, (glon, glat) in enumerate(gold):
            got = pts[k]
            n += 2
            check("%s#%d lon=%s" % (sec, k + 1, glon),
                  list(got["lon"]) == list(glon), str(got.get("lon")))
            check("%s#%d lat=%s" % (sec, k + 1, glat),
                  list(got["lat"]) == list(glat), str(got.get("lat")))
    check("黄金三元组计数=24", n == 24, str(n))

    # 需求②：起飞记 GPS——launch 默认开 + 节点块在位 + preflight ④ 探针
    rfl = open(os.path.join(WS, "src", "arena_bridges", "launch",
                            "real_fleet.launch"), encoding="utf-8").read()
    check("launch gps_home 默认 true",
          'name="gps_home" default="true"' in rfl)
    check("launch gps_home 节点块在位",
          'if="$(arg gps_home)"' in rfl and "gps_home_node.py" in rfl)
    pf = open(os.path.join(REAL, "scripts", "preflight.sh"),
              encoding="utf-8").read()
    check("preflight ④ GPS fix 探针在位（warn 档）", "④ GPS fix" in pf)

    # 需求③：投放后返出发点——replace 默认（launch 下行+executor 读参+消费）
    check("launch gps_home_mode 默认 replace",
          'name="gps_home_mode" default="replace"' in rfl)
    mx = open(os.path.join(WS, "src", "arena_mission", "scripts",
                           "mission_executor_node.py"), encoding="utf-8").read()
    check("executor 读 gps_home_mode（默认 replace）",
          '"~gps_home_mode", "replace"' in mx)
    check("executor 订阅 home/enu（绑定处全句，防注释假 OK）",
          'rospy.Subscriber(ns + "/home/enu", PoseStamped, '
          'self._on_home_corr)' in mx)
    check("executor RETURN 消费 _home_pad_apply",
          "_home_pad_apply(self.pad" in mx)


def main():
    l1_nodes()
    run_layer(l2_home_pad, "L2")
    run_layer(l3_venue, "L3")
    run_layer(l4_gcs_chain, "L4")
    run_layer(l5_c100_chain, "L5")
    run_layer(l6_gps_golden, "L6")
    if FAILS:
        print("REALPACK GPS SELFTEST FAIL (%d): %s"
              % (len(FAILS), ", ".join(FAILS)))
        sys.exit(1)
    if SKIPS:
        # 交付包在未构建的树里可跑：L1 实跑 + 其余能跑的层照跑，缺构建的层
        # 显式 SKIP 带原因。判据口径：**未构建树里的 PASS 不是权威读数**，
        # catkin build 后必须重跑取零 SKIP 的 PASS。
        print("REALPACK GPS SELFTEST PASS (%d SKIP: %s)"
              % (len(SKIPS), "; ".join(SKIPS)))
        print("NOTE: SKIP 层需 catkin build 后重跑（权威读数=零 SKIP 的 PASS）")
        return
    print("REALPACK GPS SELFTEST PASS")


if __name__ == "__main__":
    main()
