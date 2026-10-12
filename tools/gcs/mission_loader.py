#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/mission_loader.py — 真机任务指派装订器（真机适配 2026-09-26）。

sim 里任务由 task_generator_node（全局 1 份，读 scene_topology 自动分配）
latched 发 /zx2026/mission/<id>。真机各机独立 ROS master，且指派要人工
可控（赛前按抽签/平台实况改）——本加载器在每机本地把**人写的指派文件**
转成同款 Mission 消息 latched 发出，mission_executor/type_match/scorekeeper
零改动复用。

字段语义对齐 task_generator_node（科目三口径）：
  payload_type  本机携带物品类型 TYPE_A..E
  box_color     本机箱色（red/blue/yellow；空=接收端回退 color_map）
  drop_point_id 参考指派平台号（闭环 color_id 下仅审计用，执行器不得据此
                导航——目标由机载识别择定；无参考填 -1）
  marker_id     参考平台标识（tag）
  takeoff_pose  起降 pad 位姿（z=hover 高度，场地 ENU）
  cruise_z/max_vel/mission_seq 同名

指派文件格式（YAML；单文件可含全部机，本加载器按 --id 取自己那条）：
  missions:
    - drone: 0
      payload: TYPE_A
      box_color: red
      drop: 1            # 参考平台号；-1=无参考
      marker: P1         # 平台 tag；可空
      takeoff: [0.0, 0.0, 1.5]   # 场地 ENU，z=hover 高度
      cruise_z: 1.2      # 穿越区平飞高度（RECON latch 定谳：1.2 档）
      max_vel: 1.2       # 真机首飞保守档
      seq: 1

用法（机载，指派文件与工具同拷上机）：
  python3 mission_loader.py --id 0 --yaml missions.yaml
  周期重发保险（latch 已覆盖晚订户，一般不需要）：--rate 1.0
  --box-color red|blue|yellow 一次性覆盖文件色——仅限无常驻实例的一次性
  装订场景（先起后不复用）。⚠勿与 launch/boot_full 常驻实例同跑：同
  节点名会被 ROS master 击杀常驻实例，CLI 退出后机上零 loader（GCS
  改色服务与唯一 latch 发布者一起消失）。
GCS 动态指派（起飞前锁定）：常驻本节点挂 /mission_loader_<id>/set_box_color
  服务——校验色名+phase IDLE 门后改 box_color 并经本机 latch 话题原地重发
  （executor/type_match/drop_align/agent 回显全链按既有重绑语义自动生效；
  单发布者，绝不起第二实例）。phase 门从严：心跳未到（executor 未起/滞后）
  一律拒改。放行后 1.2s 补偿复核兜竞态，复核以 executor 实际箱色广播
  （/drone_<id>/mission/box_color）为准绳：广播=新色=executor 已接受（镜像
  时序 Mission 先到、起飞迁移后至）→ 不回滚保新色一致；广播≠新色 → 回滚
  重发旧色（tm/dp 回旧，executor 本就旧色）。服务入参出参见
  zx2026_common/SetBoxColor.srv。
自检：python3 mission_loader.py --selftest（无 ROS 可跑，结构 7 检）
"""
import argparse
import sys
import threading

# ---------------------------------------------------------------- 纯逻辑 --
DEFAULT_TAKEOFF = [0.0, 0.0, 1.5]   # z=hover 高度（rules takeoff_hover_z 同款）
REQUIRED_KEYS = ("payload", "drop")
VALID_COLORS = ("red", "blue", "yellow")   # config.py COLOR_TO_UINT8 同源三色


def resolve_color(req, fallback=""):
    """色参纯逻辑（service/CLI 共用，可离线测）：小写归一后对三色白名单。

    空串=回退 fallback（文件默认）。返回 (ok, color_or_errmsg)。宁可拒绝
    也不带错飞（同 load_assignment 的 panic 纪律）。"""
    c = str(req or "").strip().lower()
    if c == "":
        return True, str(fallback or "")
    if c in VALID_COLORS:
        return True, c
    return False, "%r 非法（仅 red/blue/yellow，空=回退文件默认）" % req


def load_assignment(path, drone_id):
    """读指派 yaml 取 drone_id 条目 → Mission 字段 dict（纯逻辑可离线测）。

    缺条目/缺必填键=人工配置错误，宁可启动失败也不带错飞（panic 纪律）。
    """
    import yaml
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    missions = (doc or {}).get("missions") or []
    for m in missions:
        if int(m.get("drone", -1)) != int(drone_id):
            continue
        for k in REQUIRED_KEYS:
            if k not in m:
                raise ValueError("mission[%d] 缺必填键 %r" % (drone_id, k))
        tk = m.get("takeoff") or DEFAULT_TAKEOFF
        if len(tk) != 3:
            raise ValueError("mission[%d] takeoff 须 [x,y,z]" % drone_id)
        return {"drone_id": int(drone_id),
                "payload_type": str(m["payload"]),
                "box_color": str(m.get("box_color", "")),
                "drop_point_id": int(m["drop"]),
                "marker_id": str(m.get("marker", "")),
                "takeoff": [float(c) for c in tk],
                "cruise_z": float(m.get("cruise_z", 1.2)),
                "max_vel": float(m.get("max_vel", 1.2)),
                "mission_seq": int(m.get("seq", 1))}
    raise ValueError("missions.yaml 无 drone=%s 条目" % drone_id)


def selftest():
    ok = [0]
    import tempfile
    import os
    good = """
missions:
  - drone: 0
    payload: TYPE_A
    box_color: red
    drop: 1
    marker: P1
    takeoff: [0.5, -1.0, 1.5]
    cruise_z: 1.2
    max_vel: 1.2
    seq: 1
  - drone: 1
    payload: TYPE_B
    drop: -1
"""
    fd, path = tempfile.mkstemp(suffix=".yaml")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(good)
        a = load_assignment(path, 0)
        assert a == {"drone_id": 0, "payload_type": "TYPE_A",
                     "box_color": "red", "drop_point_id": 1,
                     "marker_id": "P1", "takeoff": [0.5, -1.0, 1.5],
                     "cruise_z": 1.2, "max_vel": 1.2, "mission_seq": 1}, a
        ok[0] += 1
        # 默认值：takeoff/cruise_z/max_vel/seq/box_color/marker 缺省
        b = load_assignment(path, 1)
        assert b["takeoff"] == DEFAULT_TAKEOFF and b["cruise_z"] == 1.2 \
            and b["max_vel"] == 1.2 and b["mission_seq"] == 1 \
            and b["box_color"] == "" and b["marker_id"] == "", b
        ok[0] += 1
        # 缺条目报错
        try:
            load_assignment(path, 5)
            raise AssertionError("missing entry not detected")
        except ValueError:
            ok[0] += 1
        # 缺必填键报错
        fd2, path2 = tempfile.mkstemp(suffix=".yaml")
        with os.fdopen(fd2, "w", encoding="utf-8") as f:
            f.write("missions:\n  - drone: 2\n    payload: TYPE_C\n")
        try:
            load_assignment(path2, 2)
            raise AssertionError("missing key not detected")
        except ValueError:
            ok[0] += 1
        os.unlink(path2)
    finally:
        os.unlink(path)
    # 色参纯逻辑（GCS 改色 service/CLI 共用）
    okc, c = resolve_color("RED", "yellow")
    assert okc and c == "red", (okc, c)
    ok[0] += 1
    okc, c = resolve_color("", "red")
    assert okc and c == "red", (okc, c)   # 空串=回退文件默认
    ok[0] += 1
    okc, c = resolve_color("green", "red")
    assert not okc and "green" in c, (okc, c)
    ok[0] += 1
    print("selftest OK (%d checks)" % ok[0])


# ------------------------------------------------------------------ ROS --
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=None)
    ap.add_argument("--yaml", default=None, help="指派文件路径")
    ap.add_argument("--rate", type=float, default=0.0,
                    help=">0 则按 Hz 周期重发（latch 已覆盖晚订户，一般 0）")
    ap.add_argument("--box-color", default=None, metavar="red|blue|yellow",
                    help="覆盖指派文件 box_color（起飞前临时改色；小写归一，"
                         "空串=回退文件默认）")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    if args.id is None or args.yaml is None:
        ap.error("--id and --yaml are required")
    try:
        from zx2026_common.msg import Mission
    except ImportError:
        sys.exit("zx2026_common.msg 不可导入——先 source 工作空间 setup.bash"
                 "（或用 --selftest）")
    import rospy
    from geometry_msgs.msg import Point, Quaternion
    a = load_assignment(args.yaml, args.id)
    if args.box_color is not None:
        okc, c = resolve_color(args.box_color, a["box_color"])
        if not okc:
            sys.exit("mission_loader: box_color %s" % c)
        a["box_color"] = c
    rospy.init_node("mission_loader_%d" % args.id)
    pub = rospy.Publisher("/zx2026/mission/%d" % args.id, Mission,
                          queue_size=1, latch=True)
    m = Mission()
    m.drone_id = a["drone_id"]
    m.payload_type = a["payload_type"]
    m.box_color = a["box_color"]
    m.drop_point_id = a["drop_point_id"]
    m.marker_id = a["marker_id"]
    m.takeoff_pose.position = Point(*a["takeoff"])
    m.takeoff_pose.orientation = Quaternion(w=1.0)
    m.cruise_z = a["cruise_z"]
    m.max_vel = a["max_vel"]
    m.mission_seq = a["mission_seq"]
    rospy.loginfo("mission_loader[%d]: payload=%s box=%s ref_dp%d(%s) "
                  "cruise_z=%.1f max_vel=%.1f seq=%d", m.drone_id,
                  m.payload_type, m.box_color, m.drop_point_id, m.marker_id,
                  m.cruise_z, m.max_vel, m.mission_seq)

    # GCS 起飞前改色入口（真机适配 2026-10-12）：本常驻节点=唯一 latch
    # 发布者，service 改 m.box_color 后原地重发——不起第二实例（双 latch
    # 发布者会让晚订户收到旧色重绑）。同步回执 ok/msg 供 dispatch 判成。
    # phase 门从严（2026-10-12 复核 C0）：executor 的 IDLE 门只保自己，
    # type_match/drop_align 收到 Mission 就重绑（无自保护）——心跳未到
    # 放行会造出 executor 旧色+tm/dp 新色的"半接受"态，MATCH 门必失配；
    # 一律拒改无损（操作员稍候重试）。放行后 1.2s 补偿复核兜心跳滞后。
    from std_msgs.msg import String as _String
    from zx2026_common.srv import SetBoxColor, SetBoxColorResponse
    phase = {"v": None}
    rospy.Subscriber("/drone_%d/mission/phase" % args.id, _String,
                     lambda msg: phase.__setitem__("v", msg.data))
    # 回滚准绳（2026-10-12 审查 C0 半窗补刀）：executor 实际箱色广播
    # （_on_mission 接受 Mission 时 latched 直发）——1.2s 复核据此分辨
    # 「executor 已接受新色」（不回滚）与「executor 拒新色保旧色」（回滚）。
    bc = {"v": None}
    rospy.Subscriber("/drone_%d/mission/box_color" % args.id, _String,
                     lambda msg: bc.__setitem__("v", str(msg.data) or None))
    # 服务处理互斥：rospy 每连接独立线程，处理器含 1.2s sleep 且共享闭包
    # m——不串行则并发两次改色（面板+CLI 双开）回执串台、回滚交错可终态
    # 落在被回滚色。锁内串行=后到请求等前到走完（含 sleep），语义 last-
    # call-wins 且回执逐请求保真。
    _set_lock = threading.Lock()

    def _on_set_color(req):
        with _set_lock:
            return _set_color_impl(req)

    def _set_color_impl(req):
        okc, c = resolve_color(req.color, a["box_color"])
        if not okc:
            return SetBoxColorResponse(ok=False, msg=c)
        if phase["v"] != "IDLE":
            why = ("心跳未到=executor 未起/滞后，稍候重试"
                   if phase["v"] is None else "起飞已开始")
            return SetBoxColorResponse(
                ok=False, msg="phase=%s 非 IDLE 拒改（起飞前锁定；%s）"
                % (phase["v"], why))
        old = m.box_color
        m.box_color = c
        pub.publish(m)
        # 补偿复核：GCS stage 快照与 1Hz phase 心跳均滞后 ~1s，放行与起飞
        # 开跑可在窗内交叉。两种时序必须分判（审查 2026-10-12：原第三刀
        # 只堵「executor 先迁移」半窗——镜像时序 Mission 先到、executor
        # 已接受新色后才迁移，此际回滚重发旧色被非 IDLE executor 拒收，
        # 反而造出 executor=新/tm=dp=旧 的"半接受"态=MATCH 门必失配）：
        #   广播=新色 → executor 已接受，改色已全链生效 → 不回滚
        #   广播≠新色 → executor 拒新色保旧色（tm/dp 已绑新色）→ 回滚：
        #     重发旧色 Mission，tm/dp 按既有重绑语义回旧色，三口一致恢复
        # 广播未达（None，executor 无本广播的旧版）→ 按拒色处理（保守回滚）
        rospy.sleep(1.2)
        if phase["v"] is not None and phase["v"] != "IDLE":
            if bc["v"] == c:
                rospy.loginfo("mission_loader[%d]: set_box_color -> %s 已被 "
                              "executor 接受（起飞开跑前生效，不回滚）",
                              args.id, c or "<file-default>")
                return SetBoxColorResponse(
                    ok=True, msg="box_color=%s seq=%d（已生效）"
                    % (c, m.mission_seq))
            m.box_color = old
            pub.publish(m)
            rospy.logwarn("mission_loader[%d]: set_box_color 回滚 -> %s"
                          "（起飞已开跑 phase=%s，改色未生效）",
                          args.id, old or "<file-default>", phase["v"])
            return SetBoxColorResponse(
                ok=False, msg="起飞已开跑（phase=%s），改色未生效已回滚=%s"
                % (phase["v"], old or "<file-default>"))
        rospy.loginfo("mission_loader[%d]: set_box_color -> %s (phase=%s)",
                      args.id, c or "<file-default>", phase["v"])
        return SetBoxColorResponse(ok=True, msg="box_color=%s seq=%d"
                                   % (c, m.mission_seq))

    rospy.Service("/mission_loader_%d/set_box_color" % args.id,
                  SetBoxColor, _on_set_color)

    if args.rate > 0:
        rate = rospy.Rate(args.rate)
        while not rospy.is_shutdown():
            pub.publish(m)
            rate.sleep()
    else:
        rospy.sleep(0.3)   # 连接建立窗口（latch 兜底，双保险）
        pub.publish(m)
        rospy.spin()


if __name__ == "__main__":
    main()
