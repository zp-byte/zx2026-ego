#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端验证：启动全栈 → 等 fleet_ready → /zx2026/start → 观测 150s 全程。

记录: /zx2026/state 状态时间线、每 5s 的 drone_0/1 位置、mission/phase、
      match/result、score_summary，输出到 stdout + /tmp/zx2026_verify.txt。
用法: bash -c 'source devel/setup.bash && python3 verify_run.py'
"""
import math
import time

import rospy
import rosparam
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock


def observation_should_continue(t_sim, t0s, observe_s, wall_elapsed, wall_cap,
                                since_clock, stall_s, shutting_down):
    """纯函数：观测窗是否继续。返回 (go_on, exit_reason)。

    2026-10-08 补栅（10-07 孤儿法证）：原实现只有 `t_sim - t0s < observe_s`
    一条判据，而 t_sim 被 max() 钳在末值 ⇒ /clock 冻结时该式恒真、循环永不退、
    SIGTERM 被吞（实测空转 40 分钟）。判序即优先级：
      外部终止 > 墙钟上限 > 时钟停摆 > sim 预算耗尽（正常出口）。
    `since_clock=None` = 从未收到 /clock（回退墙钟档），不做停摆判定。
    """
    if shutting_down:
        return False, "ROS SHUTDOWN"
    if wall_elapsed > wall_cap:
        return False, "WALL CAP %.0fs" % wall_cap
    if since_clock is not None and since_clock > stall_s:
        return False, ("CLOCK STALLED (sim frozen at %.1fs, %.0fs since last /clock)"
                       % (t_sim - t0s, since_clock))
    if t_sim - t0s >= observe_s:
        return False, "BUDGET"
    return True, None


class Verifier:
    def __init__(self):
        self.state = "?"
        self.phases = {}
        self.match = {}
        self.odom = {}
        self.score_summary = None
        self.fleet_ready = False
        self.sim_t = None
        self.sim_wall = None   # 最近一次 /clock 到达的墙钟（停摆判据用，见 run()）

    def _on_state(self, m):
        self.state = m.data

    def _on_phase(self, i):
        def cb(m, i=i):
            self.phases[i] = m.data
        return cb

    def _on_match(self, i):
        def cb(m, i=i):
            self.match[i] = m.data
        return cb

    def _on_odom(self, i):
        def cb(m, i=i):
            p = m.pose.pose.position
            self.odom[i] = (round(p.x, 2), round(p.y, 2), round(p.z, 2))
        return cb

    def _on_score(self, m):
        self.score_summary = m.data

    def _on_fleet(self, m):
        self.fleet_ready = m.data

    def _on_clock(self, m):
        self.sim_t = m.clock.to_sec()
        self.sim_wall = time.time()   # 停摆判据：sim 值 + 到达墙钟成对记录

    def run(self):
        rospy.init_node("verify_run", anonymous=True)
        for i in range(6):
            rospy.Subscriber("/drone_%d/odom" % i, Odometry, self._on_odom(i))
            rospy.Subscriber("/drone_%d/mission/phase" % i, String, self._on_phase(i))
            rospy.Subscriber("/drone_%d/match/result" % i, String, self._on_match(i))
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber("/zx2026/score_summary", String, self._on_score)
        rospy.Subscriber("/zx2026/fleet_ready", Bool, self._on_fleet)
        rospy.Subscriber("/clock", Clock, self._on_clock)

        out = open("/tmp/zx2026_verify.txt", "w")
        log = lambda *a: (print(*a), out.write(" ".join(str(x) for x in a) + "\n"), out.flush())

        # 等 fleet_ready（最多 30s）
        # 节奏一律 time.sleep：本节点是监督器，心跳不绑被监督的 /clock
        # （rospy.sleep 在 use_sim_time 下吃 sim 时间，时钟冻结时永久阻塞）
        t0 = time.time()
        while not self.fleet_ready and time.time() - t0 < 30:
            if rospy.is_shutdown():
                break
            time.sleep(0.2)
        log("fleet_ready =", self.fleet_ready)

        # 等 P2_WAIT，然后 start
        t0 = time.time()
        while self.state != "P2_WAIT" and time.time() - t0 < 20:
            if rospy.is_shutdown():
                break
            time.sleep(0.2)
        try:
            srv = rospy.ServiceProxy("/zx2026/start", Trigger)
            r = srv()
            log("start service:", r.success, r.message)
        except Exception as e:
            log("start service FAIL:", e)

        # P6：观测窗跟随时限参数化（时限+60s，上限 1600；DONE 早退保留）
        try:
            from zx2026_common import config as _cfg
            tl = float(_cfg.load("competition_rules.yaml").get("time_limit_s", 600.0))
        except Exception:
            tl = 600.0
        observe_s = min(tl + 60.0, 1600.0)
        log("=== observation (%.0fs, sample every 5s) ===" % observe_s)
        last_state = None
        t0 = time.time()
        # 2026-09-29 P3 smoke 法证：预算原按墙钟（time.time），RTF 0.5（defop
        # 六机推理链）时 240s 墙钟只覆盖 ~126s sim < 任务窗 150s —— DONE 永等
        # 不到还被误判污染。改按 /clock sim 时间计预算（无 /clock 回退墙钟，
        # 基线语义不变）；t0 保留墙钟 epoch 供封卷报告 mtime 新鲜度门。
        # ⚠ 2026-10-08 补栅（10-07 孤儿法证）：此前 sim 预算**无墙钟兜底**且循环
        # 从不查 is_shutdown ⇒ /clock 一旦冻结（仿真器死/launch 半死），`_now()`
        # 被 max() 钳在末值永不增长、while 恒真、SIGTERM 被吞 —— 实测空转 **40
        # 分钟**（sim 停在 t=152），并覆盖 /tmp/zx2026_verify.txt，同批另一格因此
        # 丢掉有效单元。三重栅：
        #   ① 逐拍 rospy.is_shutdown()（SIGTERM 立即出）
        #   ② 时钟停摆判据：sim_t 连续 STALL_S 墙钟秒不动 ⇒ 判死并写明
        #   ③ 总墙钟上限：按悲观 RTF_FLOOR 反算 + 余量
        # 节奏一律 time.sleep（分片，便于及时响应信号），不绑 sim 时钟。
        t0s = self.sim_t
        wall_fallback = t0s is None
        if wall_fallback:
            log("WARN: no /clock, observation budget falls back to wall time")
            t0s = time.time()
        STALL_S = 60.0        # sim 时钟停摆判据（墙钟秒）
        RTF_FLOOR = 0.65      # 悲观 RTF 地板（=批有效性闸门的中途门），反算墙钟上限
        wall_cap = observe_s / RTF_FLOOR + 180.0
        wall_start = time.time()

        def _now():
            if wall_fallback:
                return time.time()
            return t0s if self.sim_t is None else max(self.sim_t, t0s)

        exit_reason = None
        while True:
            _go, _why = observation_should_continue(
                _now(), t0s, observe_s, time.time() - wall_start, wall_cap,
                None if self.sim_wall is None else time.time() - self.sim_wall,
                STALL_S, rospy.is_shutdown())
            if not _go:
                exit_reason = _why
                break
            tnow = _now() - t0s
            if self.state != last_state:
                last_state = self.state
                log("-- state ->", self.state, "@ %.0fs" % tnow)
            o0 = self.odom.get(0)
            o1 = self.odom.get(1)
            if o0 or o1:
                log("  t=%.0f drone0=%s drone1=%s phase0=%s match0=%s"
                    % (tnow, o0, o1,
                       self.phases.get(0), self.match.get(0)))
            if self.state == "DONE":
                time.sleep(2.0)   # 封卷报告落盘窗口（scorekeeper 收 DONE 写 yaml）
                exit_reason = "DONE"
                break
            for _ in range(25):   # 5s，分片以便及时响应 SIGTERM
                if rospy.is_shutdown():
                    break
                time.sleep(0.2)

        wall_elapsed = time.time() - wall_start
        sim_elapsed = _now() - t0s
        rtf = (sim_elapsed / wall_elapsed) if wall_elapsed > 0 else 0.0
        log("=== observation ended: %s ===" % (exit_reason or "BUDGET"))
        log("wall_elapsed=%.1fs sim_elapsed=%.1fs RTF=%.3f" % (wall_elapsed, sim_elapsed, rtf))
        log("=== final ===")
        log("state:", self.state)
        log("phases:", {k: v for k, v in sorted(self.phases.items())})
        log("match:", {k: v for k, v in sorted(self.match.items())})
        log("odom drone0:", self.odom.get(0))
        log("score_summary:", self.score_summary)
        # 通过判定（比赛口径 2026-09-09）：时限封卷语义下未完成机不再等终态，
        # "六机全 DONE+MATCH" 仅作 FULL 档对照打印。PASS = state DONE 时封卷
        # 成功：封卷报告落盘（mtime≥观测起点防旧 run 残留）+ 零退赛 + ≥1 机落地。
        full = (all(self.phases.get(i) == "DONE" for i in range(6))
                and all(self.match.get(i) == "MATCH" for i in range(6)))
        rep = None
        try:
            import glob
            import os as _os
            import yaml as _yaml
            cands = [f for f in glob.glob("/tmp/zx2026_score_*.yaml")
                     if _os.path.getmtime(f) >= t0 - 5.0]
            if cands:
                p = max(cands, key=_os.path.getmtime)
                rep = _yaml.safe_load(open(p))
        except Exception:
            rep = None
        log("full_completion:", full)
        log("report retired:", (rep or {}).get("retired"),
            "landed_n:", (rep or {}).get("landed_n"))
        ok = (self.state == "DONE" and rep is not None
              and not (rep.get("retired") or [])
              and (rep.get("landed_n") or 0) >= 1)
        log("VERDICT:", "PASS" if ok else "FAIL")
        out.close()


if __name__ == "__main__":
    try:
        Verifier().run()
    except rospy.ROSInterruptException:
        pass
