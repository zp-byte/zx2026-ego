#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_shadow_selftest.py — T2 nav_node shadow 模式（planner.source=defop）
离线自测：配置解析语义 + 发布点门控静态核查 + arrived 保活判据纯度。

用法（WSL）：
  cd ~/zx2026_arena_ws && source /opt/ros/noetic/setup.bash \
    && source devel/setup.bash && python3 tools/defop_shadow_selftest.py

结果写 /tmp/defop_shadow_selftest.txt（UTF-8，规避 GBK 控制台）。

背景（架构裁决，已定）：source=defop = shadow 模式——nav_node 全管线照跑
（A*/反应层/rescue/swarm/arrived/metrics/任务簿记全保活）。T2 起 defop_cpn
成为唯一发布者；P3 终端交接后改单写者仲裁：cpn 只在 cruise 窗口持笔
（cpn_active 旗，语义核=cpn_active_of），terminal 终端接近（2D<1.2 进入/
>1.6 退出迟滞）/ departure 出发场未清 / no_goal 时 nav 收回发布权——
arrived 由 nav 管线收尾触发，mission FSM 才能推进（cpn 在 0.8m 停悬停
且无机间感知，是原缺陷两根）。默认（键缺失/值不识）=internal，现状
逐字节不变。

覆盖：
  A. vel_source_of 解析矩阵：键缺失/段 null/段空/defop/internal/值不识/
     空串/大小写敏感——只放行 'defop' 一个外来值（保守回退 internal）
  B. 真实配置集成：cfg.load("sim_settings.yaml") 解析结果合法（键缺失
     当下必 internal；T1 落键后此处自动跟随）
  C. nav_node.py 发布点静态核查：
     C1 vel_cmd 发布点恰 3 处且全部被门控（主循环 defop 臂 nav 持笔门 +
        internal 臂直发 + landed _cpn_active 门）
     C2 发布器仍无条件创建（防下游 can't binder 无 publisher 误判）
     C3 nav/arrived 两发射点不在任何 _vel_source 门内（保活静态证据）
     C4 goal_reached 纯度：方法体零 _vel_source/零规划状态引用（纯
        odom/est-vs-goal 距离判据 → defop 模式天然保活的根因）
     C5 landed 路径 _halted 簿记仍在（landing/done 协调不动）
     C6 启动日志带 vel_src 字段（活体可辨识）
  D. cpn_active_of 纯函数矩阵（P3 终端交接闸门语义核）：internal 恒 False
     （其余三参任意组合）/ defop+cruise+清场+goal -> True / terminal ->
     False / departure 未清 -> False / no_goal -> False
  E. P3 终端交接两端静态核查：nav 持笔旗每拍广播 + 心跳扩容 + 迟滞阈值；
     defop_cpn on_timer 首行 yield（active+活性双门，连 HOLD 零速都静默）
     + 订阅 + 默认 False + 跳变日志 + 发布前复查 + 旗 staleness 活性门
  F. _cpn_gate 运行时状态机（stub 实例离线驱动，双对抗校验共同指出的覆盖
     缺口）：terminal 1.2 进 / 1.6 出迟滞 + 换腿强制解除；出发场 2.0 清 /
     1.5 回锁迟滞；冷启动无邻居 fail-closed 5s 窗；过期邻居剔除；internal
     短路
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/zx2026_arena_ws")
# 只插 arena_nav/scripts：nav_node 顶部 import zx2026_common.msg（devel 生成
# 包），源包无 msg 子包会遮蔽（与 bh_selftest 同款结构，见其头注）。
sys.path.insert(0, os.path.join(HERE, "..", "src", "arena_nav", "scripts"))

try:
    from nav_node import NavNode, vel_source_of, cpn_active_of
except ImportError as e:
    sys.stderr.write("import nav_node 失败(%s)。\n请先: source /opt/ros/noetic/"
                     "setup.bash && source devel/setup.bash\n" % e)
    sys.exit(2)

from zx2026_common import config as cfg

NAV_NODE = os.path.join(WS, "src", "arena_nav", "scripts", "nav_node.py")
CPN_NODE = os.path.join(WS, "src", "defop_nav", "scripts", "defop_cpn_node.py")
OUT = "/tmp/defop_shadow_selftest.txt"

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))


def test_parse_matrix():
    """A. vel_source_of 解析矩阵（保守回退三连 + 只放行 defop）。"""
    cases = [
        ({}, "internal", "键缺失"),
        ({"planner": None}, "internal", "planner 段 null（yaml 空段）"),
        ({"planner": {}}, "internal", "planner 段空"),
        ({"planner": {"source": "defop"}}, "defop", "defop 放行"),
        ({"planner": {"source": "internal"}}, "internal", "显式 internal"),
        ({"planner": {"source": "bogus"}}, "internal", "值不识"),
        ({"planner": {"source": ""}}, "internal", "空串"),
        ({"planner": {"source": "DEFOP"}}, "internal", "大小写敏感（保守）"),
        ({"planner": {"source": None}}, "internal", "source 键 null"),
    ]
    for settings, want, tag in cases:
        got = vel_source_of(settings)
        check("A %s" % tag, got == want, "got=%s want=%s" % (got, want))


def test_real_config():
    """B. 真实 sim_settings.yaml 解析合法（键缺失当下必 internal）。"""
    settings = cfg.load("sim_settings.yaml")
    got = vel_source_of(settings)
    src = (settings.get("planner", {}) or {}).get("source", "internal")
    want = "defop" if str(src) == "defop" else "internal"
    check("B 真实配置解析合法", got == want,
          "yaml source=%r -> %s" % (src, got))


def method_body(text, name):
    """取类方法源码体（def <name>(self...) 到下一个同级 def；参数表任意）。"""
    m = re.search(r"^    def %s\(self\b.*?\):" % re.escape(name), text, re.M)
    if not m:
        return None
    nxt = re.search(r"^    (def |@)", text[m.end():], re.M)
    return text[m.end():m.end() + nxt.start()] if nxt else text[m.end():]


def test_static():
    """C. nav_node.py 发布点门控与 arrived 保活静态核查。"""
    with open(NAV_NODE, "r", encoding="utf-8") as f:
        text = f.read()

    # C1 vel_cmd 发布点恰 3 处，且各自被门控（P3 终端交接后结构）
    n_pub = len(re.findall(r"self\.pub_vel\.publish\(", text))
    check("C1a vel_cmd 发布点恰 3 处", n_pub == 3, "found=%d" % n_pub)
    # landed 一次性零速：nav 持笔门内（internal 恒 False=行为不变；defop
    # 模式 LANDED 发生在 terminal 终端态，cpn 静默、nav 持笔照发不抢帧）
    landed = method_body(text, "_on_task_update") or ""
    m = re.search(r'if self\._vel_source == "defop":\s*\n\s+'
                  r'self\.pub_cpn_active\.publish\(Bool\(data=False\)\)\s*\n\s+'
                  r'self\.pub_vel\.publish\(Twist\(\)\)', landed)
    check("C1b landed 无条件零速 + 补一枪 False 旗", m is not None,
          "halt 契约无条件（校验 V1-B2 巡航中急停洞：掐零速+停播旗后 cpn "
          "仍持笔续飞）；internal 模式行为逐字节不变")
    # 主循环：defop 臂发布在 nav 持笔门内，internal 臂直发
    tick = method_body(text, "_tick") or ""
    m = re.search(r'if self\._vel_source == "defop":.*?'
                  r'if not self\._cpn_active:\s*\n\s+'
                  r'self\.pub_vel\.publish\(tw\)', tick, re.S)
    check("C1c 主循环 defop 臂 nav 持笔门", m is not None,
          "cpn_active=False（terminal/departure/no_goal）才由 nav 发布")
    m = re.search(r'else:\s*\n\s+self\.pub_vel\.publish\(tw\)', tick)
    check("C1d internal 臂直发保留", m is not None,
          "else: pub_vel.publish(tw) 逐字节不变")

    # C2 发布器无条件创建
    lines = [ln for ln in text.splitlines()
             if "self.pub_vel = rospy.Publisher(" in ln]
    ok = (len(lines) == 1 and "_vel_source" not in lines[0]
          and 'ns + "/vel_cmd"' in lines[0])
    check("C2 pub_vel 无条件创建", ok,
          "defop 模式也有 publisher（防下游 can't binder 误判）")

    # C3 arrived 两发射点不在门内（defop 模式天然保活）
    n_arr = len(re.findall(r"self\.pub_arrived\.publish\(", text))
    check("C3a arrived 发射点恰 2 处", n_arr == 2, "found=%d" % n_arr)
    arr_lines = [ln for ln in text.splitlines()
                 if "self.pub_arrived.publish(" in ln]
    ok = all("_vel_source" not in ln for ln in arr_lines)
    check("C3b arrived 发射线无 source 门", ok,
          "无 odom/goal 时 False 心跳 + 主循环 goal_reached 广播")

    # C4 goal_reached 纯度：零 _vel_source、零规划状态引用
    gr = method_body(text, "goal_reached") or ""
    banned = ["_vel_source", "self.path", "waypoint_idx", "_gpath",
              "_plan_fail_since", "_de_active", "_gs_active"]
    hits = [b for b in banned if b in gr]
    check("C4 goal_reached 判据纯度", not hits,
          "纯 odom/est-vs-goal 距离判据" if not hits
          else "混入规划状态: %s" % hits)

    # C5 landed 路径 _halted 簿记仍在门之前
    m = re.search(r"self\._halted = True", landed)
    check("C5 landed _halted 簿记保留", m is not None,
          "landing/done 协调不动，只掐发布")

    # C6 启动日志带 vel_src 字段
    check("C6 启动日志 vel_src 字段", "vel_src=%s" in text,
          "活体可辨识节点当前发布源")


def test_gate_matrix():
    """D. cpn_active_of 纯函数矩阵（P3 终端交接闸门语义核）。"""
    # (a) internal 恒 False：terminal/departure/goal 任意组合都不得翻 True
    for term in (False, True):
        for dep in (False, True):
            for hg in (False, True):
                got = cpn_active_of(False, term, dep, hg)
                check("D internal 恒 False (term=%s dep=%s goal=%s)"
                      % (term, dep, hg), got is False, "got=%s" % got)
    # (b) defop + cruise（未进终端）+ 出发场清空 + 有腿目标 -> cpn 持笔
    got = cpn_active_of(True, False, True, True)
    check("D defop cruise 窗口 cpn 持笔", got is True, "got=%s" % got)
    # (c) defop + terminal 终端接近 -> nav 持笔
    got = cpn_active_of(True, True, True, True)
    check("D defop terminal nav 持笔", got is False, "got=%s" % got)
    # (d) defop + 出发场未清 -> nav 持笔
    got = cpn_active_of(True, False, False, True)
    check("D defop departure 未清 nav 持笔", got is False, "got=%s" % got)
    # (e) defop + 无腿目标 -> nav 持笔
    got = cpn_active_of(True, False, True, False)
    check("D defop no_goal nav 持笔", got is False, "got=%s" % got)


def test_gate_static():
    """E. P3 终端交接两端静态核查（nav 闸门 + cpn yield 门）。"""
    with open(NAV_NODE, "r", encoding="utf-8") as f:
        text = f.read()
    # E1 闸门方法存在：迟滞阈值 1.2/1.6 在体内 + cpn_active_of 收口
    gate = method_body(text, "_cpn_gate") or ""
    ok = bool(gate) and "1.2" in gate and "1.6" in gate \
        and "cpn_active_of(" in gate
    check("E1 _cpn_gate 迟滞与语义核接线", ok,
          "2D<1.2 进入 / >1.6 退出 + cpn_active_of 收口")
    # E2 defop 门内持笔旗每拍计算并广播（cpn yield 的源头旗）
    tick = method_body(text, "_tick") or ""
    m = re.search(r'if self\._vel_source == "defop":\s*\n\s+'
                  r'self\._cpn_active = self\._cpn_gate\(now\)\s*\n\s+'
                  r'self\.pub_cpn_active\.publish\('
                  r'Bool\(data=self\._cpn_active\)\)', tick)
    check("E2 持笔旗每拍广播", m is not None,
          "nav 每拍 /planning/cpn_active；internal 不算不广播")
    # E3 SHADOW 心跳扩容：cpn=(旗)(原因)
    check("E3 SHADOW 心跳扩容", "cpn=%s(%s)" in text,
          "cpn_active + _term_reason 活体证据")
    # E4 持笔旗发布器无条件创建（internal 模式也有，防 binder 误判）
    lines = [ln for ln in text.splitlines()
             if "self.pub_cpn_active = rospy.Publisher(" in ln]
    ok = (len(lines) == 1 and "_vel_source" not in lines[0]
          and 'ns + "/planning/cpn_active"' in lines[0])
    check("E4 pub_cpn_active 无条件创建", ok, "internal 模式也有 publisher")

    # ---- defop_cpn 端 ----
    with open(CPN_NODE, "r", encoding="utf-8") as f:
        ctext = f.read()
    # E5 on_timer 首行 yield：active/活性双门先于 busy 锁（连 HOLD 零速都静默）
    m = re.search(r"def on_timer\(self, _evt\):.*?"
                  r"if not self\.active or not self\._flag_fresh\(\):\s*\n\s*"
                  r"return.*?_busy\.acquire", ctext, re.S)
    check("E5 cpn on_timer 首行 yield（active+活性双门）", m is not None,
          "not active / 旗超时 -> return（nav 持笔期零发布，含 HOLD 零速）")
    # E5b 旗 staleness 活性门（校验 V1-M5）：到达时刻簿记 + 1.0s 阈值
    ok = ("def _flag_fresh(self):" in ctext
          and "self._flag_t = rospy.Time.now().to_sec()" in ctext
          and "self._flag_t = None" in ctext
          and "<= 1.0" in ctext)
    check("E5b cpn 旗 staleness 活性门", ok,
          "_flag_t 簿记 + 1.0s 阈值（nav 死/停播 -> cpn 让位静默，"
          "gz 插件 0.5s 断链保护接管）")
    # E5c 推理后发布前复查（校验 V1-B1/V2-M3 双写瞬时窗）
    m = re.search(r"cmd, dbg = self\.planner\.plan_once\(now\).*?"
                  r"if not self\.active or not self\._flag_fresh\(\):\s*\n\s*"
                  r"return.*?dbg\['t_plan_ms'\]", ctext, re.S)
    check("E5c cpn 发布前复查旗", m is not None,
          "plan_once（0.2-0.5s）途中旗翻转 -> 丢弃本拍，双写窗压到零")
    # E6 持笔旗订阅 + 默认 False（旗未到达前 nav 持笔）+ 跳变日志
    ok = ("self.base + '/planning/cpn_active'" in ctext
          and "self.active = False" in ctext
          and "cpn_active <- %s" in ctext)
    check("E6 cpn 订阅+默认 False+跳变日志", ok,
          "默认 nav 持笔；跳变 loginfo（低频事件）")


def test_gate_runtime():
    """F. _cpn_gate 运行时状态机（stub 实例离线驱动）。

    D 段只证纯函数矩阵，状态迁移/迟滞/冷启动窗无运行时证据（双对抗校验
    共同指出的覆盖缺口）。本段用 NavNode.__new__ 绕过 __init__（无 ROS
    句柄）注入属性，直接调真方法——距离映射/迟滞带/换腿重锁/出发场迟滞/
    冷启动 fail-closed/过期邻居剔除全部落到真实代码路径。
    """
    NOW = 100.0

    def mk(goal=(10.0, 0.0, 1.2), odom=(0.0, 0.0, 1.2), neighbors=None,
           vel_source="defop", nb_age=0.0):
        s = NavNode.__new__(NavNode)      # 不跑 __init__（无 rospy 句柄）
        s._vel_source = vel_source
        s.goal = goal
        s.odom = tuple(odom)
        s.neighbors = dict(neighbors or {})
        s.neighbor_last = dict((j, NOW - nb_age) for j in s.neighbors)
        s._cpn_depart_r = 2.0
        s._term_engaged = False
        s._term_goal_id = None
        s._dep_clear = False
        s._goal_first_t = None
        s._term_reason = "init"
        return s

    def g(s, now):
        return NavNode._cpn_gate(s, now)

    NB = {1: (0.0, 5.0, 1.0)}             # 已散开的新鲜邻居
    # F1 cruise：远处目标 + 出发场已清（冷启动窗已过）-> cpn 持笔
    s = mk(neighbors=NB)
    s._goal_first_t = NOW - 10.0
    ok = g(s, NOW)
    check("F1 cruise 窗口 cpn 持笔", ok is True and s._term_reason == "cruise",
          "got=%s reason=%s" % (ok, s._term_reason))
    # F2 terminal 进入：2D d=1.19 < 1.2 -> nav 持笔
    s = mk(odom=(8.81, 0.0, 1.2), neighbors=NB)
    s._goal_first_t = NOW - 10.0
    ok = g(s, NOW)
    check("F2 terminal 进入 d=1.19", ok is False and s._term_reason == "terminal",
          "got=%s reason=%s" % (ok, s._term_reason))
    # F3 未进入：d=1.21 > 1.2（初始态）-> cpn 持笔
    s = mk(odom=(8.79, 0.0, 1.2), neighbors=NB)
    s._goal_first_t = NOW - 10.0
    ok = g(s, NOW)
    check("F3 未进入 d=1.21 仍 cruise", ok is True, "got=%s" % ok)
    # F4 退出迟滞：已终端态 d=1.5 仍 terminal；d=1.61 解除
    s = mk(odom=(8.81, 0.0, 1.2), neighbors=NB)
    s._goal_first_t = NOW - 10.0
    g(s, NOW)                              # 进入终端
    s.odom = (8.50, 0.0, 1.2)              # d=1.5（迟滞带内）
    ok = g(s, NOW + 0.1)
    check("F4 退出迟滞 d=1.5 仍 terminal", ok is False, "got=%s" % ok)
    s.odom = (8.39, 0.0, 1.2)              # d=1.61
    ok = g(s, NOW + 0.2)
    check("F4b 退出迟滞 d=1.61 解除", ok is True and s._term_engaged is False,
          "got=%s engaged=%s" % (ok, s._term_engaged))
    # F5 换腿强制解除：终端态中 goal 变更（新目标远）-> cpn 重新持笔
    s = mk(odom=(8.81, 0.0, 1.2), neighbors=NB)
    s._goal_first_t = NOW - 10.0
    g(s, NOW)
    s.goal = (-10.0, 0.0, 1.2)
    ok = g(s, NOW + 0.1)
    check("F5 换腿强制解除终端", ok is True and s._term_engaged is False,
          "got=%s engaged=%s" % (ok, s._term_engaged))
    # F6 出发场迟滞：1.5 不清 -> 2.1 清 -> 1.6 保持清 -> 1.4 重封锁
    s = mk(neighbors={1: (0.0, 1.5, 1.0)})
    s._goal_first_t = NOW - 10.0
    ok = g(s, NOW)
    check("F6 出发场 d=1.5 未清", ok is False and s._term_reason == "departure",
          "got=%s reason=%s" % (ok, s._term_reason))
    s.neighbors = {1: (0.0, 2.1, 1.0)}
    ok = g(s, NOW + 0.1)
    check("F6b 出发场 d=2.1 放行", ok is True, "got=%s" % ok)
    s.neighbors = {1: (0.0, 1.6, 1.0)}
    ok = g(s, NOW + 0.2)
    check("F6c 出发场迟滞 d=1.6 保持清空", ok is True, "got=%s" % ok)
    s.neighbors = {1: (0.0, 1.4, 1.0)}
    ok = g(s, NOW + 0.3)
    check("F6d 出发场迟滞 d=1.4 重封锁", ok is False, "got=%s" % ok)
    # F7 冷启动 fail-closed：无邻居 + 首目标刚到达 -> nav 持笔；5s 后放行
    s = mk()
    ok = g(s, NOW)
    check("F7 冷启动无邻居 fail-closed",
          ok is False and s._term_reason == "departure",
          "got=%s reason=%s" % (ok, s._term_reason))
    ok = g(s, NOW + 6.0)
    check("F7b 冷启动 5s 窗后放行", ok is True, "got=%s" % ok)
    # F8 过期邻居剔除：>1.0s 未更新=不算在场（冷启动窗语义接管）
    s = mk(neighbors={1: (0.0, 1.0, 1.0)})
    s._goal_first_t = NOW - 10.0
    s.neighbor_last = {1: NOW - 5.0}       # 贴脸但失联
    ok = g(s, NOW)
    check("F8 过期邻居剔除", ok is True, "got=%s" % ok)
    # F9 internal 短路：恒 nav 持笔（行为逐字节不变）
    s = mk(vel_source="internal")
    ok = g(s, NOW)
    check("F9 internal 恒 False 短路",
          ok is False and s._term_reason == "internal",
          "got=%s reason=%s" % (ok, s._term_reason))


def main():
    test_parse_matrix()
    test_real_config()
    test_static()
    test_gate_matrix()
    test_gate_static()
    test_gate_runtime()
    npass = sum(1 for (_, ok, _) in RESULTS if ok)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write("defop_shadow_selftest 结果 (%d/%d PASS)\n" % (npass, len(RESULTS)))
        for (name, ok, detail) in RESULTS:
            f.write("  [%s] %s%s\n" % ("PASS" if ok else "FAIL", name,
                                       (" — " + detail) if detail else ""))
    print("defop_shadow_selftest rc=%d (%d/%d)"
          % (0 if npass == len(RESULTS) else 1, npass, len(RESULTS)))
    sys.exit(0 if npass == len(RESULTS) else 1)


if __name__ == "__main__":
    main()
