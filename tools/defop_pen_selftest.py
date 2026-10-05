#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_pen_selftest.py — pen_authority 刀一 + cloud_nav 刀一B 守卫（可失败自检）。

docs/pen_authority_case.md §三+§五：主键 planner.pen_authority: nav|cpn_forever
（默认 nav = 逐字节旧行为），cpn_forever 三捆绑不可拆（起飞闸门/departure
旁路/stall 冻结）；配套 planner.cloud_nav: keep|off（默认 keep = 旧行为；
2026-10-05 用户裁决：避障=深度图单源，纯 DeFoP+坐标——off ⇒ lidar 停发 +
nav/executor 不订 /cloud）。

  A) 源码钉：五文件捆绑点全部在场 + 默认档（nav/keep）零行为变更的静态保证。
  B) 行为钉：从**真源码** ast 抽 pen_mode_of / vel_source_of / cpn_active_of /
     _cpn_beat_stale 编译执行——forever 旁路语义、关档真值表、配置保守回退。

失败即非零退出。用法: python3 tools/defop_pen_selftest.py
"""
import ast
import re
import sys

WS = "/home/ubuntu/zx2026_defop_ws"
NAV = WS + "/src/arena_nav/scripts/nav_node.py"
CPN = WS + "/src/defop_nav/scripts/defop_cpn_node.py"
EXE = WS + "/src/arena_mission/scripts/mission_executor_node.py"
LID = WS + "/src/arena_sensor/scripts/lidar_node.py"
YAML = WS + "/src/zx2026_common/config/sim_settings.yaml"

fails = []


def chk(name, cond, detail=""):
    print("%s %-52s %s" % ("[PASS]" if cond else "[FAIL]", name, detail))
    if not cond:
        fails.append(name)


def extract_fn(path, name):
    """从真源码 ast 抽函数（模块级/类内皆可），编译可执行。"""
    tree = ast.parse(open(path).read())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            return compile(mod, "<pen_selftest>", "exec")
    raise SystemExit("函数缺失: " + name)


def part_a():
    nav = open(NAV).read()
    cpn = open(CPN).read()
    exe = open(EXE).read()
    lid = open(LID).read()
    yml = open(YAML).read()

    # ---- 刀一 pen_authority ----------------------------------------------
    # A1 yaml 默认档：nav + 冻结阈 1.0（默认即旧行为）
    chk("A1 yaml pen_authority: nav 默认",
        "pen_authority: nav" in yml, "工作配置须默认 nav")
    chk("A1b yaml pen_stall_freeze_t: 1.0",
        "pen_stall_freeze_t: 1.0" in yml, "2-5Hz 实测健康间隙≤0.5s 的 2×")

    # A2 nav departure 旁路：旁路分支在 min_d 判定之前，且 reason 语义保留
    # （锚=唯一赋值行；刀一.5 后 nav 有两处 if self._pen_forever: 裸行——
    # 订阅门在 _cpn_gate 之前，裸行 find 会被抢锚，故钉赋值行不钉 if 行）
    i_byp = nav.find("departure_clear = True")
    i_mind = nav.find("elif min_d is None:")
    chk("A2 nav departure 旁路在场且序先于 min_d 判定",
        0 < i_byp < i_mind, "bypass@%d mind@%d" % (i_byp, i_mind))
    chk("A2b 旁路体唯一且无第二处赋值",
        nav.count("departure_clear = True") == 1)

    # A3 nav stall 冻结：elif（仅 CPN 持笔时触发）+ 零速 + 节流日志
    i_gate = nav.find("if not (self._cpn_active and self._cpn_ready):")
    i_frz = nav.find("elif self._pen_forever and self._cpn_beat_stale(now):")
    chk("A3 nav 冻结分支在场且为 elif（nav 持笔优先）",
        0 < i_gate < i_frz, "gate@%d freeze@%d" % (i_gate, i_frz))
    body = nav[i_frz:i_frz + 900]
    chk("A3b 冻结体发布零速 Twist()",
        "self.pub_vel.publish(Twist())" in body)
    chk("A3c 冻结体节流日志 PEN_STALL",
        "PEN_STALL" in body)
    chk("A3d _cpn_beat_stale 停摆判据（None fail-safe + 阈值）",
        "_cpn_beat_t is None" in nav and "_cpn_stall_t" in nav)

    # A4 nav dbg 心跳订阅在场（stall 看门狗输入）
    chk("A4 nav 订阅 /defop/debug 心跳",
        'Subscriber(ns + "/defop/debug", String, self._on_cpn_dbg'
        in nav)

    # A5 cpn warm 旗：latched + models_ok 仅装载成功路径 + 三输入门
    chk("A5 cpn pub_warm latched",
        "pub_warm = rospy.Publisher(self.base + '/planning/cpn_warm'"
        in cpn)
    # set_models() 传 setter 也含 "self.models = models"（534 行良性第二处）——
    # 须锚成功路径的紧邻耦合（worker: 装载后紧接置位），且全文件唯一置位点
    ok_hits = [m.start() for m in re.finditer(r"self\.models = models", cpn)]
    i_true = cpn.find("self._models_ok = True")
    coupled = any(0 < p < i_true < p + 120 for p in ok_hits)
    chk("A5b _models_ok 仅在装载成功路径置位",
        coupled and cpn.count("self._models_ok = True") == 1,
        "hits@%s true@%d" % (ok_hits, i_true))
    chk("A5c _maybe_warm 门 = models_ok ∧ 三输入",
        "if self._models_ok and all(self._have.values()):" in cpn)
    chk("A5d warm 只发 True 且一次",
        "self._warm = True" in cpn and
        "self.pub_warm.publish(Bool(data=True))" in cpn)

    # A6 executor 起飞闸门：TAKEOFF_PENDING 内、state 翻转前、60s fail-open
    i_gate = exe.find("if self._pen_forever and not self._cpn_warm:")
    i_takeoff = exe.find('self.state = "TAKEOFF"')
    i_pending = exe.find('if self.state == "TAKEOFF_PENDING":')
    chk("A6 executor 闸门在 TAKEOFF_PENDING 内且先于 state 翻转",
        0 < i_pending < i_gate < i_takeoff,
        "pending@%d gate@%d takeoff@%d" % (i_pending, i_gate, i_takeoff))
    body = exe[i_gate:i_gate + 900]
    chk("A6b 60s fail-open 超时",
        "+ 60.0" in body)
    chk("A6c fail-open logerr 留痕",
        "logerr_throttle" in body and "fail-open" in body)
    chk("A6d 订阅仅 cpn_forever 注册",
        "if self._pen_forever:" in exe and
        '"/planning/cpn_warm"' in exe)

    # A7 默认档逐字节：_pen_forever 须 require source=defop ∧ mode=cpn_forever
    chk("A7 nav _pen_forever 双条件（source∧mode）",
        'self._pen_forever = (self._vel_source == "defop"' in nav and
        'pen_mode_of(settings) == "cpn_forever")' in nav)

    # A7b nav pen 档启动回声（审查 major：与 CPN pen_relay 行 rosout 对质）
    chk("A7b nav pen 档启动回声在场",
        "pen_authority=%s" in nav and "pen_mode_of(settings)" in nav)

    # ---- 刀一B cloud_nav（2026-10-05 裁决：避障=深度图单源） ----------------
    # A8 yaml 默认档：keep
    chk("A8 yaml cloud_nav: keep 默认",
        "cloud_nav: keep" in yml, "工作配置须默认 keep=旧行为")

    # A9 nav 云订阅受 _cloud_nav 门控（closed_loop 分支内）
    i_cl = nav.find("if self.closed_loop:")
    i_cloudsub = nav.find('Subscriber(ns + "/cloud", PointCloud2, '
                          'self._on_cloud)')
    i_gate9 = nav.find("if self._cloud_nav:")
    chk("A9 nav /cloud 订阅受 _cloud_nav 门控",
        0 < i_cl < i_gate9 < i_cloudsub,
        "closed_loop@%d gate@%d sub@%d" % (i_cl, i_gate9, i_cloudsub))
    chk("A9b nav _cloud_nav 解析（YAML1.1 off 布尔双接）",
        'not (_cn_v is False or str(_cn_v).strip().lower() == "off")' in nav)

    # A10 executor via_slots 云订阅受 _cloud_nav 门控
    chk("A10 executor via_slots 云订阅门控",
        "self._vs_source == \"cloud\" \\" in exe.replace("\n", " ")
        and "and self._cloud_nav:" in exe)
    chk("A10b executor _cloud_nav 解析（YAML1.1 off 布尔双接）",
        'not (_cn_v is False or str(_cn_v).strip().lower() == "off")' in exe)

    # A11 lidar cloud_nav=off ⇒ run() 空闲挂起（不 raycast 不发布）
    i_run = lid.find("def run(self):")
    i_idle = lid.find("rospy.spin()")
    chk("A11 lidar off 档空闲挂起（run 首行门 + spin）",
        0 < i_run < i_idle < i_run + 300, "run@%d spin@%d" % (i_run, i_idle))
    chk("A11b lidar keep 档照跑（cloud_nav 解析 + 默认 keep）",
        'not (_cn_v is False or str(_cn_v).strip().lower() == "off")' in lid
        and '_cn_v = _pl.get("cloud_nav", "keep")' in lid)

    # A12 YAML 1.1 布尔陷阱回归钉（10-05 smoke 定谳：裸 off 解析为布尔 False，
    # 旧判别 str(v)!="off" 恒真 ⇒ cloud_nav=off 全链失效、六机 lidar 照跑）。
    # 三形态判别：False→off / "off"→off / "keep"→keep / True→keep。
    try:
        import yaml as _yaml
        _pl_raw = _yaml.safe_load("planner:\n  cloud_nav: off\n")["planner"]
        _det = lambda v: not (v is False or str(v).strip().lower() == "off")
        chk("A12 YAML1.1 裸 off=布尔 False 且判别式双接",
            _pl_raw["cloud_nav"] is False
            and _det(_pl_raw["cloud_nav"]) is False
            and _det("keep") is True
            and _det("off") is False
            and _det(True) is True)
    except ImportError:
        chk("A12 YAML1.1 裸 off=布尔 False 且判别式双接", True, "yaml 缺失跳过")

    # ---- 刀一.5 sep 挂 CPN 统一组装点（pen_authority_case §三.4） ----------
    # A13 CPN 配置通道 + 话题切换
    chk("A13 cpn 导入 zx2026_common config（回退路径双接）",
        "from zx2026_common import config as _zx_cfg" in cpn
        and cpn.count("from zx2026_common import config as _zx_cfg") == 2
        and "_zx_cfg.load(\"sim_settings.yaml\")" in cpn)
    chk("A13b cpn pen_relay 旗严格解析 + 读败回落 nav 档",
        "self._pen_relay" in cpn
        and '== "cpn_forever"' in cpn
        and '.strip().lower() == "cpn_forever"' not in cpn
        and "pen_relay=nav fallback" in cpn,
        "严格全等对齐 nav pen_mode_of（审查 major）")

    # A14 CPN pub_cmd 话题条件切换（cpn_vel_raw | vel_cmd）
    i_relay = cpn.find("self._pen_relay = (")
    i_pub = cpn.find("self.pub_cmd = rospy.Publisher(_cmd_topic")
    chk("A14 cpn pub_cmd 条件话题（_pen_relay 先于 pub_cmd）",
        0 < i_relay < i_pub and "'/planning/cpn_vel_raw'" in cpn
        and cpn.count("'/planning/cpn_vel_raw'") == 1,
        "relay@%d pub@%d" % (i_relay, i_pub))

    # A15 nav 中继订阅仅 cpn_forever + 话题跨文件对齐
    i_gate15 = nav.find("if self._pen_forever:\n            rospy.Subscriber"
                        "(ns + \"/planning/cpn_vel_raw\"")
    chk("A15 nav 中继订阅受 _pen_forever 门控（话题与 cpn 侧逐字对齐）",
        i_gate15 > 0
        and '"/planning/cpn_vel_raw"' in nav
        and "_on_pen_raw" in nav)

    # A16 中继回调：单写者门 + sep 混合 + 零令放行 + 失败放行
    i_raw = nav.find("def _on_pen_raw(self, msg):")
    i_raw_end = nav.find("\n    def ", i_raw + 10) if i_raw > 0 else -1
    _raw = nav[i_raw:i_raw_end] if (i_raw > 0 and i_raw_end > i_raw) else ""
    chk("A16 _on_pen_raw 四要素（门/sep/零阈/放行）",
        "not (self._cpn_active and self._cpn_ready)" in _raw
        and "self._apply_separation(" in _raw
        and "> 0.05" in _raw
        and "pass-through" in _raw,
        "def@%d end@%d" % (i_raw, i_raw_end))

    # A16b 审查修复三钉（wf_ba224dac）：np 同型 / beat 刷新 / halt 短路
    chk("A16b _on_pen_raw np.array 同型+beat 刷新+halt 短路",
        "np.array(cmd, dtype=float)" in _raw
        and "self._cpn_beat_t = rospy.get_time()" in _raw
        and "if self._halted or not (self._cpn_active" in _raw)

    # A16c halt 分支清本地旗（旗与门同源）；find 自 i_halt 起搜——
    # self._cpn_active = False 首现在 __init__ 初始化，全局 find 会被抢锚
    i_halt = nav.find("self._halted = True")
    i_halt_clr = nav.find("self._cpn_active = False", i_halt + 1)
    chk("A16c halt 分支清 _cpn_active（_tick 早退后中继门同源关闭）",
        0 < i_halt < i_halt_clr < i_halt + 900,
        "halt@%d clr@%d" % (i_halt, i_halt_clr))

    # A17 中继保真：angular.z 透传 + 不挂 swarm/vo/cloud（sep 专属中继）
    chk("A17 _on_pen_raw angular.z 透传且不含 swarm/vo/cloud",
        "tw.angular.z = msg.angular.z" in _raw
        and "_apply_swarm" not in _raw
        and "_apply_vo_avoid" not in _raw
        and "_apply_cloud_avoidance" not in _raw)


def part_b():
    ns = {}
    exec(extract_fn(NAV, "pen_mode_of"), ns)
    exec(extract_fn(NAV, "vel_source_of"), ns)
    exec(extract_fn(NAV, "cpn_active_of"), ns)
    exec(extract_fn(NAV, "_cpn_beat_stale"), ns)
    pm, vs, ca = ns["pen_mode_of"], ns["vel_source_of"], ns["cpn_active_of"]

    # B1 pen_mode_of 保守回退五连（缺省/None 段/标量段/空段/未知值）
    chk("B1 pen_mode_of 回退五连全 nav",
        pm({}) == "nav"
        and pm({"planner": None}) == "nav"
        and pm({"planner": "x"}) == "nav"
        and pm({"planner": {}}) == "nav"
        and pm({"planner": {"pen_authority": "cpn_forEver"}}) == "nav"
        and pm({"planner": {"pen_authority": "cpn_forever"}}) == "cpn_forever")

    # B2 vel_source_of 逐字节不动（防回归）
    chk("B2 vel_source_of 语义不动",
        vs({"planner": {"source": "defop"}}) == "defop"
        and vs({}) == "internal"
        and vs({"planner": {"source": "cpn"}}) == "internal")

    # B3 cpn_active_of forever=False 旧真值表逐字节（5 组全覆盖）
    chk("B3 cpn_active_of 关档旧真值表",
        ca(False, False, True, True) is False      # 非 defop
        and ca(True, False, False, True) is False  # departure 未清
        and ca(True, False, True, False) is False  # 无 goal
        and ca(True, True, True, True) is False    # terminal 交接
        and ca(True, False, True, True) is True)   # cruise

    # B4 cpn_active_of forever=True：departure 旁路、terminal/no_goal/defop 保持
    chk("B4 forever=True departure 旁路生效",
        ca(True, False, False, True, forever=True) is True)
    chk("B4b forever=True terminal 交接保持",
        ca(True, True, False, True, forever=True) is False)
    chk("B4c forever=True no_goal 保持（起飞前不持笔）",
        ca(True, False, False, False, forever=True) is False)
    chk("B4d forever=True 非 defop 恒 False",
        ca(False, False, False, True, forever=True) is False)

    # B5 _cpn_beat_stale：None fail-safe / 阈内不冻 / 超阈冻结
    bs = ns["_cpn_beat_stale"]

    class D:
        pass
    d = D()
    d._cpn_beat_t, d._cpn_stall_t = None, 1.0
    r1 = bs(d, 100.0)
    d._cpn_beat_t = 99.5
    r2 = bs(d, 100.0)
    d._cpn_beat_t = 98.0
    r3 = bs(d, 100.0)
    chk("B5 beat_stale 三态（None 冻/0.5s 不冻/2.0s 冻）",
        r1 is True and r2 is False and r3 is True,
        "%s/%s/%s" % (r1, r2, r3))


def main():
    part_a()
    part_b()
    print()
    if fails:
        print("FAIL %d: %s" % (len(fails), ", ".join(fails)))
        sys.exit(1)
    print("ALL PASS（pen_authority 刀一 + cloud_nav 刀一B + 行为钉 全绿）")


if __name__ == "__main__":
    main()
