#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_run.py 观测窗判据干测（2026-10-08 孤儿法证回归）。

背景：10-07 我的批跑单元 `timeout 560 python3 verify_run.py` 空转 40 分钟，
SIM 地板停在 t=152，SIGTERM 被吞，还覆盖了 /tmp/zx2026_verify.txt 使同批
另一格失去有效单元。根因 = 观测循环唯一判据 `t_sim - t0s < observe_s`，而
t_sim 被 max() 钳住 ⇒ /clock 冻结时恒真。

本测**不依赖仿真**：只喂 observation_should_continue() 纯函数。
用法: python3 tools/verify_gate_selftest.py
"""
import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)


def _load():
    p = os.path.join(_ROOT, "verify_run.py")
    spec = importlib.util.spec_from_file_location("verify_run_under_test", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    m = _load()
    if not hasattr(m, "observation_should_continue"):
        print("FAIL: verify_run.py 无 observation_should_continue")
        return 1
    f = m.observation_should_continue
    fails = []

    def chk(name, got, want):
        if got != want:
            fails.append("%s: got %r want %r" % (name, got, want))

    # 常用参数：180 → observe 240 的档位任取；这里用 250+60=310 / cap 800
    K = dict(t0s=100.0, observe_s=310.0, wall_cap=800.0, stall_s=60.0)
    base = dict(t_sim=150.0, wall_elapsed=50.0, since_clock=0.5, shutting_down=False)

    def call(**over):
        a = dict(K)
        a.update(base)
        a.update(over)
        return f(a["t_sim"], a["t0s"], a["observe_s"], a["wall_elapsed"],
                 a["wall_cap"], a["since_clock"], a["stall_s"], a["shutting_down"])

    # 1 常规：窗内、时钟活、未关停 → 继续
    chk("normal", call(), (True, None))

    # 2 ★回归主角：/clock 冻结（sim 停在 152，61s 墙钟无新 clock）
    got = call(t_sim=152.0, wall_elapsed=560.0, since_clock=61.0)
    chk("clock-stall", got[0], False)
    if got[0] is False and "CLOCK STALLED" not in (got[1] or ""):
        fails.append("clock-stall 原因串未写明: %r" % (got[1],))

    # 3 ★旧实现反证：同一输入，旧判据恒真（这就是 40 分钟空转）
    old_go = (152.0 - K["t0s"]) < K["observe_s"]
    chk("legacy-invariant-provably-stuck", old_go, True)   # 旧式=永远继续
    chk("new-breaks-it", call(t_sim=152.0, wall_elapsed=560.0, since_clock=61.0)[0], False)

    # 4 停摆判据只在确实过期时触发（59.9s 不杀）
    chk("stall-boundary-under", call(t_sim=152.0, since_clock=59.9)[0], True)
    chk("stall-boundary-over", call(t_sim=152.0, since_clock=60.1)[0], False)

    # 5 墙钟上限
    chk("wall-cap", call(wall_elapsed=800.1)[0], False)
    chk("wall-cap-under", call(wall_elapsed=799.9)[0], True)

    # 6 sim 预算耗尽 = 正常出口，原因串必须是 BUDGET（不是被误报成停摆）
    chk("budget", call(t_sim=410.0, since_clock=0.5), (False, "BUDGET"))

    # 7 SIGTERM：优先级最高，必须压过其它
    chk("shutdown-wins", call(shutting_down=True, wall_elapsed=9999.0, since_clock=9999.0),
        (False, "ROS SHUTDOWN"))

    # 8 无 /clock（墙钟回退档）：since_clock=None 不做停摆判定，语义不变
    chk("wall-fallback-no-stall", call(t_sim=152.0, since_clock=None)[0], True)

    if fails:
        print("VERIFY GATE SELFTEST FAIL")
        for x in fails:
            print("  -", x)
        return 1
    print("VERIFY GATE SELFTEST PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
