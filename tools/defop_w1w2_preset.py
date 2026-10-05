#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_w1w2_preset.py off|on|show — W1/W2 时序修复档切换（A/B 用）。

W1 = 进度感知腿看门狗（只作用于 GOTO_HI 长腿）：stall 捕真卡死、cap 捕慢爬。
W2 = 碰撞恢复窗收窄：cooldown_dt 2.0->1.5，**必须与** defop_nav 的
     collision_settle_s 3.0->2.5 成对改——CPN 在窗口内持笔（nav 静默），
     只砍世界侧 cooldown 等于零收益。配对不变量 = bounce_dt+cooldown_dt
     与 collision_settle_s 相等，脚本每次跑都断言。

  档   scan_leg_stall_s  cooldown_dt  collision_settle_s  恢复窗
  off       0.0（关）        2.0            3.0            3.0s（旧行为）
  on       10.0             1.5            2.5            2.5s

注意：本脚本改 defop_nav.yaml 的 collision_settle_s，而 tools/defop_cfg_preset.py
会把同一键钉回 3.0 —— **本脚本须在 defop_cfg_preset.py 之后应用**。
"""
import io
import re
import sys

WS = "/home/ubuntu/zx2026_defop_ws/src"
RULES = WS + "/zx2026_common/config/competition_rules.yaml"
SIMS = WS + "/zx2026_common/config/sim_settings.yaml"
NAVY = WS + "/defop_nav/config/defop_nav.yaml"
CPNPY = WS + "/defop_nav/scripts/defop_cpn_node.py"
BOUNCE_DT = 1.0                # sim_settings.collision.recovery.bounce_dt（本脚本不动）

PRESETS = {
    "off": {"scan_leg_stall_s": "0.0", "cooldown_dt": "2.0",
            "collision_settle_s": "3.0"},
    "on": {"scan_leg_stall_s": "10.0", "cooldown_dt": "1.5",
           "collision_settle_s": "2.5"},
}
FILES = [(RULES, "scan_leg_stall_s"), (SIMS, "cooldown_dt"),
         (NAVY, "collision_settle_s")]


def sub_line(text, key, val):
    """yaml 形态：`<indent>key: <num>`（保留行尾注释）。"""
    pat = re.compile(r"^(\s*%s\s*:\s*)[0-9.]+" % re.escape(key), re.M)
    new, n = pat.subn(lambda m: m.group(1) + val, text)
    if n != 1:
        raise SystemExit("FAIL %s: 命中 %d 次（期望 1）" % (key, n))
    return new


def sub_dict(text, key, val):
    """python dict 形态：`'key': <num>`（defop_cpn_node 的 default_cfg）。"""
    pat = re.compile(r"('%s':\s*)[0-9.]+" % re.escape(key))
    new, n = pat.subn(lambda m: m.group(1) + val, text)
    if n != 1:
        raise SystemExit("FAIL py %s: 命中 %d 次（期望 1）" % (key, n))
    return new


def read(path, key):
    m = re.search(r"^\s*%s\s*:\s*([0-9.]+)" % re.escape(key),
                  io.open(path, encoding="utf-8").read(), re.M)
    return float(m.group(1)) if m else None


def show():
    for path, key in FILES:
        print("  %-52s %s = %s" % (path.split("/src/")[1], key, read(path, key)))
    cd, cs = read(SIMS, "cooldown_dt"), read(NAVY, "collision_settle_s")
    exp = BOUNCE_DT + cd
    ok = abs(exp - cs) < 1e-9
    print("  配对不变量: bounce_dt(%.1f)+cooldown_dt(%s) = %.1f vs settle %s -> %s"
          % (BOUNCE_DT, cd, exp, cs, "OK" if ok else "**MISMATCH**"))
    return ok


def apply(preset):
    vals = PRESETS[preset]
    for path, _ in FILES:
        s = io.open(path, encoding="utf-8").read()
        for k, v in vals.items():
            if re.search(r"^\s*%s\s*:" % re.escape(k), s, re.M):
                s = sub_line(s, k, v)
        io.open(path, "w", encoding="utf-8").write(s)
    # python 侧默认值同步（YAML 有键时以 YAML 为准，但留着旧值是不一致暗雷）
    s = io.open(CPNPY, encoding="utf-8").read()
    s = sub_dict(s, "collision_settle_s", vals["collision_settle_s"])
    io.open(CPNPY, "w", encoding="utf-8").write(s)
    print("preset=%s applied" % preset)
    if not show():
        raise SystemExit("FAIL 配对不变量（W2 单侧改动会零收益）")
    return 0


if __name__ == "__main__":
    a = sys.argv[1] if len(sys.argv) > 1 else "show"
    if a == "show":
        sys.exit(0 if show() else 1)
    if a in PRESETS:
        sys.exit(apply(a))
    raise SystemExit("usage: off|on|show")
