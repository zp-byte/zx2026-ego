#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_w3_gov_preset.py off|on|show — ② 预测性减速档切换（A/B 用）。

W3-② = CPN 前视速度调节器（DefopCPNPlanner._gov_scale）：硬否决 sup_* 只在
min_dist(1.0m) 内生效且无前视，本器自 gov_dist(2.0m) 起按比例压水平模长，
补上 2-5Hz 流水线吃掉的反应裕度。只减速不改向、不动 vz、地板永不 0。

  档   gov_enable   gov_dist  gov_stop  gov_floor
  off   false        2.0       0.35      0.45   （键在但关=逐字节旧行为）
  on    true         2.0       0.35      0.45

只动 src/defop_nav/config/defop_nav.yaml 一个文件（node 侧默认 dict 也是
False，双保险：yaml 缺席/被覆盖都不会意外开门）。
"""
import io
import re
import sys

YAML = "/home/ubuntu/zx2026_defop_ws/src/defop_nav/config/defop_nav.yaml"

PRESETS = {
    "off": {"gov_enable": "false", "gov_dist": "2.0",
            "gov_stop": "0.35", "gov_floor": "0.45"},
    "on": {"gov_enable": "true", "gov_dist": "2.0",
           "gov_stop": "0.35", "gov_floor": "0.45"},
}


def sub_line(text, key, val):
    pat = re.compile(r"^(\s*%s\s*:\s*)(true|false|[0-9.]+)" % re.escape(key), re.M)
    new, n = pat.subn(lambda m: m.group(1) + val, text)
    if n != 1:
        raise SystemExit("FAIL %s: 命中 %d 次（期望 1）" % (key, n))
    return new


def read(path, key):
    m = re.search(r"^\s*%s\s*:\s*(true|false|[0-9.]+)" % re.escape(key),
                  io.open(path, encoding="utf-8").read(), re.M)
    return m.group(1) if m else None


def show():
    for k in ("gov_enable", "gov_dist", "gov_stop", "gov_floor"):
        print("  %-12s = %s" % (k, read(YAML, k)))
    return True


def apply(preset):
    s = io.open(YAML, encoding="utf-8").read()
    for k, v in PRESETS[preset].items():
        s = sub_line(s, k, v)
    io.open(YAML, "w", encoding="utf-8").write(s)
    print("preset=%s applied" % preset)
    show()
    return 0


if __name__ == "__main__":
    a = sys.argv[1] if len(sys.argv) > 1 else "show"
    if a == "show":
        sys.exit(0 if show() else 1)
    if a in PRESETS:
        sys.exit(apply(a))
    raise SystemExit("usage: off|on|show")
