#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_zfloor_preset.py — 切换 closed_loop.z_floor_post（贴地 vz 地板二次兜底）。

  off  = false（默认，逐字节旧行为）   on = true    show = 当前值

只改 sim_settings.yaml 的这一个键，其余逐字节不动（sed 原地替换，不留
格式漂移）；顺带打印 nav 侧读取点的行，防"改了 yaml 没改载入点"。
"""
import io
import re
import sys

YAML = "/home/ubuntu/zx2026_defop_ws/src/zx2026_common/config/sim_settings.yaml"
NODE = "/home/ubuntu/zx2026_defop_ws/src/arena_nav/scripts/nav_node.py"
KEY = "z_floor_post"
PAT = re.compile(r"^(\s*%s:\s*)(true|false)\s*$" % KEY, re.M)


def read():
    s = io.open(YAML, encoding="utf-8").read()
    m = PAT.search(s)
    if not m:
        raise SystemExit("FAIL: yaml 里找不到 %s（补丁未施加？）" % KEY)
    return s, m


def main():
    arg = (sys.argv[1] if len(sys.argv) > 1 else "show").strip().lower()
    s, m = read()
    cur = m.group(2)
    if arg == "show":
        print("%s = %s" % (KEY, cur))
        return 0
    if arg not in ("on", "off"):
        raise SystemExit("用法: %s {on|off|show}" % sys.argv[0])
    want = "true" if arg == "on" else "false"
    s2 = PAT.sub(lambda mm: mm.group(1) + want, s, count=1)
    if s2 != s:
        io.open(YAML, "w", encoding="utf-8").write(s2)
    print("%s: %s -> %s" % (KEY, cur, want))
    # 联动自检：nav 侧确实有读取点
    n = io.open(NODE, encoding="utf-8").read()
    hits = n.count('cl.get("%s"' % KEY)
    print("nav 读取点: %d 处%s" % (hits, "" if hits == 1 else "  <-- 期望 1，异常！"))
    return 0 if hits == 1 else 1


if __name__ == "__main__":
    raise SystemExit(main())
