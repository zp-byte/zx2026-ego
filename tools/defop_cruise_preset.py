#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_cruise_preset.py — 切换林带巡航高度解绑（heights.cruise_free / belt_entry_z）。

  off  = 旧行为（cruise_free=false，巡航腿拉到 cruise_z）
  b    = 解绑，进林带高度 1.5（= 离台悬停高度，用户指令字面实现）
  c    = 解绑，进林带高度 2.4（探索臂：飞高让开树干）
  d    = 解绑，进林带高度 3.0（2026-10-01 用户裁决的"保持 3m"臂 —— **明知越界**）
  show = 当前值（含 yaml 类型往返校验）

只改 competition_rules.yaml 的这两个键，其余逐字节不动。

【2026-10-01 血案】上一版写回时把「值」与「# 注释」之间的空白吃掉了，
产出 `belt_entry_z: 1.5# 开时…` —— YAML 里 `#` 前无空白**不是注释**，于是
整个变成字符串，executor `float()` 当场炸（arm A 整臂作废）。`cruise_free`
同病更阴：`false#…` 是非空字符串，`bool()` 恒 True，会**静默把对照臂开成
处置臂**。故本版两道锁：①保留原空白；②写完立刻回读 yaml 断言**类型**。

硬约束提醒：belt_entry_z < 冠下缘 2.81（sim_settings.yaml 内联不变式），
宜 <= map_z_band 顶 2.60。越顶 = 冠撞。
"""
import io
import re
import sys

YAML = "/home/ubuntu/zx2026_defop_ws/src/zx2026_common/config/competition_rules.yaml"
NODE = "/home/ubuntu/zx2026_defop_ws/src/arena_mission/scripts/mission_executor_node.py"
# 捕获组 3 = 值后的空白（必须原样保留），组 4 = 注释
P_FREE = re.compile(r"^(\s*cruise_free:\s*)(true|false)(\s*)(#.*)?$", re.M)
P_BELT = re.compile(r"^(\s*belt_entry_z:\s*)([-\d.]+)(\s*)(#.*)?$", re.M)
CEIL = 2.60     # map_z_band 证据窗顶；冠下缘 2.81 是硬顶
CROWN = 2.81    # 冠下缘（sim_settings.yaml 内联不变式；203 棵树实测最低值）

MODES = {"off": ("false", "1.5"), "b": ("true", "1.5"),
         "c": ("true", "2.4"), "d": ("true", "3.0")}


def warn_invariant(val):
    """越界告警：把"越了多少、代价是什么"印成人话，杜绝事后自欺。"""
    v = float(val)
    if v < CROWN:
        return
    print("WARN: belt_entry_z=%s **越过冠下缘硬不变式 %.2f**。" % (val, CROWN))
    print("      203 棵树实测：冠底 <=3.00 者 8 棵(3.9%%)、<=3.50 者 62 棵(30.5%%)。")
    print("      被卷入的树，障碍半径从树干 0.13~0.21 涨到冠包络 1.83~2.82（10~20 倍）。")
    if v >= CEIL:
        print("      且 %s >= map_z_band 顶 %.2f —— 该高度在 nav 点云证据带**之外**。" % (val, CEIL))


def sub_keep(pat, val, s):
    """替换值，**原样保留** 值↔注释 之间的空白（本版的核心修复）。"""
    def rep(m):
        sep = m.group(3) or (" " if m.group(4) else "")
        return m.group(1) + val + sep + (m.group(4) or "")
    return pat.sub(rep, s, count=1)


def load():
    s = io.open(YAML, encoding="utf-8").read()
    mf, mb = P_FREE.search(s), P_BELT.search(s)
    if not mf or not mb:
        raise SystemExit("FAIL: yaml 里找不到 cruise_free / belt_entry_z（补丁未施加？）")
    return s, mf, mb


def roundtrip():
    """回读 yaml，断言**类型**正确（bool / float）——字符串化即判负。"""
    import yaml
    try:
        h = yaml.safe_load(io.open(YAML, encoding="utf-8"))["heights"]
    except Exception as e:                                   # noqa: BLE001
        return False, "yaml 解析失败: %s" % e
    f, b = h.get("cruise_free"), h.get("belt_entry_z")
    if not isinstance(f, bool):
        return False, "cruise_free 不是 bool 而是 %r（值↔注释空白被吃？）" % (f,)
    if not isinstance(b, float):
        return False, "belt_entry_z 不是 float 而是 %r（值↔注释空白被吃？）" % (b,)
    return True, "cruise_free=%s(bool)  belt_entry_z=%s(float)" % (f, b)


def selfcheck():
    n = io.open(NODE, encoding="utf-8").read()
    a = n.count('h.get("cruise_free"')
    c = n.count("self.belt_entry_z")
    b = n.count("self._belt_z()")
    ok = (a == 1 and c == 3 and b == 4)
    print("executor 读取点: cruise_free 载入 %d 处 / belt_entry_z %d 处 / _belt_z() %d 处%s"
          % (a, c, b, "" if ok else "  <-- 期望 载入1·belt_entry_z 3(载入+_belt_z体+起飞else)·_belt_z() 4(3腿+_wait_pos)，异常！"))
    return ok


def main():
    arg = (sys.argv[1] if len(sys.argv) > 1 else "show").strip().lower()
    s, mf, mb = load()
    cur = (mf.group(2), mb.group(2))
    if arg == "show":
        ok, msg = roundtrip()
        print("cruise_free = %s   belt_entry_z = %s" % cur)
        print("yaml 往返: %s%s" % (msg, "" if ok else "   <-- FAIL"))
        return 0 if (ok and selfcheck()) else 1
    if arg not in MODES:
        raise SystemExit("用法: %s {off|b|c|d|show}" % sys.argv[0])
    want_f, want_b = MODES[arg]
    warn_invariant(want_b)
    bak = s
    s = sub_keep(P_FREE, want_f, s)
    s = sub_keep(P_BELT, want_b, s)
    io.open(YAML, "w", encoding="utf-8").write(s)
    ok, msg = roundtrip()
    if not ok:
        io.open(YAML, "w", encoding="utf-8").write(bak)     # 坏了就回滚，绝不带病开跑
        print("FAIL 写回后 yaml 往返校验不过，已回滚。%s" % msg)
        return 1
    print("cruise_free: %s -> %s   belt_entry_z: %s -> %s   [mode=%s]"
          % (cur[0], want_f, cur[1], want_b, arg))
    print("yaml 往返: %s" % msg)
    return 0 if selfcheck() else 1


if __name__ == "__main__":
    raise SystemExit(main())
