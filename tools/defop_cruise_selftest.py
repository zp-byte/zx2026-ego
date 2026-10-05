#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_cruise_selftest.py — 巡航高度解绑干测（结构断言 + 真源码功能测）。

结构断言：把 mission_executor_node.py 里 26 个 self._goto(...) 调用点
**逐个抠出来**（括号配平，含跨行），断言 z 表达式的类别计数逐一精确——
防"改了 A 类漏了 B 类"或"C 类被误伤"。含**非默认档**断言。

功能测：把**真源码**的 _cruise_z_goal / _belt_z 抠出来 exec 进桩类，
True/False 两档都断言（库内教训：只测默认档等于没测）。

【2026-10-01 二补】B 类腿原为**无条件** self.belt_entry_z，缺省 1.5 ≠
旧值 cruise_z 1.2 ⇒ "关 = 逐字节旧行为"是假话。现改条件式 _belt_z()。
本测新增「_goto 内裸 belt_entry_z 必须带 cruise_free 守卫」哨兵，防复发。
"""
import io
import re
import sys

NODE = "/home/ubuntu/zx2026_defop_ws/src/arena_mission/scripts/mission_executor_node.py"
src = io.open(NODE, encoding="utf-8").read()
fails = 0


def chk(label, got, want):
    global fails
    ok = got == want
    if not ok:
        fails += 1
    print("  %-46s %-4s got=%s want=%s" % (label, "OK" if ok else "FAIL", got, want))


# ---------------- 结构断言 ----------------
print("== 结构断言：26 个 _goto 调用点的 z 表达式分类 ==")
calls = []
for m in re.finditer(r"self\._goto\(", src):
    i = src.index("(", m.end() - 1)
    depth, j = 0, i
    while j < len(src):
        if src[j] == "(":
            depth += 1
        elif src[j] == ")":
            depth -= 1
            if depth == 0:
                break
        j += 1
    calls.append(src[i:j + 1])
print("  抠出调用点 %d 个" % len(calls))
chk("总数（8 A + 3 B + 14 C + 1 _wait_pos）", len(calls), 26)

CATS = [
    ("A 类 _cruise_z_goal()",   "self._cruise_z_goal()", 8),
    ("B 类条件式 _belt_z()",     "self._belt_z()", 3),
    ("C 投放 self.identify_z",  "self.identify_z", 3),
    # 5 = 397/563/867/1157 四处 + L852 drop_mid_z 的兜底分支（同一次调用内）
    ("C 投放 self.drop_hover_z", "self.drop_hover_z", 5),
    ("C 投放 self.drop_mid_z",  "self.drop_mid_z", 1),
    ("C 起降 self.takeoff_hover_z", "self.takeoff_hover_z", 2),
    ("C 起降 self.touchdown_z", "self.touchdown_z", 1),
    ("C 急停/退赛 o[0],o[1],o[2]", "o[0], o[1], o[2]", 2),
    ("C 让位位调用 _wait_pos(dp)", "self._wait_pos(dp)", 1),
    ("C 投放点 z（cid 分档）", ", z))", 1),
]
for label, needle, want in CATS:
    chk(label, sum(1 for c in calls if needle in c), want)

# 旧值残留哨兵：任何 _goto 调用点都不该再直接引用 self.cruise_z
chk("无 _goto 残留裸 self.cruise_z", sum(1 for c in calls if "self.cruise_z" in c), 0)
# 二补哨兵：_goto 里出现的 belt_entry_z 必须全部带 cruise_free 守卫
chk("_goto 内裸 belt_entry_z（全须带守卫）",
    sum(1 for c in calls if "self.belt_entry_z" in c and "_cruise_free" not in c), 0)
# 起飞腿的守卫必须在
chk("起飞腿含 cruise_free 守卫",
    sum(1 for c in calls if "self.takeoff_hover_z" in c and "_cruise_free" in c), 1)
# _wait_pos 的返回（不是 _goto 调用，单独断言）
mw = re.search(r"def _wait_pos\(self, dp\):.*?return \((.*?)\)\n", src, re.S)
chk("_wait_pos 返回含 _belt_z()",
    (mw is not None) and ("self._belt_z()" in mw.group(1)), True)

# ---------------- 功能测（真源码） ----------------
print("\n== 功能测：真源码抠取，两档都测 ==")


def extract(head):
    i = src.index(head)
    j = src.index("\n    def ", i + len(head))
    body = src[i:j]
    lines = [(l[4:] if l.startswith("    ") else l) for l in body.split("\n")]
    ns = {}
    exec(compile("\n".join(lines), "<%s>" % head.strip(), "exec"), ns)
    print("  %-40s 真源码 %d 行" % (head.strip(), len(lines)))
    return ns[head.strip().split("(")[0].replace("def ", "")]


fn_goal = extract("    def _cruise_z_goal(self):\n")
fn_belt = extract("    def _belt_z(self):\n")


class Stub(object):
    pass


def run(fn, flag, odom_z, cruise_z, belt_z):
    o = Stub()
    o._cruise_free = flag
    o.odom = (0.0, 0.0, odom_z)
    o.cruise_z = cruise_z
    o.belt_entry_z = belt_z
    return fn(o)


print("  -- _cruise_z_goal --")
chk("默认档 off → cruise_z", run(fn_goal, False, 1.53, 1.2, 1.5), 1.2)
chk("非默认档 on → 当时高度", run(fn_goal, True, 1.53, 1.2, 1.5), 1.53)
chk("非默认档 on 且 cruise_z!=1.2", run(fn_goal, True, 2.41, 9.9, 1.5), 2.41)
chk("非默认档 on 高度为 0 → 仍取 0（不回落）", run(fn_goal, True, 0.0, 1.2, 1.5), 0.0)

print("  -- _belt_z（二补：关必须回旧值，不许走 belt_entry_z）--")
chk("默认档 off → cruise_z（不是 1.5！）", run(fn_belt, False, 0.9, 1.2, 1.5), 1.2)
chk("非默认档 on → belt_entry_z", run(fn_belt, True, 0.9, 1.2, 1.5), 1.5)
chk("非默认档 on bind 2.4", run(fn_belt, True, 0.9, 1.2, 2.4), 2.4)
chk("非默认档 on 且 cruise_z 被挪", run(fn_belt, True, 0.9, 7.7, 2.4), 2.4)
chk("默认档 off 且 cruise_z 被挪 → 跟 cruise_z", run(fn_belt, False, 0.9, 7.7, 1.5), 7.7)

# ---------------- 关档等价证明（arm A 必须是历史基线） ----------------
print("\n== 关档等价证明：常量折叠后逐调用点对质 HEAD ==")
import subprocess

REPO = NODE.split("/src/")[0]
TAIL = ("self.belt_entry_z if self._cruise_free\n"
        "                            else self.takeoff_hover_z")


def _calls(txt):
    out = []
    for m in re.finditer(r"self\._goto\(", txt):
        i = txt.index("(", m.end() - 1)
        d, j = 0, i
        while j < len(txt):
            if txt[j] == "(":
                d += 1
            elif txt[j] == ")":
                d -= 1
                if d == 0:
                    break
            j += 1
        out.append(" ".join(txt[i:j + 1].split()))
    return out


try:
    old = subprocess.check_output(
        ["git", "show", "HEAD:src/arena_mission/scripts/mission_executor_node.py"],
        cwd=REPO, stderr=subprocess.DEVNULL).decode("utf-8")
except Exception as e:                                          # noqa: BLE001
    old = ""
    print("  (跳过：取不到 HEAD 版本：%s)" % e)

if old:
    folded = (src.replace("self._cruise_z_goal()", "self.cruise_z")
                 .replace("self._belt_z()", "self.cruise_z")
                 .replace(TAIL, "self.takeoff_hover_z"))
    co, cf = _calls(old), _calls(folded)
    bad = [i for i, (a, b) in enumerate(zip(co, cf)) if a != b]
    for i in bad:
        print("      #%02d 旧 %s\n          折 %s" % (i, co[i], cf[i]))
    chk("关档折叠后 26 点全等 HEAD", len(bad), 0)
    on = _calls(src.replace("self._cruise_z_goal()", "self.odom[2]")
                   .replace("self._belt_z()", "self.belt_entry_z")
                   .replace(TAIL, "self.belt_entry_z"))
    chk("开档 A 类 -> odom[2]", sum(1 for c in on if "self.odom[2]" in c), 8)
    chk("开档 B腿+起飞 -> belt_entry_z",
        sum(1 for c in on if "self.belt_entry_z" in c), 4)

print("\n%s" % ("全部 PASS" if fails == 0 else "%d 项 FAIL" % fails))
sys.exit(1 if fails else 0)
