#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_geo_selftest.py — deadend_geo 门守卫（2026-10-03 立案，可失败自检）。

两层：
  A) 从**真源码** ast 抽取 _deadend_step_logic/_sup_bad/_sup_dwell 编译执行：
     geo 语义四断言（低分也停 / 关档逐字节 / 自清后迟滞退出照旧 / 超时兜底）。
  B) 门计算镜像回放两局 probe dmin：触发边沿与 18 起撞案抓到数 ≈ 离线 H2
     （1.17 边沿/分/机、抓 15/18），镜像窗与节点码同参（0.20~0.60s 回看）。

失败即非零退出。用法: python3 tools/defop_geo_selftest.py [--skip-b]
"""
import ast
import json
import re
import sys

NODE = ("/home/ubuntu/zx2026_defop_ws/src/defop_nav/scripts/"
        "defop_cpn_node.py")
RUNS = [
    ("225237", "/home/ubuntu/zx2026_defop_ws/run_logs/defop_ab_20261002_225237/00_defop_seed43_r0"),
    ("001331", "/home/ubuntu/zx2026_defop_ws/run_logs/defop_ab_20261003_001331/00_defop_seed43_r0"),
]
COLL = re.compile(r'^([0-9.]+).*drone (\d+) COLLISION #\d+')

fails = []


def chk(name, cond, detail=""):
    print("%s %-46s %s" % ("[PASS]" if cond else "[FAIL]", name, detail))
    if not cond:
        fails.append(name)


def extract_fn(src, name):
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            return compile(mod, "<selftest>", "exec")
    raise SystemExit("函数缺失: " + name)


def part_a():
    src = open(NODE).read()
    ns = {}
    for name in ("_sup_bad", "_sup_dwell", "_deadend_step_logic"):
        exec(extract_fn(src, name), ns)
    dsl = ns["_deadend_step_logic"]
    st = lambda: {"t0": None, "ok_t0": None}
    HI, LO, MAXS, HYST = 6.5, 5.0, 4.0, 1.0

    # A1 关档逐字节：低分 + geo_bad=False → 正常跟飞
    s, a, f = dsl(0, 1.0, 5, 100.0, st(), HI, LO, MAXS, HYST, geo_bad=False)
    chk("A1 关档低分=success", (s, a, f) == (0, "success", False),
        "%s/%s/%s" % (s, a, f))
    # A2 开档新行为：**低分也停**（OOD 分低位正是病根）
    s, a, f = dsl(0, 1.0, 5, 100.0, st(), HI, LO, MAXS, HYST, geo_bad=True)
    chk("A2 geo_bad=True 低分也 stop", (s, a, f) == (1, "stop", False),
        "%s/%s/%s" % (s, a, f))
    # A3 咬合期不退出：step1 + geo 持续 + lo 本应真 → 仍 rotate（lo 被同置假）
    s3 = st()
    s3["t0"] = 100.0
    s, a, f = dsl(1, 1.0, 5, 100.5, s3, HI, LO, MAXS, HYST, geo_bad=True)
    chk("A3 咬合期 lo_ok 被压住=rotate", (s, a) == (1, "rotate"),
        "%s/%s" % (s, a))
    # A4 geo 自清后走既有迟滞退出（geo 不闩死）：首个 lo_ok 拍迟滞起算，
    # 连续满 1.0s 后退——两次调用推进钟。
    s4 = st()
    s4["t0"] = 100.0
    s, a, f = dsl(1, 1.0, 5, 101.5, s4, HI, LO, MAXS, HYST, geo_bad=False)
    ok1 = (s, a) == (1, "rotate")
    s, a, f = dsl(1, 1.0, 5, 102.6, s4, HI, LO, MAXS, HYST, geo_bad=False)
    ok2 = (s, a) == (0, "success")
    chk("A4 geo 自清后 hyst 退出=success", ok1 and ok2,
        "首拍 rotate=%s, 满窗退出=%s" % (ok1, ok2))
    # A5 超时兜底不变：step1 + geo 持续 + 超 max_s → 强制放宽
    s5 = st()
    s5["t0"] = 100.0
    s, a, f = dsl(1, 1.0, 5, 105.0, s5, HI, LO, MAXS, HYST, geo_bad=True)
    chk("A5 max_s 兜底=forced", (s, a, f) == (2, "forced", True),
        "%s/%s/%s" % (s, a, f))
    # A6 veto 通道与 geo 并存不互扰（都关=旧行为）
    s, a, f = dsl(0, 1.0, 5, 100.0, st(), HI, LO, MAXS, HYST,
                  veto_frac=0.9, veto_hi=0.0)
    chk("A6 veto 关+geo 关=旧", (s, a) == (0, "success"), "%s/%s" % (s, a))


def mirror_gate(rows):
    """节点 _deadend_action 门计算镜像（位置回看恰 2 拍，与节点码同构）。"""
    hist = []
    run = 0
    out = []
    for r in rows:
        now, dmin = r[0], r[8]
        hist.append((now, dmin))
        if len(hist) > 8:
            del hist[0]
        clo = None
        if dmin is not None and len(hist) >= 3:
            t2, d2 = hist[-3]
            dt = now - t2
            if dt > 0.05 and d2 is not None:
                clo = (dmin - d2) / dt
        on = (dmin is not None and dmin < 1.2
              and clo is not None and clo <= -0.3)
        run = run + 1 if on else 0
        out.append(run >= 2)
    return out


def part_b():
    tot_on = tot = edges = 0
    mins = 0.0
    caught = miss = 0
    leads = []
    for tag, rd in RUNS:
        tl = json.load(open(rd + "/probe.json"))["defop_debug"]
        colls = []
        for ln in open(rd + "/collisions.txt", errors="ignore"):
            m = COLL.search(ln)
            if m:
                colls.append((float(m.group(1)), int(m.group(2))))
        for d in range(6):
            rows = (tl.get(str(d)) or {}).get("tl", [])
            if len(rows) < 3:
                continue
            g = mirror_gate(rows)
            tot_on += sum(g)
            tot += len(g)
            edges += sum(1 for i in range(1, len(g))
                         if g[i] and not g[i - 1] and rows[i][1] == 0)
            mins += (rows[-1][0] - rows[0][0]) / 60.0
        for t, d in colls:
            rows = [r for r in (tl.get(str(d)) or {}).get("tl", [])
                    if t - 3.0 <= r[0] <= t + 0.2]
            if len(rows) < 3:
                continue
            g = mirror_gate(rows)
            lead = None
            i = 0
            while i < len(g):
                if g[i]:
                    j = i
                    while j + 1 < len(g) and g[j + 1]:
                        j += 1
                    if j >= i + 1:
                        lead = t - rows[i][0]
                        break
                    i = j + 1
                else:
                    i += 1
            if lead is None:
                miss += 1
            else:
                caught += 1
                leads.append(lead)
    onpct = 100.0 * tot_on / max(1, tot)
    epm = edges / max(0.1, mins) / 6.0
    leads.sort()
    med = leads[len(leads) // 2] if leads else float("nan")
    print("  镜像回放: ON=%.1f%% 边沿/分/机=%.2f 抓到=%d/18 中位lead=%.2fs"
          % (onpct, epm, caught, med))
    chk("B1 抓到≥13 (离线H2=15)", caught >= 13, "got %d" % caught)
    chk("B2 边沿/分/机∈[0.5,2.0] (H2=1.17)", 0.5 <= epm <= 2.0,
        "got %.2f" % epm)
    chk("B3 中位lead≥1.0s (H2=1.53)", med >= 1.0, "got %.2f" % med)


print("=== A. 状态机 geo 语义（真源码抽取） ===")
part_a()
if "--skip-b" not in sys.argv:
    print("=== B. 门计算镜像回放（两局 probe） ===")
    part_b()
print()
if fails:
    print("FAIL x%d: %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("全过。")
