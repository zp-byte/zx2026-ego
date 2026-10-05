#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_reactwin.py 的可失败守卫（合成样本，零归档依赖）。

判据仪一旦判错，批 3/批 4 的归因就全歪，故把分类边界钉成断言。

⚠ 本案重要更正（2026-10-02，读码 + 副本实测）：`min_score` 是**碰撞分，
越低越安全**（`hi_ok = min_score < deadend_high`）。故"撞前 min_score 低"
= 监督层判它**安全**却仍撞 = ③ 未拦住。原 §7 规则写成"远低于 low"像是在
描述危险，方向易读反；本器把语义写死为"监督层当时判定安全"。

断言（任一破即 exit 1）：
  [A] dt 边界：0.3 => ①；0.8（含）不再算 ①
  [B] 副本占用优先：rot=True => ②；rot=False => ③（不论 ms 高低）
  [C] 无副本（off 档）退回 ms 规则：<5 且新鲜 => ③；<5 但滞后 46s => 证据不足；
      >=5 => ②
  [D] 尾部窗：最后一个采样离碰撞太远 / 窗内无连续采样 => None（不可判）
  [E] 端到端：合成 rosout 复刻批 1 bis cell00 的五起碰撞
      （d3 dt=0.450、d2 dt=0.626 => ①；d4 dt=3.504 且撞前最后 1 s
        t_rot_cur 未涨 => ③；d1 dt=8.604 / d5 dt=115.1 => ③）

用法：python3 tools/defop_reactwin_selftest.py
"""
import os
import sys

FAILS = 0
HERE = os.path.dirname(os.path.abspath(__file__))


def chk(tag, got, want):
    global FAILS
    ok = got == want
    if not ok:
        FAILS += 1
    print("  [%s] %s   got=%r want=%r" % ("OK" if ok else "FAIL", tag, got, want))


def _load():
    p = os.path.join(HERE, "defop_reactwin.py")
    src = open(p, encoding="utf-8").read().replace("\nsys.exit(main())", "\n")
    ns = {"__name__": "rw", "__file__": p}
    exec(compile(src, p, "exec"), ns)
    return ns


def _sh(t, v, d=4):
    return ("%f INFO /drone_%d/defop_cpn [x] [defop_cpn] SHADOW bearing cyc=%d "
            "cur_dead=13 cur_ep=2 | t_rot_cur=%.1f t_rot_beta=7.0 dt=0.0"
            % (t, d, int(t) % 900 + 1, v))


# 合成 rosout：真机日志行的形状（时间戳 + 节点 + [x] + 正文）
SYNTH = "\n".join([
    # d3: 死端入口 68.452 -> 碰撞 68.902 => dt=0.450 => ①（①压过占用）
    "68.452000000 INFO /drone_3/defop_cpn [x] [defop_cpn] DEADEND enter (min_score=3.1, safe=9, dir=1)",
    _sh(68.500, 3.0, d=3), _sh(69.200, 3.4, d=3),
    "68.902000000 INFO /drone_3/defop_cpn [x] drone 3 COLLISION #1 kind=trunk at (1.0,2.0,1.2) dir=(0,0,0)",
    # d2: 164.602 -> 165.228 => dt=0.626 => ①
    "164.602000000 INFO /drone_2/defop_cpn [x] [defop_cpn] DEADEND enter (min_score=3.4, safe=4, dir=-1)",
    "165.228000000 INFO /drone_2/defop_cpn [x] drone 2 COLLISION #1 kind=trunk at (3.0,4.0,1.2) dir=(0,0,0)",
    # d4: 撞前最后 1 s 副本 t_rot_cur 冻结（7.1 -> 7.1）=> 未在转 => ③
    "138.294000000 INFO /drone_4/defop_cpn [x] [defop_cpn] DEADEND enter (min_score=6.77, safe=0, dir=1)",
    _sh(138.100, 6.8), _sh(139.196, 7.1), _sh(140.392, 7.1), _sh(141.434, 7.1),
    "140.442000000 INFO /drone_4/defop_cpn [x] [defop_cpn] drone_4 veto=216 safe=40 min_score=4.70",
    "141.798000000 INFO /drone_4/defop_cpn [x] drone 4 COLLISION #1 kind=trunk at (5.0,6.0,1.2) dir=(0,0,0)",
    # d1: 从未进死端；撞前 3.93s ms=1.99（监督层判"安全"）=> ③
    "142.830000000 INFO /drone_1/defop_cpn [x] [defop_cpn] drone_1 veto=96 safe=160 min_score=1.99",
    "146.760000000 INFO /drone_1/defop_cpn [x] drone 1 COLLISION #1 kind=trunk at (7.0,8.0,1.2) dir=(0,0,0)",
    # d5: 从未进死端；撞前 1.37s ms=1.36 => ③
    "257.576000000 INFO /drone_5/defop_cpn [x] [defop_cpn] drone_5 veto=144 safe=112 min_score=1.36",
    "258.942000000 INFO /drone_5/defop_cpn [x] drone 5 COLLISION #1 kind=trunk at (9.0,10.0,1.2) dir=(0,0,0)",
    # 影子行必须**不被**当真机：它不构成 DEADEND enter
    "142.100000000 INFO /drone_1/defop_cpn [x] [defop_cpn] SHADOW bearing SUMMARY cyc=9 cur_dead=1",
])


def main():
    m = _load()
    classify, in_rot, analyze = m["classify"], m["in_rot"], m["analyze"]

    print("[A] 分类边界（dt）")
    chk("dt=0.3 => ①", classify(0.3, None, None), m["CLS_LATE"])
    chk("dt=0.0 => ①", classify(0.0, None, None), m["CLS_LATE"])
    chk("dt=0.8 不算①", classify(m["LATE_S"], None, None) == m["CLS_LATE"], False)

    print("[B] 副本占用优先于 ms")
    chk("rot=True => ②", classify(3.0, 1.3, 0.5, True), m["CLS_INDE"])
    chk("rot=False + ms 低 => ③", classify(3.0, 1.3, 0.5, False), m["CLS_SUP"])
    chk("rot=False + ms 高 => 仍③", classify(3.0, 6.7, 0.5, False), m["CLS_SUP"])

    print("[C] 无副本（off 档）退回 ms 规则")
    chk("ms=1.3 新鲜 => ③", classify(None, 1.3, 1.0, None), m["CLS_SUP"])
    chk("ms=1.3 lag=46s => 证据不足", classify(None, 1.3, 46.0, None), m["CLS_GAP"])
    chk("ms=1.99 lag=3.93 => ③", classify(3.0, 1.99, 3.93, None), m["CLS_SUP"])
    chk("ms=6.0 => ②", classify(3.0, 6.0, 0.5, None), m["CLS_INDE"])
    chk("dt/ms 全缺 => ②", classify(None, None, None, None), m["CLS_INDE"])

    print("[D] 尾部窗不可判")
    chk("无影子行 => None", in_rot([], 10.0), None)
    chk("只有一条 => None", in_rot([(9.0, 1.0)], 10.0), None)
    chk("窗内采样间隔 4s => None", in_rot([(5.0, 1.0), (9.0, 2.0)], 10.0), None)
    chk("末次采样离碰撞 4s => None", in_rot([(4.0, 1.0), (6.0, 2.0)], 10.0), None)
    chk("窗内涨 => True", in_rot([(9.0, 1.0), (9.8, 1.4)], 10.0), True)
    chk("窗内平 => False", in_rot([(9.0, 1.0), (9.8, 1.0)], 10.0), False)

    print("[E] 端到端（合成 rosout 复刻批 1 bis cell00 五起）")
    import tempfile
    d = tempfile.mkdtemp(prefix="rwselftest_")
    with open(os.path.join(d, "rosout.log"), "w") as f:
        f.write(SYNTH + "\n")
    rows, n_en, n_fr, n_sh = analyze(d)
    chk("碰撞条数", len(rows), 5)
    chk("真机死端入口数", n_en, 3)          # d3/d2/d4；SUMMARY 行不得计入
    chk("影子线条数", n_sh, 6)              # d3 两条 + d4 四条（SUMMARY 行须排除）
    by = {r["d"]: r for r in rows}
    chk("d3 dt", round(by[3]["dt"], 3), 0.450)
    chk("d2 dt", round(by[2]["dt"], 3), 0.626)
    chk("d4 dt", round(by[4]["dt"], 3), 3.504)
    chk("d4 占用", by[4]["rot"], False)
    chk("d4 归类", by[4]["cls"], m["CLS_SUP"])
    chk("d1 ms/lag", (by[1]["ms"], round(by[1]["lag"], 2)), (1.99, 3.93))
    chk("d5 ms/lag", (by[5]["ms"], round(by[5]["lag"], 2)), (1.36, 1.37))
    tally = {}
    for r in rows:
        tally[r["cls"]] = tally.get(r["cls"], 0) + 1
    chk("① 计数", tally.get(m["CLS_LATE"], 0), 2)
    chk("② 计数", tally.get(m["CLS_INDE"], 0), 0)
    chk("③ 计数", tally.get(m["CLS_SUP"], 0), 3)

    print("[F] Tier 2 逐拍线压过 rosout")
    ft = m["from_ticks"]
    # 行 = [sim_t, cod, col_prob, veto, safe, forced]
    rows = [[119.0, 0, 0.9, 5, 20, 0], [119.8, 0, 0.9, 5, 20, 0]]
    chk("无上升沿 => dt None", ft(rows, 120.0)[0], None)
    chk("ms 取最近非 None", (ft(rows, 120.0)[1], round(ft(rows, 120.0)[2], 3)),
        (0.9, 0.2))
    chk("末拍在死端外 => rot False", ft(rows, 120.0)[3], False)
    rows2 = [[138.0, 0, 6.0, 9, 9, 0], [139.0, 1, 6.7, 9, 0, 0],
             [139.9, 2, 6.7, 9, 0, 0]]
    chk("上升沿 => dt", round(ft(rows2, 140.0)[0], 2), 1.0)
    chk("末拍在死端内 => rot True", ft(rows2, 140.0)[3], True)
    chk("末拍太旧 => rot 不可判", ft([[1.0, 2, 1.0, 0, 0, 0]], 10.0)[3], None)
    chk("col_prob 全 None => ms None", ft([[9.0, 0, None, 0, 0, 0]], 10.0)[1], None)

    # 端到端：同一格，rosout 说 ②（旧口径），逐拍线说 ③ —— 必须采信逐拍线
    d2 = tempfile.mkdtemp(prefix="rwselftest2_")
    with open(os.path.join(d2, "rosout.log"), "w") as f:
        f.write("120.000000000 INFO /drone_4/defop_cpn [x] drone 4 COLLISION #1 "
                "kind=trunk at (1.0,2.0,1.2) dir=(0,0,0)\n"
                "90.000000000 INFO /drone_4/defop_cpn [x] [defop_cpn] drone_4 "
                "veto=1 safe=1 min_score=8.00\n")
    with open(os.path.join(d2, "probe.json"), "w") as f:
        import json as _j
        _j.dump({"defop_debug": {"4": {"tl": [[119.0, 0, 0.9, 5, 20, 0],
                                             [119.8, 0, 0.9, 5, 20, 0]]}}}, f)
    r2 = analyze(d2)[0][0]
    chk("源为 tick", r2["src"], "tick")
    chk("ms 用逐拍(0.9) 非 rosout(8.0)", r2["ms"], 0.9)
    chk("归类 ③（旧口径会判②）", r2["cls"], m["CLS_SUP"])

    print("\n%s" % ("ALL PASS —— 判据仪与手工锚点一致"
                    if not FAILS else "%d FAIL" % FAILS))
    return 1 if FAILS else 0


sys.exit(main())
