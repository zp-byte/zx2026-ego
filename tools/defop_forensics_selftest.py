#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_collision_forensics.py 的正则可失败守卫（合成样本，零归档依赖）。

背景：日志行自 `kind=` 字段落地后变成
    `drone 1 COLLISION #1 kind=trunk at (x,y,z) dir=(dx,dy,dz)`
旧 COLL 正则不含该段 ⇒ **静默解析 0 条**：110544 批真值 5、旧版报 0，而报告
仍然打印"碰撞解剖: 共 0"——不是报错，是**假阴性**。历次归因都靠这台仪器，
故把"格式脱节必须响"钉成守卫，而不是靠人记得看。

断言（任一破即 exit 1）：
  [A] 新格式（带 kind）三类字段解析正确；
  [B] 旧格式（无 kind）向后兼容，kind 记 '?'；
  [C] 无碰撞文本 => 0 条（**不得**误触发断言）；
  [D] 格式脱节（`COLLISION #` 在、行不可解析）=> 必须 SystemExit；
  [E] 部分脱节（1 条可解析 + 1 条不可）=> 必须 SystemExit（不是只看总数）。

用法：python3 tools/defop_forensics_selftest.py
"""
import os
import sys

FAILS = 0


def chk(tag, got, want):
    global FAILS
    ok = got == want
    if not ok:
        FAILS += 1
    print("  [%s] %s   got=%r want=%r"
          % ("OK" if ok else "FAIL", tag, got, want))


def _load():
    """载入被测模块（其末尾 sys.exit(main()) 须先摘掉，否则 import 即退）。"""
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "defop_collision_forensics.py")
    src = open(p, encoding="utf-8").read().replace("\nsys.exit(main())", "\n")
    ns = {"__name__": "fx", "__file__": p}
    exec(compile(src, p, "exec"), ns)
    return ns


NEW = ("76.47 INFO /drone_1/defop_cpn [x] drone 1 COLLISION #1 kind=trunk "
       "at (10.79,-1.16,1.16) dir=(-0.44,-0.90,0.00)\n")
OLD = ("111.32 INFO /drone_0/defop_cpn [x] drone 0 COLLISION #1 "
       "at (29.11,-7.53,0.89) dir=(-0.10,-0.99,0.00)\n")
MANGLE = ("76.47 INFO /drone_1/defop_cpn [x] drone 1 COLLISION #1 kind=trunk "
          "at 10.79,-1.16,1.16 dir=(-0.44,-0.90,0.00)\n")   # 丢了 '(x,y,z)'


def _raises(fn):
    try:
        fn()
    except SystemExit:
        return "SystemExit"
    except Exception as e:                                  # noqa: BLE001
        return type(e).__name__
    return "未抛"


def main():
    m = _load()
    co = m["collisions_of"]

    print("[A] 新格式（带 kind）")
    c = co(NEW)[0]
    chk("kind", c["kind"], "trunk")
    chk("t/x/n", (c["t"], c["xyz"][0], c["n"]), (76.47, 10.79, 1))

    print("[B] 旧格式（无 kind）向后兼容")
    chk("kind", co(OLD)[0]["kind"], "?")

    print("[C] 无碰撞文本 => 0 条，且不得误触发断言")
    chk("条数", len(co("nothing here\n")), 0)

    print("[D] 格式脱节 => 必须响")
    chk("SystemExit", _raises(lambda: co(MANGLE)), "SystemExit")

    print("[E] 部分脱节 => 必须响（不是只看总数）")
    chk("SystemExit", _raises(lambda: co(NEW + MANGLE)), "SystemExit")

    print("\n%s" % ("ALL PASS —— 取证仪不会静默少报"
                    if not FAILS else "%d FAIL" % FAILS))
    return 1 if FAILS else 0


sys.exit(main())
