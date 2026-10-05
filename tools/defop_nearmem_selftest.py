#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_nearmem_selftest.py — 近场记忆影守卫（docs/near_field_memory_case.md）。

A 部分（源码钉）：键缺省 0 / 环初始化 / 四处注入 / off 档本尊直通 /
影子 replica 不吃影（byte-old 对照组）。
B 部分（行为钉，真 defop_common 监督器 + 影函数 ast 抽取）：
  B1 罪案重现：合成终端逼近帧（干带 1.2→0.22m → 填充抹平 3.5m）——
     影关（K=0）时抹平帧 safe 必 256（罪证钉：监督器读全清）；
  B2 修证：影开（K=6）时抹平帧 safe 必仍被否决（<256，且否决数与 0.22m
     帧同量级）；
  B3 老化：连续 K 拍无近事件后影自然过期（safe 回 256）= 旋转逃逸语义；
  B4 单元：近事件进环/远事件不进环/当前更近者赢/无效语义 0.0。

用法：python3 tools/defop_nearmem_selftest.py   （零 ROS 依赖，纯离线）
"""
import ast
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src", "defop_nav", "scripts")
NODE = os.path.join(SRC, "defop_cpn_node.py")

PASS = []


def ok(name, cond, detail=""):
    PASS.append(bool(cond))
    print("  [%s] %s%s" % ("PASS" if cond else "FAIL", name,
                           (" — " + detail) if detail else ""))


def load_src():
    with open(NODE, encoding="utf-8") as f:
        return f.read()


def ast_extract(src, names):
    """抽出模块级函数源码并 exec（真机同款实现，geo selftest 同款纪律）。"""
    tree = ast.parse(src)
    out = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            out[node.name] = ast.get_source_segment(src, node)
    missing = set(names) - set(out)
    assert not missing, "ast 抽取缺函数: %s" % missing
    ns = {"np": np}
    for code in out.values():
        exec(compile(code, NODE, "exec"), ns)
    return ns


def real_supervisor():
    """真 defop_common（纯 numpy）+ 真基元库。"""
    sys.path.insert(0, SRC)
    import defop_common as dc
    lib = dc.build_action_library(1, 8, 32, 15, 1.0, 0.0, 87.0, 50.0)
    return dc, lib


def make_frame(trunk_d, w=480, h=270, band=(200, 280)):
    """背景 4.0m + 中央干带 trunk_d（trunk_d=None 即纯背景）。"""
    f = np.full((h, w), 4.0, dtype=np.float32)
    if trunk_d is not None:
        f[:, band[0]:band[1]] = float(trunk_d)
    return f


def sup_safe(dc, lib, frame):
    """真监督器 safe 计数（与节点同参）。"""
    _, _, safe_mask, _, _, _ = dc.filter_actions_by_grid_3(
        frame, np.deg2rad(87.0), np.deg2rad(50.0),
        lib[0][:, 0, 2], lib[0][:, 0, 1], lib[1], lib[2],
        num_yaw=32, num_vel_z=8, min_dist=1.0, min_ratio=0.05,
        drone_radius=0.35, margin_fn=dc.compute_margin_cone)
    return int(np.asarray(safe_mask).ravel().sum())


def main():
    print("== A 源码钉 ==")
    src = load_src()
    ok("A1 default_cfg 键缺省 0", "'near_mem_frames': 0," in src)
    ok("A2 环/影状态初始化",
       "self._near_ring = []" in src and "self._sup_depth = None" in src)
    # plan 段四注入（按出现顺序）
    i_sup = src.find("safe_mask, blocked_with_margin, _, _ = dc.filter_actions_by_grid_3(")
    ok("A3a 监督器吃影", i_sup > 0 and
       "sup_depth," in src[i_sup:i_sup + 120])
    ok("A3b dbg 吃影", "_dd = sup_depth" in src)
    i_rot = src.find("rot_dir = dc.choose_dead_end_rotation(")
    ok("A3c 死端旋转方向吃影", i_rot > 0 and
       "sup_depth," in src[i_rot:i_rot + 120])
    ok("A3d gov 与监督器同源",
       "d = self._sup_depth if self._sup_depth is not None else self.depth" in src)
    ok("A4 off 档本尊直通",
       "else:\n            sup_depth = self.depth\n        self._sup_depth = sup_depth" in src)
    # 影子 replica 的 filter 调用必须仍是 self.depth（对照组）
    i_rep = src.find("safe_mask_b, _, _, _ = dc.filter_actions_by_grid_3(")
    ok("A5 影子 replica 不吃影（byte-old 对照）",
       i_rep > 0 and "self.depth," in src[i_rep:i_rep + 120])

    print("== B 行为钉（真监督器） ==")
    dc, lib = real_supervisor()
    ns = ast_extract(src, {"_near_field_shadow", "_near_event_frame"})
    shadow_fn, event_fn = ns["_near_field_shadow"], ns["_near_event_frame"]

    # 终端逼近帧序：真监督器视角的干带（1.2m 起进否决带）
    seq = [1.2, 0.8, 0.5, 0.3, 0.22]
    frames = [make_frame(d) for d in seq]
    smear = make_frame(3.5)          # 填充抹平后的「假背景」帧
    safe_seq = [sup_safe(dc, lib, f) for f in frames]
    safe_smear = sup_safe(dc, lib, smear)
    ok("B0 罪案前提：0.22m 帧确被否决 / 抹平帧确读全清",
       safe_seq[-1] < 128 and safe_smear == 256,
       "safe(0.22m)=%d safe(smear)=%d" % (safe_seq[-1], safe_smear))

    # B1 影关（K=0）：抹平帧 safe=256 = 罪证钉
    ok("B1 影关=罪案重现（safe 256）",
       sup_safe(dc, lib, smear) == 256)

    # B2 影开（K=6）：近事件环 + 抹平帧 ⇒ 否决顶住
    r_near = 1.0
    ring = [event_fn(f, r_near) for f in frames]
    shadowed = shadow_fn(ring, smear, r_near)
    safe_fix = sup_safe(dc, lib, shadowed)
    ok("B2 影开=抹平帧否决顶住", safe_fix <= safe_seq[-1] + 32,
       "safe(shadow)=%d vs safe(0.22m)=%d" % (safe_fix, safe_seq[-1]))
    # 影的像素值：抹平帧干带被记忆顶回 0.22
    ok("B2b 影像素值=最后近事件",
       abs(float(shadowed[135, 240]) - 0.22) < 1e-4,
       "shadow[135,240]=%.3f" % float(shadowed[135, 240]))

    # B3 老化：K 拍纯背景近事件（远帧不进环）后影过期
    ring2 = [event_fn(f, r_near) for f in frames]
    cur = smear
    for _ in range(6):               # 6 拍无近事件 ⇒ 5 帧旧环全被挤出
        ring2.append(event_fn(make_frame(None), r_near))
        del ring2[0]
    shadow_aged = shadow_fn(ring2, cur, r_near)
    ok("B3 老化后影过期（旋转逃逸语义）",
       sup_safe(dc, lib, shadow_aged) == 256)

    # B4 单元语义（0.5/0.25 = float32 精确可表示，避免相等断言假失败）
    f_near = make_frame(0.5)
    ev = event_fn(f_near, r_near)
    ok("B4a 近事件进环", abs(float(ev[135, 240]) - 0.5) < 1e-6 and
       float(ev[0, 0]) == np.inf)
    ev_far = event_fn(make_frame(2.0), r_near)
    ok("B4b 远事件不进环", float(ev_far[135, 240]) == np.inf)
    sh2 = shadow_fn([ev], make_frame(0.25), r_near)  # 当前更近者赢
    ok("B4c 当前更近者赢", abs(float(sh2[135, 240]) - 0.25) < 1e-6)
    sh3 = shadow_fn([], np.zeros((270, 480), np.float32), r_near)
    ok("B4d 无效帧影不发明正值", float((sh3 > 0).sum()) == 0)

    n_pass = sum(PASS)
    print("== %d/%d PASS ==" % (n_pass, len(PASS)))
    return 0 if n_pass == len(PASS) else 1


if __name__ == "__main__":
    sys.exit(main())
