#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""杨树主枝段抽取 —— 从程序化生成器精确重放，导出枝碰撞模板。

**出处**：`/mnt/d/gazebo_forest/tree_mesh_gen.py`（生成
`meshes/poplar/tree_{0,1,2}_{wood,leaves}.stl` 的那支脚本）。该生成器完全确定性
（`random.Random(100 + i*7)`，i=0,1,2），主枝的每一段都是显式
`cyl(p0, p1, r0, r1)` 调用 —— 故本工具**逐行重放同一 RNG 调用序列**直接取出枝段
参数，而不是从 STL 三角汤反推。

为什么不解析 mesh：干与枝混在同一个 wood 网格里，靠连通域 + PCA 反推轴心既脆
又不唯一；而生成器把几何参数写死在代码里，重放是精确解。

**RNG 纪律（改本文件前必读）**：`build_tree` 里每一次 `rng.uniform/randint` 都会
推进 Mersenne Twister 状态，**顺序与次数必须逐位复刻** —— 所以下面的 `build_tree`
是生成器的逐行拷贝，只把 `cyl()` 换成录制器、`blob()` 换成 no-op（`blob` 本身不消耗
rng，其 `rng.uniform(...)` 参数由**调用点**在传参时求值，故重放顺序不受影响）。
漏掉任何一次 rng 调用，后面所有枝的参数都会整体错位。

**输出**：`src/zx2026_common/config/forest_poplar_branches.json`
    {"tree_0": {"seed":..., "H":..., "trunk_top":..., "R":..., "n_main":...,
                "branches": [[x0,y0,z0, x1,y1,z1, r], ...]}, ...}
**为什么放 config 目录而不是 meshes 目录**：Scene 用 `cfg.pkg_config_dir()`
解析表相对路径（见 `_load_tree_table`），枝模板与树表是同一批**场景数据**、
同生共死，放同侧即可**复用同一条解析惯例**；放 `meshes/`（属另一个包
arena_world_gazebo）则要为它单造一条跨包寻径规则。世界发射侧不需要这个文件
——它读的是 Scene 已经算好的 `ob.branches`。
坐标 = **模型局部系**（原点在干基、z=0 在地面），与 wood/leaves mesh 同系；
Scene 侧按 yaw 旋转 + scale 缩放 + 落地平移实例化到世界系。

**口径（用户裁决 2026-10-01）**：**只保留主枝**（二级枝不建碰撞），每条主枝把两段
锥形柱**合并成 1 段**（p_start→p_end），半径取两段长度加权均 —— 即 ~0.22·R，
R∈[0.13,0.19] → 枝半径 ~3–4cm。
"""
import json
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
OUT = os.path.join(WS, "src", "zx2026_common", "config",
                   "forest_poplar_branches.json")

# 与生成器 `build_tree(100 + i * 7)` 一致
SEEDS = [100, 107, 114]

# 生成器里主枝两段锥形柱的半径系数（× 干基半径 R）
R_NEAR, R_MID, R_TIP = 0.38, 0.20, 0.08
R_SEC_NEAR, R_SEC_TIP = 0.15, 0.06


# ---- 极简三维向量（不引 numpy：本工具只做标量几何，与生成器的差异在 1e-16 级，
#      而枝位置对 RNG 无影响——RNG 只由 uniform/randint 的**次数与顺序**决定）----
def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _len(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def build_tree(seed):
    """生成器 `build_tree()` 的逐行重放，只录制圆柱段。

    返回 (meta, trunk_segs, main_segs, sec_segs)：
      meta      -- {"H","trunk_top","R","n_main"}
      trunk_segs-- [(p0,p1,r0,r1)]  主干
      main_segs -- [(p0,p1,r0,r1)]  主枝各段（每主枝 2 段，按 K 顺序平铺）
      sec_segs  -- [(p0,p1,r0,r1)]  二级枝各段（不建碰撞，仅供统计）
    """
    rng = random.Random(seed)
    trunk_segs, main_segs, sec_segs = [], [], []

    def cyl(p0, p1, r0, r1, seg, sink):
        """生成器 cyl() 的录制替身：不产三角面，只登记线段+半径。"""
        sink.append((tuple(float(v) for v in p0), tuple(float(v) for v in p1),
                     float(r0), float(r1)))

    def blob(c, r, sink):
        """生成器 blob() 的 no-op 替身（叶团不进碰撞；体本身不消耗 rng）。"""
        pass

    # ---- 以下与 /mnt/d/gazebo_forest/tree_mesh_gen.py:build_tree 逐行对应 ----
    H = rng.uniform(8.5, 11.0)               # 全树高
    trunk_top = H * rng.uniform(0.72, 0.82)  # 干高
    R = rng.uniform(0.13, 0.19)
    cyl((0, 0, 0), (0, 0, trunk_top), R, R * 0.45, 10, trunk_segs)

    n_main = rng.randint(7, 10)
    for k in range(n_main):
        h0 = H * rng.uniform(0.42, 0.86)
        az = 2 * math.pi * k / n_main + rng.uniform(-0.35, 0.35)
        el = math.radians(rng.uniform(48, 68))          # 上举角
        Lb = rng.uniform(1.7, 3.1) * (1.0 - 0.55 * h0 / H)
        d = (math.cos(el) * math.cos(az),
             math.cos(el) * math.sin(az),
             math.sin(el))
        p_start = (0.0, 0.0, h0)
        p_mid = _add(_add(p_start, _mul(d, Lb * 0.55)), (0.0, 0.0, 0.15))
        p_end = _add(_add(p_mid, _mul(d, Lb * 0.45)), (0.0, 0.0, 0.25))
        cyl(p_start, p_mid, R * R_NEAR, R * R_MID, 6, main_segs)
        cyl(p_mid, p_end, R * R_MID, R * R_TIP, 6, main_segs)
        # 二级枝（不建碰撞，但**必须照跑 rng**）
        for _ in range(rng.randint(1, 2)):
            t = rng.uniform(0.4, 0.85)
            base = _add(p_start, _mul(_sub(p_end, p_start), t))
            az2 = az + rng.uniform(-0.9, 0.9)
            el2 = math.radians(rng.uniform(40, 65))
            L2 = Lb * rng.uniform(0.35, 0.6)
            d2 = (math.cos(el2) * math.cos(az2),
                  math.cos(el2) * math.sin(az2),
                  math.sin(el2))
            e2 = _add(base, _mul(d2, L2))
            cyl(base, e2, R * R_SEC_NEAR, R * R_SEC_TIP, 5, sec_segs)
            blob(_add(e2, _mul(d2, 0.15)), rng.uniform(0.35, 0.6), None)
        # 沿枝外段叶团（同样只为推进 rng）
        for _ in range(rng.randint(3, 5)):
            t = rng.uniform(0.45, 1.0)
            c = _add(_add(p_start, _mul(_sub(p_end, p_start), t)),
                     (rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2),
                      rng.uniform(0.0, 0.3)))
            blob(c, rng.uniform(0.4, 0.75), None)
    # 顶部冠团（只为推进 rng）
    for _ in range(rng.randint(7, 11)):
        c = (rng.uniform(-0.9, 0.9), rng.uniform(-0.9, 0.9),
             H + rng.uniform(-0.6, 0.9))
        blob(c, rng.uniform(0.45, 0.85), None)

    meta = {"H": H, "trunk_top": trunk_top, "R": R, "n_main": n_main}
    return meta, trunk_segs, main_segs, sec_segs


def merge_main(main_segs, R):
    """把每条主枝的两段锥形柱合并成 1 段（p_start→p_end），文件头口径。

    合并半径 = 两段**长度加权**的平均半径（每段自身半径取其两端均值）：
        r1m = (0.38R + 0.20R)/2 = 0.29R
        r2m = (0.20R + 0.08R)/2 = 0.14R
        r   = (L1·r1m + L2·r2m) / (L1 + L2)
    """
    assert len(main_segs) % 2 == 0, "主枝段数应为偶数（每主枝 2 段）"
    out = []
    for i in range(0, len(main_segs), 2):
        a0, a1, _ra0, _ra1 = main_segs[i]
        b0, b1, _rb0, _rb1 = main_segs[i + 1]
        assert a1 == b0, "主枝两段应首尾相接"
        L1, L2 = _len(_sub(a1, a0)), _len(_sub(b1, b0))
        tot = L1 + L2
        if tot < 1e-9:
            continue
        r = (L1 * (R_NEAR + R_MID) / 2.0 + L2 * (R_MID + R_TIP) / 2.0) / tot * R
        out.append((a0, b1, r))
    return out


def main():
    doc = {}
    for i, seed in enumerate(SEEDS):
        meta, trunk, main_segs, sec = build_tree(seed)
        brs = merge_main(main_segs, meta["R"])
        name = "tree_%d" % i

        # ---- 断言（口径护栏，改生成器口径时这里先炸） ----
        assert 7 <= len(brs) <= 10, (name, "主枝数越界", len(brs))
        assert len(brs) == meta["n_main"], (name, "合并后条数应等于 n_main")
        for (p0, p1, r) in brs:
            assert abs(p0[0]) < 1e-9 and abs(p0[1]) < 1e-9, (name, "枝基应在干轴上")
            assert 0.40 * meta["H"] < p0[2] < 0.95 * meta["H"], (name, "枝基高越界", p0[2])
            assert p1[2] > p0[2], (name, "主枝应上举")
            assert 0.02 <= r <= 0.06, (name, "合并枝半径越界", r)

        doc[name] = {
            "seed": seed,
            "H": round(meta["H"], 6),
            "trunk_top": round(meta["trunk_top"], 6),
            "R": round(meta["R"], 6),
            "n_main": meta["n_main"],
            "branches": [[round(v, 6) for v in (p0 + p1)] + [round(r, 6)]
                         for (p0, p1, r) in brs],
        }
        zs = [b[2] for b in doc[name]["branches"]]
        print("  %s seed=%d  H=%.2f 干顶=%.2f R=%.3f 主枝=%d  "
              "枝基 z∈[%.2f, %.2f]  二级枝=%d 段(未建碰撞)"
              % (name, seed, meta["H"], meta["trunk_top"], meta["R"], len(brs),
                 min(zs), max(zs), len(sec)))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as fp:
        json.dump(doc, fp, indent=1, sort_keys=True)
    print("写出:", OUT)
    print("总计主枝段: %d（全林 203 棵 ≈ %d 个碰撞体）"
          % (sum(len(v["branches"]) for v in doc.values()),
             round(sum(len(v["branches"]) for v in doc.values()) / 3.0 * 203)))


if __name__ == "__main__":
    main()
