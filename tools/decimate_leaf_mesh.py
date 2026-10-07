#!/usr/bin/env python3
"""顶点聚类抽稀：把杨树叶 mesh 降到目标面数（2026-10-07）。

**为什么需要它**：换场（ego_ws → zx2026_ws 场景）恢复了 `v_leaf` 叶视觉，
203 棵 × 约 4100 面 ≈ 83 万面进了 Gazebo 的软件渲染管线，把 ego 臂**独有的**
D435 深度相机从 2.28 Hz 打到 0.595 Hz —— 而 `grid_map/odom_depth_timeout = 1.0 s`
⇒ 帧间隔 1.68 s ≫ 1.0 s ⇒ 六机永久 EMERGENCY_STOP。受控对照（同一台机、同一套
launch、单变量=有无叶）：有叶 0.595 Hz vs 无叶 2.277 Hz。

用户裁决：**不删叶，改抽稀**。本工具是那条路的第一步。

**算法 = 顶点聚类（vertex clustering）**，不引入第三方依赖（本机只有 numpy）：
  1. 读二进制 STL → 三角面顶点；
  2. 把空间按边长 `cell` 的立方格切分，每个格内顶点取均值当代表点；
  3. 三角面按代表点重映射，丢掉退化面（任意两索引相同）；
  4. 按几何重算法线，写回二进制 STL。
用二分搜索 `cell` 命中目标面数。**确定性**：无随机数，同样输入同输出。

**口径**：这是**视觉件**。`Scene`（depth_sim / camera_render 链）里没有叶的表示，
叶只对 gzclient 与 Gazebo 真实相机传感器可见 ⇒ 抽稀**不改变任何碰撞/判据几何**。
被改的资产是 ego 侧副本（`src/arena_world_gazebo/meshes/poplar/`），
**arena 侧同名文件是另一份，绝不碰**。

用法：
  python3 tools/decimate_leaf_mesh.py --facets 512            # 就地（自动备份 .orig）
  python3 tools/decimate_leaf_mesh.py --facets 512 --out DIR  # 写到别处（试验用）
  python3 tools/decimate_leaf_mesh.py --restore               # 从 .orig 还原
"""
import argparse
import os
import shutil
import struct
import sys

import numpy as np

MESH_DIR = "/home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/meshes/poplar"
VARIANTS = (0, 1, 2)


def read_stl(path):
    """读二进制 STL → (F,3,3) float32 顶点。ASCII STL 直接报错（本仓全是二进制）。"""
    with open(path, "rb") as fh:
        blob = fh.read()
    if len(blob) < 84:
        raise ValueError("%s 太短" % path)
    n = struct.unpack("<I", blob[80:84])[0]
    if 84 + n * 50 != len(blob):
        raise ValueError("%s 不是二进制 STL（面数 %d 推算 %d 字节 vs 实际 %d）"
                         % (path, n, 84 + n * 50, len(blob)))
    arr = np.frombuffer(blob, dtype=np.uint8, count=n * 50, offset=84)
    arr = arr.reshape(n, 50)
    # 前 12 字节是法线（丢弃，后面按几何重算），接着 36 字节是 9 个 float 顶点
    verts = arr[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    return verts.astype(np.float32)


def write_stl(path, tris):
    """(F,3,3) → 二进制 STL，法线按几何重算。"""
    a = tris[:, 1] - tris[:, 0]
    b = tris[:, 2] - tris[:, 0]
    nrm = np.cross(a, b)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    ln[ln < 1e-20] = 1.0
    nrm = (nrm / ln).astype(np.float32)
    n = len(tris)
    rec = np.zeros((n, 50), dtype=np.uint8)
    rec[:, 0:12] = nrm.view(np.uint8).reshape(n, 12)
    rec[:, 12:48] = tris.astype(np.float32).reshape(n, 9).view(np.uint8).reshape(n, 36)
    with open(path, "wb") as fh:
        fh.write(b"decimate_leaf_mesh.py vertex-clustering (2026-10-07)".ljust(80, b"\0"))
        fh.write(struct.pack("<I", n))
        fh.write(rec.tobytes())
    return n


def cluster(tris, cell):
    """一次顶点聚类。返回新三角面数组。"""
    V = tris.reshape(-1, 3)
    lo = V.min(axis=0)
    idx = np.floor((V - lo) / cell).astype(np.int64)
    uniq, inv = np.unique(idx, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    k = len(uniq)
    sums = np.zeros((k, 3), dtype=np.float64)
    np.add.at(sums, inv, V.astype(np.float64))
    cnt = np.bincount(inv, minlength=k).astype(np.float64)
    cnt[cnt == 0] = 1.0
    rep = (sums / cnt[:, None]).astype(np.float32)
    F = inv.reshape(-1, 3)
    ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    F = F[ok]
    if len(F) == 0:
        return F.reshape(0, 3, 3)
    return rep[F]


def decimate_to(tris, target):
    """二分 cell 边长命中 target 面数。返回 (新面, 命中面数, cell)。"""
    lo = 0.0
    diag = float(np.linalg.norm(tris.reshape(-1, 3).max(0) - tris.reshape(-1, 3).min(0)))
    hi = diag if diag > 0 else 1.0
    best = None
    for _ in range(48):
        mid = 0.5 * (lo + hi)
        out = cluster(tris, mid)
        n = len(out)
        if best is None or abs(n - target) < abs(best[1] - target):
            best = (out, n, mid)
        if n > target:
            lo = mid          # 面还太多 ⇒ cell 放大
        else:
            hi = mid
        if abs(lo - hi) < 1e-9 * max(1.0, diag):
            break
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--facets", type=int, default=512, help="每棵树的目标面数")
    ap.add_argument("--out", default=None, help="输出目录（缺省=就地覆盖，先备份 .orig）")
    ap.add_argument("--restore", action="store_true", help="从 .orig 还原")
    ap.add_argument("--mesh-dir", default=MESH_DIR)
    args = ap.parse_args()

    md = args.mesh_dir

    if args.restore:
        got = 0
        for v in VARIANTS:
            src = os.path.join(md, ".orig", "tree_%d_leaves.stl" % v)
            dst = os.path.join(md, "tree_%d_leaves.stl" % v)
            if os.path.exists(src):
                shutil.copy2(src, dst)
                got += 1
                print("restore %s" % dst)
        print("restored %d/%d" % (got, len(VARIANTS)))
        return 0 if got == len(VARIANTS) else 1

    outdir = args.out or md
    if outdir != md:
        os.makedirs(outdir, exist_ok=True)
    elif not os.path.isdir(os.path.join(md, ".orig")):
        os.makedirs(os.path.join(md, ".orig"), exist_ok=True)
        for v in VARIANTS:
            src = os.path.join(md, "tree_%d_leaves.stl" % v)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(md, ".orig", "tree_%d_leaves.stl" % v))
                print("backup -> .orig/tree_%d_leaves.stl" % v)

    total_before = total_after = 0
    for v in VARIANTS:
        src = os.path.join(md, ".orig", "tree_%d_leaves.stl" % v)
        if not os.path.exists(src):
            src = os.path.join(md, "tree_%d_leaves.stl" % v)
        tris = read_stl(src)
        nb = len(tris)
        out, na, cell = decimate_to(tris, args.facets)
        if len(out) == 0:
            print("!! tree_%d 抽稀后空了，跳过" % v, file=sys.stderr)
            return 1
        dst = os.path.join(outdir, "tree_%d_leaves.stl" % v)
        write_stl(dst, out)
        total_before += nb
        total_after += na
        print("tree_%d: %5d -> %5d facets (cell=%.5f)  %s" % (v, nb, na, cell, dst))
    print("合计 %d -> %d facets (%.1f%%)" % (
        total_before, total_after, 100.0 * total_after / max(1, total_before)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
