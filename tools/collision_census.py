#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""碰撞计数普查：**多通道并列**，禁止用单一 token 出数。

起因（2026-10-07，我的口径事故）：我用 `grep -c COLLISION` 数碰撞，得出
「碰撞 0」并写进给用户的结论；而世界层的实际日志串是 **`COLLIDED obstacle`**
（`world_node.py:335`），真值是 **6**。用户在下一轮问「这局有碰撞吗」才暴露。
本工具把那次的教训固化成机制：任何一次计数都必须**同时**给出全部可用通道。

口径分层（这是本工具的核心概念，别把两层混成一个数）：
  [真值层] 世界层几何命中 —— 三源互校，**不一致即报错**
    A rosout.log        `COLLIDED obstacle`（含 (x,y,z)）
    C world_node-*.log  同 A 的逐节点副本（A 被截断时的交叉源）
    D gazebo_collision_monitor-*.log  gz 判官；⚠ 判官自身漏计「走 freeze 分支
                        的那一撞」（不发 Bool 不打行）⇒ gz 真值 = D 数 + frozen 数
  [感知层] 导航当拍自己记下的撞击 —— **结构上允许 < 真值**
    B nav_node_<i>.log  `collision marker ... close= clr=`（盲撞判据就在这里）
  [交叉]  E nav_metrics 按局快照 run_logs/nav_metrics_<启动时刻>.yaml
          ⚠ 其日志行 `NAV_METRICS_SUMMARY:` 在 logging 层被**截断在 86 字符**
            （rosout.log 与 nav_metrics-<pid>.log 两处皆然，只剩 drone 0）⇒
            照日志数会得 1 而真值 6。其 CSV 的 collisions 又会中途复位 ⇒ 不能
            取末值。故只认按局 yaml（节点内 sim_t 精确对局）。

用法:
  python3 tools/collision_census.py                # 最新一局
  python3 tools/collision_census.py <uuid 前缀>     # 指定局
  python3 tools/collision_census.py --list          # 列近 12 局
  python3 tools/collision_census.py --blind <uuid>  # 追加盲撞/挤入分类
退出码: 0=真值层内部一致  1=真值层不一致(可疑)  2=用法/找不到局
"""
import glob
import io
import os
import re
import sys
import time

LOG_ROOT = "/home/ubuntu/.ros/log"
SMOKE = "/tmp/zx2026_smoke.log"
# nav_metrics 的按局快照目录（权威 E 通道；日志行会被截断，见 ch_metrics）
YAML_DIR = "/home/ubuntu/zx2026_ego_ws/run_logs"
CSV = os.path.join(YAML_DIR, "nav_metrics_ts.csv")   # 兜底：逐拍逐机滚动追加
# ⚠ 正确 token 是 COLLIDED；COLLISION 在世界层**从来不是**碰撞事件
#（rosout 里的 `collision` 全是话题名/订阅名，2026-10-07 逐字核过）
TOKEN = "COLLIDED obstacle"
WRONG_TOKEN = "COLLISION"
# 盲撞判据（tools/ 既有口径）：clr = 撞拍本机点云最近点距离。
# clr ≈ drone_radius(0.35) 且 close>0 ⇒ 看见仍撞（挤入）；clr ≫ 0.35 ⇒ 盲撞。
DRONE_R = 0.35


def pick_run(arg):
    if arg and arg != "--latest":
        cands = [d for d in glob.glob(os.path.join(LOG_ROOT, "*"))
                 if os.path.isdir(d) and os.path.basename(d).startswith(arg)]
        if not cands:
            return None
        return max(cands, key=os.path.getmtime)
    dirs = [d for d in glob.glob(os.path.join(LOG_ROOT, "*")) if os.path.isdir(d)]
    return max(dirs, key=os.path.getmtime) if dirs else None


def read(p):
    try:
        return io.open(p, encoding="utf-8", errors="ignore").read()
    except Exception:
        return ""


def parse_pos(lines):
    out = []
    for ln in lines:
        if TOKEN not in ln:
            continue
        try:
            d = ln.split("drone ")[1].split(" ")[0]
        except Exception:
            d = "?"
        m = re.search(r"at \((-?[\d.]+), ?(-?[\d.]+), ?(-?[\d.]+)\)", ln)
        out.append((d, tuple(float(x) for x in m.groups()) if m else None))
    return out


def per_drone(hits):
    c = {}
    for d, _ in hits:
        c[d] = c.get(d, 0) + 1
    return dict(sorted(c.items(), key=lambda kv: int(kv[0]) if kv[0].isdigit() else 99))


def _nlog(f):
    return "stdout" not in f and "stderr" not in f


def ch_rosout(d):
    p = os.path.join(d, "rosout.log")
    if not os.path.isfile(p):
        return None, "rosout.log 缺失"
    return parse_pos(read(p).split("\n")), None


def ch_nodes(d):
    hits, files = [], []
    for f in glob.glob(os.path.join(d, "nav_node_*-*.log")):
        if not _nlog(f):
            continue
        files.append(f)
        for ln in read(f).split("\n"):
            if "collision marker" in ln:
                dr = re.search(r"drone (\d+)", ln)
                clr = re.search(r"clr=(-?[\d.]+)", ln)
                cl = re.search(r"close=(-?[\d.]+)", ln)
                hits.append((dr.group(1) if dr else "?",
                             (float(cl.group(1)) if cl else None,
                              float(clr.group(1)) if clr else None)))
    if not files:
        return None, "无 nav_node_*.log"
    return hits, None


def ch_world(d):
    hits, files = [], []
    for f in glob.glob(os.path.join(d, "world_node*.log")):
        if not _nlog(f):
            continue
        files.append(f)
        hits += parse_pos(read(f).split("\n"))
    if not files:
        return None, "无 world_node*.log"
    return hits, None


def ch_judge(d):
    fs = [f for f in glob.glob(os.path.join(d, "gazebo_collision_monitor-*.log")) if _nlog(f)]
    if not fs:
        return None, "py 后端无判官节点"
    hits = []
    for f in fs:
        for ln in read(f).split("\n"):
            if re.search(r"COLLISION|collided|hit", ln, re.I):
                hits.append((os.path.basename(f)[:20], None))
    frozen = 0
    for f in glob.glob(os.path.join(d, "*")):
        if os.path.isfile(f) and _nlog(f):
            frozen += len(re.findall(r"FREEZE|frozen", read(f)))
    if frozen:
        print("      (判官通道补 frozen=%d 才是 gz 真值：判官漏计走 freeze 的那一撞)" % frozen)
    return hits, None


def _run_window(d):
    """从 rosout.log 的 epoch 戳推出本局的墙钟窗口。"""
    ts = [float(x) for x in re.findall(r"(17\d{8}\.\d+)", read(os.path.join(d, "rosout.log")))]
    return (min(ts) - 5.0, max(ts) + 180.0) if ts else None


def ch_metrics(d):
    """E 通道：nav_metrics。

    ⚠ 三处坑（都为 2026-10-08 实测，不是推测）：
    1. **日志行不可用**：`NAV_METRICS_SUMMARY:` 在 logging 层被截断
       （rosout.log 与 nav_metrics-<pid>.log 两处都断在 86 字符，只剩 drone 0
       且尾部停在半个 dict 里）⇒ 照它数会得 1 而真值 6。与「launch log 的 col
       整局假 0」同族，故此处根本不读它。
    2. **CSV 的 collisions 会中途复位**（`d7c638fa` 末值 0/1/1/1/0/0，而最大值
       {0:4,1:2,2:8,3:5,4:3,5:6}=28 ≈ 真值 29）⇒ 绝不能取"最后一行"。
    3. **CSV 会跨局**：它按墙钟滚动追加，窗口若取太宽（rosout 戳有时越过局界）
       会把下一局的行并进来。
    权威源 = `run_logs/nav_metrics_<局启动时刻>.yaml`：按局落盘的干净快照，其
    `drones[*].sim_t` 就是本局 epoch，可**精确对局**（不靠 mtime、不靠文件名）。
    CSV 只作兜底（取每机**最大值**并显式加注）。
    """
    win = _run_window(d)
    if not win:
        return None, "rosout 无 epoch 戳，无法对局"
    try:
        import yaml as _yaml
    except Exception as e:
        return None, "无 yaml 模块: %s" % e
    snap = [f for f in glob.glob(os.path.join(YAML_DIR, "nav_metrics_*.yaml"))
            if os.path.basename(f) != "nav_metrics_latest.yaml"]
    for f in sorted(snap, key=os.path.getmtime, reverse=True):
        if not (win[0] - 120 <= os.path.getmtime(f) <= win[1] + 120):
            continue
        try:
            doc = _yaml.safe_load(io.open(f, encoding="utf-8", errors="ignore"))
        except Exception:
            continue
        dr = (doc or {}).get("drones") or {}
        sim = [v.get("sim_t") for v in dr.values() if isinstance(v, dict) and v.get("sim_t")]
        if not sim or not (win[0] <= max(sim) <= win[1]):
            continue      # 落盘时间近但 sim_t 不属于本局 ⇒ 是邻局快照
        per = {k: int(float(v.get("collisions") or 0)) for k, v in dr.items()}
        return (sum(per.values()), dict(sorted(per.items(), key=lambda kv: int(kv[0]))),
                "%s state=%s" % (os.path.basename(f), (doc or {}).get("state"))), None

    if not os.path.exists(CSV):
        return None, "无按局 yaml、亦无 %s" % CSV
    try:
        import csv as _csv
        rows = [r for r in _csv.DictReader(io.open(CSV, encoding="utf-8", errors="ignore"))
                if r.get("wall_t") and win[0] <= float(r["wall_t"]) <= win[1]]
    except Exception as e:
        return None, "CSV 读取失败: %s" % e
    if not rows:
        return None, "窗口 %.0f-%.0f 内无行" % win
    mx = {}
    for r in rows:
        k = r.get("drone")
        if k is None:
            continue
        mx[k] = max(mx.get(k, 0.0), float(r.get("collisions") or 0))
    per = {k: int(v) for k, v in mx.items()}
    return (sum(per.values()), dict(sorted(per.items(), key=lambda kv: int(kv[0]))),
            "兜底 CSV 取每机**最大值**；窗口内 %d 行，可能含邻局" % len(rows)), None


def blind_split(d):
    """盲撞 vs 挤入：clr 是本机点云最近点，close 是闭合速度。"""
    blind, squeeze = [], []
    for f in glob.glob(os.path.join(d, "nav_node_*-*.log")):
        if not _nlog(f):
            continue
        for ln in read(f).split("\n"):
            if "collision marker" not in ln:
                continue
            dr = re.search(r"drone (\d+)", ln)
            cl = re.search(r"close=(-?[\d.]+)", ln)
            clr = re.search(r"clr=(-?[\d.]+)", ln)
            if not clr:
                continue
            clr = float(clr.group(1))
            cl = float(cl.group(1)) if cl else 0.0
            rec = (dr.group(1) if dr else "?", cl, clr)
            # clr 贴着机身半径 ⇒ 撞击当拍点云确实看见了这棵树（挤入）
            (squeeze if clr <= DRONE_R * 1.5 else blind).append(rec)
    print("      [B 通道细分] 盲撞(clr≫机身半径)=%d  挤入(clr≈%.2f 且 close>0)=%d"
          % (len(blind), DRONE_R, len(squeeze)))
    for tag, lst in (("盲撞", blind), ("挤入", squeeze)):
        for dr, cl, clr in lst:
            print("         %s drone %s close=%+.2f clr=%.2f" % (tag, dr, cl, clr))
    if not blind and not squeeze:
        print("         (本局无 marker 行)")
    elif not [r for r in blind + squeeze if r[2] <= DRONE_R * 1.5]:
        print("         ⚠ 无小 clr 样本 ⇒ 整列判据退化，不得据此断「盲/不盲」")


def main():
    argv = [a for a in sys.argv[1:] if a != "--blind"]
    want_blind = "--blind" in sys.argv
    a = argv[0] if argv else "--latest"
    if a == "--list":
        for d in sorted(glob.glob(os.path.join(LOG_ROOT, "*")), key=os.path.getmtime)[-12:]:
            if os.path.isdir(d):
                print("  %s  COLLIDED=%d" % (os.path.basename(d),
                                             read(os.path.join(d, "rosout.log")).count(TOKEN)))
        return 0
    d = pick_run(a)
    if not d:
        print("找不到局: %s（或用 --list 看可选）" % a)
        return 2
    print("局: %s" % os.path.basename(d))
    print()

    rows = []   # (层, 名称, 计数 or None, 逐机, 备注)

    A, errA = ch_rosout(d)
    rows.append(("[真值]", "A rosout COLLIDED", len(A) if A is not None else None,
                 per_drone(A or []), errA or ""))
    C, errC = ch_world(d)
    rows.append(("[真值]", "C world_node 副本", len(C) if C is not None else None,
                 per_drone(C or []), errC or ""))
    D, errD = ch_judge(d)
    rows.append(("[真值]", "D 判官(gz)", len(D) if D is not None else None, {}, errD or ""))
    B, errB = ch_nodes(d)
    rows.append(("[感知]", "B nav_node marker", len(B) if B is not None else None,
                 per_drone(B or []), errB or ""))
    E = ch_metrics(d)
    if E[0] is None:
        rows.append(("[交叉]", "E nav_metrics", None, {}, E[1]))
    else:
        tot, per, note = E[0]
        rows.append(("[交叉]", "E nav_metrics", tot, per, note))

    print("  层      通道                       计数   逐机分布")
    for layer, name, n, pd, note in rows:
        print("  %-6s  %-26s %-6s %s%s" % (layer, name, "N/A" if n is None else n, pd,
                                           ("  " + note) if note else ""))
    print()

    if want_blind:
        blind_split(d)

    wrong = read(os.path.join(d, "rosout.log")).count(WRONG_TOKEN)
    print("  ⚠ 口径陷阱自查：用 `%s` 数会得 %d 次（全是话题名/订阅名，**非碰撞**）。"
          % (WRONG_TOKEN, wrong))
    print("    世界层碰撞的唯一 token 是 `%s`。`COLLISION_THRESHOLD` 等也含该串，"
          "故连 `grep -c COLLISION` 这种写法都要禁。" % TOKEN)
    print()

    truth = {name: n for layer, name, n, _, _ in rows if layer == "[真值]" and n is not None}
    bad = False
    if len(set(truth.values())) > 1:
        print("  ✗ 真值层三源不一致：%s" % truth)
        print("    已知成因：rosout 会被截断（grep 不到不等于没撞）；gz 判官漏计"
              "走 freeze 分支的那一撞（gz 真值 = D + frozen，见上方提示）。")
        bad = True
    if A is not None and B is not None and len(A) != len(B):
        print("  ⚠ A(真值 %d) vs B(感知 %d)：差 %+d。真值 > 感知 ⇒ 有「撞了但导航当拍"
              "没记」；感知 > 真值 ⇒ 恢复反弹连撞两棵。" % (len(A), len(B), len(A) - len(B)))
    print()
    print("  权威计数 = 真值层 A = %s" % (len(A) if A is not None else "?"))
    if bad:
        print("  ⚠ 真值层不一致，**不要**直接采信单数，请人工核 above 的备注。")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
