#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/c3_tip_pilot.py — W7-C3 tip_slow pilot（gz 实弹 A/B，2026-09-28）。

立案（memory/zx2026-gz-crown-collision-forensics + se-band-gate）：gz 复验
seed42 默认栈残余=冠缘热点 tree(-7.88,5.88) tip_slow 靶；SE 带双树窄门
（8.96,-2.57 / 9.16,-5.96 冠缘间隙 ~1m）跨三臂持续。机制已实现默认关
（sim_settings branch_handling.tip_slow），w7_selftest C6/C7/C9 干测过。

臂（2 臂 × seed{42,43,44} × rep{1,2} = 12 cells，rep 内 A→B 交错防时段漂移）：
  A  tip=False（=现行默认栈：W1 三旗+W2-P2+mem6+W7-C1/C2 全 ON）
  B  tip=True （唯一臂变量=tip_slow，其余逐位同 A）

后端：bash run_verify_gazebo.sh（无头 gzserver，端口 11411）——残余热点
即在 gz 实弹定谳，pilot 对 gz。

毙杀口径（预登记，W7 同款 + 减速代价门）：
  证据门：B 臂每 cell banner 须回显 tip=True 且 TIP-SLOW 触发行 ≥1
         （无触发行=机制未咬合，col 差异不可归因）。
  翻线：B 臂 Σcol ≤ A 臂（逐 seed/rep 配对对账）∧ score 无配对劣化 ∧
        零 FAIL（6/6 DONE）∧ done_t 回归 ≤20s（减速代价门；W4 预算律
        slack≥41.3s 兜底）∧ minclr 不劣化（风险再分布对账）。
  判毙：任一 FAIL / score 降 / done_t 超 20s / minclr 崩 / col 挪移到
        别的障碍类 / B 臂零触发行。

用法:
  python3 tools/c3_tip_pilot.py smoke         # Phase 0：B seed42 咬合门
  python3 tools/c3_tip_pilot.py rest <outdir>  # 其余 cells 续跑
  python3 tools/c3_tip_pilot.py all           # 全 12 cells（跳过 smoke 门）
"""
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

import yaml

WS = os.path.expanduser("~/zx2026_arena_ws")
CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
ROS_LOG = os.path.expanduser("~/.ros/log")

sys.path.insert(0, os.path.join(WS, "tools"))
from matrix_run import VERIFY_TXT, SMOKE_LOG, set_top, set_in_block, \
    set_key_in_block, parse_metrics


def _time_limit_s():
    try:
        with open(WS + "/src/zx2026_common/config/competition_rules.yaml") as f:
            return float(yaml.safe_load(f).get("time_limit_s", 600.0))
    except Exception:
        return 600.0


CELLS = []
for _rep in (1, 2):
    for _s in (42, 43, 44):
        CELLS.append(dict(tag="A", seed=_s, rep=_rep, tip=False))
        CELLS.append(dict(tag="B", seed=_s, rep=_rep, tip=True))

EVIDENCE_PATS = [
    "closed_loop=",            # banner（含 tip= 回显）
    "TIP-SLOW",                # W7-C3 触发证据（预测性减速）
    "_guard hit",
    "swarm quiet window",
    "collision marker at",
    "COLLISION",               # gazebo_collision_monitor logwarn（带坐标）
    "DEAD-END escape engaged",
    "dead-end escape done",
    "goal-seal",
    "POST-HIT CALM",
    "POST-HIT FREEZE",
    "POST-HIT HOLD",
    "HAZARD-SHARE",
]


def patch_cell(cell):
    """显式落盘全部旗（滚动基线 + tip 臂变量）：补丁跨 cell 累积，缺省也必须写。

    基线=W7 定谳栈（W1 三旗+W2-P2+mem6），C1/C2 按现行默认双开（两臂同置），
    杨树林外置 SDF 契约下 procedural 枝层关（两臂同置）。
    """
    set_top(CFG, "run_seed", cell["seed"])
    set_in_block(CFG, "temporal_consistency", "false")
    set_in_block(CFG, "wind", "false")
    set_in_block(CFG, "comms", "false")
    set_in_block(CFG, "deadend_escape", "true")
    set_key_in_block(CFG, "deadend_escape", "exit_hyst_s", 2.5)
    set_in_block(CFG, "veto_gate", "false")
    set_in_block(CFG, "point_cache", "false")
    set_in_block(CFG, "drift_aware_margin", "false")
    set_key_in_block(CFG, "near_stop_zone", "enabled", "false")
    set_key_in_block(CFG, "nav", "inflation", 0.4)
    set_in_block(CFG, "rescue_mutex", "true")
    set_in_block(CFG, "bounce_corridor", "true")
    set_in_block(CFG, "sep_obs_guard", "true")
    set_in_block(CFG, "hotspot_inflate", "false")
    set_key_in_block(CFG, "via_slots", "enabled", "true")
    set_key_in_block(CFG, "rescue_quiet", "enabled", "true")
    set_key_in_block(CFG, "obs_guard", "enabled", "true")
    set_key_in_block(CFG, "vo_avoid", "enabled", "false")
    set_key_in_block(CFG, "boundary_margin", "enabled", "true")
    set_key_in_block(CFG, "return_route", "enabled", "false")
    set_key_in_block(CFG, "branch_handling", "contact_mark", "false")
    set_key_in_block(CFG, "branch_handling", "ca_iso_floor", 0.0)
    set_key_in_block(CFG, "branch_handling", "cloud_mem_frames", 6)
    set_key_in_block(CFG, "post_hit_calm", "enabled", "true")
    set_key_in_block(CFG, "hazard_share", "enabled", "true")
    set_in_block(CFG, "branches", "false")
    # ---- 臂变量：唯一差异 = tip_slow ----
    set_key_in_block(CFG, "tip_slow", "enabled",
                     "true" if cell["tip"] else "false")


def capture_evidence(outdir):
    """从 ~/.ros/log 最新 run 目录采证据行 → <outdir>/evidence.txt。"""
    dirs = [d for d in glob.glob(os.path.join(ROS_LOG, "*"))
            if os.path.isdir(d) and os.path.basename(d) != "latest"]
    if not dirs:
        open(os.path.join(outdir, "evidence.txt"), "w").write("(no ros log dir)")
        return "(none)"
    run_dir = max(dirs, key=os.path.getmtime)
    lines = []
    for fp in sorted(glob.glob(os.path.join(run_dir, "*.log"))):
        try:
            txt = open(fp, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        base = os.path.basename(fp)
        for ln in txt.splitlines():
            if any(p in ln for p in EVIDENCE_PATS):
                lines.append("%s | %s" % (base, ln.strip()))
    with open(os.path.join(outdir, "evidence.txt"), "w", encoding="utf-8") as f:
        f.write("run_dir=%s\n" % os.path.basename(run_dir))
        f.write("\n".join(lines) + "\n")
    return os.path.basename(run_dir)


def _wait_clean_field(timeout_s=90.0):
    """清场点名：零 ROS/GZ 进程才发车（防串局污染）。"""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        r = subprocess.run(["pgrep", "-fc", "rosmaster|gzserver|roslaunch"],
                           capture_output=True, text=True)
        cnt = (r.stdout or "").strip()
        if cnt in ("", "0"):
            return True
        print("    清场等待：残留进程 %s" % cnt, flush=True)
        time.sleep(3)
    return False


def _run_once(cell, outdir):
    """跑一次 run_verify_gazebo.sh 并收集结果 + 证据；返回 (row, sleep_drift_s)。"""
    tag, seed = cell["tag"], cell["seed"]
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    patch_cell(cell)
    if not _wait_clean_field():
        print("    !! 清场超时，强制 pkill 兜底", flush=True)
        subprocess.run(["pkill", "-f", "rosmaster.*11411"], capture_output=True)
        subprocess.run(["pkill", "-f", "gzserver"], capture_output=True)
        time.sleep(3)

    t0 = time.time()
    m0 = time.monotonic()
    timed_out = False
    try:
        subprocess.run(["bash", WS + "/run_verify_gazebo.sh"],
                       capture_output=True, text=True,
                       timeout=_time_limit_s() + 300.0)
    except subprocess.TimeoutExpired:
        timed_out = True
        subprocess.run(["pkill", "-f", "rosmaster.*11411"], capture_output=True)
        subprocess.run(["pkill", "-f", "gazebo.launch"], capture_output=True)
        subprocess.run(["pkill", "-f", "gzserver"], capture_output=True)
        time.sleep(2)
    # 封卷汇合窗：scorekeeper report_settle_s 延迟写可能落在脚本退出之后
    time.sleep(3.0)
    dt = time.time() - t0
    drift = dt - (time.monotonic() - m0)

    row = dict(tag=tag, seed=seed, rep=cell["rep"], verdict="FAIL", score=-1,
               done_t=-1, col=-1, stuck_s=-1, stuck_n=-1, flips=-1, dist=-1,
               minclr=-1, runtime_s=round(dt), timeout=timed_out,
               sleep_s=round(drift, 1))
    try:
        txt = open(VERIFY_TXT).read()
        if "VERDICT: PASS" in txt:
            row["verdict"] = "PASS"
        m = re.search(r"-- state -> DONE @ (\d+)s", txt)
        if m:
            row["done_t"] = int(m.group(1))
    except OSError:
        pass
    scores = glob.glob("/tmp/zx2026_score_*.yaml")
    if scores:
        try:
            sy = yaml.safe_load(open(max(scores, key=os.path.getmtime)))
            t = sy.get("total")
            if isinstance(t, (int, float)):
                row["score"] = int(t)
        except Exception:
            pass

    ymls = sorted(glob.glob(WS + "/run_logs/nav_metrics_2*.yaml"),
                  key=os.path.getmtime)
    if ymls:
        try:
            row.update(parse_metrics(ymls[-1]))
        except Exception as e:
            print("metrics parse fail:", e, flush=True)

    os.makedirs(outdir, exist_ok=True)
    for src, dst in ((VERIFY_TXT, "verify.txt"), (SMOKE_LOG, "smoke.log")):
        try:
            shutil.copy(src, os.path.join(outdir, dst))
        except OSError:
            pass
    if ymls:
        shutil.copy(ymls[-1], os.path.join(outdir, "nav_metrics.yaml"))
    scores = glob.glob("/tmp/zx2026_score_*.yaml")
    if scores:
        shutil.copy(max(scores, key=os.path.getmtime),
                    os.path.join(outdir, "score.yaml"))
    run_dir = capture_evidence(outdir)
    row["run_dir"] = run_dir
    # 证据门计数：B 臂零 TIP-SLOW=机制未咬合；banner tip= 旗位回显
    ev = os.path.join(outdir, "evidence.txt")
    try:
        etxt = open(ev, encoding="utf-8").read()
    except OSError:
        etxt = ""
    row["n_tipslow"] = etxt.count("TIP-SLOW")
    row["banner_tip"] = etxt.count("tip=True")
    row["banner_tip_off"] = etxt.count("tip=False")
    row["n_col_gz"] = etxt.count("COLLISION")
    for pat, key in (("POST-HIT CALM", "n_calm"), ("HAZARD-SHARE", "n_hz")):
        row[key] = etxt.count(pat)
    return row, drift


def run_cell(cell, outdir, rerun_on_sleep=True):
    """跑一个 cell；检出睡眠/挂死污染则留档并自动重跑一次。"""
    row, drift = _run_once(cell, outdir)
    contaminated = drift > 30.0 or (row["score"] == -1 and not row["timeout"])
    if contaminated and rerun_on_sleep:
        why = ("wall-monotonic 漂移 %.0fs" % drift) if drift > 30.0 \
            else "verify 无 total（rosmaster 残留挂死）"
        print("    !! %s —— cell 污染：留档 __contaminated，自动重跑" % why,
              flush=True)
        shutil.rmtree(outdir + "__contaminated", ignore_errors=True)
        try:
            os.rename(outdir, outdir + "__contaminated")
        except OSError:
            pass
        row, drift = _run_once(cell, outdir)
    row["sleep_s"] = round(drift, 1)
    return row


def summary(rows):
    print("\n===== C3 tip_slow pilot 汇总（gz 实弹）=====")
    for tag in ("A", "B"):
        arm = [r for r in rows if r["tag"] == tag]
        if not arm:
            continue
        pv = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%s: PASS %d/%d" % (tag, pv, len(arm)))
        for r in sorted(arm, key=lambda x: (x["seed"], x["rep"])):
            print("   s%d r%d: %s score=%s done_t=%s col=%s minclr=%s "
                  "TIP-SLOW=%s banner(tipT/tipF)=%d/%d COLLISION行=%d" % (
                      r["seed"], r["rep"], r["verdict"], r["score"],
                      r["done_t"], r.get("col"), r.get("minclr"),
                      r.get("n_tipslow", 0), r.get("banner_tip", 0),
                      r.get("banner_tip_off", 0), r.get("n_col_gz", 0)))
    keys = sorted(set((r["seed"], r["rep"]) for r in rows))
    print("\n配对差（B−A）：")
    hard_bad = []
    for s, rep in keys:
        ra = next((r for r in rows if r["tag"] == "A" and r["seed"] == s
                   and r["rep"] == rep), None)
        rb = next((r for r in rows if r["tag"] == "B" and r["seed"] == s
                   and r["rep"] == rep), None)
        if not ra or not rb:
            continue
        d_done = (rb["done_t"] - ra["done_t"]) \
            if (ra["done_t"] >= 0 and rb["done_t"] >= 0) else None
        d_score = (rb["score"] - ra["score"]) \
            if (ra["score"] >= 0 and rb["score"] >= 0) else None
        print("  s%d r%d: dCol=%+d dScore=%s dDone=%s" % (
            s, rep, (rb.get("col", 0) - ra.get("col", 0)),
            ("+%d" % d_score) if d_score is not None else "?",
            ("+%ds" % d_done) if d_done is not None else "?"))
        if rb["verdict"] != "PASS":
            hard_bad.append("s%dr%d FAIL" % (s, rep))
        if d_score is not None and d_score < 0:
            hard_bad.append("s%dr%d score 降" % (s, rep))
        if rb.get("col", 0) > ra.get("col", 0):
            hard_bad.append("s%dr%d col 升" % (s, rep))
        if d_done is not None and d_done > 20:
            hard_bad.append("s%dr%d done_t 超 20s" % (s, rep))
    b_arm = [r for r in rows if r["tag"] == "B"]
    no_bite = [r for r in b_arm
               if r.get("banner_tip", 0) < 1 or r.get("n_tipslow", 0) < 1]
    print("\n证据门：B 臂零咬合 cells=%s" %
          (["s%dr%d" % (r["seed"], r["rep"]) for r in no_bite] or "无"))
    print("硬门违例：%s" % (hard_bad or "无"))
    if len(b_arm) == len(rows) // 2 and len(rows) >= 2:
        if not hard_bad and not no_bite:
            print("判定：硬门全过+全咬合 → 翻线候选（TIP 类归因对账后定谳）")
        else:
            print("判定：判毙/存疑——见违例清单")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    os.environ.setdefault("GAZEBO_MODEL_PATH",
                          "/opt/ros/noetic/share/gazebo-11/models")
    outbase = WS + "/run_logs/c3_pilot_%s" % time.strftime("%Y%m%d_%H%M%S")
    # 配装备份：任何退出路径都恢复原 yaml（含行尾注释逐位）
    bak = CFG + ".c3pilot_bak"
    shutil.copy(CFG, bak)
    try:
        # exec bit 自愈（124209 教训）：UNC 编辑 node 脚本会掉可执行位
        subprocess.run(["bash", "-c",
                        "find %s/src -name '*.py' -path '*/scripts/*' "
                        "-exec chmod +x {} \\;" % WS], capture_output=True)
        if mode == "smoke":
            cell = next(c for c in CELLS if c["tag"] == "B" and c["seed"] == 42
                        and c["rep"] == 1)
            outdir = outbase + "_%s_s%d_r%d" % (cell["tag"], cell["seed"],
                                                cell["rep"])
            print("Phase 0 smoke: %s seed%d rep%d -> %s" % (
                cell["tag"], cell["seed"], cell["rep"], outdir), flush=True)
            row = run_cell(cell, outdir)
            print("smoke row:", json.dumps(row, ensure_ascii=False), flush=True)
            bad = []
            if row["verdict"] != "PASS":
                bad.append("verdict 非 PASS")
            if row["score"] < 0:
                bad.append("score 缺失")
            if row.get("banner_tip", 0) < 1:
                bad.append("banner 未回显 tip=True（旗未咬合）")
            if row.get("n_tipslow", 0) < 1:
                bad.append("TIP-SLOW 触发行=0（机制未咬合，col 差异不可归因）")
            if bad:
                print("SMOKE FAIL — %s —— 不进入矩阵" % "；".join(bad),
                      flush=True)
                sys.exit(2)
            print("SMOKE PASS（咬合门过：tip=True 回显 + TIP-SLOW≥1）",
                  flush=True)
            return
        if mode == "rest":
            outbase = sys.argv[2]
            rows = []
            done = set()
            for d in glob.glob(outbase + "_*"):
                m = re.match(r".*_([AB])_s(\d+)_r(\d+)$", d)
                if m and not d.endswith("__contaminated"):
                    done.add((m.group(1), int(m.group(2)), int(m.group(3))))
            cells = [c for c in CELLS
                     if (c["tag"], c["seed"], c["rep"]) not in done]
        else:
            cells = CELLS
            rows = []
        for i, cell in enumerate(cells):
            outdir = "%s_%s_s%d_r%d" % (outbase, cell["tag"], cell["seed"],
                                        cell["rep"])
            print("[%d/%d] %s seed%d rep%d ..." % (
                i + 1, len(cells), cell["tag"], cell["seed"], cell["rep"]),
                flush=True)
            row = run_cell(cell, outdir)
            print("    ", json.dumps(row, ensure_ascii=False), flush=True)
            rows.append(row)
            try:
                with open(outbase + "_results.json", "w", encoding="utf-8") as f:
                    json.dump(rows, f, ensure_ascii=False, indent=1)
            except OSError:
                pass
            summary(rows)
        print("DONE -> %s" % outbase, flush=True)
    finally:
        shutil.copy(bak, CFG)
        print("config restored from %s" % bak, flush=True)


if __name__ == "__main__":
    main()
