#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/defop_ab_run.py — T4 DeFoP A/B 跑批器（baseline vs defop 两臂矩阵）。

复用现成机制：matrix_run（补丁函数/指标解析）、run_verify_gazebo.sh 机制
（官方判分后端，本工具按其机制仿写驱动脚本——baseline 臂逐字节复刻，defop 臂
追加 6 机 defop.launch 推理链）、gz_closure_run 的门序列（P2_WAIT / scorekeeper
话题在列 / start 服务在列 / 清场点名）、genval_run 的污染重跑律。

每 run 装订（补丁只动值不动注释，run 间备份还原）：
  run_seed=S                    两臂
  planner.source=internal|defop 消费方=nav_node.vel_source_of（T2 shadow）
  sensor.depth.enabled=true     仅 defop 臂（baseline 显式落 false=默认值零改动）

臂驱动：
  baseline = 逐字节复刻 run_verify_gazebo.sh / run_verify.sh 机制（默认栈零改动）
  defop    = 同机制 + roslaunch defop_nav defop.launch drone_id:=0..5
             （gazebo.launch 未透传 defop_arm arg，CLI 附参会报 unused args，
              故单发 6 个 defop.launch——与 stack_nodes defop 组 include 等价）
后端：默认 gazebo（官方判分后端）；/clock 门失败=gz 基建坏 → auto 档自动回退
classic（run_verify.sh 机制）并在 summary 声明。

采集：verdict/score/landed/Σcol/minclr/done_t/RTF + defop 臂活体探针
（另计 /planning/cpn_active 旗到达率与持笔占比 cpn_ratio——A/B 前提校验：
defop 臂 cpn 真持过 cruise，而非旗恒 False 静默失效跑成基线）
（每机 vel_cmd 与 latent_space 频率，rostopic 口径）+ OOM 守卫（dmesg）。
CSV 追加式（results/defop_ab.csv），列头含 arm/seed/run/时间戳；
每 cell 归档 run_logs/defop_ab_<ts>/<cell>/（verify/score/metrics/probe/rosout）。

用法:
  python3 tools/defop_ab_run.py --seeds 43,46,47 --runs 2 \
      --arms baseline,defop --out results/defop_ab.csv [--dry-run]
  python3 tools/defop_ab_run.py --arms baseline --seeds 43 --runs 1   # 实弹单验
"""
import argparse
import csv
import filecmp
import glob
import json
import os
import re
import shutil
import signal
import stat
import subprocess
import sys
import time

import yaml

WS = os.path.expanduser("~/zx2026_defop_ws")
sys.path.insert(0, os.path.join(WS, "tools"))
from matrix_run import CFG, VERIFY_TXT, SMOKE_LOG, VERIFY_TIMEOUT, \
    parse_metrics, set_in_block, set_key_in_block, set_top  # noqa: E402

# 候选臂配置（2026-10-05 立案）：arm -> defop_nav.yaml 键覆盖。
# baseline/defop 无覆盖（= 现状）；c1=候选1（死端 n_safe 门限）、c2=候选2
# （选择代价间隙项）、c12=两者并行。臂名进 ARMSPEC，驱动侧 `!= baseline`
# 即拉起 defop 链（2026-10-05 改），故 c* 与 defop 同链、仅配置不同。
NAVCFG = os.path.join(WS, "src/defop_nav/config/defop_nav.yaml")
# c1 门限 20：离线重放 3 局 31 起撞的 n_indx1 曲线定（dmin=1.0m 处中位 ~25、
# 0.5m 处 ~12）——T=20 咬合余量 0.5~3.8m、只漏 2/19；全时间线误触发 7.5~8.5%
# （T=10 只 2% 但漏 7/19；T=30 覆盖更全但停转代价更高）。8% 是诚实成本，A/B 量它。
# c2 恒开（follow_clear_nsafe=0）：间隙场在开阔区自然为 0（无否决⇒无偏置）=
# 自门控，比设阈值干净。w=0.5 rad ≈ 28.6° 上限偏置（目标对齐代价 0~1.5 rad 量级）。
ARM_KEYS = {
    "baseline": {},
    "defop": {},
    "c1": {"deadend_nsafe_min": 20},
    "c2": {"follow_clear_w": 0.5, "follow_clear_nsafe": 0},
    "c12": {"deadend_nsafe_min": 20, "follow_clear_w": 0.5,
            "follow_clear_nsafe": 0},
}

# 端口分配口径：本工具全链只用 11411（run_verify*.sh 既有端口）
MASTER = "http://127.0.0.1:11411"
CLEAN_PAT = r"rosmaster.*11411|roscore.*11411|gzserver|gzclient"
GATE_BUDGET_S = 360.0          # 发车+门序列预算（gz 冷启动含余量）
# verify 包装 timeout 在模板里加 240s（2026-09-29 smoke 法证：观测预算已改
# 按 sim 时间计，defop 臂 RTF~0.5 时 150s sim≈300s 墙钟，480s 包装太贴边；
# 基线臂 DONE 早退不受影响，只是包装上限抬高分母）
# 2026-09-29 矩阵首轮法证：原式 = GATE + VERIFY_TIMEOUT + 180 + 240 只算了
# **未加 240 的** VERIFY_TIMEOUT，与模板里真正下发的 timeout（+240）不自洽
# ——defop seed47 r0 门序列慢（gz 冷启动）后 verify 全窗未及收尾即被 cell
# 预算砍，且砍后残留 rosmaster/gzserver 毒死其后 12 个 cell。故按**实际下发
# 值**计：GATE + (VERIFY_TIMEOUT+240) + 300（发车/probe/归档/清场余量）。
CELL_BUDGET_S = GATE_BUDGET_S + (VERIFY_TIMEOUT + 240.0) + 300.0
COLS = ["arm", "seed", "run", "ts", "backend", "verdict", "score", "landed",
        "retired_n", "col", "minclr", "done_t", "rtf", "vel_cmd_hz",
        "latent_hz", "latent_live", "cpn_live", "cpn_duty_s", "cpn_ratio",
        "oom_hits", "runtime_s", "sleep_s",
        "timeout", "note", "cell_dir"]
FALLBACK = {"n": 0}            # gz 基建坏回退 classic 的次数（summary 声明用）

# VAE 推理设备（2026-10-01 GPU 加速立项）：driver 里 export VAE_DEVICE，由
# defop.launch 的 $(optenv VAE_DEVICE cpu) 消费 → 6 机全跟随。main() 由
# --vae-device 覆写。默认 cpu = 逐字节保留旧行为，A/B 时只差这一个开关。
VAE_DEVICE = "cpu"

# 门失败退码（driver.sh）：3=通用门失败（cell 作废）；4=gz 基建坏信号
# （/clock 缺席）——auto 档据此回退 classic
EXIT_GATE = 3
EXIT_GZBROKE = 4


# ---------------------------------------------------------------------------
# 配置补丁（沿用 matrix_run 只动值不动注释口径；planner 段缺省时新建）
# ---------------------------------------------------------------------------
def set_planner_source(path, value):
    """planner.source 装订。段存在→块内改/插 source（保行内注释）；
    缺段→文件尾新增顶级段。消费方 nav_node.vel_source_of（键缺失默认
    internal 逐字节不变，故新建段对现栈零行为变更）。
    返回动作描述：set | insert | append。"""
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    for i, ln in enumerate(lines):
        if re.match(r"^planner\s*:", ln):
            # 段内找 source:（缩进须大于段键；遇下一顶级键即段尽）
            for j in range(i + 1, len(lines)):
                s = lines[j].strip()
                if not s or s.startswith("#"):
                    continue
                if lines[j][:1] not in (" ", "\t"):
                    break
                m = re.match(r"^(\s*)source\s*:", lines[j])
                if m:
                    lines[j] = re.sub(r"^(\s*source\s*:\s*)\S+",
                                      lambda mm: mm.group(1) + str(value),
                                      lines[j], count=1)
                    with open(path, "w", encoding="utf-8") as f:
                        f.writelines(lines)
                    return "set"
            lines.insert(i + 1, "  source: %s\n" % value)
            with open(path, "w", encoding="utf-8") as f:
                f.writelines(lines)
            return "insert"
    txt = "".join(lines)
    if txt and not txt.endswith("\n"):
        txt += "\n"
    txt += ("\n# ---- 规划源（T4 defop_ab_run 每 run 装订；消费方"
            "nav_node.vel_source_of，键缺失默认 internal） ----\n"
            "planner:\n  source: %s\n" % value)
    with open(path, "w", encoding="utf-8") as f:
        f.write(txt)
    return "append"


def patch_cell(arm, seed):
    """单 cell 配置装订：run_seed + planner.source（两臂）+ depth（defop 真、
    baseline 显式落 false=默认值，防跨 cell 补丁泄漏——matrix_run 同律）+
    候选臂的 defop_nav.yaml 键覆盖（2026-10-05 立案）。"""
    is_defop = arm != "baseline"
    set_top(CFG, "run_seed", seed)
    set_planner_source(CFG, "defop" if is_defop else "internal")
    set_key_in_block(CFG, "depth", "enabled",
                     "true" if is_defop else "false")
    patch_navcfg(arm)


def patch_navcfg(arm):
    """候选臂：按 ARM_KEYS 覆盖 defop_nav.yaml 的键（原地改值、保留注释）。
    无覆盖的臂（baseline/defop）= no-op = 零写盘。命中数 != 1 即抛（防键名
    写错时静默不改=假 A/B——本仓"配置补丁 ^ 锚点缩进失配"前科）。"""
    keys = ARM_KEYS.get(arm) or {}
    if not keys:
        return
    with open(NAVCFG, encoding="utf-8") as f:
        txt = f.read()
    for k, v in keys.items():
        txt, n = re.subn(r"(?m)^%s:.*$" % re.escape(k),
                         "%s: %s" % (k, v), txt)
        if n != 1:
            raise RuntimeError("navcfg 键 %r 命中 %d 次（应恰 1）" % (k, n))
    with open(NAVCFG, "w", encoding="utf-8") as f:
        f.write(txt)


# ---------------------------------------------------------------------------
# OOM 守卫（dmesg 为主，journalctl -k 兜底；都不可用记 -1 如实上报）
# ---------------------------------------------------------------------------
def oom_count():
    r = subprocess.run(["bash", "-c", "dmesg >/dev/null 2>&1 && "
                        "dmesg 2>/dev/null | grep -ic oom"],
                       capture_output=True, text=True)
    if r.returncode == 0 and r.stdout.strip().isdigit():
        return int(r.stdout.strip())
    r = subprocess.run(["bash", "-c", "journalctl -k --no-pager 2>/dev/null "
                        "| grep -ic oom"], capture_output=True, text=True)
    if r.returncode in (0, 1) and r.stdout.strip().isdigit():
        return int(r.stdout.strip())
    return -1


def oom_snapshot(path):
    """OOM 命中现场留档（tail 20 行）。"""
    r = subprocess.run(["bash", "-c",
                        "dmesg 2>/dev/null | grep -i oom | tail -20"],
                       capture_output=True, text=True)
    if r.stdout.strip():
        with open(path, "w", encoding="utf-8") as f:
            f.write(r.stdout)


# ---------------------------------------------------------------------------
# 活体探针（每 cell 由驱动脚本拉起；SIGTERM 落 JSON）
# ---------------------------------------------------------------------------
PROBE_SRC = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_ab 活体探针（T4）：/clock 采样 + 每机 vel_cmd/latent_space 消息计数
（rostopic 口径）。SIGTERM 或超时后 finally 落 JSON；无消息话题计 0 不报错
（baseline 臂 latent_space 无发布者=常态）。first/last=(墙钟, sim 时钟)。"""
import json
import os
import signal
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32MultiArray, String

OUT = sys.argv[1]
MAX_S = float(sys.argv[2]) if len(sys.argv) > 2 else 2400.0


def _stat():
    return {"n": 0, "first": None, "last": None}


clock = _stat()
_sim = [0.0]
vel = {d: _stat() for d in range(6)}
lat = {d: _stat() for d in range(6)}
# P3 终端交接闸门：cpn_active 旗到达统计（A/B 前提校验——cpn 真持过 cruise）
# n=0 -> 无发布者（baseline 臂/节点没起来）；n>0 而 n_true=0 -> 旗恒 False
# （defop 臂静默失效=跑成基线）。duty_s=旗为 True 的 sim 时长积分（20Hz 采样
# 的一阶近似），与 clock 总时长之比即 cpn 持笔占比。
act = {d: _stat() for d in range(6)}
n_true = {d: 0 for d in range(6)}
duty = {d: 0.0 for d in range(6)}
_prev_s = {d: None for d in range(6)}
# 死端真值（2026-10-02 立案）：/drone_N/defop/debug 逐拍无节流发 JSON，mode 在
# 死端拍为 deadend_stop/deadend_rotate，超时强制放宽拍另有 forced=True。
# **零改 planner** 即可量到回合时长 —— 用来对拍影子档 replica 的 t_rot
# （对拍失配则 Δ 判据作废，见 docs/deadend_timeout_case.md）。
ddbg = {d: {"n": 0, "rot_s": 0.0, "stop_s": 0.0, "forced_s": 0.0,
            "ep_n": 0, "mode": None, "t": None,
            "ep_acc": 0.0, "hist": [0] * 12, "ep_max": 0.0, "tl": []}
        for d in range(6)}
# 逐拍时间线（Tier 2，2026-10-02 立案）：rosout 里 `[defop_cpn] drone_N veto=..
# safe=.. min_score=..` 是 loginfo_throttle，实测每机每格仅 14~19 条、最大间隔
# 46s ⇒ 撞前**瞬时**读数取不到，`defop_reactwin.py` 只能用"最近一条"定罪
# （故设 lag>5s 归"证据不足"）。/drone_N/defop/debug 逐拍发（2~5Hz），此处把
# **逐拍** mode/col_prob/veto/safe 落成紧凑数组，撞前滞后可压到 ~0.3s。
# 行 = [sim_t, cod, col_prob, veto_cnt, safe_cnt, forced]，
#   cod: 0=非死端 1=deadend_stop 2=deadend_rotate（forced 拍另由 forced 标）。
# 键**逐拍变化**且 col_prob 可为 None（defop_cpn_node.py:831/:840）⇒ 三点均容缺。
TL_MAX = 4000


def on_clock(m):
    w, s = time.time(), m.clock.to_sec()
    if clock["first"] is None:
        clock["first"] = (w, s)
    clock["last"] = (w, s)
    clock["n"] += 1
    _sim[0] = s


def _mk(bucket, d):
    def cb(_m):
        w, s = time.time(), _sim[0]
        b = bucket[d]
        if b["first"] is None:
            b["first"] = (w, s)
        b["last"] = (w, s)
        b["n"] += 1
    return cb


def _mk_act(d):
    def cb(m):
        w, s = time.time(), _sim[0]
        b = act[d]
        if b["first"] is None:
            b["first"] = (w, s)
        b["last"] = (w, s)
        b["n"] += 1
        if m.data:
            n_true[d] += 1
            ps = _prev_s[d]
            if ps is not None and s > ps:
                duty[d] += s - ps      # 上一拍旗为 True 的持续时长
        _prev_s[d] = s
    return cb


def _mk_ddbg(d):
    """死端真值：按 mode 的**前一条**归属把 sim 时长记入 rot/stop。
    上升沿（上拍非死端、本拍死端）= 一次回合开始，与 `DEADEND enter` 同义。

    2026-10-02 增：**逐回合时长直方图**（hist/ep_max）。这是死端回合时长的
    **权威源** —— 直接取自真机 mode 线，不经过 planner 的 replica
    （replica 曾因跳过拍充能而虚高一个量级，110544/00 d1 已法证）。
    replica 只作对拍旁证。"""
    def cb(m):
        s = _sim[0]
        r = ddbg[d]
        r["n"] += 1
        try:
            j = json.loads(m.data)
        except Exception:
            return
        md = j.get("mode")
        cp = j.get("col_prob")
        cod = 2 if md == "deadend_rotate" else (1 if md == "deadend_stop" else 0)
        tl = r["tl"]
        # 行 = [sim_t, cod, col_prob, veto, safe, forced,
        #       dnear, dnear2, dmin, near_az_deg, yaw_deg, vwx, vwy, n_indx1]
        # 前 6 列**位置不变**（defop_reactwin.py 按下标读，向后兼容）；
        # 后 8 列 = ③ 监督层可观测性（见 defop_cpn_node.py plan_once 遥测段）。
        _vw = j.get("vw") or [None, None]
        tl.append([round(s, 3), cod,
                   (round(float(cp), 3) if cp is not None else None),
                   j.get("veto_cnt"), j.get("safe_cnt"),
                   1 if j.get("forced") else 0,
                   j.get("dnear"), j.get("dnear2"), j.get("dmin"),
                   j.get("near_az_deg"), j.get("yaw_deg"),
                   _vw[0], _vw[1], j.get("n_indx1")])
        if len(tl) > TL_MAX:
            del tl[:len(tl) - TL_MAX]
        prev_md, prev_s = r["mode"], r["t"]
        gap = (s - prev_s) if (prev_s is not None and s > prev_s) else 0.0
        if prev_md in ("deadend_rotate", "deadend_stop") and gap > 0.0:
            if prev_md == "deadend_rotate":
                r["rot_s"] += gap
            else:
                r["stop_s"] += gap
        if bool(j.get("forced")) and gap > 0.0:
            r["forced_s"] += gap
        cur_dead = md in ("deadend_rotate", "deadend_stop")
        prev_dead = prev_md in ("deadend_rotate", "deadend_stop")
        if cur_dead and not prev_dead:
            r["ep_n"] += 1
            r["ep_acc"] = 0.0
        elif cur_dead:
            r["ep_acc"] += gap
        elif prev_dead:
            ep = r["ep_acc"]
            r["ep_acc"] = 0.0
            r["ep_max"] = max(r["ep_max"], ep)
            r["hist"][int(min(11, ep))] += 1
        r["mode"], r["t"] = md, s
    return cb


def dump():
    out = {"ts": time.strftime("%Y-%m-%d %H:%M:%S"), "clock": clock,
           "vel_cmd": vel, "latent_space": lat,
           "cpn_active": act, "cpn_true": n_true, "cpn_duty_s": duty,
           "defop_debug": ddbg}
    tmp = OUT + ".tmp"
    with open(tmp, "w") as f:
        json.dump(out, f, indent=1)
    os.replace(tmp, OUT)


def on_term(sig, frm):
    rospy.signal_shutdown("sigterm")


signal.signal(signal.SIGTERM, on_term)
rospy.init_node("defop_ab_probe", anonymous=True, disable_signals=True)
subs = [rospy.Subscriber("/clock", Clock, on_clock)]
for d in range(6):
    subs.append(rospy.Subscriber("/drone_%d/vel_cmd" % d, Twist, _mk(vel, d)))
    subs.append(rospy.Subscriber("/drone_%d/latent_space" % d,
                                 Float32MultiArray, _mk(lat, d)))
    subs.append(rospy.Subscriber("/drone_%d/planning/cpn_active" % d,
                                 Bool, _mk_act(d)))
    subs.append(rospy.Subscriber("/drone_%d/defop/debug" % d, String,
                                 _mk_ddbg(d)))
t0 = time.time()
try:
    while not rospy.is_shutdown() and time.time() - t0 < MAX_S:
        time.sleep(0.5)
finally:
    dump()
'''


def write_probe(outdir):
    p = os.path.join(outdir, "probe_node.py")
    with open(p, "w", encoding="utf-8") as f:
        f.write(PROBE_SRC)
    return p


# ---------------------------------------------------------------------------
# 单 cell 驱动脚本（仿 run_verify_gazebo.sh 机制 + 门序列，由工具生成）
# ---------------------------------------------------------------------------
DRIVER_TMPL = r"""#!/usr/bin/env bash
# defop_ab_run 单 cell 驱动（T4 生成）——机制=run_verify_gazebo.sh +
# gz_closure_run 门序列（P2_WAIT/scorekeeper 在列/start 服务在列/清场点名）。
# 全程 ROS_MASTER_URI=11411（本工具分配端口）；结束只清本脚本所拉进程树。
WS=/home/ubuntu/zx2026_defop_ws
CELL=__CELL__
BACKEND=__BACKEND__
ARMSPEC=__ARMSPEC__
PROBE=__PROBE__
PROBE_OUT=__PROBE_OUT__
VERIFY_TIMEOUT=__VERIFY_TIMEOUT_EFF__
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
# VAE 推理设备：defop.launch 的 $(optenv VAE_DEVICE cpu) 消费（6 机全跟随）。
export VAE_DEVICE=__VAE_DEVICE__
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models
PROBE_PID=""

cleanup() {
  # 只杀本脚本记名启动的 PID；11411/gz 残留点名后才补刀（孤儿=本单元所拉：
  # 发车前已点名零残留，故此处残留必属本单元）
  [ -f "$CELL/launch.pid" ] && kill "$(cat $CELL/launch.pid)" 2>/dev/null
  [ -f "$CELL/defop_lids" ] && while read -r p; do kill "$p" 2>/dev/null; done < "$CELL/defop_lids"
  [ -n "$PROBE_PID" ] && kill -TERM "$PROBE_PID" 2>/dev/null
  sleep 3
  pgrep -f "gzserver" >/dev/null 2>&1 && pkill -f gzserver 2>/dev/null
  pgrep -f "rosmaster.*11411" >/dev/null 2>&1 && pkill -f "rosmaster.*11411" 2>/dev/null
  pgrep -f "roscore.*11411" >/dev/null 2>&1 && pkill -f "roscore.*11411" 2>/dev/null
  sleep 2
  # 二轮 KILL 升级（gzserver 在 master 先死后会挂死忽略 TERM——本次实弹首犯）；
  # 发车前已点名零残留，此处残留必属本单元孤儿，-KILL 收割不涉他人
  pgrep -f "gzserver" >/dev/null 2>&1 && pkill -KILL -f gzserver 2>/dev/null
  pgrep -f "rosmaster.*11411" >/dev/null 2>&1 && pkill -KILL -f "rosmaster.*11411" 2>/dev/null
  pgrep -f "roscore.*11411" >/dev/null 2>&1 && pkill -KILL -f "roscore.*11411" 2>/dev/null
  sleep 1
}
trap cleanup EXIT
# 2026-09-29 矩阵首轮法证：只有 EXIT 陷阱时，外层 cell 预算 killpg SIGTERM
# 到组，bash 未必走到 EXIT（残留 rosmaster/gzserver 毒死其后 12 个 cell）。
# 显式 TERM/INT 陷阱 -> exit -> EXIT 陷阱 -> cleanup，清场必达。
trap 'exit 143' TERM INT

# exec-bit 自愈（124209 教训：UNC 编辑掉位 → roslaunch 静默 spawn 失败）
find $WS/src -name '*.py' -path '*/scripts/*' -exec chmod +x {} + 2>/dev/null

# 发车（官方判分后端 gazebo 优先；classic=run_verify.sh 机制回退）
if [ "$BACKEND" = "gazebo" ]; then
  nohup roslaunch arena_world_gazebo gazebo.launch > "$CELL/launch.log" 2>&1 &
else
  nohup roslaunch arena_world zx2026_all.launch > "$CELL/launch.log" 2>&1 &
fi
echo $! > "$CELL/launch.pid"

# 门0 master 就绪（2026-09-29 smoke 法证定谳：defop 臂 gazebo.launch+
# 6×defop.launch 七个 roslaunch 并发抢建 11411 master=创建竞态，败者
# "Unable to start XML-RPC server" bind 死场残——必须先等 master 起来
# 再发 defop，后续 defop.launch 即为 join 不再抢建）
for i in $(seq 1 120); do
  rostopic list >/dev/null 2>&1 && break; sleep 0.5
done
rostopic list >/dev/null 2>&1 || { echo "GATE_FAIL:master"; exit 3; }

# defop 臂：6 机推理链单独发车（gazebo.launch 未透传 defop_arm arg，CLI 附参
# 会报 unused args；单发 defop.launch×6 与 stack_nodes defop 组 include 等价）
if [ "$ARMSPEC" != "baseline" ]; then
  : > "$CELL/defop_lids"
  for d in 0 1 2 3 4 5; do
    nohup roslaunch defop_nav defop.launch drone_id:=$d >> "$CELL/defop_launch.log" 2>&1 &
    echo $! >> "$CELL/defop_lids"
  done
fi

# 门1 master/话题在列
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
rostopic list >/dev/null 2>&1 || { echo "GATE_FAIL:topic_list"; exit 3; }
# 门2 节点数>=20（stock 同款）
for i in $(seq 1 120); do
  N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 20 ] && break; sleep 0.5
done
[ "$(rosnode list 2>/dev/null | wc -l)" -ge 20 ] || { echo "GATE_FAIL:nodes20"; exit 3; }
# 门3 /clock（gzserver 就绪判据；缺席=gz 基建坏 → 退码 4）
if [ "$BACKEND" = "gazebo" ]; then
  for i in $(seq 1 240); do
    rostopic list 2>/dev/null | grep -q '^/clock' && break; sleep 0.5
  done
  rostopic list 2>/dev/null | grep -q '^/clock' || { echo "GATE_FAIL:clock"; exit 4; }
fi
# 门4 P2_WAIT（防 start 早调被拒；echo 带 timeout 防 latched 空转挂死）
ST=""
for i in $(seq 1 60); do
  ST=$(timeout -k 1 2 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"' | head -1)
  [ "$ST" = '"P2_WAIT"' ] && break; sleep 1
done
[ "$ST" = '"P2_WAIT"' ] || { echo "GATE_FAIL:p2wait"; exit 3; }
# 门5 scorekeeper 活体=其话题在列（不能等消息：latched 首发布前 echo 必空转）
for i in $(seq 1 10); do
  rostopic list 2>/dev/null | grep -q '^/zx2026/score/0$' && break; sleep 1
done
rostopic list 2>/dev/null | grep -q '^/zx2026/score/0$' || { echo "GATE_FAIL:scorekeeper"; exit 3; }
# 门6 start 服务在列（bkmfoi31i：服务未注册时调用无超时挂死）
for i in $(seq 1 30); do
  rosservice list 2>/dev/null | grep -q '^/zx2026/start$' && break; sleep 1
done
rosservice list 2>/dev/null | grep -q '^/zx2026/start$' || { echo "GATE_FAIL:start_svc"; exit 3; }
# 门7 defop 臂：latent_space 话题在列（vae 活体判据；话题在列≠有消息）
if [ "$ARMSPEC" != "baseline" ]; then
  for i in $(seq 1 20); do
    rostopic list 2>/dev/null | grep -q '^/drone_0/latent_space$' && break; sleep 1
  done
  rostopic list 2>/dev/null | grep -q '^/drone_0/latent_space$' || { echo "GATE_FAIL:latent"; exit 3; }
fi
echo "GATES_OK"

# 活体探针（RTF + vel_cmd/latent 频率；SIGTERM 落 JSON，cleanup 兜底补发）
python3 "$PROBE" "$PROBE_OUT" 2400 &
PROBE_PID=$!

# E2E 观测（stock verify_run.py：fleet_ready/P2_WAIT/start/观测窗/VERDICT）
timeout "$VERIFY_TIMEOUT" python3 $WS/verify_run.py
VRC=$?
[ "$VRC" = "124" ] && echo "VERIFY_TIMEOUT"

kill -TERM "$PROBE_PID" 2>/dev/null
sleep 3.5   # 封卷汇合窗（report_settle_s=1.0 race 修复）+ nav_metrics 落盘
# 设备活体回声（2026-10-01）：defop_vae_node 在 CUDA 不可用时是 warn 后静默回落
# cpu——「请求了 cuda:0 却跑在 cpu」与「GPU 无收益」在 CSV 上完全同形。故把各机
# 实际生效设备点出来：请求 cuda:0 时必须 6 个 device=cuda:0，否则本跑作废。
if [ "$ARMSPEC" != "baseline" ]; then
  echo "VAE_DEV_SUMMARY: $(grep -ao 'device=[a-z0-9:]*' "$CELL/defop_launch.log" 2>/dev/null | sort | uniq -c | tr '\n' ' ')"
fi
echo "DRIVER_DONE rc=$VRC"
exit 0
"""


def write_driver(outdir, arm, backend):
    p = os.path.join(outdir, "driver.sh")
    txt = DRIVER_TMPL
    txt = txt.replace("__CELL__", outdir)
    txt = txt.replace("__BACKEND__", backend)
    txt = txt.replace("__ARMSPEC__", arm)
    txt = txt.replace("__PROBE__", os.path.join(outdir, "probe_node.py"))
    txt = txt.replace("__PROBE_OUT__", os.path.join(outdir, "probe.json"))
    txt = txt.replace("__VERIFY_TIMEOUT__", str(int(VERIFY_TIMEOUT)))
    txt = txt.replace("__VERIFY_TIMEOUT_EFF__", str(int(VERIFY_TIMEOUT) + 240))
    txt = txt.replace("__VAE_DEVICE__", VAE_DEVICE)
    with open(p, "w", encoding="utf-8") as f:
        f.write(txt)
    os.chmod(p, os.stat(p).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return p


# ---------------------------------------------------------------------------
# 点名/清场
# ---------------------------------------------------------------------------
def wait_clean_slate(timeout_s=20.0, own=False):
    """清场点名：11411 栈与 gz 进程零残留才发车。

    own=False（批首 cell）：残留可能是别人的会话 → 不杀，返回 False 由调用方
    中止该 cell（严禁宽域 pkill）。
    own=True（批中后续 cell）：本批发车前已证零残留，此刻的栈签名残留必属
    本工具上一 cell 的孤儿（矩阵首轮预算 kill 后毒死 12 cell 的教训）→ PID
    点名清杀后复点。
    """
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        r = subprocess.run(["pgrep", "-f", CLEAN_PAT], capture_output=True)
        if r.returncode != 0:
            return True
        time.sleep(0.5)
    if own:
        scoped_cleanup_own("pre-launch")
        r = subprocess.run(["pgrep", "-f", CLEAN_PAT], capture_output=True)
        if r.returncode != 0:
            return True
    r = subprocess.run(["pgrep", "-af", CLEAN_PAT], capture_output=True,
                       text=True)
    print("    !! 清场点名失败，残留进程（不属本工具，不代杀）：\n%s"
          % (r.stdout or "").strip(), flush=True)
    return False


def confirm_zero_residue():
    """收尾自证：pgrep 复点，返回 (零残留?, 残留行)。"""
    r = subprocess.run(["pgrep", "-af", CLEAN_PAT], capture_output=True,
                       text=True)
    return r.returncode != 0, (r.stdout or "").strip()


def _ancestors():
    """自身进程树（含祖先）PID 集：PID 点名清杀时的自杀护栏。"""
    me = {os.getpid()}
    p = os.getppid()
    for _ in range(8):
        if p <= 1 or p in me:
            break
        me.add(p)
        try:
            with open("/proc/%d/stat" % p) as f:
                p = int(f.read().rsplit(") ", 1)[1].split()[1])
        except Exception:
            break
    return me


def scoped_cleanup_own(tag=""):
    """cell 收尾残留兜底（2026-09-29 矩阵首轮法证）：只清本栈签名进程。

    发车前 wait_clean_slate 已点名零残留，故此刻的栈签名残留必属本单元孤儿
    （预算 kill 后 driver 未及清场的情形）。仍**一律 PID 点名**（pgrep 列
    PID -> 排除自身进程树 -> os.kill 逐个），绝不宽域 pkill——命令行含模式
    词的进程（外层 bash -lc 等）天然免疫自杀事故。返回被清 PID 列表。
    """
    me = _ancestors()
    pids = []
    for pat in ("gzserver", r"rosmaster.*11411", r"roscore.*11411"):
        r = subprocess.run(["pgrep", "-f", pat], capture_output=True, text=True)
        for s in (r.stdout or "").split():
            try:
                pid = int(s)
            except ValueError:
                continue
            if pid not in me and pid not in pids:
                pids.append(pid)
    if not pids:
        return []
    print("    !! cell 残留点名（%s），PID 点名清杀: %s" % (tag, pids), flush=True)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    time.sleep(3)
    for pid in pids:
        try:
            os.kill(pid, 0)                 # 还在 -> KILL 升级
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(1)
    return pids


# ---------------------------------------------------------------------------
# 采集
# ---------------------------------------------------------------------------
def _parse_verify(path, t_since=0.0):
    """verify.txt → verdict/done_t/start_ok。**新鲜度门**（2026-09-29 矩阵首轮
    法证）：verify.txt 是 /tmp 定路径，cell 被预算砍时本轮从未落盘，裸读会
    把上一 cell 的 done_t 冒名计入（defop seed47 r0 报 done=185s 实为前一个
    baseline cell 的数）——非 fresh 一律判 stale 走缺省，与 _archive 同口径。
    """
    d = {"verdict": "FAIL", "done_t": -1, "start_ok": None, "stale": False}
    try:
        if t_since and os.path.getmtime(path) < t_since - 5.0:
            d["stale"] = True
            return d
        txt = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        d["stale"] = True
        return d
    if "VERDICT: PASS" in txt:
        d["verdict"] = "PASS"
    m = re.search(r"-- state -> DONE @ (\d+)s", txt)
    if m:
        d["done_t"] = int(m.group(1))
    m = re.search(r"start service: (\w+)", txt)
    if m:
        d["start_ok"] = m.group(1) == "True"
    return d


def _parse_score(t_since):
    """最新封卷 yaml（mtime 防旧 run 残留，verify_run 同口径）。"""
    cands = [f for f in glob.glob("/tmp/zx2026_score_*.yaml")
             if os.path.getmtime(f) >= t_since - 5.0]
    if not cands:
        return None
    try:
        return yaml.safe_load(open(max(cands, key=os.path.getmtime)))
    except Exception:
        return None


def _probe_summary(path):
    """probe.json → (vel_hz_mean, lat_hz_mean, latent_live, rtf)。
    hz = n/活跃窗（该话题首末消息墙钟差）；latent_live=有 latent 消息机数
    （baseline 臂无发布者 → 0，如实非异常）；rtf = 任务窗（各机 vel_cmd 首末
    Δsim/Δwall 中位数），无 vel 流则退 /clock 全窗。探针缺失 → -1（探针坏，
    与 0=话题静默区分）。"""
    if not os.path.exists(path):
        return -1.0, -1.0, 0, -1.0
    try:
        j = json.load(open(path))
    except Exception:
        return -1.0, -1.0, 0, -1.0

    def hzs(bucket):
        out = []
        for d in range(6):
            b = bucket.get(str(d)) or bucket.get(d) or {}
            n = b.get("n", 0)
            if n and b.get("first") and b.get("last"):
                w = b["last"][0] - b["first"][0]
                if w > 0.5:
                    out.append((n / w, b))
        return out

    vh = hzs(j.get("vel_cmd", {}))
    lh = hzs(j.get("latent_space", {}))
    vel_mean = round(sum(h for h, _ in vh) / len(vh), 2) if vh else -1.0
    lat_mean = round(sum(h for h, _ in lh) / len(lh), 2) if lh else 0.0
    latent_live = len(lh)
    rt = []
    for _, b in vh:
        w = b["last"][0] - b["first"][0]
        s = b["last"][1] - b["first"][1]
        if w > 0.5 and s > 0:
            rt.append(s / w)
    if not rt:
        c = j.get("clock", {})
        if c.get("first") and c.get("last"):
            w = c["last"][0] - c["first"][0]
            s = c["last"][1] - c["first"][1]
            if w > 1 and s > 0:
                rt.append(s / w)
    rtf = round(sorted(rt)[len(rt) // 2], 3) if rt else -1.0
    return vel_mean, lat_mean, latent_live, rtf


def _cpn_summary(path):
    """probe.json → (cpn_live, cpn_duty_s, cpn_ratio)：A/B 前提校验（cpn 真
    持过 cruise 吗——P3 终端交接闸门落地后新加，2026-09-29）。

    cpn_live  = 收到 /planning/cpn_active 旗的机数（0=无发布者：baseline 臂
                常态，或 nav 没起来）。
    cpn_duty_s= 旗为 True 的 sim 时长均值（20Hz 采样一阶近似；-1=探针坏）。
    cpn_ratio = 每机 duty/该机 vel_cmd 活跃窗（sim）的中位数。
    defop 臂健康 = live=6 且 ratio 显著 >0（出发窗/终端/返航窗口归 nav，
    cruise 占大头）；live>0 而 ratio≈0 = 旗恒 False（cpn 静默失效=跑成基线，
    defop.launch 联锁缺键的典型签名）。
    """
    if not os.path.exists(path):
        return 0, -1.0, -1.0
    try:
        j = json.load(open(path))
    except Exception:
        return 0, -1.0, -1.0
    act = j.get("cpn_active", {}) or {}
    duty = j.get("cpn_duty_s", {}) or {}
    velb = j.get("vel_cmd", {}) or {}

    def _g(bucket, d):
        return bucket.get(str(d), bucket.get(d))

    live = 0
    ds, ratios = [], []
    for d in range(6):
        b = _g(act, d) or {}
        if b.get("n", 0) > 0:
            live += 1
        v = _g(duty, d)
        if v is None:
            continue
        ds.append(float(v))
        vb = _g(velb, d) or {}
        if vb.get("first") and vb.get("last"):
            w = vb["last"][1] - vb["first"][1]
            if w > 1.0:
                ratios.append(float(v) / w)
    duty_mean = round(sum(ds) / len(ds), 1) if ds else -1.0
    ratio = round(sorted(ratios)[len(ratios) // 2], 3) if ratios else -1.0
    return live, duty_mean, ratio


def _archive(outdir, t0=0.0, seed=None):
    """verify/smoke/最新 nav_metrics/封卷 score/rosout+碰撞行 留档。"""
    for src, dst in ((VERIFY_TXT, "verify.txt"), (SMOKE_LOG, "smoke.log")):
        # D2：/tmp 定路径盲拷会带入上一 run 残留（smoke 实锤冒名），
        # 非 fresh 一律不拷=诚实缺失
        try:
            if os.path.getmtime(src) >= t0 - 5.0:
                shutil.copy(src, os.path.join(outdir, dst))
        except OSError:
            pass
    ymls = sorted(glob.glob(WS + "/run_logs/nav_metrics_2*.yaml"),
                  key=os.path.getmtime)
    # verify minor：新鲜度门+seed 对质——收集器未落盘/上一 run 残留时
    # 宁缺勿错（返回空=调用方记 metrics_stale），防冒名归因
    ymls = [q for q in ymls if os.path.getmtime(q) >= t0 - 5.0]
    if ymls and seed is not None:
        _hit = None
        for q in reversed(ymls):
            try:
                with open(q) as _f:
                    _meta = yaml.safe_load(_f) or {}
                if int(_meta.get("run_seed", -1)) == int(seed):
                    _hit = q
                    break
            except Exception:
                continue
        ymls = [_hit] if _hit else []
    if ymls:
        try:
            shutil.copy(ymls[0], os.path.join(outdir, "nav_metrics.yaml"))
        except OSError:
            pass
    scores = glob.glob("/tmp/zx2026_score_*.yaml")
    if scores:
        try:
            shutil.copy(max(scores, key=os.path.getmtime),
                        os.path.join(outdir, "score.yaml"))
        except OSError:
            pass
    rosout = os.path.expanduser("~/.ros/log/latest/rosout.log")
    if os.path.exists(rosout):
        try:
            shutil.copy(rosout, os.path.join(outdir, "rosout.log"))
            with open(os.path.join(outdir, "rosout.log"),
                      errors="replace") as f:
                hits = [ln.rstrip() for ln in f if "COLLISION #" in ln]
            with open(os.path.join(outdir, "collisions.txt"), "w",
                      encoding="utf-8") as f:
                f.write("\n".join(hits) + "\n")
        except OSError:
            pass
    return ymls


def _run_once(cell, outdir, backend):
    """跑一个 cell（备份→补丁→驱动→采集→还原）；返回行 dict（gate=内部键）。
    异常安全：finally 还原 sim_settings.yaml 且 filecmp 校验 diff 为空。"""
    arm, seed, rep = cell["arm"], cell["seed"], cell["run"]
    os.makedirs(outdir, exist_ok=True)
    oom0 = oom_count()
    bak = os.path.join(outdir, "sim_settings.yaml.bak")
    shutil.copy(CFG, bak)                       # 先备份
    nbak = os.path.join(outdir, "defop_nav.yaml.bak")
    shutil.copy(NAVCFG, nbak)                   # 候选臂的 CPN 配置同备份
    notes, gate = [], 0
    timed_out = False
    so, dt = "", 0.0
    t0 = time.time()
    m0 = time.monotonic()
    try:
        patch_cell(arm, seed)                   # 后补丁
        driver = write_driver(outdir, arm, backend)
        write_probe(outdir)
        # start_new_session=独立进程组：超时 killpg 一网打尽 roslaunch/rosmaster/
        # gzserver 全树（只清本 cell 所拉，不宽域）
        proc = subprocess.Popen(["bash", driver], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True,
                                start_new_session=True)
        try:
            so, _ = proc.communicate(timeout=CELL_BUDGET_S)
        except subprocess.TimeoutExpired:
            timed_out = True
            notes.append("cell_budget_kill")
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                time.sleep(3)
                os.killpg(proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                so = proc.communicate(timeout=10)[0] or ""
            except Exception:
                pass
        dt = time.time() - t0
    finally:
        shutil.copy(bak, CFG)                   # 后还原
        if not filecmp.cmp(bak, CFG, shallow=False):
            shutil.copy(bak, CFG)               # 重试一次
            time.sleep(0.2)
            if not filecmp.cmp(bak, CFG, shallow=False):
                notes.append("RESTORE_MISMATCH")
        shutil.copy(nbak, NAVCFG)               # 候选臂 CPN 配置同还原
        if not filecmp.cmp(nbak, NAVCFG, shallow=False):
            shutil.copy(nbak, NAVCFG)
            time.sleep(0.2)
            if not filecmp.cmp(nbak, NAVCFG, shallow=False):
                notes.append("RESTORE_MISMATCH_NAV")
        try:
            os.remove(bak)
        except OSError:
            pass
        try:
            os.remove(nbak)
        except OSError:
            pass

    drift = dt - (time.monotonic() - m0)        # wall-monotonic 漂移（睡眠/断档）
    # cell 收尾兜底：driver 清场未达（预算 kill 等）时的栈签名残留 PID 点名
    # 清杀——防一个坏 cell 毒死其后全部 cell（矩阵首轮 12 cell 连坐教训）
    ok_end, resid_end = confirm_zero_residue()
    if not ok_end:
        scoped_cleanup_own(os.path.basename(outdir))
        ok_end2, resid_end2 = confirm_zero_residue()
        if not ok_end2:
            notes.append("residue_after_cell")
            print("    !! cell 收尾仍有残留（不属本工具签名，留观）:\n%s"
                  % resid_end2, flush=True)
    if "GATE_FAIL:" in so:
        m = re.search(r"GATE_FAIL:(\w+)", so)
        gname = m.group(1) if m else "?"
        gate = EXIT_GZBROKE if gname == "clock" else EXIT_GATE
        notes.append("gate_fail:" + gname)
    if "VERIFY_TIMEOUT" in so:
        timed_out = True

    row = dict(arm=arm, seed=seed, run=rep,
               ts=time.strftime("%Y-%m-%d %H:%M:%S"), backend=backend,
               verdict="FAIL", score=-1, landed=-1, retired_n=-1, col=-1,
               minclr=-1, done_t=-1, rtf=-1.0, vel_cmd_hz=-1.0, latent_hz=0.0,
               latent_live=0, cpn_live=0, cpn_duty_s=-1.0, cpn_ratio=-1.0,
               oom_hits=-1, runtime_s=round(dt),
               sleep_s=round(drift, 1), timeout=timed_out,
               note="|".join(n for n in notes if n), cell_dir=outdir,
               gate=gate)

    vd = _parse_verify(VERIFY_TXT, t0)
    row["verdict"] = vd["verdict"]
    row["done_t"] = vd["done_t"]
    if vd.get("stale"):
        notes.append("verify_stale")
    if vd["start_ok"] is False:
        notes.append("start_rejected")
    sy = _parse_score(t0)
    if sy:
        t = sy.get("total")
        if isinstance(t, (int, float)):
            row["score"] = int(t)
        if isinstance(sy.get("landed_n"), int):
            row["landed"] = sy["landed_n"]
        # 退赛名单（retired 非空=比赛口径 FAIL 直接成因，A/B 归因要害）
        try:
            row["retired_n"] = len(sy.get("retired") or [])
        except Exception:
            pass
    ymls = _archive(outdir, t0, seed)
    if ymls:
        try:
            row.update(parse_metrics(ymls[-1]))
        except Exception as e:
            print("    metrics parse fail:", e, flush=True)
    else:
        notes.append("metrics_stale")
    vel_hz, lat_hz, lat_live, rtf = _probe_summary(
        os.path.join(outdir, "probe.json"))
    row["vel_cmd_hz"], row["latent_hz"] = vel_hz, lat_hz
    row["latent_live"], row["rtf"] = lat_live, rtf
    cl, cd, cr = _cpn_summary(os.path.join(outdir, "probe.json"))
    row["cpn_live"], row["cpn_duty_s"], row["cpn_ratio"] = cl, cd, cr
    if arm != "baseline" and cl > 0 and cr >= 0 and cr < 0.05:
        notes.append("cpn_flag_dead")     # 旗恒 False=cpn 静默失效跑成基线

    oom1 = oom_count()
    if oom0 >= 0 and oom1 >= 0:
        row["oom_hits"] = max(0, oom1 - oom0)
        if row["oom_hits"] > 0:
            oom_snapshot(os.path.join(outdir, "oom.txt"))
            notes.append("oom+%d" % row["oom_hits"])
    row["note"] = "|".join(n for n in notes if n)
    return row


def run_cell(cell, outroot, idx, backend_request, rerun_on_sleep=True):
    """跑一个 cell：点名→_run_once；gz 基建坏 auto 回退 classic；污染重跑一次。"""
    tag = "%02d_%s_seed%d_r%d" % (idx, cell["arm"], cell["seed"], cell["run"])
    outdir = os.path.join(outroot, tag)
    print("[%s] backend=%s ..." % (tag, backend_request), flush=True)

    if not wait_clean_slate(own=(idx > 0)):
        return dict(arm=cell["arm"], seed=cell["seed"], run=cell["run"],
                    ts=time.strftime("%Y-%m-%d %H:%M:%S"),
                    backend="gazebo" if backend_request != "classic"
                    else "classic",
                    verdict="ABORT", score=-1, landed=-1, col=-1, minclr=-1,
                    done_t=-1, rtf=-1.0, vel_cmd_hz=-1.0, latent_hz=0.0,
                    latent_live=0, cpn_live=0, cpn_duty_s=-1.0, cpn_ratio=-1.0,
                    oom_hits=-1, runtime_s=0, sleep_s=0.0,
                    timeout=False, note="pre_launch_residual_procs",
                    cell_dir="-", gate=0)

    backend = "gazebo" if backend_request != "classic" else "classic"
    row = _run_once(cell, outdir, backend)

    # gz 基建坏（/clock 缺席）→ auto 档回退 classic 机制，gzbroke 目录留档
    if row.get("gate") == EXIT_GZBROKE and backend_request == "auto":
        print("    !! /clock 缺席=gz 基建坏 —— 回退 run_verify.sh 机制"
              "（classic），summary 将声明", flush=True)
        FALLBACK["n"] += 1
        try:
            os.rename(outdir, outdir + "__gzbroke")
        except OSError:
            pass
        backend = "classic"
        row = _run_once(cell, outdir, backend)
        row["note"] = (row["note"] + "|gz_fallback").strip("|")

    # 污染重跑（genval 律：墙钟漂移>30s=睡眠/断档；或无 total 非 timeout=挂死）
    contaminated = (row["sleep_s"] > 30.0
                    or (row["score"] == -1 and not row["timeout"]
                        and row.get("gate", 0) != EXIT_GZBROKE))
    if contaminated and rerun_on_sleep:
        why = ("wall-monotonic 漂移 %.0fs" % row["sleep_s"]
               if row["sleep_s"] > 30.0 else "verify 无 total（挂死/门失败）")
        print("    !! %s —— cell 污染：留档 __contaminated，自动重跑" % why,
              flush=True)
        try:
            os.rename(outdir, outdir + "__contaminated")
        except OSError:
            pass
        row = _run_once(cell, outdir, backend)
    return row


# ---------------------------------------------------------------------------
# dry-run
# ---------------------------------------------------------------------------
def dry_run(cells, out_csv, backend_request):
    """只打印将做的 patch 与将跑的命令；不动任何文件不启进程。"""
    with open(CFG, encoding="utf-8") as f:
        raw = f.read()
    try:
        s = yaml.safe_load(raw) or {}
    except Exception:
        s = {}
    has_planner = bool(re.search(r"^planner\s*:", raw, re.M))
    cur_src = (s.get("planner") or {}).get("source")
    cur_dep = ((s.get("sensor") or {}).get("depth") or {}).get("enabled")
    cur_seed = s.get("run_seed")
    outroot = os.path.join(WS, "run_logs",
                           "defop_ab_" + time.strftime("%Y%m%d_%H%M%S"))
    print("==== DRY-RUN（不动任何文件、不启任何进程） ====")
    print("backend 请求: %s（auto=gazebo 官方判分后端优先；/clock 门失败=gz "
          "基建坏 → 回退 classic 并在 summary 声明）" % backend_request)
    print("CSV 追加: %s（缺文件先写列头）" % out_csv)
    print("归档根: %s" % outroot)
    print("采集: verify.txt(VERDICT/DONE@Ns)/封卷 yaml(total,landed_n)/"
          "nav_metrics(Σcol,minclr)/探针(/drone_*/vel_cmd、/drone_*/latent_space"
          " @%s)/OOM(dmesg grep -ic oom 前后差)" % MASTER)
    for k, c in enumerate(cells):
        arm = c["arm"]
        is_defop = arm != "baseline"
        want_src = "defop" if is_defop else "internal"
        want_dep = "true" if is_defop else "false"
        print("\n[cell %d/%d] %02d_%s_seed%d_r%d  backend=%s"
              % (k + 1, len(cells), k, arm, c["seed"], c["run"],
                 "gazebo" if backend_request in ("auto", "gazebo")
                 else "classic"))
        print("  patch run_seed: %s -> %s" % (cur_seed, c["seed"]))
        if has_planner:
            print("  patch planner.source: %s -> %s（planner 块内改值）"
                  % (cur_src, want_src))
        else:
            print("  patch planner.source: (现配置缺 planner 段) -> %s"
                  "（文件尾新增顶级段；T2 消费方 vel_source_of 键缺失默认"
                  " internal，故零行为变更）" % want_src)
        print("  patch sensor.depth.enabled: %s -> %s%s"
              % (cur_dep, want_dep,
                 "  # 仅 defop 臂" if want_dep == "true"
                 else "  # baseline=默认值，逐字节不变"))
        nk = ARM_KEYS.get(arm) or {}
        if nk:
            print("  patch defop_nav.yaml: %s"
                  % ", ".join("%s=%s" % (k, v) for k, v in nk.items()))
        else:
            print("  patch defop_nav.yaml: (无，逐字节不变)")
        if is_defop:
            print("  cmd: bash <cell>/driver.sh —— roslaunch arena_world_gazebo"
                  " gazebo.launch + roslaunch defop_nav defop.launch "
                  "drone_id:=0..5")
        else:
            print("  cmd: bash <cell>/driver.sh —— 机制=run_verify_gazebo.sh "
                  "逐字节复刻（baseline 默认栈零改动）")
        print("  门: rostopic list → nodes>=20 → /clock(gz) → P2_WAIT → "
              "score/0 在列 → /zx2026/start 在列%s"
              % (" → latent_space 在列" if is_defop else ""))
        print("  采后: probe.json/launch.log/verify/score/metrics/rosout 归档；"
              "finally 还原 sim_settings.yaml（备份回拷+filecmp 校验 diff 空）")
    print("\nDRY-RUN 结束（共 %d cells）" % len(cells))


# ---------------------------------------------------------------------------
# CSV / main
# ---------------------------------------------------------------------------
def _csv_append(path, row):
    """追加式 CSV（文件缺/空先写列头；csv 模块防 note 含逗号破列）。"""
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    need_header = not (os.path.exists(path) and os.path.getsize(path) > 0)
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        if need_header:
            w.writerow(COLS)
        w.writerow([row.get(c, "") for c in COLS])


def main():
    ap = argparse.ArgumentParser(description="DeFoP A/B 跑批器（T4）")
    ap.add_argument("--seeds", default="43", help="逗号分隔，如 43,46,47")
    ap.add_argument("--runs", type=int, default=1, help="每 seed 重复遍数")
    ap.add_argument("--arms", default="baseline,defop",
                    help="逗号分隔，baseline 与/或 defop")
    ap.add_argument("--out", default="results/defop_ab.csv",
                    help="CSV 路径（相对路径锚在 WS 根）")
    ap.add_argument("--backend", default="auto",
                    choices=["auto", "gazebo", "classic"],
                    help="auto=gazebo 优先，gz 基建坏回退 classic")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印补丁与命令，不动文件不启进程")
    ap.add_argument("--no-rerun", action="store_true",
                    help="关闭污染自动重跑（默认开）")
    ap.add_argument("--vae-device", default="cpu",
                    choices=["cpu", "cuda:0", "auto"],
                    help="VAE 推理设备（2026-10-01 GPU 加速立项）；"
                         "经 defop.launch $(optenv VAE_DEVICE) 施加到全部 6 机")
    args = ap.parse_args()

    global VAE_DEVICE
    VAE_DEVICE = args.vae_device

    seeds = [int(x) for x in str(args.seeds).split(",") if x.strip()]
    arms = [a.strip() for a in str(args.arms).split(",") if a.strip()]
    for a in arms:
        if a not in ARM_KEYS:
            ap.error("未知臂 %r（%s）" % (a, "|".join(ARM_KEYS)))
    out_csv = args.out if os.path.isabs(args.out) else os.path.join(WS, args.out)

    # 臂交错（同 seed/run 内 A/B 相邻，防时段漂移混淆臂间差异——matrix_run 同律）
    cells = []
    for rep in range(args.runs):
        for seed in seeds:
            for arm in arms:
                cells.append(dict(arm=arm, seed=seed, run=rep))

    if args.dry_run:
        dry_run(cells, out_csv, args.backend)
        return

    outroot = os.path.join(WS, "run_logs",
                           "defop_ab_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outroot, exist_ok=True)
    print("defop_ab start -> %s\ncsv -> %s\ncells=%d backend=%s"
          % (outroot, out_csv, len(cells), args.backend), flush=True)

    rows = []
    for k, cell in enumerate(cells):
        try:
            row = run_cell(cell, outroot, k, args.backend,
                           rerun_on_sleep=not args.no_rerun)
        except Exception as e:            # 单 cell 崩溃不拖垮整批（如实记行）
            row = dict(arm=cell["arm"], seed=cell["seed"], run=cell["run"],
                       ts=time.strftime("%Y-%m-%d %H:%M:%S"), backend="?",
                       verdict="ERROR", score=-1, landed=-1, col=-1,
                       minclr=-1, done_t=-1, rtf=-1.0, vel_cmd_hz=-1.0,
                       latent_hz=0.0, latent_live=0, cpn_live=0,
                       cpn_duty_s=-1.0, cpn_ratio=-1.0, oom_hits=-1,
                       runtime_s=0, sleep_s=0.0, timeout=False,
                       note="exc:" + repr(e)[:80], cell_dir="-", gate=0)
        rows.append(row)
        _csv_append(out_csv, row)
        print("    -> %s score=%s landed=%s col=%s minclr=%s done=%ss rtf=%s "
              "vel=%s lat=%s(%d) cpn=%s/%ss(%d机) (%.0fs) %s"
              % (row["verdict"], row["score"], row["landed"], row["col"],
                 row["minclr"], row["done_t"], row["rtf"], row["vel_cmd_hz"],
                 row["latent_hz"], row["latent_live"], row["cpn_ratio"],
                 row["cpn_duty_s"], row["cpn_live"], row["runtime_s"],
                 row["note"]), flush=True)

    ok, resid = confirm_zero_residue()
    print("\n残留点名: %s" % ("零残留" if ok else "有残留:\n" + resid),
          flush=True)

    print("\n===== SUMMARY (mean per arm) =====", flush=True)
    if FALLBACK["n"]:
        print("!! gz 基建坏回退 classic（run_verify.sh 机制）%d 次 —— 本次"
              "结果非纯官方 gz 后端口径" % FALLBACK["n"], flush=True)

    def _mean(armr, key):
        vals = [r[key] for r in armr
                if isinstance(r.get(key), (int, float)) and r[key] >= 0]
        return (sum(vals) / len(vals), len(vals)) if vals else (-1.0, 0)

    for arm in arms:
        armr = [r for r in rows if r["arm"] == arm
                and r["verdict"] not in ("ABORT", "ERROR")]
        if not armr:
            print("%-8s 无有效行" % arm, flush=True)
            continue
        n = len(armr)
        pv = sum(1 for r in armr if r["verdict"] == "PASS")
        sc, _ = _mean(armr, "score")
        ld, _ = _mean(armr, "landed")
        co, _ = _mean(armr, "col")
        dn, _ = _mean(armr, "done_t")
        rt, _ = _mean(armr, "rtf")
        vh, _ = _mean(armr, "vel_cmd_hz")
        lh, _ = _mean(armr, "latent_hz")
        cl, _ = _mean(armr, "cpn_live")
        cr, _ = _mean(armr, "cpn_ratio")
        print("%-8s n=%d pass=%d score=%.1f landed=%.1f col=%.1f done=%.0fs "
              "rtf=%.2f vel_hz=%.1f lat_hz=%.1f cpn_live=%.1f cpn_ratio=%.2f"
              % (arm, n, pv, sc, ld, co, dn, rt, vh, lh, cl, cr), flush=True)
    oomt = sum(r["oom_hits"] for r in rows if r["oom_hits"] > 0)
    print("OOM 新增命中: %d\ncsv: %s" % (oomt, out_csv), flush=True)


if __name__ == "__main__":
    main()
