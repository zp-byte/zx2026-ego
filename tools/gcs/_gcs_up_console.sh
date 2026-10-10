#!/bin/bash
# _gcs_up_console.sh — GCS agent+hub+面板 控制台宿主（2026-10-09）
# 前置：_gz_up_console.sh 控制台已起（gz 后端全栈）。本脚本自带等栈活循环。
# agent/hub 宿主=本终端会话（nohup+孤儿化），面板前台——关窗=GCS 停。
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source "$HOME/zx2026_ego_ws/devel/setup.bash"
ROOT="$HOME/zx2026_ego_ws"

# 等栈活（最多 120s）：节点数 ≥30 且 /zx2026/start service 在
N=0; S=""
for i in $(seq 1 120); do
  N=$(rosnode list 2>/dev/null | wc -l)
  if timeout 3 rosservice info /zx2026/start >/dev/null 2>&1; then S=ok; break; fi
  sleep 1
done
echo "[gcs_up] nodes=$N start_service=${S:-MISSING}"

nohup python3 "$ROOT/tools/gcs/gcs_agent.py" \
  --profile "$ROOT/tools/gcs/profile_sim.yaml" > /tmp/gcs_agent_stack.log 2>&1 &
disown
sleep 4
nohup python3 "$ROOT/tools/gcs/gcs_hub.py" --ids 0,1,2,3,4,5 --port 9870 \
  > /tmp/gcs_hub_stack.log 2>&1 &
disown
sleep 8

python3 - <<'PY'
import json, os, time
p = "/home/ubuntu/zx2026_arena_ws/run_logs/gcs_status.json"
try:
    age = time.time() - os.stat(p).st_mtime
    d = json.load(open(p))
    print("[gcs_up] snap age %.1fs stage=%s linked=%s" % (age, d.get("stage"), d.get("linked")))
except Exception as e:
    print("[gcs_up] status not ready:", e)
PY
echo "[gcs_up] panel starting (close this console = GCS agent/hub/panel down)"
exec python3 "$ROOT/tools/gcs/gcs_panel.py" --profile "$ROOT/tools/gcs/profile_sim.yaml"
