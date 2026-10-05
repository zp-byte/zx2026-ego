#!/usr/bin/env bash
# 残留勘察 + 清理（只动真残留，不 pkill -f 自杀）。先报后清。
cd /home/ubuntu/zx2026_defop_ws || exit 9
echo "=== 谁在跑（排除本脚本自身与 bash -lc 包装）==="
ps -eo pid,ppid,etimes,stat,comm,args --no-headers \
  | grep -Ev 'grep|b3_residue|ps -eo' \
  | grep -E 'gzserver|gzclient|rosmaster|roslaunch|rospy|defop|nav_node|mission_executor|depth_sim|probe_node|vae_node' \
  || echo "  （无）"
echo
echo "=== gzserver 详情（若有）==="
for p in $(pgrep -x gzserver); do
  echo "  pid=$p etimes=$(ps -o etimes= -p $p | tr -d ' ')s ppid=$(ps -o ppid= -p $p | tr -d ' ')"
  echo "  cwd=$(readlink /proc/$p/cwd 2>/dev/null)"
  tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null; echo
done
echo
echo "=== 端口 11411 / 11311 ==="
ss -ltnp 2>/dev/null | grep -E '11411|11311' || echo "  （无监听）"
