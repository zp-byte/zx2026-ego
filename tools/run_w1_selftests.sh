#!/bin/bash
# W1 全量 selftest 套件（新 + 存量回归）
# 2026-10-07：路径从 ~/zx2026_arena_ws 改到 ego_ws——旧路径会让本套件在**别仓**上跑，
# 回归变成空转（改了 nav_node 也测不到）。新增 heading（机头对齐）入列。
source /opt/ros/noetic/setup.bash
source ~/zx2026_ego_ws/devel/setup.bash
cd ~/zx2026_ego_ws
for t in w1_swarm via_slots avoid dam de nstop p4 plan rescue veto heading; do
  echo "== $t =="
  python3 "tools/${t}_selftest.py" 2>&1 | tail -4
done
