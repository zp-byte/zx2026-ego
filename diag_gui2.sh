#!/usr/bin/env bash
# v2: 字面路径读 rviz/gazebo 日志 + 栈节点 warn 统计
cd /home/ubuntu/zx2026_ego_ws || exit 1
S=/home/ubuntu/.ros/log/7c7728d4-a1f7-11f1-8e43-00155d8bdb99
echo "=== dir exists:"; ls -d "$S" 2>&1
echo "=== rviz/gazebo log files:"; ls -la "$S" | grep -E "rviz|gazebo"
echo "=== rviz log content (last 20):"
cat "$S"/rviz*.log 2>/dev/null | tail -20
echo "=== gazebo log content (last 20):"
cat "$S"/gazebo*.log 2>/dev/null | tail -20
echo "=== gazebo warn/error only:"
grep -iE "warn|error" "$S"/gazebo*.log 2>/dev/null | tail -20
echo "== DONE =="
