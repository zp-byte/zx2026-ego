#!/usr/bin/env bash
# GUI 会话诊断：rviz/gazebo/栈节点日志中的 warn/error + 话题健康
cd /home/ubuntu/zx2026_ego_ws || exit 1
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1

SESS=$(ls -td /home/ubuntu/.ros/log/*/ | head -1)
echo "session: $SESS"
echo "=== rviz log ($(wc -c < "${SESS}rviz"*.log 2>/dev/null) bytes):"
cat "${SESS}"rviz*.log 2>/dev/null | tail -15
echo "=== gazebo log ($(wc -c < "${SESS}"gazebo-2.log 2>/dev/null) bytes):"
grep -iE "warn|error" "${SESS}"gazebo-2.log 2>/dev/null | tail -15
echo "=== stack stdout WARN lines (unique):"
grep -h "WARN" "${SESS}"*-stdout.log 2>/dev/null | sed 's/\[.*\]//' | sort | uniq -c | sort -rn | head -15
echo "=== node count / clock / markers:"
echo "nodes=$(rosnode list 2>/dev/null | wc -l) clock=$(rostopic list 2>/dev/null | grep -c '^/clock')"
timeout 4 rostopic hz /zx2026/markers 2>/dev/null | head -2
echo "== DIAG DONE =="
