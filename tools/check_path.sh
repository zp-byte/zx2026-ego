#!/usr/bin/env bash
# 验证插件发布的 /drone_<id>/path（Path 轨迹）数据
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_defop_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; sleep 2
setsid nohup roslaunch arena_world_gazebo gazebo.launch >/tmp/gz_path.log 2>&1 < /dev/null &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
sleep 25

echo "=== /drone_0/path topic present? ==="
timeout 4 rostopic list 2>/dev/null | grep -c "/drone_0/path"

echo "=== path message (first 22 lines) ==="
timeout 4 rostopic echo -n1 /drone_0/path 2>/dev/null | head -22

echo "=== pose bullet count ==="
timeout 4 rostopic echo -n1 /drone_0/path 2>/dev/null | grep -cE "^ +- *$"

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null
echo "== PATH CHECK DONE =="
