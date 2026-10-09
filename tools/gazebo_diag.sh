#!/usr/bin/env bash
# 诊断：启动 gazebo.launch → 检查 odom/ready/state
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; sleep 2

nohup roslaunch arena_world_gazebo gazebo.launch >/tmp/gz_diag.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 120); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 25 ] && break; sleep 0.5; done

echo "=== nodes up ==="
rosnode list 2>/dev/null | wc -l

echo "=== odom topics ==="
timeout 5 rostopic list 2>/dev/null | grep odom

echo "=== per-drone odom (3s each) ==="
for i in 0 1 2 3 4 5; do
  C=$(timeout 3 rostopic echo -n1 /drone_$i/odom 2>/dev/null | grep -c "position")
  POS=$(timeout 3 rostopic echo -n1 /drone_$i/odom 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' ')
  echo "drone$i frames=$C pos=$POS"
done

echo "=== fleet_ready ==="
timeout 3 rostopic echo -n1 /zx2026/fleet_ready 2>/dev/null | grep data
echo "=== state ==="
timeout 3 rostopic echo -n1 /zx2026/state 2>/dev/null | grep data

echo "=== sim time /clock ==="
timeout 3 rostopic echo -n1 /clock 2>/dev/null | grep -E "secs|nsecs" | head -2

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null
echo "== DIAG DONE =="
