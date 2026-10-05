#!/usr/bin/env bash
# Gazebo 后端冒烟测试：gzserver(forest_world) + 全栈 → 检查插件/odom → start → 观测阶段
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_defop_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

find /home/ubuntu/zx2026_defop_ws/src -name "*.py" -exec chmod +x {} +
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 2

nohup roslaunch arena_world_gazebo gazebo.launch >/tmp/zx2026_gz_stack.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 120); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 25 ] && break; sleep 0.5; done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"
echo "gz nodes: $(rosnode list 2>/dev/null | grep -c gazebo)"
echo "--- plugin loads (should be 6) ---"
grep -c "DroneVelPlugin\[drone_" /tmp/zx2026_gz_stack.log
echo "--- world loaded? ---"
grep -E "Loading model|loaded.*forest|gzserver.*started" /tmp/zx2026_gz_stack.log | head -3
echo "--- odom publishers ---"
timeout 8 rostopic echo -n1 /drone_0/odom 2>/dev/null | grep -A2 position | head -4

echo "--- start ---"
rosservice call /zx2026/start 2>&1
echo "--- observe 60s (every 10s) ---"
for k in $(seq 1 6); do
  sleep 10
  PH=$(timeout 3 rostopic echo -n1 /drone_0/mission/phase 2>/dev/null | grep data)
  P0=$(timeout 3 rostopic echo -n1 /drone_0/odom 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' ')
  echo "t+$((k*10))s drone0 phase=$PH pos=$P0"
done

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null
echo "== GAZEBO TEST DONE =="
