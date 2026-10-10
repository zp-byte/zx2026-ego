#!/usr/bin/env bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash
find /home/ubuntu/zx2026_ego_ws/src -name '*.py' -exec chmod +x {} +

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "zx2026_all.launch" 2>/dev/null; sleep 1
nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_smoke.log 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 90); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 20 ] && break; sleep 0.5; done
echo "nodes: $(rosnode list 2>/dev/null | wc -l)"

# 等 P2_WAIT → start
for i in $(seq 1 40); do
  S=$(timeout 2 rostopic echo -n 1 /zx2026/state 2>/dev/null | grep data: | tr -d ' "')
  [ "$S" = "P2_WAIT" ] && break
  sleep 0.5
done
rosservice call /zx2026/start "{}" >/dev/null 2>&1
echo "started. waiting 25s for drones to reach forest edge..."
sleep 25

echo "== nav processes alive =="
rosnode ping -n 1 nav_node_0 2>&1 | tail -1
echo "== collision flags (latched) =="
for i in 0 1 2 3 4 5; do
  V=$(timeout 2 rostopic echo -n 1 /drone_$i/collision 2>/dev/null | grep data: | tr -d ' ')
  echo "  drone_$i collision=$V"
done
echo "== drone positions now =="
for i in 0 1 2; do
  P=$(timeout 2 rostopic echo -n 1 /drone_$i/odom 2>/dev/null | grep -E 'x: |y: |z: ' | head -3 | tr '\n' ' ')
  echo "  drone_$i: $P"
done
echo "== vel_cmd stream (drone_0, 3 msgs) =="
timeout 3 rostopic echo -n 3 /drone_0/vel_cmd 2>/dev/null | grep -E 'x: |y: |z: ' | head -9
echo "== planning goal (drone_0, latched) =="
timeout 2 rostopic echo -n 1 /drone_0/planning/goal 2>/dev/null | grep -E 'x: |y: |z: ' | head -3

kill $LAUNCH_PID 2>/dev/null; pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "roscore.*11411" 2>/dev/null
sleep 1
echo "== DIAG DONE =="
