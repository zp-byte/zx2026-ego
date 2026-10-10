#!/usr/bin/env bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash
find /home/ubuntu/zx2026_ego_ws/src -name '*.py' -exec chmod +x {} +

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "zx2026_all.launch" 2>/dev/null; sleep 1
rm -f /tmp/zx2026_smoke.log
nohup roslaunch arena_world zx2026_all.launch >/tmp/zx2026_smoke.log 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 90); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 20 ] && break; sleep 0.5; done
for i in $(seq 1 40); do
  S=$(timeout 2 rostopic echo -n 1 /zx2026/state 2>/dev/null | grep data: | tr -d ' "')
  [ "$S" = "P2_WAIT" ] && break
  sleep 0.5
done
rosservice call /zx2026/start "{}" >/dev/null 2>&1
echo "started, observing 40s..."
sleep 40
echo "== COLLISION EVENTS (world_node) =="
grep -E 'COLLIDED' /tmp/zx2026_smoke.log
echo "== drone positions =="
for i in 0 1 2 3 4 5; do
  P=$(timeout 2 rostopic echo -n 1 /drone_$i/odom 2>/dev/null | grep -E 'x: |y: |z: ' | head -3 | tr '\n' ' ')
  C=$(timeout 2 rostopic echo -n 1 /drone_$i/collision 2>/dev/null | grep data: | tr -d ' ')
  echo "  drone_$i: $P | collision=$C"
done
kill $LAUNCH_PID 2>/dev/null; pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f "roscore.*11411" 2>/dev/null
sleep 1
echo "== DIAG DONE =="
