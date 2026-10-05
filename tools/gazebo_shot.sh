#!/usr/bin/env bash
# Gazebo 后端 + RViz 截图：启动 gazebo.launch viewer:=true → start → 截图 /tmp/zx2026_gazebo_rviz.png
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_defop_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

find /home/ubuntu/zx2026_defop_ws/src -name "*.py" -exec chmod +x {} +
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 2

nohup roslaunch arena_world_gazebo gazebo.launch viewer:=true >/tmp/zx2026_gz_viz.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 150); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 25 ] && break; sleep 0.5; done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"
echo "rviz nodes: $(rosnode list 2>/dev/null | grep -c rviz)"

# 等 P2_WAIT 后 start，让无人机起飞进入画面
for i in $(seq 1 60); do
  ST=$(timeout 2 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
  [ "$ST" = '"P2_WAIT"' ] && break; sleep 1
done
rosservice call /zx2026/start >/dev/null 2>&1 && echo "start called (state was $ST)"

sleep 20

WID=$(DISPLAY=:0 xdotool search --class rviz 2>/dev/null | tail -1)
[ -z "$WID" ] && WID=$(DISPLAY=:0 xdotool search --name "RViz" 2>/dev/null | tail -1)
echo "rviz window id: $WID"
if [ -n "$WID" ]; then
  DISPLAY=:0 import -window "$WID" /tmp/zx2026_gazebo_rviz.png 2>/tmp/zx2026_gz_shot.log \
    && echo "screenshot OK $(du -h /tmp/zx2026_gazebo_rviz.png | cut -f1)" || cat /tmp/zx2026_gz_shot.log
else
  echo "no rviz window; root fallback"
  DISPLAY=:0 import -window root /tmp/zx2026_gazebo_rviz.png 2>&1 | head -2
fi

echo "--- drone0 position (sanity) ---"
timeout 3 rostopic echo -n1 /drone_0/odom 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' '; echo
echo "--- marker count published ---"
timeout 3 rostopic echo -n1 /zx2026/markers 2>/dev/null | grep -c "ns:"

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f rviz 2>/dev/null
echo "== GAZEBO SHOT DONE =="
