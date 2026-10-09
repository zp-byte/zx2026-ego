#!/usr/bin/env bash
# Gazebo + RViz 大图截图：放大 rviz 窗口 → start → 截图（相机聚焦森林）
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_arena_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f rviz 2>/dev/null; sleep 2

nohup roslaunch arena_world_gazebo gazebo.launch viewer:=true >/tmp/zx2026_gz_viz2.log 2>&1 &
for i in $(seq 1 60); do rostopic list >/dev/null 2>&1 && break; sleep 0.5; done
for i in $(seq 1 150); do N=$(rosnode list 2>/dev/null | wc -l); [ "$N" -ge 25 ] && break; sleep 0.5; done

# 放大 rviz 窗口
sleep 8
WID=$(DISPLAY=:0 xdotool search --class rviz 2>/dev/null | tail -1)
[ -z "$WID" ] && WID=$(DISPLAY=:0 xdotool search --name "RViz" 2>/dev/null | tail -1)
echo "rviz window id: $WID"
if [ -n "$WID" ]; then
  DISPLAY=:0 xdotool windowsize "$WID" 1280 800
  DISPLAY=:0 xdotool windowmove "$WID" 60 40
  sleep 3
fi

# start 后让无人机飞一会再拍（进入森林中央）
for i in $(seq 1 60); do
  ST=$(timeout 2 rostopic echo -n1 /zx2026/state 2>/dev/null | grep -o '"P[0-9A-Z_]*"')
  [ "$ST" = '"P2_WAIT"' ] && break; sleep 1
done
rosservice call /zx2026/start >/dev/null 2>&1
sleep 45   # 无人机应已深入林区

# 微调相机：拉近到森林中央（yaw/pitch 由用户自己动，这里用 RViz 默认）
if [ -n "$WID" ]; then
  DISPLAY=:0 import -window "$WID" /tmp/zx2026_gazebo_rviz_big.png 2>/tmp/zx2026_gz_shot2.log \
    && echo "screenshot OK $(du -h /tmp/zx2026_gazebo_rviz_big.png | cut -f1)" || cat /tmp/zx2026_gz_shot2.log
else
  DISPLAY=:0 import -window root /tmp/zx2026_gazebo_rviz_big.png 2>&1 | head -2
fi

echo "--- drone0 pos ---"
timeout 3 rostopic echo -n1 /drone_0/odom 2>/dev/null | grep -E "x:|y:" | head -2 | tr '\n' ' '; echo
echo "--- all drones done? ---"
for i in 0 1 2 3 4 5; do timeout 2 rostopic echo -n1 /drone_$i/mission/phase 2>/dev/null | grep -o 'data: "[A-Z_]*"' | head -1; done

cp /tmp/zx2026_gazebo_rviz_big.png /mnt/c/Users/24882/Desktop/资料/gazebo_rviz_big.png 2>/dev/null && echo "copied to Desktop"
pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f rviz 2>/dev/null
echo "== GAZEBO SHOT2 DONE =="
