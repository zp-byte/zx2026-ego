#!/usr/bin/env bash
# 启动 gzserver(无头) + gzclient(图形窗口)，均 setsid 分离，立即返回
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_defop_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; sleep 2

setsid nohup gzserver /home/ubuntu/zx2026_defop_ws/src/arena_world_gazebo/worlds/forest_world.world \
  >/tmp/gz_srv_only.log 2>&1 < /dev/null &
setsid nohup gzclient >/tmp/gzclient_live.log 2>&1 < /dev/null &

sleep 8
echo "gzserver: $(pgrep -f gzserver >/dev/null && echo RUNNING || echo dead)"
echo "gzclient: $(pgrep -f gzclient >/dev/null && echo RUNNING || echo dead)"
echo "== started detached =="
