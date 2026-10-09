#!/usr/bin/env bash
# gzserver + gzclient 都自灭式启动，日志分文件，立即返回
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; sleep 2
setsid nohup timeout -k 5 120 gzserver /home/ubuntu/zx2026_arena_ws/src/arena_world_gazebo/worlds/forest_world.world >/tmp/gzs.log 2>&1 < /dev/null &
sleep 15
setsid nohup timeout -k 5 90 gzclient >/tmp/gzc.log 2>&1 < /dev/null &
sleep 5
echo "gzserver: $(pgrep -f 'gzserver /' >/dev/null && echo UP || echo down) | gzclient: $(pgrep -f 'gzclient$' >/dev/null && echo UP || echo down)"
exit 0
