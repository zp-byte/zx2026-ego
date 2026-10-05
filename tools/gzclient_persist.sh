#!/usr/bin/env bash
# 分离启动 gzserver + gzclient（不设 timeout，窗口保持），立即截图分析
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; sleep 2
setsid nohup gzserver /home/ubuntu/zx2026_defop_ws/src/arena_world_gazebo/worlds/forest_world.world >/tmp/gzs_p.log 2>&1 < /dev/null &
sleep 12
setsid nohup gzclient >/tmp/gzc_p.log 2>&1 < /dev/null &

sleep 20
echo "gzserver: $(pgrep -f 'gzserver /' >/dev/null && echo UP || echo down)"
echo "gzclient: $(pgrep -f 'gzclient$' >/dev/null && echo UP || echo down)"
WID=$(xdotool search --name gazebo 2>/dev/null | tail -1)
echo "gazebo wid: $WID"
if [ -n "$WID" ]; then
  xdotool getwindowgeometry "$WID" 2>/dev/null
  import -window "$WID" /tmp/gzclient_p.png 2>&1 | head -2
  identify /tmp/gzclient_p.png 2>/dev/null
fi
exit 0
