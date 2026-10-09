#!/usr/bin/env bash
# 持久启动 gzserver + gzclient（D3D12 原生 GL，不强制软件渲染，不设 timeout）
# 窗口保持直到手动关闭；用于让用户直接查看 Gazebo 画面
export DISPLAY=:0
unset LIBGL_ALWAYS_SOFTWARE
unset QT_X11_FORCE_SOFTWARE_GL
unset GALLIUM_DRIVER
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; sleep 2
setsid nohup gzserver /home/ubuntu/zx2026_arena_ws/src/arena_world_gazebo/worlds/forest_world.world >/tmp/gzs_n.log 2>&1 < /dev/null &
sleep 15
setsid nohup gzclient >/tmp/gzc_n.log 2>&1 < /dev/null &
sleep 12

echo "gzserver: $(pgrep -f 'gzserver /' >/dev/null && echo UP || echo down)"
echo "gzclient: $(pgrep -f 'gzclient$' >/dev/null && echo UP || echo down)"
echo "GL: $(glxinfo 2>/dev/null | grep -E 'OpenGL renderer|OpenGL version' | head -2)"
WID=$(xdotool search --name gazebo 2>/dev/null | tail -1)
echo "gazebo wid: $WID"
if [ -n "$WID" ]; then xdotool getwindowgeometry "$WID" 2>/dev/null | grep -E 'Geometry|Position'; fi
echo "gzc log tail:"; tail -5 /tmp/gzc_n.log 2>/dev/null
exit 0
