#!/usr/bin/env bash
# 用 WSLg 原生 D3D12 GL（不强制软件渲染）测试 gzclient 是否出画面
export DISPLAY=:0
unset LIBGL_ALWAYS_SOFTWARE
unset QT_X11_FORCE_SOFTWARE_GL
unset GALLIUM_DRIVER
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null; sleep 2
setsid nohup gzserver /home/ubuntu/zx2026_defop_ws/src/arena_world_gazebo/worlds/forest_world.world >/tmp/gzs_n.log 2>&1 < /dev/null &
sleep 12

# gzclient 后台自灭式（45s），同一脚本内截图
setsid nohup timeout -k 3 45 gzclient >/tmp/gzc_n.log 2>&1 < /dev/null &
sleep 22

echo "gzclient: $(pgrep -f 'gzclient$' >/dev/null && echo UP || echo down)"
WID=$(xdotool search --name gazebo 2>/dev/null | tail -1)
echo "gazebo wid: $WID"
if [ -n "$WID" ]; then
  xdotool getwindowgeometry "$WID" 2>/dev/null | grep -E 'Geometry|Position'
  import -window "$WID" /tmp/gz_native.png 2>/tmp/gz_native_shot.log
  identify /tmp/gz_native.png 2>/dev/null
fi
echo "gzc log:"; tail -8 /tmp/gzc_n.log 2>/dev/null
exit 0
