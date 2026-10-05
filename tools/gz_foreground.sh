#!/usr/bin/env bash
# 前台跑 gzclient 25s，捕获全部输出/崩溃栈
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

echo "=== gzserver running? $(pgrep -f 'gzserver /' | wc -l) ==="
echo "=== gzclient foreground 25s ==="
timeout -k 3 25 gzclient 2>&1
echo "=== rc=$? ==="
