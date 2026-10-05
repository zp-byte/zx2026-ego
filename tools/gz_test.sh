#!/usr/bin/env bash
# 自灭式测试 gzclient：timeout 60s 自动结束，日志写到 /tmp/gztest.log，立即返回
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_FORCE_SOFTWARE_GL=1
export GALLIUM_DRIVER=llvmpipe
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -f gzclient 2>/dev/null; sleep 1
setsid nohup timeout -k 5 60 gzclient >/tmp/gztest.log 2>&1 < /dev/null &
echo "launched (rc captured in log)"
exit 0
