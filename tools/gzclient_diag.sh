#!/usr/bin/env bash
# 诊断 gzclient：gzserver 先起，再前台跑 gzclient 抓输出（带超时）
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

# 只启 gzserver（不跑任务栈，专注渲染）
setsid nohup gzserver /home/ubuntu/zx2026_defop_ws/src/arena_world_gazebo/worlds/forest_world.world \
  >/tmp/gz_srv_only.log 2>&1 < /dev/null &
sleep 20
echo "gzserver alive: $(pgrep -f gzserver >/dev/null && echo yes || echo no)"
echo "gzserver log tail:"; tail -3 /tmp/gz_srv_only.log

echo "=== run gzclient (timeout 90s) ==="
timeout 90 gzclient >/tmp/gzclient_diag.log 2>&1
RC=$?
echo "gzclient exit: $RC"
echo "=== gzclient log tail ==="
tail -40 /tmp/gzclient_diag.log

echo "=== X windows ==="
DISPLAY=:0 xdotool search --class gazebo 2>/dev/null | head -2
DISPLAY=:0 xdotool search --name "gzclient" 2>/dev/null | head -2

pkill -f "rosmaster.*11411" 2>/dev/null; pkill -f gzserver 2>/dev/null
echo "== GZCLIENT DIAG DONE =="
