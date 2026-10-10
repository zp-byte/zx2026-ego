#!/bin/bash
# 第二轮诊断：全量看 gazebo 话题、完整日志、Gazebo 自身模型列表
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
export OGRE_RTT_MODE=Copy
export GAZEBO_MODEL_PATH=/opt/ros/noetic/share/gazebo-11/models

pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; pkill -9 -f rviz 2>/dev/null
pkill -9 -f "rosmaster.*11411" 2>/dev/null; sleep 1

setsid roslaunch arena_world_gazebo gazebo.launch \
  >/tmp/zx2026_diag2_launch.log 2>&1 < /dev/null &
echo "启动中，等待 40s..."
for i in $(seq 1 40); do sleep 1; done

echo "=== 完整 launch 日志（前 40 行） ==="
head -40 /tmp/zx2026_diag2_launch.log

echo
echo "=== 完整 launch 日志（尾部 30 行） ==="
tail -30 /tmp/zx2026_diag2_launch.log

echo
echo "=== 所有 gazebo 相关话题 ==="
rostopic list 2>/dev/null | grep -i gazebo

echo
echo "=== /clock 是否在走 ==="
timeout 3 rostopic echo -n1 /clock 2>/dev/null | grep -o "secs: [0-9]*" | head -1

echo
echo "=== gz 命令模型列表 ==="
gz model --list 2>/dev/null | head -20 || echo "gz model 命令不可用"

pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; pkill -9 -f rviz 2>/dev/null
pkill -9 -f "rosmaster.*11411" 2>/dev/null; pkill -9 -f "roslaunch arena_world_gazebo" 2>/dev/null
echo "=== diag2 done ==="
