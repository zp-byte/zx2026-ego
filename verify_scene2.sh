#!/bin/bash
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
  >/tmp/zx2026_verify2_launch.log 2>&1 < /dev/null &
echo "启动，等待 35s..."
for i in $(seq 1 35); do sleep 1; done

echo "=== 精确模型计数（Python） ==="
python3 /home/ubuntu/zx2026_ego_ws/count_models.py

pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; pkill -9 -f rviz 2>/dev/null
pkill -9 -f "rosmaster.*11411" 2>/dev/null; pkill -9 -f "roslaunch arena_world_gazebo" 2>/dev/null
echo "=== verify2 done ==="
