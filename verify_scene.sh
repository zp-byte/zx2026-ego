#!/bin/bash
# 最终验证：模型数量 + 是否含树/无人机/围栏
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
  >/tmp/zx2026_verify_launch.log 2>&1 < /dev/null &
echo "启动，等待 35s..."
for i in $(seq 1 35); do sleep 1; done

echo "=== model_states 模型总数 ==="
timeout 6 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -c "name:"

echo
echo "=== 模型名分类统计 ==="
timeout 6 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep "name:" | sed 's/.*name: *//' | sed -E 's/_[0-9]+$//' | sort | uniq -c | sort -rn

echo
echo "=== /clock ==="
timeout 3 rostopic echo -n1 /clock 2>/dev/null | grep -o "secs: [0-9]*" | head -1

echo
echo "=== 是否有 static 属性告警残留（应=0） ==="
grep -ac "static.*not defined" /tmp/zx2026_verify_launch.log

pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; pkill -9 -f rviz 2>/dev/null
pkill -9 -f "rosmaster.*11411" 2>/dev/null; pkill -9 -f "roslaunch arena_world_gazebo" 2>/dev/null
echo "=== verify done ==="
