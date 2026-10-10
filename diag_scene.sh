#!/bin/bash
# 诊断 Gazebo 无场景：用用户实际路径 roslaunch gazebo.launch 复现，查模型插入数与加载错误
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

# 干净启动
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; pkill -9 -f rviz 2>/dev/null
pkill -9 -f "rosmaster.*11411" 2>/dev/null; sleep 1

# 用用户实际路径启动（headless，不带 viewer，避免 rviz/gzclient 干扰）
setsid roslaunch arena_world_gazebo gazebo.launch \
  >/tmp/zx2026_diag_launch.log 2>&1 < /dev/null &
echo "roslaunch gazebo.launch 启动，等待 30s..."

for i in $(seq 1 30); do sleep 1; done

echo "=== 进程 ==="
pgrep -f "gzserver" >/dev/null && echo "gzserver UP" || echo "gzserver DOWN"
echo "节点数: $(rosnode list 2>/dev/null | wc -l)"

echo
echo "=== model_states 模型数（应为 ~111） ==="
timeout 5 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep -c "  name:" || echo "无 /gazebo/model_states 话题"

echo
echo "=== 模型名字（前 20） ==="
timeout 5 rostopic echo -n1 /gazebo/model_states 2>/dev/null | grep "  name:" | head -20

echo
echo "=== launch 日志：错误行 ==="
grep -aE "Error|error|Unable|unable|Failed|failed|not found|Exception|SpawnEntity|terminate called" /tmp/zx2026_diag_launch.log | grep -av "registerPublisher\|XmlRpcClient\|doConnect" | head -40 || echo "无关键错误"

echo
echo "=== launch 日志尾部 20 行 ==="
tail -20 /tmp/zx2026_diag_launch.log

# 收尾
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; pkill -9 -f rviz 2>/dev/null
pkill -9 -f "rosmaster.*11411" 2>/dev/null; pkill -9 -f "roslaunch arena_world_gazebo" 2>/dev/null
echo "=== diag done ==="
