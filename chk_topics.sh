#!/usr/bin/env bash
# 检查 RViz 各显示依赖话题的发布频率
cd /home/ubuntu/zx2026_ego_ws || exit 1
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
for t in /zx2026/markers /drone_0/odom /drone_0/path /drone_0/occ_grid \
         /drone_0/vel_cmd /drone_0/mission/phase /tf /clock /drone_0/cloud; do
  echo "== $t:"
  timeout 3 rostopic hz "$t" 2>&1 | head -1
done
echo "== DONE =="
