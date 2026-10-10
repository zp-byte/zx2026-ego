#!/bin/bash
# 遍历所有 gazebo 日志，统计 static 警告数 / 是否完成 API 插件加载 / 是否卡在等 world 服务
echo "dir          static  APIloaded  waitingSvc  size"
for f in /home/ubuntu/.ros/log/*/gazebo-*.log; do
  [ -f "$f" ] || continue
  d=$(dirname "$f")
  s=$(grep -ac "static.*not defined" "$f" 2>/dev/null)
  fin=$(grep -ac "Finished loading Gazebo ROS API" "$f" 2>/dev/null)
  ms=$(grep -ac "set_physics_properties.*has not been" "$f" 2>/dev/null)
  sz=$(stat -c%s "$f" 2>/dev/null)
  echo "$(basename "$d" | cut -c1-12)  $s  $fin  $ms  $sz"
done
