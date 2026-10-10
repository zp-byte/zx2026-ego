#!/bin/bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash

echo "=== 总分 / 摘要 ==="
timeout 4 rostopic echo -n1 /zx2026/score_summary 2>/dev/null

echo
echo "=== 各机 分数 ==="
for i in 0 1 2 3 4 5; do
  s=$(timeout 3 rostopic echo -n1 /zx2026/score/$i 2>/dev/null | grep -oE "(total|score|points|phase):? ?[0-9.]+" | tr '\n' ' ')
  echo "drone_$i : $s"
done

echo
echo "=== 各机 类型匹配结果 ==="
for i in 0 1 2 3 4 5; do
  r=$(timeout 3 rostopic echo -n1 /drone_$i/match/result 2>/dev/null | grep -oE "(match|ok|result|type|correct):? ?[A-Za-z0-9_]*" | tr '\n' ' ')
  echo "drone_$i : $r"
done

echo
echo "=== 各机 投放完成 ==="
for i in 0 1 2 3 4 5; do
  d=$(timeout 3 rostopic echo -n1 /drone_$i/payload/done 2>/dev/null | grep -oE "data: (True|False)" | tr '\n' ' ')
  echo "drone_$i : $d"
done

echo
echo "=== 各机 最终位置 ==="
for i in 0 1 2 3 4 5; do
  p=$(timeout 3 rostopic echo -n1 /drone_$i/odom 2>/dev/null | grep -oE "position: ?\n? *x: -?[0-9.]+|x: -?[0-9.]+|y: -?[0-9.]+|z: -?[0-9.]+" | tr '\n' ' ')
  echo "drone_$i : $p"
done
