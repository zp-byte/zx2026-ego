#!/usr/bin/env bash
# 检查最近一次 roslaunch 的节点日志：重点 nav / executor / type_match / scorekeeper
LATEST=$(ls -td /home/ubuntu/.ros/log/*/ 2>/dev/null | grep -vE 'rostopic|rosclean|install' | head -1 | xargs -n1 basename)
echo "logdir: $LATEST"
for ptn in nav_node_0 executor_0 type_match_node scorekeeper; do
  echo "===== $ptn ====="
  ls /home/ubuntu/.ros/log/$LATEST/$ptn-*.log 2>/dev/null | while read f; do
    echo "--- $f ---"
    tail -25 "$f"
  done
done
echo "===== launch 概要 ====="
grep -iE "nav_node|executor|type_match|scorekeeper|MATCH|TIMEOUT|FAILED|goal|collision" /tmp/zx2026_smoke.log | grep -vE "Traceback|Error" | tail -40
