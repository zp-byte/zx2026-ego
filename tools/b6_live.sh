#!/usr/bin/env bash
# 批 6 活体检查：新门是否真的咬合（防"跑了但机制没开"的空转臂）。
# 权威源 = ~/.ros/log/<uuid>/ 下的 node 日志（stdout 有缓冲坑；rosout 才是活体证据）。
B=${1:-run_logs/defop_ab_20261002_143210}
cd /home/ubuntu/zx2026_defop_ws || exit 9

echo "=== 格目录 ==="
ls -d "$B"/*/ 2>/dev/null | sed 's|.*/||'

echo
echo "=== 跑批残留/进行中进程 ==="
ps -eo etime,args --no-headers \
  | grep -E "defop_cpn_node|gzserver|rosmaster|nav_node|miss" | grep -v grep \
  | awk '{print "  " $1 "  " $2 " " $3}' | head -12

L=$(ls -dt /home/ubuntu/.ros/log/*/ 2>/dev/null | head -1)
echo
echo "=== 最新 ROS logdir: $L ==="
if [ -z "$L" ]; then
  echo "  （还没有 ROS 日志目录）"
  exit 0
fi

echo "--- 启动回声（配置是否真到达节点；静默误解析的唯一对质）---"
grep -h "deadend: max_s" "$L"*.log 2>/dev/null | head -3 | sed 's/^/  /'
grep -h "sup_bearing_mode=" "$L"*.log 2>/dev/null | head -2 | sed 's/^/  /'

echo "--- DEADEND enter（新门咬合的触发证据）---"
tot=$(grep -h "DEADEND enter" "$L"*.log 2>/dev/null | wc -l)
vf=$(grep -h "DEADEND enter" "$L"*.log 2>/dev/null | grep -c "vf=")
echo "  总数=$tot  其中带 vf= 的=$vf"
grep -ho "DEADEND enter ([^)]*)" "$L"*.log 2>/dev/null | head -12 | sed 's/^/    /'

echo "--- forced-relax ---"
grep -h "DEADEND forced-relax" "$L"*.log 2>/dev/null | wc -l | sed 's/^/  /'

echo "--- 碰撞（若已开始）---"
for d in "$B"/*/; do
  col=$(grep -c "COLLISION #" "$d"collisions.txt 2>/dev/null || true)
  echo "  $(basename "$d")  碰撞=${col:-0}"
done
