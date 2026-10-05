#!/usr/bin/env bash
# 批 6 逐格死端/落地对质：把 enter 按 vf 分档，看新增的咬合是"挡真障碍"
# 还是"误锁"（vf 高但其实无路可走 / 短回合反复进出）。
B=${1:-run_logs/defop_ab_20261002_143210}
cd /home/ubuntu/zx2026_defop_ws || exit 9
L=$(ls -dt /home/ubuntu/.ros/log/*/ 2>/dev/null | head -1)
echo "logdir=$L"
echo "=== 全部 DEADEND enter：按 vf 分档 ==="
grep -ho "DEADEND enter (min_score=[-0-9.]*, safe=[0-9]*, dir=[-0-9]*, vf=[0-9.]*)" \
     "$L"*.log 2>/dev/null | sed 's/.*vf=//; s/)$//' | sort -n | uniq -c
echo
echo "=== vf=0.000（旧门也能进）的 min_score 分布 ==="
grep -ho "DEADEND enter (min_score=[-0-9.]*, safe=[0-9]*, dir=[-0-9]*, vf=0.000)" \
     "$L"*.log 2>/dev/null | wc -l
echo
echo "=== vf>0 的（**只有新门能进**）逐条 ==="
grep -ho "DEADEND enter (min_score=[-0-9.]*, safe=[0-9]*, dir=[-0-9]*, vf=[0-9.]*)" \
     "$L"*.log 2>/dev/null | awk -F'vf=' '$2+0 > 0' | head -40
echo
echo "=== forced-relax 次数 ==="
grep -h "DEADEND forced-relax" "$L"*.log 2>/dev/null | wc -l
echo
echo "=== 逐格结果 ==="
for d in "$B"/*/; do
  echo "  $(basename "$d")  col=$(grep -c 'COLLISION #' "$d"collisions.txt 2>/dev/null || echo 0)"
done
