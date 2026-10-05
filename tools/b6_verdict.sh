#!/usr/bin/env bash
# 批 6 判词四件套：P4 余量 / DE 实测 / 机制占比 / 触发咬合 vs 真碰撞 对质。
cd /home/ubuntu/zx2026_defop_ws || exit 9
B5=run_logs/defop_ab_20261002_135937
B6=run_logs/defop_ab_20261002_143210

echo "########## 1) 归因四件套 ##########"
for B in $B5 $B6; do
  echo "---- $B ----"
  python3 tools/defop_collision_forensics.py "$B" 2>&1 | tail -25
done

echo
echo "########## 2) 机制占比（reactwin）##########"
for B in $B5 $B6; do
  echo "---- $B ----"
  python3 tools/defop_reactwin.py "$B" 2>&1 | tail -20
done

echo
echo "########## 3) DEADEND enter / forced-relax 逐格（真机 mode 线）##########"
for B in $B5 $B6; do
  echo "---- $B ----"
  for d in "$B"/*/; do
    c=$(basename "$d"); c=${c%%_*}
    # 各格独立的 rosout 归档
    n=$(grep -h "DEADEND enter" "$d"*.log 2>/dev/null | wc -l)
    f=$(grep -h "DEADEND forced-relax" "$d"*.log 2>/dev/null | wc -l)
    echo "  cell$c  enter=$n  forced=$f"
  done
done

echo
echo "########## 4) P4 余量 ##########"
if [ -f /tmp/p4calc.py ]; then
  for B in $B5 $B6; do
    echo "---- $B ----"
    python3 /tmp/p4calc.py "$B" 2>&1 | tail -12
  done
else
  echo "  （/tmp/p4calc.py 不在，跳过）"
fi
