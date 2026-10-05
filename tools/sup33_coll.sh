#!/usr/bin/env bash
# 列出某批各格的碰撞原文（drone/kind/位置）。
B=${1:-run_logs/defop_ab_20261002_135937}
cd /home/ubuntu/zx2026_defop_ws || exit 9
for d in "$B"/*/; do
  c=$(basename "$d")
  n=$(grep -c 'COLLISION #' "$d/collisions.txt" 2>/dev/null || echo 0)
  echo "=== $c  碰撞=$n ==="
  grep -o 'drone [0-9] COLLISION #[0-9]* kind=[a-z]* at ([^)]*)' \
       "$d/collisions.txt" 2>/dev/null
done
