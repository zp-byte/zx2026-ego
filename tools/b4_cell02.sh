#!/usr/bin/env bash
# 批 2 vs 批 4 的 cell02 细节对拍（含全批卡滞计数）。
cd /home/ubuntu/zx2026_defop_ws || exit 9
for B in defop_ab_20261002_121827 defop_ab_20261002_132328; do
  C=run_logs/$B/02_defop_seed43_r2
  echo "===================== $B / 02 ====================="
  echo "--- score.yaml ---"
  head -20 "$C/score.yaml" 2>/dev/null
  echo "--- 事件计数（STALL/STUCK/LOW_SLACK/UNAUTHORIZED/RETIRE） ---"
  for k in STALL STUCK LOW_SLACK UNAUTHORIZED RETIRE; do
    printf "  %-14s %s\n" "$k" "$(grep -c "$k" "$C/rosout.log" 2>/dev/null)"
  done
  echo "--- state 时序 ---"
  grep -E "^-- state ->" "$C/verify.txt" 2>/dev/null
  echo "--- 碰撞 ---"
  grep -c "COLLISION #" "$C/collisions.txt" 2>/dev/null
done
echo
echo "===================== 全批卡滞（stuck）====================="
for B in defop_ab_20261002_121827 defop_ab_20261002_132328; do
  echo "--- $B ---"
  for c in run_logs/$B/*/; do
    printf "  %-24s STALL=%-4s STUCK=%-4s LOW_SLACK=%-4s RETIRE=%s\n" "$(basename "$c")" \
      "$(grep -c STALL "$c/rosout.log" 2>/dev/null)" \
      "$(grep -c STUCK "$c/rosout.log" 2>/dev/null)" \
      "$(grep -c LOW_SLACK "$c/rosout.log" 2>/dev/null)" \
      "$(grep -c RETIRE "$c/rosout.log" 2>/dev/null)"
  done
done
