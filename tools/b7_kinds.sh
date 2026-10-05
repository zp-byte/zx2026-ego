#!/usr/bin/env bash
# 碰撞种类 / 高度带 / 位置分布 — 最近几批横向对质。
cd /home/ubuntu/zx2026_defop_ws || exit 9
for B in "$@"; do
  echo "===== $B ====="
  n=$(grep -h 'COLLISION #' run_logs/$B/*/collisions.txt 2>/dev/null | wc -l)
  echo "  总碰撞 = $n"
  grep -h 'COLLISION #' run_logs/$B/*/collisions.txt 2>/dev/null \
    | grep -o 'kind=[a-z]*' | sort | uniq -c | sed 's/^/    /'
  echo -n "  z 带: "
  grep -h 'COLLISION #' run_logs/$B/*/collisions.txt 2>/dev/null \
    | grep -o 'at ([^)]*)' | sed 's/.*,\([0-9.]*\))/\1/' | sort -n | tr '\n' ' '
  echo
  echo -n "  机: "
  grep -h 'COLLISION #' run_logs/$B/*/collisions.txt 2>/dev/null \
    | grep -o 'drone [0-9]' | sort | uniq -c | tr '\n' ' '
  echo
  echo "  位置:"
  grep -h 'COLLISION #' run_logs/$B/*/collisions.txt 2>/dev/null \
    | grep -o 'at ([^,]*,[^,]*,[^)]*)' | sed 's/^/    /'
done
