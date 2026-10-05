#!/usr/bin/env bash
# P4 余量（对照臂/处理臂）。⚠ 不信 CSV 的 timeout 列（对 250s 截断无感知）。
cd /home/ubuntu/zx2026_defop_ws || exit 9
for c in b5_20261002 b6_20261002; do
  echo "---- $c ----"
  python3 /tmp/p4calc.py "results/$c.csv" 2>&1 | tail -10
done
