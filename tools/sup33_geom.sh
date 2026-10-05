#!/usr/bin/env bash
# ③ 几何法证：碰撞原文（含 dir/距离）+ 该机撞前 6s 的位姿/速度（verify.txt 1Hz）。
cd /home/ubuntu/zx2026_defop_ws || exit 9
B=${1:-run_logs/defop_ab_20261002_132328}
C=$2
D=$3
T=$4
echo "=== 该格碰撞原文（全字段）==="
grep "COLLISION #" "$B/$C/collisions.txt" 2>/dev/null | head -10
echo
echo "=== 碰撞监视器 rosout 原文（前 12 行）==="
grep -iE "COLLISION|碰撞" "$B/$C/rosout.log" 2>/dev/null | head -12
echo
echo "=== drone$D 撞前 8s 位姿（verify.txt）==="
awk -v D="$D" -v T="$T" '
  match($0, /t=([0-9]+) drone([0-9]+)=\(([-0-9.]+), ([-0-9.]+), ([-0-9.]+)\)/, a) {
    if (a[2] == D && a[1] + 0 >= T - 8 && a[1] + 0 <= T + 2) {
      t = a[1] + 0; x = a[3] + 0; y = a[4] + 0; z = a[5] + 0
      if (pt != "") {
        dt = t - pt; dd = sqrt((x-px)^2 + (y-py)^2)
        printf "  t=%4d  (%7.2f,%7.2f,%5.2f)  v=%5.2f m/s\n", t, x, y, z, dd/dt
      } else {
        printf "  t=%4d  (%7.2f,%7.2f,%5.2f)  v=  -\n", t, x, y, z
      }
      pt = t; px = x; py = y
    }
  }' "$B/$C/verify.txt" 2>/dev/null
