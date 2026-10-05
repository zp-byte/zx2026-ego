#!/usr/bin/env bash
# 批 3 发车前离线门（档位自检 + 守卫回归 + 残留门）。不发车，只报。
cd /home/ubuntu/zx2026_defop_ws || exit 9
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source devel/setup.bash >/dev/null 2>&1

CFG=src/defop_nav/config/defop_nav.yaml
echo "=== 档位自检 ==="
M=$(grep -E "^sup_bearing_mode:" $CFG | awk '{print $2}' | tr -d "'\"")
A=$(grep -E "^deadend_max_s:" $CFG | awk '{print $2}')
H=$(grep -E "^deadend_exit_hyst_s:" $CFG | awk '{print $2}')
C=$(grep -E "^sup_de_dt_cap_s:" $CFG | awk '{print $2}')
G=$(grep -E "^gov_enable:" $CFG | awk '{print $2}')
S=$(grep -E "^yaw_rate_spot_deg:" $CFG | awk '{print $2}')
echo "  sup_bearing_mode=$M deadend_max_s=$A deadend_exit_hyst_s=$H cap=$C gov=$G spot=$S"
[ "$M" = "on" ] && echo "  OK 模式=on（批 3 靶子）" || echo "  XX 模式不是 on"
[ "$S" = "30.0" ] && echo "  OK yaw_rate_spot 未动（批 4 才动）" || echo "  XX spot 被误改"

echo
echo "=== 守卫回归 ==="
for t in defop_deadend_selftest defop_sup_geo_selftest defop_forensics_selftest defop_reactwin_selftest; do
  printf "  %-28s " "$t"
  python3 tools/$t.py >/tmp/gate_$t.log 2>&1 && echo "PASS" || echo "FAIL(见 /tmp/gate_$t.log)"
done

echo
echo "=== exec bit ==="
git diff --summary | grep "mode change" && echo "  XX 有 mode change" || echo "  OK 无 mode change"

echo
echo "=== 残留门 ==="
R=$(pgrep -cf 'rosmaster|roslaunch|gzserver|defop_|nav_node|mission_executor|depth_sim|probe_node')
echo "  匹配进程数=$R"
pgrep -af 'rosmaster|roslaunch|gzserver|defop_|nav_node|mission_executor|depth_sim|probe_node' | head -5
free -m | head -2
