#!/usr/bin/env bash
# 批 4 离线门：档位自检 + 四守卫回归 + exec bit + 残留。不发车，只报。
cd /home/ubuntu/zx2026_defop_ws || exit 9
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source devel/setup.bash >/dev/null 2>&1

CFG=src/defop_nav/config/defop_nav.yaml
echo "=== 档位自检 ==="
M=$(grep -E "^sup_bearing_mode:" "$CFG" | awk '{print $2}' | tr -d "'\"")
A=$(grep -E "^deadend_max_s:" "$CFG" | awk '{print $2}')
H=$(grep -E "^deadend_exit_hyst_s:" "$CFG" | awk '{print $2}')
C=$(grep -E "^sup_de_dt_cap_s:" "$CFG" | awk '{print $2}')
G=$(grep -E "^gov_enable:" "$CFG" | awk '{print $2}')
S=$(grep -E "^yaw_rate_spot_deg:" "$CFG" | awk '{print $2}')
L=$(grep -E "^yaw_rate_limit_deg:" "$CFG" | awk '{print $2}')
echo "  sup_bearing_mode=$M max_s=$A hyst=$H cap=$C gov=$G spot=$S limit=$L"
[ "$M" = "off" ] && echo "  OK 模式=off（批 3 已判毙回滚）" || echo "  XX 模式不是 off"
[ "$S" = "45.0" ] && echo "  OK yaw_rate_spot=45（批 4 靶子）" || echo "  XX spot 不是 45"
[ "$S" = "$L" ] && echo "  OK spot==limit 不越限" || echo "  XX spot 越限"

echo
echo "=== 守卫回归 ==="
for t in defop_deadend_selftest defop_sup_geo_selftest defop_forensics_selftest defop_reactwin_selftest; do
  printf "  %-28s " "$t"
  python3 "tools/$t.py" >"/tmp/g4_$t.log" 2>&1 && echo "PASS" || echo "FAIL(见 /tmp/g4_$t.log)"
done

echo
echo "=== exec bit ==="
if git diff --summary | grep "mode change"; then echo "  XX 有 mode change"; else echo "  OK 无 mode change"; fi

echo
echo "=== 残留门（ps，避免 pgrep -f 自匹配）==="
R=$(ps -eo args --no-headers | grep -Ev 'grep|b4_gate' \
      | grep -cE 'gzserver|gzclient|rosmaster|roslaunch|defop_cpn|defop_vae|nav_node|mission_executor|depth_sim|probe_node')
echo "  匹配进程数=$R"
ps -eo pid,args --no-headers | grep -Ev 'grep|b4_gate' \
  | grep -E 'gzserver|rosmaster|roslaunch|defop_cpn|nav_node|probe_node' | head -5
free -m | head -2
