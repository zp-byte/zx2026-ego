#!/usr/bin/env bash
# 批 3 —— 翻 β on（只动 sup_bearing_mode；死端两键保持批 2 定值 4.0/1.0）。
# 对照 = 批 2（defop_ab_20261002_121827，shadow 档，发布路径与 off 逐字节同）。
# 效应归因：批3 − 批2 = β 修正的净效应。
# 杀线（任一破即回 'off'）：P4 余量 > 0 且 landed 不掉 且 Σcol 不升。
cd /home/ubuntu/zx2026_defop_ws || exit 9
# ⚠ set -u 必须**在** ROS source 之后（setup.bash 引用未绑定变量 ⇒ 静默中止）
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source devel/setup.bash >/dev/null 2>&1
set -u

CFG=src/defop_nav/config/defop_nav.yaml
echo "=== 档位自检（不符即拒发）==="
M=$(grep -E "^sup_bearing_mode:" $CFG | awk '{print $2}' | tr -d "'\"")
A=$(grep -E "^deadend_max_s:" $CFG | awk '{print $2}')
H=$(grep -E "^deadend_exit_hyst_s:" $CFG | awk '{print $2}')
C=$(grep -E "^sup_de_dt_cap_s:" $CFG | awk '{print $2}')
S=$(grep -E "^yaw_rate_spot_deg:" $CFG | awk '{print $2}')
echo "  sup_bearing_mode=$M  deadend_max_s=$A  exit_hyst_s=$H  cap=$C  spot=$S"
if [ "$M" != "on" ] || [ "$A" != "4.0" ] || [ "$H" != "1.0" ] || [ "$S" != "30.0" ]; then
  echo "  XX 档位不符，拒发批"; exit 8
fi
echo "  OK 档位正确（on / 4.0 / 1.0，spot 未动）"
echo

echo "=== 发车前残留门（必须 0，用 ps 避免 pgrep -f 自匹配）==="
R=$(ps -eo args --no-headers | grep -Ev 'grep|b3_run' \
      | grep -cE 'gzserver|gzclient|rosmaster|roslaunch|defop_cpn|defop_vae|nav_node|mission_executor|depth_sim|probe_node')
echo "  匹配进程数=$R"
if [ "${R:-0}" != "0" ]; then
  ps -eo pid,args --no-headers | grep -Ev 'grep|b3_run' \
    | grep -E 'gzserver|rosmaster|roslaunch|defop_cpn|nav_node|probe_node' | head -5
  echo "  XX 有残留，拒发"; exit 7
fi
free -m | head -2
echo

echo "=== 发车（setsid 脱离会话）==="
LOG=/tmp/b3_console.log
: > "$LOG"
setsid nohup python3 -u tools/defop_ab_run.py --arms defop --seeds 43 --runs 3 \
  --vae-device cuda:0 --out results/b3_20261002.csv >> "$LOG" 2>&1 &
echo "  runner pid=$!"
sleep 8
echo "  存活: $(pgrep -cf 'tools/defop_ab_run.py') 个 runner"
tail -5 "$LOG" 2>/dev/null | sed 's/^/    /'
