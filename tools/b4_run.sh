#!/usr/bin/env bash
# 批 4 —— 死端旋转速率 yaw_rate_spot 30 → 45°/s（**只动这一个键**）。
# 基线 = 批 2（defop_ab_20261002_121827，off/4.0/1.0/30.0）。
#   ⚠ 批 3 的 β 已判毙回 'off' ⇒ 当前工作区与批 2 逐字节同，效应 = 批4 − 批2。
# 杀线（任一破即回 30.0）：P4 余量>0 · landed 不降(<17) · retired_n 不升 ·
#   Σcol 不升(<9) · stuck_s 不升 · 杠杆咬合(dt 右移 / cell01 P4 余量 >5s)。
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
L=$(grep -E "^yaw_rate_limit_deg:" $CFG | awk '{print $2}')
echo "  sup_bearing_mode=$M  max_s=$A  hyst=$H  cap=$C  spot=$S  limit=$L"
if [ "$M" != "off" ] || [ "$A" != "4.0" ] || [ "$H" != "1.0" ] || [ "$S" != "45.0" ]; then
  echo "  XX 档位不符，拒发批"; exit 8
fi
if [ "$S" != "$L" ]; then
  echo "  XX spot($S) != limit($L)，越限；拒发"; exit 8
fi
echo "  OK 档位正确（off / 4.0 / 1.0 / spot=45=limit）"
echo

echo "=== 发车前残留门（必须 0，用 ps 避免 pgrep -f 自匹配）==="
R=$(ps -eo args --no-headers | grep -Ev 'grep|b4_run' \
      | grep -cE 'gzserver|gzclient|rosmaster|roslaunch|defop_cpn|defop_vae|nav_node|mission_executor|depth_sim|probe_node')
echo "  匹配进程数=$R"
if [ "${R:-0}" != "0" ]; then
  ps -eo pid,args --no-headers | grep -Ev 'grep|b4_run' \
    | grep -E 'gzserver|rosmaster|roslaunch|defop_cpn|nav_node|probe_node' | head -5
  echo "  XX 有残留，拒发"; exit 7
fi
free -m | head -2
echo

echo "=== 发车（setsid 脱离会话）==="
LOG=/tmp/b4_console.log
: > "$LOG"
setsid nohup python3 -u tools/defop_ab_run.py --arms defop --seeds 43 --runs 3 \
  --vae-device cuda:0 --out results/b4_20261002.csv >> "$LOG" 2>&1 &
echo "  runner pid=$!"
sleep 8
echo "  存活: $(pgrep -cf 'tools/defop_ab_run.py') 个 runner"
tail -5 "$LOG" 2>/dev/null | sed 's/^/    /'
