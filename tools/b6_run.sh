#!/usr/bin/env bash
# 批 6 —— ③「监督层未拦住」**A/B 处理臂**：翻 deadend_veto_frac=0.60 / dwell=0.30。
#   对照 = 批 5（run_logs/defop_ab_20261002_135937，同 seed43 三格、同插桩、门关）。
#   唯一自变量 = 这两个新键（其余 off/4.0/1.0/45.0 逐字节与批 5 同）。
# 杀线（预登记，任一破即回 0.0/0.0）：P4 余量>0 · landed 不降 · Σcol 不升
#   · DE 增幅须可解释（离线预登记 +46.7% 死端占用，见 yaml 注释的复算表）。
cd /home/ubuntu/zx2026_defop_ws || exit 9
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source devel/setup.bash >/dev/null 2>&1
set -u

CFG=src/defop_nav/config/defop_nav.yaml
echo "=== 档位自检 ==="
M=$(grep -E "^sup_bearing_mode:" $CFG | awk '{print $2}' | tr -d "'\"")
A=$(grep -E "^deadend_max_s:" $CFG | awk '{print $2}')
H=$(grep -E "^deadend_exit_hyst_s:" $CFG | awk '{print $2}')
S=$(grep -E "^yaw_rate_spot_deg:" $CFG | awk '{print $2}')
L=$(grep -E "^yaw_rate_limit_deg:" $CFG | awk '{print $2}')
V=$(grep -E "^deadend_veto_frac:" $CFG | awk '{print $2}')
D=$(grep -E "^deadend_veto_dwell_s:" $CFG | awk '{print $2}')
echo "  mode=$M max_s=$A hyst=$H spot=$S limit=$L veto_frac=$V dwell=$D"
if [ "$M" != "off" ] || [ "$A" != "4.0" ] || [ "$H" != "1.0" ] \
   || [ "$S" != "45.0" ] || [ "$V" != "0.60" ] || [ "$D" != "0.30" ]; then
  echo "  XX 档位不符（本批须 off/4.0/1.0/45.0 + veto 0.60/0.30），拒发批"; exit 8
fi
echo "  OK 唯一自变量 = deadend_veto_frac/dwell"

echo "=== 插桩自检（A/B 的触发证据链）==="
for k in dnear dnear2 near_az_deg n_indx1; do
  c=$(grep -c "'$k'" src/defop_nav/scripts/defop_cpn_node.py)
  echo "  $k 出现 $c 次"; [ "$c" -ge 1 ] || { echo "  XX 插桩缺失"; exit 6; }
done
for k in deadend_veto_frac deadend_veto_dwell_s sup_bad sup_dwell; do
  c=$(grep -c "$k" src/defop_nav/scripts/defop_cpn_node.py)
  echo "  $k 出现 $c 次"; [ "$c" -ge 1 ] || { echo "  XX ③ 代码缺失"; exit 6; }
done
# 触发证据：DEADEND enter 行必须带 vf=
c=$(grep -c 'vf=%s' src/defop_nav/scripts/defop_cpn_node.py)
[ "$c" -ge 1 ] || { echo "  XX DEADEND enter 未带 vf，触发证据链断"; exit 6; }
echo "  ok DEADEND enter 带 vf="
python3 -m py_compile src/defop_nav/scripts/defop_cpn_node.py \
  tools/defop_ab_run.py && echo "  ok py_compile" || { echo "  XX 编译失败"; exit 6; }

echo "=== 发车前残留门（必须 0）==="
R=$(ps -eo args --no-headers | grep -Ev 'grep|b6_run' \
      | grep -cE 'gzserver|gzclient|rosmaster|roslaunch|defop_cpn|defop_vae|nav_node|mission_executor|depth_sim|probe_node|rosout')
echo "  匹配进程数=$R"
if [ "${R:-0}" != "0" ]; then
  ps -eo pid,args --no-headers | grep -Ev 'grep|b6_run' \
    | grep -E 'gzserver|rosmaster|roslaunch|defop_cpn|nav_node|probe_node|rosout' | head -5
  echo "  XX 有残留，拒发"; exit 7
fi
free -m | head -2

echo "=== 发车（setsid 脱离会话）==="
LOG=/tmp/b6_console.log
: > "$LOG"
setsid nohup python3 -u tools/defop_ab_run.py --arms defop --seeds 43 --runs 3 \
  --vae-device cuda:0 --out results/b6_20261002.csv >> "$LOG" 2>&1 &
echo "  runner pid=$!"
sleep 8
echo "  存活: $(pgrep -cf 'tools/defop_ab_run.py') 个 runner"
tail -5 "$LOG" 2>/dev/null | sed 's/^/    /'
