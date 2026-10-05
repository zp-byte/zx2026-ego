#!/usr/bin/env bash
# 批 5 —— ③「监督层未拦住」可观测性档：**纯遥测插桩，零行为改动**。
#   defop_cpn_node.py /defop/debug 增 dnear/dnear2/dmin/near_az_deg/yaw_deg/vw/
#   n_indx1/n_indx；探针 tl 行由 6 列扩到 14 列（前 6 列位置不变）。
# 档位 = 批 4 默认值（off/4.0/1.0/45.0）逐字节不变 ⇒ 与批 4 同分布，可对读。
cd /home/ubuntu/zx2026_defop_ws || exit 9
# ⚠ set -u 必须**在** ROS source 之后（setup.bash 引用未绑定变量 ⇒ 静默中止）
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source devel/setup.bash >/dev/null 2>&1
set -u

CFG=src/defop_nav/config/defop_nav.yaml
echo "=== 档位自检（应与批 4 逐字节同）==="
M=$(grep -E "^sup_bearing_mode:" $CFG | awk '{print $2}' | tr -d "'\"")
A=$(grep -E "^deadend_max_s:" $CFG | awk '{print $2}')
H=$(grep -E "^deadend_exit_hyst_s:" $CFG | awk '{print $2}')
S=$(grep -E "^yaw_rate_spot_deg:" $CFG | awk '{print $2}')
L=$(grep -E "^yaw_rate_limit_deg:" $CFG | awk '{print $2}')
echo "  sup_bearing_mode=$M  max_s=$A  hyst=$H  spot=$S  limit=$L"
if [ "$M" != "off" ] || [ "$A" != "4.0" ] || [ "$H" != "1.0" ] || [ "$S" != "45.0" ]; then
  echo "  XX 档位不符，拒发批"; exit 8
fi
if [ "$S" != "$L" ]; then
  echo "  XX spot($S) != limit($L)，越限；拒发"; exit 8
fi
echo "  OK 档位正确（off / 4.0 / 1.0 / spot=45=limit）"
echo

echo "=== 插桩自检：源码里必须能找到新字段 ==="
for k in dnear dnear2 near_az_deg n_indx1; do
  c=$(grep -c "'$k'" src/defop_nav/scripts/defop_cpn_node.py)
  echo "  $k 在 cpn_node 出现 $c 次"
  [ "$c" -ge 1 ] || { echo "  XX 插桩缺失"; exit 6; }
done
c=$(grep -c 'n_indx1' tools/defop_ab_run.py)
echo "  n_indx1 在探针 PROBE_SRC 出现 $c 次"
[ "$c" -ge 1 ] || { echo "  XX 探针未扩列"; exit 6; }
python3 -m py_compile src/defop_nav/scripts/defop_cpn_node.py tools/defop_ab_run.py \
  && echo "  ok py_compile" || { echo "  XX 编译失败"; exit 6; }
echo

echo "=== 发车前残留门（必须 0，用 ps 避免 pgrep -f 自匹配）==="
R=$(ps -eo args --no-headers | grep -Ev 'grep|b5_run' \
      | grep -cE 'gzserver|gzclient|rosmaster|roslaunch|defop_cpn|defop_vae|nav_node|mission_executor|depth_sim|probe_node|rosout')
echo "  匹配进程数=$R"
if [ "${R:-0}" != "0" ]; then
  ps -eo pid,args --no-headers | grep -Ev 'grep|b5_run' \
    | grep -E 'gzserver|rosmaster|roslaunch|defop_cpn|nav_node|probe_node|rosout' | head -5
  echo "  XX 有残留，拒发"; exit 7
fi
free -m | head -2
echo

echo "=== 发车（setsid 脱离会话）==="
LOG=/tmp/b5_console.log
: > "$LOG"
setsid nohup python3 -u tools/defop_ab_run.py --arms defop --seeds 43 --runs 3 \
  --vae-device cuda:0 --out results/b5_20261002.csv >> "$LOG" 2>&1 &
echo "  runner pid=$!"
sleep 8
echo "  存活: $(pgrep -cf 'tools/defop_ab_run.py') 个 runner"
tail -5 "$LOG" 2>/dev/null | sed 's/^/    /'
