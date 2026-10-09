#!/bin/bash
# =============================================================================
# preflight.sh — 真机起飞前九检（2026-10-09）
# =============================================================================
# 时机：官方飞行栈（mavros/LIO/EKF/px4ctrl）+ real_d435.launch 全部拉起之后、
#       发 /zx2026/fleet_ready 之前。任何 FAIL 都不许飞。
# 用法：
#   bash preflight.sh [id]            # 默认 id=0
#   bash preflight.sh 0 && rostopic pub -1 /zx2026/fleet_ready std_msgs/Bool "data: true"
# 判据纪要：
#   - D435 hz 判「帧间隔」不判平均速率（ego RTF 定谳：间歇断流平均速率抓不住），
#     rostopic hz 连续窗内若断流会掉到低值，故阈值取得保守（≥10，标称 15）。
#   - USB error-71 断流病（换口定谳）的现场签名就是第 4 检掉零——先换 USB3 口
#     直插（禁 hub），再重启 nodelet（rosnode cleanup + 重拉 launch）。
# =============================================================================
ID=${1:-0}
FAIL=0; WARN=0

# ROS 环境：脚本独立跑（新终端/双击）也必须能找到 rostopic/rospack
source /opt/ros/noetic/setup.bash 2>/dev/null
for f in "$HOME/zx2026_ego_ws/devel/setup.bash" "$(dirname "$0")/../../devel/setup.bash"; do
  [ -f "$f" ] && source "$f" && break
done
pass(){ echo "  [PASS] $1"; }
fail(){ echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
warn(){ echo "  [WARN] $1"; WARN=$((WARN+1)); }

# hz 采样：6s 窗，返回平均速率（无发布者/超时=0）
hz_of(){
  timeout 6 rostopic hz "$1" 2>/dev/null | awk '/average rate/{printf "%.1f", $3; exit}'
  return 0
}
ge(){ awk -v a="$1" -v b="$2" 'BEGIN{print (a>=b)?1:0}'; }

echo "=== 真机 preflight 九检（drone_$ID） $(date '+%F %T') ==="

# ① master 活体
echo "[1/9] ROS master"
if rostopic list >/dev/null 2>&1; then pass "rostopic list 可用（master 活）"
else fail "master 不可达——先 roscore / 检查 ROS_MASTER_URI"; fi

# ② mavros FCU 连接
echo "[2/9] mavros FCU"
ST=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null)
if echo "$ST" | grep -q 'connected: True'; then pass "FCU connected"
else fail "FCU 未连接（查 tty 线序/波特率 921600；mavros 拉起失败）"; fi
echo "$ST" | grep -q 'mode:.*OFFBOARD' && warn "已在 OFFBOARD——确认是本脚本调用前有意进入"

# ③ EKF/LIO 里程计
echo "[3/9] /ekf/ekf_odom（LIO+EKF 就绪判据，real_fleet.launch 头部前置）"
H=$(hz_of /ekf/ekf_odom)
if [ "$(ge "$H" 20)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <20——LIO/EKF 没就绪，三桥的 odom 源会是死话题"; fi

# ④ D435 点云
echo "[4/9] /camera/depth/color/points（D435 降流档标称 15）"
H=$(hz_of /camera/depth/color/points)
if [ "$(ge "$H" 10)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <10——D435 断流（error-71 接触病：换 USB3 口直插禁 hub）或 nodelet 空壳（重启）"; fi

# ⑤ odom_tf 出云
echo "[5/9] /drone_${ID}/odom（odom_tf ENU 变换输出）"
H=$(hz_of /drone_${ID}/odom)
if [ "$(ge "$H" 20)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <20——odom_tf 死了或 odom_src 配错"; fi

# ⑥ cloud_adapter 出云
echo "[6/9] /drone_${ID}/cloud（四道滤后 nav 建图输入；地面滤最要害）"
H=$(hz_of /drone_${ID}/cloud)
if [ "$(ge "$H" 3)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <3——cloud_adapter 死了；查其 console：frame=camera 档须 odom 四元数活"; fi

# ⑦ 磁盘 + 温度 + 内存
echo "[7/9] 磁盘/温度/内存（8GB Orin NX 预算）"
FREE_GB=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
if [ "$FREE_GB" -ge 2 ]; then pass "根分区余 ${FREE_GB}GB"
else fail "根分区余 ${FREE_GB}GB <2——nav_metrics/日志会写爆"; fi
TMAX=0
for z in /sys/devices/virtual/thermal/thermal_zone*/temp; do
  [ -r "$z" ] || continue
  T=$(cat "$z"); [ "$T" -gt "$TMAX" ] && TMAX=$T
done
TC=$((TMAX/1000))
if [ "$TC" -le 85 ]; then pass "CPU 温度 ${TC}°C"
else warn "CPU 温度 ${TC}°C >85——Jetson 过频降频会吃掉 vel_bridge 50Hz"; fi
MEM_AVAIL_MB=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
if [ "$MEM_AVAIL_MB" -ge 800 ]; then pass "可用内存 ${MEM_AVAIL_MB}MB"
else warn "可用内存 ${MEM_AVAIL_MB}MB <800——TF/LIO/渲染挤占后 vel_bridge 50Hz 有迟滞风险"; fi

# ⑧ 设置装订一致性
echo "[8/9] settings 装订（拷到机上须改名 sim_settings.yaml）"
PKG="$(dirname "$0")/../config/settings_real.yaml"
DEPLOYED="$(rospack find zx2026_common 2>/dev/null)/config/sim_settings.yaml"
if [ ! -r "$DEPLOYED" ]; then
  warn "rospack 找不到 zx2026_common——本机未 source ego_ws devel？装订检查跳过"
elif [ -f "$PKG" ] && cmp -s "$PKG" "$DEPLOYED"; then
  pass "sim_settings.yaml 与包内 settings_real.yaml 逐位一致"
else
  fail "装订不一致——重拷：cp $PKG $DEPLOYED（部署方式=改名覆盖，见 README）"
fi
BOUND=$(grep -m1 'source: cloud' "$DEPLOYED" 2>/dev/null)
[ -n "$BOUND" ] && pass "mission.via_slots.source=cloud（真值泄漏审计过）" \
                 || warn "via_slots.source 非 cloud——真值泄漏审计红线，部署后必须复核"

# ⑨ 可执行位（UNC/拷贝会剥 exec bit，二犯定谳）
echo "[9/9] exec bits"
BADS=""
for f in "$0" \
         "$(rospack find arena_bridges 2>/dev/null)/odom_tf.py" \
         "$(rospack find arena_bridges 2>/dev/null)/cloud_adapter.py" \
         "$(rospack find arena_bridges 2>/dev/null)/vel_bridge.py" \
         "$(dirname "$0")/estop.sh"; do
  [ -f "$f" ] || continue
  [ -x "$f" ] || BADS="$BADS $f"
done
if [ -z "$BADS" ]; then pass "脚本/桥全部可执行"
else fail "缺 exec bit：$BADS —— chmod +x 之（spawn 静默失败定谳）"; fi

# 附件提醒（非检）
if ! timeout 3 rostopic echo -n1 /zx2026/fleet_ready 2>/dev/null | grep -q 'data: True'; then
  warn "fleet_ready 未置位——全部 PASS 后人工发：rostopic pub -1 /zx2026/fleet_ready std_msgs/Bool \"data: true\""
fi
echo "=== 九检完：FAIL=$FAIL WARN=$WARN ==="
[ "$FAIL" -eq 0 ] && echo "PREFLIGHT PASS" || echo "PREFLIGHT FAIL——禁止起飞"
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
