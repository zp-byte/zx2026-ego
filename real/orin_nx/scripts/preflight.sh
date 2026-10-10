#!/bin/bash
# =============================================================================
# preflight.sh — 真机起飞前十一检（2026-10-09，原九检 + GPS/下视 RGB 两检）
# =============================================================================
# 时机：官方飞行栈（mavros/LIO/EKF/px4ctrl）+ real_d435.launch 全部拉起之后、
#       发 /zx2026/fleet_ready 之前。任何 FAIL 都不许飞。
# 用法：
#   bash preflight.sh [id]            # 默认 id=0
#   bash preflight.sh 0 && rostopic pub -1 /zx2026/fleet_ready std_msgs/Bool "data: true"
# 判据纪要：
#   - D435 hz 判「帧间隔」不判平均速率（ego RTF 定谳：间歇断流平均速率抓不住），
#     rostopic hz 连续窗内若断流会掉到低值，故阈值取得保守（≥10，标称 15）。
#   - USB error-71 断流病（换口定谳）的现场签名就是第 5/6 检掉零——先换
#     USB3 口直插（禁 hub），再重启 nodelet（rosnode cleanup + 重拉 launch）。
#   - odom 源（新③）双探：VIO 主档 VINS 优先（/vins/imu_propagate，换 VINS
#     定夺 2026-10-09），LIO/EKF 备档回退——VINS 缺而 EKF 活给 warn 提示备档，
#     双双无才 FAIL。VINS 未初始化的常见根因=infra 流没开（检 5）/IMU 流没起。
#   - GPS 检（新④）与下视 RGB 检（新⑥）取 warn 不取 fail：短暂无 fix / 相机
#     断流都不直接使飞行不安全（返航回退 scene pad、对齐回退平台心），禁不禁
#     飞留给操作员裁决——别让辅助链的坏消息堵死飞行许可。
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

echo "=== 真机 preflight 十一检（drone_$ID） $(date '+%F %T') ==="

# ① master 活体
echo "[1/11] ROS master"
if rostopic list >/dev/null 2>&1; then pass "rostopic list 可用（master 活）"
else fail "master 不可达——先 roscore / 检查 ROS_MASTER_URI"; fi

# ② mavros FCU 连接
echo "[2/11] mavros FCU"
ST=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null)
if echo "$ST" | grep -q 'connected: True'; then pass "FCU connected"
else fail "FCU 未连接（查 tty 线序/波特率 921600；mavros 拉起失败）"; fi
echo "$ST" | grep -q 'mode:.*OFFBOARD' && warn "已在 OFFBOARD——确认是本脚本调用前有意进入"

# ③ odom 源双探（VIO 主档 VINS 优先 / LIO-EKF 备档；换 VINS 定夺 2026-10-09）
echo "[3/11] odom 源（/vins/imu_propagate 主档 / /ekf/ekf_odom 备档）"
HV=$(hz_of /vins/imu_propagate)
HE=$(hz_of /ekf/ekf_odom)
if [ "$(ge "$HV" 50)" = "1" ]; then
  pass "VINS hz=$HV（VIO 主档；odom_src 须=/vins/imu_propagate）"
elif [ "$(ge "$HE" 20)" = "1" ]; then
  warn "VINS 未起（hz=$HV），LIO/EKF 活 hz=$HE——走备档须显式 odom_src:=/ekf/ekf_odom cloud_frame:=world"
else
  fail "VINS 与 EKF odom 全无——VINS 未初始化（infra 流/IMU 流见检 5/8）或 LIO 未起，禁飞"
fi

# ④ GPS fix（GPS 锚定返航输入；warn 不 fail——返航可回退场景 pad）
echo "[4/11] /mavros/global_position/global（GPS 记点/返航修正输入）"
H=$(hz_of /mavros/global_position/global)
if [ "$(ge "$H" 1)" != "1" ]; then
  warn "hz=$H <1——global_position 死：返航退化为场景 pad 原行为"
else
  FIX=$(timeout 4 rostopic echo -n1 /mavros/global_position/status 2>/dev/null | grep -m1 '^status' | grep -o '[-0-9]\+$')
  if [ -n "$FIX" ] && [ "$FIX" -ge 0 ] 2>/dev/null; then pass "hz=$H fix status=$FIX"
  else warn "hz=$H 但无 fix——室内/RTK 未就绪，起飞前须室外复核"; fi
fi

# ⑤ D435 点云
echo "[5/11] /camera/depth/color/points（D435 降流档标称 15）"
H=$(hz_of /camera/depth/color/points)
if [ "$(ge "$H" 10)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <10——D435 断流（error-71 接触病：换 USB3 口直插禁 hub）或 nodelet 空壳（重启）"; fi

# ⑥ 下视 RGB（WHEELTEC C100/usb_cam，2026-10-10 定谳；warn 不 fail——对齐回退平台心）
echo "[6/11] /camera_down/image_raw（下视 C100，drop_align/识别线输入）"
H=$(hz_of /camera_down/image_raw)
if [ "$(ge "$H" 10)" = "1" ]; then pass "hz=$H"
else warn "hz=$H <10——C100 断：对齐退化平台心、识别线同断；查 /dev/camera_down 软链（udev 装订+拔插）与 USB 口"; fi
ENC=$(timeout 4 rostopic echo -n1 /camera_down/image_raw/encoding 2>/dev/null | head -1 | tr -d '"' | tr -d "'")
case "$ENC" in
  bgr8|rgb8|yuyv|yuv422|yuv422_yuy2|yuy2|uyvy|yuv422_uyvy)
    pass "encoding=$ENC（消费白名单内）" ;;
  "")
    warn "encoding 读不到（话题无数据时随之缺）" ;;
  *)
    warn "encoding=$ENC 不在消费白名单——帧会被拒帧（查 camera_down.launch pixel_format 或升级编码转换）" ;;
esac

# ⑦ odom_tf 出云
echo "[7/11] /drone_${ID}/odom（odom_tf ENU 变换输出）"
H=$(hz_of /drone_${ID}/odom)
if [ "$(ge "$H" 20)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <20——odom_tf 死了或 odom_src 配错"; fi

# ⑧ cloud_adapter 出云
echo "[8/11] /drone_${ID}/cloud（四道滤后 nav 建图输入；地面滤最要害）"
H=$(hz_of /drone_${ID}/cloud)
if [ "$(ge "$H" 3)" = "1" ]; then pass "hz=$H"
else fail "hz=$H <3——cloud_adapter 死了；查其 console：frame=camera 档须 odom 四元数活"; fi

# ⑨ 磁盘 + 温度 + 内存
echo "[9/11] 磁盘/温度/内存（8GB Orin NX 预算）"
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

# ⑩ 设置装订一致性
echo "[10/11] settings 装订（拷到机上须改名 sim_settings.yaml）"
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

# ⑪ 可执行位（UNC/拷贝会剥 exec bit，二犯定谳）
echo "[11/11] exec bits"
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
echo "=== 十一检完：FAIL=$FAIL WARN=$WARN ==="
[ "$FAIL" -eq 0 ] && echo "PREFLIGHT PASS" || echo "PREFLIGHT FAIL——禁止起飞"
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
