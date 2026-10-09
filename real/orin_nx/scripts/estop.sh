#!/bin/bash
# =============================================================================
# estop.sh — 软件级急停两模式（2026-10-09）
# =============================================================================
# 定位：真机急停三层中的软件层（最外层）。硬层=RC 三通（6 通悬停/5 通接管/
#       7 通锁桨，px4ctrl 断链兜底同源）+ GCS panic。本脚本永远不是最后防线。
#
# 用法：
#   estop.sh kill [id]   # 默认。断链急停：杀 vel_bridge ⇒ /setpoints_cmd 断流
#                        # ⇒ px4ctrl 0.5s 看门狗 → HOLD 定点（断链定谳）。
#                        # 对 px4ctrl 语义最干净：无新指令、无状态跃迁。
#   estop.sh zero [id]   # 零速指令：停 nav_node 写者，本脚本以 50Hz 直发零速
#                        # vel_cmd，经 vel_bridge 正常链路下压 ⇒ 平滑刹停悬停。
#                        # 适用：不想惊动 px4ctrl 看门狗的场景（演示/回收）。
#   estop.sh status      # 只看现状：桥在否、cmd 频率、nav 在否
#
# ⚠ 恢复流程：kill 模式后必须重启 real_d435.launch（vel_bridge 是被杀方）；
#   zero 模式 Ctrl-C 停发后，nav_node 已死不会自己复活——重拉 launch。
# ⚠ 两模式都不落桨：锁桨只能 RC 7 通或 px4ctrl 层操作（留给硬层）。
# =============================================================================
MODE=${1:-kill}
ID=${2:-0}
VC="/drone_${ID}/vel_cmd"

status(){
  echo "== estop status (drone_$ID) =="
  pgrep -af 'vel_bridge.py' >/dev/null && echo "vel_bridge: 活" || echo "vel_bridge: 无（kill 态或未启动）"
  pgrep -af 'nav_node.py'   >/dev/null && echo "nav_node:   活" || echo "nav_node:   无"
  ( timeout 4 rostopic hz /setpoints_cmd 2>/dev/null | grep -m1 'average rate' ) \
    || echo "/setpoints_cmd: 4s 内无发布（px4ctrl 将在 0.5s 后 HOLD）"
  exit 0
}
[ "$MODE" = "status" ] && status

# 先 source 才有 rostopic
source /opt/ros/noetic/setup.bash 2>/dev/null
for f in "$(dirname "$0")/../../devel/setup.bash" "$HOME/zx2026_ego_ws/devel/setup.bash"; do
  [ -f "$f" ] && source "$f" && break
done

case "$MODE" in
  kill)
    echo "[estop-kill] 杀 vel_bridge → /setpoints_cmd 断流 → px4ctrl HOLD…"
    pkill -f 'vel_bridge.py' && echo "  vel_bridge 已杀" || echo "  vel_bridge 本就不在"
    sleep 1.0
    pgrep -af 'vel_bridge.py' >/dev/null && echo "  [WARN] 仍有 vel_bridge 残留——查多实例" \
                                            || echo "  断链生效（px4ctrl 应已 HOLD）"
    echo "  恢复：重拉 real_d435.launch"
    ;;
  zero)
    if pgrep -f 'nav_node.py' >/dev/null; then
      echo "[estop-zero] 停 nav_node 写者（防它覆写零令）…"
      pkill -f 'nav_node.py'
      sleep 0.5
    fi
    echo "[estop-zero] 50Hz 直发零速 vel_cmd（Ctrl-C 停）…"
    # Twist 全零=悬停期望；vel_bridge 积分链照常，链路语义无跃迁
    timeout 3600 rostopic pub -r 50 "$VC" geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" &
    PUB=$!
    trap 'kill $PUB 2>/dev/null; exit 0' INT TERM
    wait $PUB
    ;;
  *)
    echo "用法: estop.sh {kill|zero|status} [id]"; exit 2;;
esac
