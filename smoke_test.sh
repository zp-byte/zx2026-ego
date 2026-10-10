#!/usr/bin/env bash
# 智信2026科目三 新仿真平台 一键冒烟测试
# 在独立 roscore 端口 11411 上启动，驱动到 DONE，输出阶段/分数汇总。
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash

# 防止 9P 写入重置可执行位：启动前统一补 chmod
find /home/ubuntu/zx2026_ego_ws/src -name '*.py' -exec chmod +x {} +

LOG=/tmp/zx2026_smoke.log
RES=/tmp/zx2026_smoke_result.txt
: > "$RES"

echo "== 清理旧进程(11411) =="
pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "zx2026_all.launch" 2>/dev/null
sleep 1

echo "== 启动 roslaunch (后台) =="
nohup roslaunch arena_world zx2026_all.launch >"$LOG" 2>&1 &
LAUNCH_PID=$!

echo "== 等待 roscore 就绪 =="
for i in $(seq 1 60); do
  if rostopic list >/dev/null 2>&1; then break; fi
  sleep 0.5
done

echo "== 等待所有节点注册 (期望 35 节点) =="
for i in $(seq 1 90); do
  N=$(rosnode list 2>/dev/null | wc -l)
  if [ "$N" -ge 20 ]; then
    echo "nodes=$N"; break
  fi
  sleep 0.5
done

echo "== 节点列表 =="
rosnode list >>"$RES" 2>&1
echo "== 话题快照 =="
rostopic list >>"$RES" 2>&1

echo "== 等待 fleet_ready =="
timeout 40 rostopic echo -n 1 /zx2026/fleet_ready >>"$RES" 2>&1 || echo "FLEET_READY_TIMEOUT" >>"$RES"

echo "== 调用 /zx2026/start =="
rosservice call /zx2026/start "{}" >>"$RES" 2>&1 || echo "START_SERVICE_FAIL" >>"$RES"

echo "== 采样阶段状态 (至多 150s) =="
echo "-- state timeline --" >>"$RES"
timeout 150 rostopic echo -n 3000 /zx2026/state 2>/dev/null \
  | grep -oP 'data: \K[^ ]+' | uniq >>"$RES"

echo "== 采样 3s 后的 drone_0 odom =="
timeout 10 rostopic echo -n 1 /drone_0/odom >>"$RES" 2>&1

echo "== 采样 drone_1 位置 (验证运动) =="
timeout 10 rostopic echo -n 1 /drone_1/odom >>"$RES" 2>&1

echo "== 最终分数摘要 =="
timeout 10 rostopic echo -n 1 /zx2026/score_summary >>"$RES" 2>&1

echo "== 计分报告文件 =="
ls -t /tmp/zx2026_score_*.yaml 2>/dev/null | head -1 >>"$RES"

# 结束后台 launch
kill $LAUNCH_PID 2>/dev/null
pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "roscore.*11411" 2>/dev/null
sleep 1
echo "== SMOKE TEST DONE =="
