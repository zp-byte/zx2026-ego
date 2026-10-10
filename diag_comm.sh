#!/usr/bin/env bash
# 实时诊断：comm 模型延迟的真实性检查
# 量 /use_sim_time 参数 + /drone_0/odom 与 /drone_1/odom_from_0 的 header.stamp
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "zx2026_all.launch" 2>/dev/null
sleep 1
nohup roslaunch arena_world zx2026_all.launch >/tmp/diag.log 2>&1 &
for i in $(seq 1 90); do
  N=$(rosnode list 2>/dev/null | wc -l)
  [ "$N" -ge 20 ] && break
  sleep 0.5
done
echo "nodes up: $(rosnode list 2>/dev/null | wc -l)"
echo "use_sim_time param: $(rosparam get /use_sim_time 2>&1)"

python3 - <<'PY'
import rospy
from nav_msgs.msg import Odometry
import time

rospy.init_node("diag", anonymous=True)
got = {}
def mk(topic, key):
    def cb(m):
        if key not in got:
            got[key] = (time.time(), m.header.stamp.to_sec(), m.header.seq)
    rospy.Subscriber(topic, Odometry, cb)

mk("/drone_0/odom", "own")
mk("/drone_1/odom_from_0", "relay")
mk("/drone_1/odom", "other_own")

# 触发任务，保证 odom 流动
from std_srvs.srv import Trigger
time.sleep(2)
try:
    srv = rospy.ServiceProxy("/zx2026/start", Trigger)
    srv()
except Exception as e:
    print("start srv:", e)

t0 = time.time()
while time.time() - t0 < 12 and len(got) < 3:
    time.sleep(0.2)

wall = time.time()
print("wall_now=%.6f" % wall)
for k, (rt, stamp, seq) in sorted(got.items()):
    print("%-12s rx_wall=%8.3f header_stamp=%8.3f seq=%d  (wall - stamp)=%.4f"
          % (k, rt, stamp, seq, wall - stamp))
print("own(0) vs relay: stamp diff = %.4f" % (got.get("relay", (0,0,0))[1] - got.get("own", (0,0,0))[1]))
PY

pkill -f "rosmaster.*11411" 2>/dev/null
pkill -f "zx2026_all.launch" 2>/dev/null
sleep 1
echo "== DIAG DONE =="
