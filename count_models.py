#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates

names = [None]

def cb(m):
    names[0] = list(m.name)

rospy.init_node("count_models", anonymous=True)
rospy.Subscriber("/gazebo/model_states", ModelStates, cb)

t = rospy.Time.now().to_sec()
while names[0] is None and not rospy.is_shutdown():
    if rospy.Time.now().to_sec() - t > 8:
        break
    rospy.sleep(0.1)

if names[0] is None:
    print("未收到 model_states")
else:
    print("模型总数 =", len(names[0]))
    kinds = {}
    for n in names[0]:
        k = n.rsplit("_", 1)[0] if n[0].isalpha() and "_" in n else n
        kinds[k] = kinds.get(k, 0) + 1
    for k, v in sorted(kinds.items(), key=lambda x: -x[1]):
        print("  %-16s %d" % (k, v))
    print("示例:", names[0][:6])
