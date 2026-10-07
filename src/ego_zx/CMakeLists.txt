cmake_minimum_required(VERSION 3.0.2)
project(ego_zx)

find_package(catkin REQUIRED)

catkin_package()

# 脚本经 roslaunch type="<file>" 直跑（scripts/ + exec bit 约定，同 arena_* 家族）。
# 注意：UNC 编辑会掉 exec bit——提交前必须 chmod +x（见工作区纪律）。
install(PROGRAMS
  scripts/ego_bridge_node.py
  scripts/global_route_node.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
