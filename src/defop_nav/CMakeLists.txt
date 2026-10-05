cmake_minimum_required(VERSION 3.0.2)
project(defop_nav)

# 纯脚本包（无编译产物；devel 空间 roslaunch 按 pkg/type 发现 scripts/）
# 模式照抄 arena_bridges（catkin dummy）。
find_package(catkin REQUIRED)
catkin_package()
