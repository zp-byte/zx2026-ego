#!/bin/bash
# _gz_up_console.sh — gz 后端全栈控制台宿主（2026-10-09，cmd start 脱离会话专用）
# 用法（Windows 侧）: cmd start 一个控制台跑本脚本
# 关闭本控制台窗口 = 整个仿真栈停止（栈宿主设计，同 stack_up.sh 纪律）
# 起完: rosservice call /zx2026/start 开始比赛（或 GCS 面板 START 键）
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
source /opt/ros/noetic/setup.bash
source "$HOME/zx2026_ego_ws/devel/setup.bash"
echo "[gz_up] master=$ROS_MASTER_URI DISPLAY=${DISPLAY:-MISSING} WAYLAND=${WAYLAND_DISPLAY:-none}"
exec roslaunch arena_world_gazebo gazebo.launch viewer:=true
