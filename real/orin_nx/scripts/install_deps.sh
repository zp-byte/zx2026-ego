#!/bin/bash
# =============================================================================
# install_deps.sh — Orin NX 8GB 真机依赖安装（Ubuntu 20.04 + ROS Noetic）
# =============================================================================
# 范围：本包直接依赖。**官方飞行栈不在本脚本范围**——mavros 之外，
# faster-lio / ekf / px4ctrl 属 Diff-planner 仓库（sh_files 拉起），
# 须另行 catkin build（README「0 步」）。
# 用法：bash install_deps.sh          （需 sudo 权限，会提示输密码）
# =============================================================================
set -e

echo "== [1/4] apt 包 =="
sudo apt update
sudo apt install -y \
  ros-noetic-realsense2-camera \
  ros-noetic-usb-cam \
  ros-noetic-mavros ros-noetic-mavros-extras \
  python3-rospkg python3-numpy python3-yaml \
  rosbash ros-noetic-rosbash

echo "== [2/4] mavros 地理库数据（Geographiclib datasets，mavros 首启卡死预防） =="
if [ -x /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh ]; then
  sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
else
  echo "  [WARN] 找不到 install_geographiclib_datasets.sh——mavros 版本差异，手工装："
  echo "    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh"
fi

echo "== [3/4] udev 规则（D435 非 root 可读 + C100 稳定软链） =="
RULES="$(dirname "$0")/../udev/99-realsense-libusb.rules"
if [ -f "$RULES" ]; then
  sudo cp "$RULES" /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "  已装 99-realsense-libusb.rules——**拔插一次 D435 生效**"
else
  echo "  [WARN] 包内无 rules 文件，跳过"
fi
CD_RULES="$(dirname "$0")/../udev/99-camera-down.rules"
if [ -f "$CD_RULES" ]; then
  sudo cp "$CD_RULES" /etc/udev/rules.d/
  echo "  已装 99-camera-down.rules 模板——**先用 lsusb 的 VID:PID 替换模板"
  echo "  里 XXXX/yyyy 再拔插 C100**，确认 ls -l /dev/camera_down 出现"
else
  echo "  [WARN] 包内无 99-camera-down.rules，跳过"
fi

echo "== [4/4] 自检 =="
python3 -c "import numpy, yaml; print('numpy', numpy.__version__, '/ yaml OK')"
rospack find realsense2_camera >/dev/null 2>&1 && echo "realsense2_camera OK" \
  || { echo "[FAIL] realsense2_camera 未找到——重开终端再试或查 apt 报错"; exit 1; }
rospack find mavros >/dev/null 2>&1 && echo "mavros OK" \
  || { echo "[FAIL] mavros 未找到"; exit 1; }
rospack find usb_cam >/dev/null 2>&1 && echo "usb_cam OK（C100 驱动）" \
  || echo "[WARN] usb_cam 未找到——C100 拉不起；查 apt 源或 vendor 份 usb_cam 进 ws"

cat <<'EOF'

== 未覆盖项（非本脚本职责，列清单防漏）==
1. Diff-planner 飞行栈：~/Diff-planner catkin build（faster-lio/ekf/px4ctrl +
   quadrotor_msgs——vel_bridge import 它，没 build 桥起不来）
2. ego_ws 本体：~/zx2026_ego_ws catkin build（arena_bridges/arena_nav/...）
3. 部署三件套②：settings_real.yaml → 机上 zx2026_common/config/sim_settings.yaml
   （改名覆盖）；missions.yaml / profile_real_lio.yaml → tools/gcs/
4. Orin NX 功率模式：sudo nvpmodel -m 0 && sudo jetson_clocks（8GB 板 MAXN，
   vel_bridge 50Hz 对调度迟滞敏感）
EOF
echo "install_deps DONE"
