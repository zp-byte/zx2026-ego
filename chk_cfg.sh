#!/usr/bin/env bash
cd /home/ubuntu/zx2026_ego_ws || exit 1
source /opt/ros/noetic/setup.bash
source /home/ubuntu/zx2026_ego_ws/devel/setup.bash
python3 - <<'PY'
from zx2026_common import config as cfg
w = cfg.load("sim_settings.yaml")["wind"]
print("wind cfg keys:", sorted(w.keys()))
print("enabled:", w.get("enabled"), "feedforward:", w.get("feedforward"),
      "gust_std:", w.get("gust_std"), "turb_std:", w.get("turb_std"))
PY
