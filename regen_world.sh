#!/bin/bash
cd /home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/scripts
echo "=== 重新生成世界 ==="
python3 world_builder.py

echo
echo "=== 校验：残留 static 属性应=0，static 元素应>0 ==="
W=/home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/worlds/forest_world.world
echo "static 属性残留: $(grep -c 'static="true"' "$W")"
echo "static 元素: $(grep -c '<static>true</static>' "$W")"
echo "模型数: $(grep -c '<model' "$W")"

echo
echo "=== XML 校验 ==="
python3 -c "import xml.dom.minidom as m; m.parse('$W'); print('XML OK')"
