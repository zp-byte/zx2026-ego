#!/bin/bash
echo "=== sdformat 版本 ==="
dpkg -l 2>/dev/null | grep -i sdformat | awk '{print $2, $3}'

echo
echo "=== 定位 model.sdf schema ==="
SCHEMA=$(find /usr/share/sdformat* /opt/ros/noetic/share/sdformat* -name "model.sdf" 2>/dev/null | head -1)
echo "文件: $SCHEMA"

echo
echo "=== schema 中 static 相关定义 ==="
grep -n "static" "$SCHEMA" 2>/dev/null

echo
echo "=== model 元素的 attribute 定义段 ==="
grep -n "attribute name=" "$SCHEMA" 2>/dev/null | head -30
