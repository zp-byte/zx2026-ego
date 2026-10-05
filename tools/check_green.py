#!/usr/bin/env python3
# 统计截图中的森林绿色像素（G 显著高于 R/B）与投放点红像素
import sys
from PIL import Image

img = Image.open(sys.argv[1]).convert("RGB")
w, h = img.size
px = img.load()
green = red = blue = 0
for y in range(h):
    for x in range(w):
        r, g, b = px[x, y]
        if g > 110 and g > r + 40 and g > b + 40:
            green += 1
        if r > 200 and g < 120 and b < 120:
            red += 1
        if b > 200 and r < 120 and g < 120:
            blue += 1
total = w * h
print("size=%dx%d" % (w, h))
print("forest-green px=%d (%.2f%%)" % (green, 100.0 * green / total))
print("drop-red px=%d (%.2f%%)" % (red, 100.0 * red / total))
print("drone-blue px=%d (%.2f%%)" % (blue, 100.0 * blue / total))
