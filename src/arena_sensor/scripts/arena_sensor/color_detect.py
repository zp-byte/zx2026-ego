# -*- coding: utf-8 -*-
"""arena_sensor::color_detect — 相机颜色检测（纯函数，无 ROS、无真值）。

color_id.source=camera 的检测算法（2026-09-10 立项）。只吃图像+配置参数，
不读 Scene/odom——这份文件在真机 Jetson 上原样运行（D435 RGB 只换话题）。

算法：HSV 域值分割（红双带 union 防 360° 回绕）→ 3×3 开运算去孤立噪点 →
7×7 闭运算桥接高光洞（2026-10-10 真机数据集 A/B：红色亮面塑料镜面高光）
→ 最大连通域 → conf = w_circ*圆形度 + w_pur*色纯度（面积只作噪声地板门限，
不入 conf——真机规则高 0.5-0.6m 时 1.2m 盘溢出 D435 帧，面积/帧归一化
无意义；纯度/形状对距离与偏移不敏感，这正是"悬停漂移 0.3m 内 conf 稳"
的标定来源）。边缘情形按 bucket_select 语义输出：
  无足够大色块 → (NO_DET, 0.0, 255)（清零）
  色块破碎/半遮挡 → conf 低（冻结）
  完整盘面 → conf 高（可 LOCK）
"""
import math

import numpy as np
import cv2

NO_DET = 255  # 与 bucket_select / 真值检测器同哨兵


def make_detect_params(rules_cfg):
    """competition_rules.color_id.camera_detector 段 → 检测参数 dict。"""
    cd = (rules_cfg.get("color_id", {}) or {}).get("camera_detector", {}) or {}
    hsv = cd.get("hsv", {}) or {}
    conf = cd.get("conf", {}) or {}
    return {
        "min_blob_area_px": int(cd.get("min_blob_area_px", 40)),
        # 形态学核（2026-10-10 真机数据集 A/B）：开运算去孤立噪点 + 闭运算
        # 桥接高光洞。闭运算=唯一经验证有效杠杆（真机红色亮面塑料的镜面高光
        # 把色块打成碎环→圆度/纯度塌陷；闭运算恢复实心盘，同公式 conf 中位
        # 0.72→0.79）。对 sim 纯色盘≈恒等，零行为风险。1=关闭该步。
        "open_k": int(cd.get("open_k", 3)),
        "close_k": int(cd.get("close_k", 7)),
        "red": ((int(hsv.get("red_lo1", 0)), int(hsv.get("red_hi1", 8))),
                (int(hsv.get("red_lo2", 172)), int(hsv.get("red_hi2", 179)))),
        "blue": (int(hsv.get("blue_lo", 100)), int(hsv.get("blue_hi", 115))),
        "yellow": (int(hsv.get("yellow_lo", 20)), int(hsv.get("yellow_hi", 35))),
        "s_min": int(hsv.get("s_min", 120)),
        "v_min": int(hsv.get("v_min", 80)),
        "w_circularity": float(conf.get("w_circularity", 0.6)),
        "w_purity": float(conf.get("w_purity", 0.4)),
        "w_area": float(conf.get("w_area", 0.0)),
        "area_floor_px": int(conf.get("area_floor_px", 40)),
    }


# ------------------------------------------------------------- 编码 → BGR --
# 唯一入口（2026-10-10 C100 定谳）。旧版两消费者各自写「非 rgb8/bgr8 一律
# 当 bgr8」——usb_cam 默认出 YUYV 时那是**静默腐蚀**：颜色全错、检测恒
# NO_DET，日志里只有一行 warning。现改为显式白名单：认得就转，不认得返
# None 由调用方**拒帧**（走 stale 哨兵，响亮失败）。
_BGR_ENC = ("bgr8", "8uc3")
_RGB_ENC = ("rgb8",)
_YUYV_ENC = ("yuyv", "yuv422", "yuv422_yuy2", "yuy2")   # 打包 4:2:2，Y 在前
_UYVY_ENC = ("uyvy", "yuv422_uyvy")                     # 打包 4:2:2，U 在前


def to_bgr(data, encoding, width, height):
    """raw sensor_msgs/Image.data → BGR ndarray（cv2 口径）。

    返回 None = 编码不认识或数据不足 ⇒ 调用方必须拒帧，禁当 bgr8 硬解。
    """
    enc = (encoding or "").strip().lower()
    if width <= 0 or height <= 0:
        return None
    raw = np.frombuffer(data, np.uint8)
    if enc in _BGR_ENC:
        n = height * width * 3
        return raw[:n].reshape(height, width, 3) if raw.size >= n else None
    if enc in _RGB_ENC:
        n = height * width * 3
        if raw.size < n:
            return None
        return np.ascontiguousarray(
            raw[:n].reshape(height, width, 3)[:, :, ::-1])
    if enc in _YUYV_ENC or enc in _UYVY_ENC:
        n = height * width * 2
        if raw.size < n or width % 2:
            return None
        # cv2 的 YUYV/UYVY 转换吃「(h, w, 2) 的原始打包字节」形态
        packed = np.ascontiguousarray(raw[:n]).reshape(height, width, 2)
        code = (cv2.COLOR_YUV2BGR_YUYV if enc in _YUYV_ENC
                else cv2.COLOR_YUV2BGR_UYVY)
        return cv2.cvtColor(packed, code)
    return None


def _color_mask(hsv, lo, hi, s_min, v_min):
    return ((hsv[:, :, 0] >= lo) & (hsv[:, :, 0] <= hi)
            & (hsv[:, :, 1] >= s_min) & (hsv[:, :, 2] >= v_min))


def _clean_mask(mask, params):
    """形态学清洗：开运算去孤立噪点 → 闭运算桥接高光洞（2026-10-10）。

    闭运算针对真机红色亮面塑料的镜面高光：高光把色块打成碎环，圆度/纯度
    塌陷；闭运算桥接后恢复实心盘。核大小 1=关闭该步（open_k/close_k 可配）。
    """
    m = mask.astype(np.uint8)
    ko = int(params.get("open_k", 3))
    if ko > 1:
        m = cv2.morphologyEx(
            m, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    kc = int(params.get("close_k", 7))
    if kc > 1:
        m = cv2.morphologyEx(
            m, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc, kc)))
    return m.astype(bool)


def _largest_blob(mask):
    """最大连通域 → (area, 质心, 外接框, mask) 或 None。"""
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[i, cv2.CC_STAT_AREA])
    cent = (float(cents[i][0]), float(cents[i][1]))
    box = (int(stats[i, cv2.CC_STAT_LEFT]), int(stats[i, cv2.CC_STAT_TOP]),
           int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT]))
    return area, cent, box, (labels == i)


def _circularity(blob_mask):
    contours, _ = cv2.findContours(blob_mask.astype(np.uint8),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0
    c = max(contours, key=cv2.contourArea)
    a = cv2.contourArea(c)
    p = cv2.arcLength(c, True)
    if a <= 0.0 or p <= 1e-6:
        return 0.0
    return min(1.0, 4.0 * math.pi * a / (p * p))


def _purity(blob_mask, area, cent):
    """色纯度：色块外接圆内命中像素占比。完整盘≈1；裁边/破碎<1。"""
    r = math.sqrt(max(area, 1) / math.pi)
    x0 = max(0, int(cent[0] - r) - 1)
    x1 = min(blob_mask.shape[1], int(cent[0] + r) + 2)
    y0 = max(0, int(cent[1] - r) - 1)
    y1 = min(blob_mask.shape[0], int(cent[1] + r) + 2)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    yy, xx = np.mgrid[y0:y1, x0:x1]
    inside = (xx - cent[0]) ** 2 + (yy - cent[1]) ** 2 <= r * r
    circle_area = math.pi * r * r
    hits = int(np.count_nonzero(blob_mask[y0:y1, x0:x1] & inside))
    return max(0.0, min(1.0, hits / circle_area))


def detect(img_bgr, params):
    """单帧检测。

    img_bgr: (H,W,3) uint8 BGR（调用方负责编码转换；D435 rgb8 先翻转）
    返回 (color_u8, conf, type_u8)；color_u8=NO_DET 时 conf=0.0、type=255。
    type_u8 不在本层解析（同色多类型歧义属配置层语义），恒返 NO_DET，
    由节点按 color_map 反查表回填。
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    s_min, v_min = params["s_min"], params["v_min"]
    candidates = (
        (0, _color_mask(hsv, params["red"][0][0], params["red"][0][1], s_min, v_min)
           | _color_mask(hsv, params["red"][1][0], params["red"][1][1], s_min, v_min)),
        (1, _color_mask(hsv, params["blue"][0], params["blue"][1], s_min, v_min)),
        (2, _color_mask(hsv, params["yellow"][0], params["yellow"][1], s_min, v_min)),
    )
    best_color, best = NO_DET, None
    for cu8, mask in candidates:
        blob = _largest_blob(_clean_mask(mask, params))
        if blob is None:
            continue
        if best is None or blob[0] > best[0]:
            best_color, best = cu8, blob
    if best is None or best[0] < params["min_blob_area_px"]:
        return NO_DET, 0.0, NO_DET

    area, cent, _, blob_mask = best
    circ = _circularity(blob_mask)
    pur = _purity(blob_mask, area, cent)
    conf = params["w_circularity"] * circ + params["w_purity"] * pur
    if params["w_area"] > 0.0:
        conf += params["w_area"] * min(1.0, area / max(1.0, 4.0 * params["area_floor_px"]))
    conf = max(0.0, min(1.0, conf))
    return best_color, conf, NO_DET  # type_u8 由节点按反查表回填


def detect_with_centroid(img_bgr, params, color_u8=None):
    """单帧检测 + 几何透出（drop_align 视觉伺服用，2026-10-09）。

    detect() 的质心保留版：同一 HSV/连通域管线，额外返回目标色块质心
    （像素，图像系 u 右 v 下）与面积。detect() 内部算完即弃（:56），对齐
    需要像素误差，故单独透出——不动 detect() 签名（bucket_select/type_match
    消费契约零变更）。

    color_u8=None：全色扫描取最大块（同 detect 语义）；指定 0/1/2 时只在
    该色 mask 内找最大块（对齐专用：像素误差只对期望色有意义，异色块必须
    忽略而非竞争）。
    返回 (color_u8, conf, cx_px, cy_px, area_px)；无有效块 (NO_DET, 0.0, -1, -1, 0)。
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    s_min, v_min = params["s_min"], params["v_min"]
    if color_u8 == 0:
        mask = (_color_mask(hsv, params["red"][0][0], params["red"][0][1], s_min, v_min)
                | _color_mask(hsv, params["red"][1][0], params["red"][1][1], s_min, v_min))
        best = _largest_blob(_clean_mask(mask, params))
        best_color = 0 if best is not None else NO_DET
    elif color_u8 in (1, 2):
        lo, hi = params["blue" if color_u8 == 1 else "yellow"]
        best = _largest_blob(_clean_mask(
            _color_mask(hsv, lo, hi, s_min, v_min), params))
        best_color = color_u8 if best is not None else NO_DET
    else:
        best_color, best = NO_DET, None
        for cu8, m in (
            (0, _color_mask(hsv, params["red"][0][0], params["red"][0][1], s_min, v_min)
               | _color_mask(hsv, params["red"][1][0], params["red"][1][1], s_min, v_min)),
            (1, _color_mask(hsv, params["blue"][0], params["blue"][1], s_min, v_min)),
            (2, _color_mask(hsv, params["yellow"][0], params["yellow"][1], s_min, v_min)),
        ):
            blob = _largest_blob(_clean_mask(m, params))
            if blob is not None and (best is None or blob[0] > best[0]):
                best_color, best = cu8, blob

    if best is None or best[0] < params["min_blob_area_px"]:
        return NO_DET, 0.0, -1, -1, 0

    area, cent, _, blob_mask = best
    circ = _circularity(blob_mask)
    pur = _purity(blob_mask, area, cent)
    conf = params["w_circularity"] * circ + params["w_purity"] * pur
    if params["w_area"] > 0.0:
        conf += params["w_area"] * min(1.0, area / max(1.0, 4.0 * params["area_floor_px"]))
    conf = max(0.0, min(1.0, conf))
    return best_color, conf, float(cent[0]), float(cent[1]), area
