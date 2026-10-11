#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""venue_real_gen.py — 真机场地派生数唯一生成/校验入口（2026-10-11）。

口径链（GPS 真值 → 场地 ENU 模型）：
  venue_gps.yaml（官方度分秒唯一真值，头注铁律：禁写派生数）
    → geo_enu.frame_from_venue（锚点=起降区 4 点均值，2026-10-09 定谳不改）
    → 本工具运行时派生全部场地几何
    → scene_topology_real.yaml / competition_rules_real.yaml 的 ENU 数。

派生口径（本轮定谳 2026-10-11）：
  * venue.size    = perimeter ENU bbox 双向外扩 5m 取整到米（grid 中心硬
    编码 0——nav_node g_x0=-size/2，size 须双向覆盖 |coord|max）
  * pads 6 点     = 起降区 quad 内 3×2（scene._make_pads cols=3 硬编码），
    间距=min(4.0, floor(quad 短边/6))
  * crossing_zone = 起降区南缘外推 10m 到补给区北缘外推 10m 之间、场地
    东西界内的四边形走廊——规则原文无穿越区坐标，派生假定（⚠）
  * drop_points   = 补给区内 mower 搜索点阵（2026-10-11 用户定谳：投放
    平台是一个区域，在区域里面搜寻——官方只给补给区周界不给平台点）。
    点阵是 RECON 扫描航路（途中检测的行覆盖）+ 末轮悬停确认锚点，非平台
    精确点；行距 1.5m = D435 color 窄轴 42.5°@巡航 2.4m 交轨带 1.87m×0.8。
    每点带 gps 度分秒字段（xyz↔gps 严格互证，手改 xyz 过不了 --check）
  * fence 4 AABB  = perimeter bbox 四边条带 z 0..3.0（保守外近似；斜边外
    越界由 rules geofence 8 点多边形 + PX4 GF_* 双层兜底，不做楼梯分解）
  * geofence      = perimeter ENU 8 点多边形（rule_monitor point_in_poly
    本就支持任意多边形；_assert_fences 前提「fence bbox ⊇ geofence bbox」
    由 --check 断言）

用法：
  python3 venue_real_gen.py                    # 打印派生值（落文件依据）
  python3 venue_real_gen.py --check            # 比对部署文件，不符 exit 1
  python3 venue_real_gen.py --check --root ~/zx2026_ws   # 机上部署树

零 ROS 依赖（纯离线）；机上校验由 deploy_real.sh 调 --check。
"""
import argparse
import math
import os
import sys


def _add_repo_paths(root=None):
    here = os.path.dirname(os.path.abspath(__file__))
    ws = root or os.path.dirname(os.path.dirname(here))
    p = os.path.join(ws, "src", "arena_bridges", "scripts")
    if os.path.isfile(os.path.join(p, "geo_enu.py")) and p not in sys.path:
        sys.path.insert(0, p)
    return ws


WS = _add_repo_paths()
import geo_enu as geo  # noqa: E402  (路径注入后导入；纯函数零重依赖)


# ------------------------------------------------------------------ 工具 --
def r2(x):
    """米制统一两位小数（派生数入库精度）。"""
    return round(float(x) + 0.0, 2)


def deg2dms(v):
    """十进制度 → [度, 分, 秒]（秒保留 2 位，进位防御）。"""
    v = abs(float(v))
    d = int(v)
    m = int((v - d) * 60)
    s = round(((v - d) * 60 - m) * 60, 2)
    if s >= 60.0:
        s -= 60.0
        m += 1
    if m >= 60:
        m -= 60
        d += 1
    return [d, m, s]


def _xs_at(poly, nc):
    """水平线 n=nc 与多边形各边交点的 (最西, 最东) x。"""
    xs = []
    k = len(poly)
    j = k - 1
    for i in range(k):
        (x1, y1), (x2, y2) = poly[j], poly[i]
        if (y1 > nc) != (y2 > nc):
            xs.append(x1 + (x2 - x1) * (nc - y1) / (y2 - y1))
        j = i
    if not xs:
        raise AssertionError("水平线 n=%.1f 与场地周界无交点" % nc)
    return min(xs), max(xs)


# ------------------------------------------------------------------ 派生 --
def derive():
    """从 venue_gps.yaml 派生全部场地几何（唯一派生入口）。"""
    fr, poly = geo.frame_from_venue(yaml_load_venue(WS))
    per, take, sup = poly["perimeter"], poly["takeoff"], poly["supply"]

    # docx 量级互证（venue_gps 抄录/换算回归闸；越界即真值源损坏）
    es = [e for e, _ in per]
    ns = [n for _, n in per]
    e_lo, e_hi, n_lo, n_hi = min(es), max(es), min(ns), max(ns)
    assert 200.0 < (e_hi - e_lo) < 245.0, "周界东西 %.1f 越量级" % (e_hi - e_lo)
    assert 400.0 < (n_hi - n_lo) < 445.0, "周界南北 %.1f 越量级" % (n_hi - n_lo)
    assert geo.point_in_poly(0.0, 0.0, per), "ENU 原点（起降区心）不在场内"

    # venue.size：bbox 双向外扩取整到米。grid 中心=0 硬编码（nav_node
    # g_x0=-size/2，覆盖 ±size/2）→ size = 2×(max|coord|+5)
    m = 5.0
    size = [2 * int(math.ceil(max(e_hi, -e_lo) + m)),
            2 * int(math.ceil(max(n_hi, -n_lo) + m))]

    # 起降区 quad → 6 pads（3 列 × 2 行，块中心=quad 外接框中心）
    te = [e for e, _ in take]
    tn = [n for _, n in take]
    tw, th = max(te) - min(te), max(tn) - min(tn)
    assert 30.0 < tw < 50.0 and 30.0 < th < 50.0, "起降区 %.0fx%.0f 越量级" % (tw, th)
    sp = float(min(4.0, math.floor(min(tw, th) / 6.0)))
    cx, cy = (max(te) + min(te)) / 2.0, (max(tn) + min(tn)) / 2.0
    pad_origin = [r2(cx - sp), r2(cy - sp / 2.0)]

    # 补给区 quad → mower 搜索点阵（2026-10-11 用户定谳：投放平台是一个
    # 区域，在区域里面搜寻——官方只给补给区周界不给平台点）。quad 实为
    # 斜置平行四边形（长边 ~31m、短边 ~8m，bbox 34×19 是倾斜投影）——
    # mower 沿其长轴铺行：行距 1.5m = D435 color 窄轴 42.5°@巡航 2.4m
    # 交轨带 1.87m×0.8 重叠收。端点在长轴系内按行线与边界求交、边内缩
    # 0.5m（锚点不出区只服务 PIP 簿记；区内无 fence/障碍，贴边飞行无害）。
    se = [e for e, _ in sup]
    sn = [n for _, n in sup]
    sw_, sh_ = max(se) - min(se), max(sn) - min(sn)
    assert 25.0 < sw_ < 45.0 and 12.0 < sh_ < 28.0, "补给区 %.0fx%.0f 越量级" % (sw_, sh_)
    e_lens = [(sup[(k + 1) % 4][0] - sup[k][0],
               sup[(k + 1) % 4][1] - sup[k][1]) for k in range(4)]
    lx, ly = max(e_lens, key=lambda t: t[0] ** 2 + t[1] ** 2)
    ll = math.hypot(lx, ly)
    ux, uy = lx / ll, ly / ll
    vx, vy = -uy, ux
    quad_uv = [(p[0] * ux + p[1] * uy, p[0] * vx + p[1] * vy) for p in sup]
    # 行带行走：quad 近平行四边形但 P2 角沿 P2→P3 边快速收窄（~10m/m），
    # 行只铺「跨度 ≥ 3.0m」的纬带（=平台 1.2m + 交轨走廊 1.87m 单侧可达
    # 0.93m；更窄的尖端楔形放不进 1.2m 平台盘，无需覆盖）。端点边内缩
    # 0.3m 只服务 PIP 簿记（区内无 fence/障碍，贴边飞行无害）。
    def _row_span(bb):
        w, e = _xs_at(quad_uv, bb)
        return w, e, e - w
    bs = [b for _, b in quad_uv]
    b0 = b1 = None
    b = min(bs) + 0.3
    while b <= max(bs) - 0.3:
        if _row_span(b)[2] >= 3.0:
            b0 = b
            break
        b += 0.05
    b = max(bs) - 0.3
    while b >= min(bs) + 0.3:
        if _row_span(b)[2] >= 3.0:
            b1 = b
            break
        b -= 0.05
    assert b0 is not None and b1 is not None and b1 - b0 > 3.0, \
        "补给区无可用行带"
    nrows = int(math.floor((b1 - b0) / 1.5)) + 1
    assert nrows >= 3, "补给区行带 %.1f 只够 %d 行" % (b1 - b0, nrows)
    types = (("TYPE_A", "red"), ("TYPE_B", "blue"), ("TYPE_C", "yellow"))
    cen = (sum(p[0] for p in sup) / 4.0, sum(p[1] for p in sup) / 4.0)
    lat_pts = []
    for r in range(nrows):
        bb = b0 + (b1 - b0) * (r / max(nrows - 1, 1))
        w_at, e_at, spn = _row_span(bb)
        assert spn >= 2.9, "补给区行 %d 跨度 %.1f 不足" % (r, spn)
        # 端点内缩 0.3m 须走「向质心」方向：u 与长边近乎平行（cos≈0.995），
        # 沿 u 内缩是切向滑动，法向净空只剩 ~0.03m，取整多边形 PIP 会翻出
        ends = []
        for a, b2 in ((w_at, bb), (e_at, bb)):
            ex, ny = a * ux + b2 * vx, a * uy + b2 * vy
            dl = math.hypot(cen[0] - ex, cen[1] - ny)
            ends.append((ex + (cen[0] - ex) * 0.3 / dl,
                         ny + (cen[1] - ny) * 0.3 / dl))
        if r % 2 == 1:
            ends.reverse()                 # boustrophedon：往返回折
        lat_pts.extend(ends)
    drops = []
    for k, (e, nn) in enumerate(lat_pts):
        lon, lat = fr.to_gps(e, nn)
        gps = {"lon": deg2dms(lon), "lat": deg2dms(lat)}
        # xyz snap 过取整 gps：保证 gps→ENU == xyz 严格互证（0.011m 内），
        # 未来实测换 gps 后同口径重算 xyz 仍过 --check
        e2, n2 = fr.to_enu(geo.dms2deg(gps["lon"]), geo.dms2deg(gps["lat"]))
        t, c = types[k % 3]
        drops.append({"id": k + 1, "xyz": [r2(e2), r2(n2), 1.5],
                      "gps": gps, "type_id": t, "color": c, "marker": ""})

    # 穿越走廊（派生假定）：起降区南缘-10m 到补给区北缘+10m
    n_north = r2(min(tn) - 10.0)
    n_south = r2(max(sn) + 10.0)
    wN, eN = _xs_at(per, n_north)
    wS, eS = _xs_at(per, n_south)
    corridor = [[r2(wN), n_north], [r2(eN), n_north],
                [r2(eS), n_south], [r2(wS), n_south]]

    # fence 4 AABB：perimeter bbox 四边条带（厚 0.3 向外，z 0..3.0 反应带）
    px = 0.3
    fences = [
        {"kind": "fence", "lo": [r2(e_lo - px), r2(n_lo - px), 0.0],
         "hi": [r2(e_lo), r2(n_hi + px), 3.0]},
        {"kind": "fence", "lo": [r2(e_hi), r2(n_lo - px), 0.0],
         "hi": [r2(e_hi + px), r2(n_hi + px), 3.0]},
        {"kind": "fence", "lo": [r2(e_lo - px), r2(n_lo - px), 0.0],
         "hi": [r2(e_hi + px), r2(n_lo), 3.0]},
        {"kind": "fence", "lo": [r2(e_lo - px), r2(n_hi), 0.0],
         "hi": [r2(e_hi + px), r2(n_hi + px), 3.0]},
    ]
    return {
        "anchor": (fr.lon0, fr.lat0),
        "perimeter": [[r2(e), r2(n)] for e, n in per],
        "takeoff_poly": [[r2(e), r2(n)] for e, n in take],
        "supply_poly": [[r2(e), r2(n)] for e, n in sup],
        "size": size,
        "pad_spacing": [r2(sp), r2(sp)],
        "pad_origin": pad_origin,
        "corridor": corridor,
        "drops": drops,
        "fences": fences,
        "geofence": [[r2(e), r2(n)] for e, n in per],
        "bbox": (r2(e_lo), r2(e_hi), r2(n_lo), r2(n_hi)),
        "supply": {"poly": [[r2(e), r2(n)] for e, n in sup],
                   "w": r2(sw_), "h": r2(sh_)},
        "takeoff": {"w": r2(tw), "h": r2(th)},
    }


# ------------------------------------------------------------------ 校验 --
def _close(a, b, tol=0.011):
    return abs(float(a) - float(b)) <= tol


def _poly_close(p, q, tol=0.011):
    return len(p) == len(q) and all(
        _close(a[0], b[0], tol) and _close(a[1], b[1], tol) for a, b in zip(p, q))


def check_deployed(root, scene, rules):
    """部署闸：scene/rules 两份已加载 dict 与派生数逐项比对，不符即 AssertionError。

    root: ws 根（定位 venue_gps.yaml；本模块导入时已按默认根注入 geo_enu，
    机上部署树与仓内同布局，无需重导）。
    """
    global WS
    if root:
        WS = _add_repo_paths(root)
    d = derive()
    geofence = [[float(x), float(y)] for x, y in rules["rule"]["geofence"]]

    # venue / zones
    assert list(scene["venue"]["size"])[:2] == d["size"], \
        "venue.size %s != 派生 %s" % (scene["venue"]["size"], d["size"])
    zones = {z["id"]: z for z in scene["zones"]}
    assert not any(z.get("kind") == "forest" for z in scene["zones"]), \
        "真机场景不得带树"
    assert _poly_close(zones["takeoff"]["polygon"], d["takeoff_poly"]), \
        "takeoff polygon 与派生不符"
    assert zones["takeoff"]["pads"] == 6, "pads 数"
    assert _close(zones["takeoff"]["pad_spacing"][0], d["pad_spacing"][0]) \
        and _close(zones["takeoff"]["pad_spacing"][1], d["pad_spacing"][1]), \
        "pad_spacing 与派生不符"
    assert _close(zones["takeoff"]["pad_origin"][0], d["pad_origin"][0]) \
        and _close(zones["takeoff"]["pad_origin"][1], d["pad_origin"][1]), \
        "pad_origin 与派生不符"
    assert _poly_close(scene["crossing_zone"]["polygon"], d["corridor"]), \
        "crossing_zone 与派生走廊不符"

    # drop_points：搜索点阵（非平台精确点），xyz 必须等于 gps→ENU（工具
    # 重算口径）；改动点阵走本工具重派生重贴，手改单字段过不了 --check。
    assert len(scene["drop_points"]) == len(d["drops"]), \
        "点阵数 %d 与派生 %d 不符（改点阵走 venue_real_gen 重派生）" \
        % (len(scene["drop_points"]), len(d["drops"]))
    # 逐值对质（review 2026-10-11 坐实的洞：自洽但陈旧/异参的点阵只要点数
    # 对就全绿过闸——按 id 对齐钉死 xyz+gps，重派生忘贴新数/树上留旧产物必炸）
    got = {dp["id"]: dp for dp in scene["drop_points"]}
    assert set(got) == {dd["id"] for dd in d["drops"]}, \
        "点阵 id 集 %s 与派生 %s 不符" \
        % (sorted(got), sorted(dd["id"] for dd in d["drops"]))
    for dd in d["drops"]:
        dp = got[dd["id"]]
        assert _close(dp["xyz"][0], dd["xyz"][0]) \
            and _close(dp["xyz"][1], dd["xyz"][1]), \
            "平台 %s xyz %s 与派生 %s 不符（重派生后忘贴新数）" \
            % (dd["id"], dp["xyz"], dd["xyz"])
        assert dp["gps"]["lon"] == dd["gps"]["lon"] \
            and dp["gps"]["lat"] == dd["gps"]["lat"], \
            "平台 %s gps %s 与派生 %s 不符" \
            % (dd["id"], dp["gps"], dd["gps"])
    fr, _ = geo.frame_from_venue(
        yaml_load_venue(root))
    for dp in scene["drop_points"]:
        g = dp.get("gps")
        assert g and "lon" in g and "lat" in g, \
            "平台 %s 缺 gps 字段（真机包坐标口径=GPS 源头）" % dp.get("id")
        e, n = fr.to_enu(geo.dms2deg(g["lon"]), geo.dms2deg(g["lat"]))
        assert _close(dp["xyz"][0], e) and _close(dp["xyz"][1], n), \
            "平台 %s xyz %s 与 gps→ENU (%.2f,%.2f) 不符——只改了单字段" \
            % (dp.get("id"), dp["xyz"], e, n)
        if not geo.point_in_poly(dp["xyz"][0], dp["xyz"][1],
                                 d["supply"]["poly"]):
            # 实测容差：贴边 1m 内仍收（测量噪声 vs 周界取整），再远即真出区
            assert geo.dist_to_poly(dp["xyz"][0], dp["xyz"][1],
                                    d["supply"]["poly"]) <= 1.0, \
                "平台 %s 出补给区" % dp.get("id")
        assert dp["type_id"] in ("TYPE_A", "TYPE_B", "TYPE_C"), \
            "平台 %s type" % dp.get("id")

    # fences：集合相等（顺序无关）
    got = sorted([(o["lo"], o["hi"]) for o in scene["static_obstacles"]
                  if o.get("kind") == "fence"])
    exp = sorted([(f["lo"], f["hi"]) for f in d["fences"]])
    assert len(got) == 4 and all(
        all(_close(a, b) for a, b in zip(ga, ea))
        and all(_close(a, b) for a, b in zip(gb, eb))
        for (ga, gb), (ea, eb) in zip(got, exp)), "fence AABB 与派生不符"

    # geofence 多边形 + fence bbox ⊇ geofence bbox（rule_monitor._assert_fences
    # 前提：四面 fence AABB 必须罩住 geofence 包络，缺边=启动断言炸）
    assert _poly_close(geofence, d["geofence"]), "geofence 与派生周界不符"
    gxe = max(x for x, _ in geofence)
    gxn = max(y for _, y in geofence)
    gxs = min(x for x, _ in geofence)
    gyn = min(y for _, y in geofence)
    west = [g for g in got if (g[1][0] - g[0][0]) <= 0.31
            and g[1][0] <= gxs + 0.31]
    east = [g for g in got if (g[1][0] - g[0][0]) <= 0.31
            and g[0][0] >= gxe - 0.31]
    south = [g for g in got if (g[1][1] - g[0][1]) <= 0.31
            and g[1][1] <= gyn + 0.31]
    north = [g for g in got if (g[1][1] - g[0][1]) <= 0.31
            and g[0][1] >= gxn - 0.31]
    assert west and east and south and north, \
        "fence 未四面罩住 geofence bbox（w%d e%d s%d n%d）" \
        % (len(west), len(east), len(south), len(north))

    # 派生自洽：pads ⊆ 起降区，drop ⊆ 补给区（PIP 用未取整派生多边形）
    sp = d["pad_spacing"][0]
    for i in range(6):
        pe = d["pad_origin"][0] + (i % 3) * sp
        pn = d["pad_origin"][1] + (i // 3) * sp
        assert geo.point_in_poly(pe, pn, d["takeoff_poly"]), \
            "pad %d 出起降区" % i
    for dd in d["drops"]:
        assert geo.point_in_poly(dd["xyz"][0], dd["xyz"][1],
                                 d["supply"]["poly"]), \
            "平台 %d 出补给区" % dd["id"]


def yaml_load_venue(root):
    import yaml
    vp = os.path.join(root, "src", "zx2026_common", "config", "venue_gps.yaml")
    with open(vp, encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--emit-xyz", action="store_true",
                    help="读 scene_topology_real.yaml 各平台 gps 字段，"
                         "打印重算 xyz（实测换 gps 后贴回用）")
    ap.add_argument("--root", default="")
    args = ap.parse_args()

    if args.root:
        _add_repo_paths(args.root)

    if not args.check:
        d = derive()
        print("锚点(起降区心) lon=%.7f lat=%.7f" % d["anchor"])
        print("perimeter bbox e[%.1f,%.1f] n[%.1f,%.1f]" % d["bbox"])
        print("venue.size: [%d, %d, 6]" % (d["size"][0], d["size"][1]))
        print("takeoff quad %sm x %sm" % (d["takeoff"]["w"], d["takeoff"]["h"]))
        print("pad_spacing: %s  pad_origin: %s" % (d["pad_spacing"], d["pad_origin"]))
        print("supply %sm x %sm" % (d["supply"]["w"], d["supply"]["h"]))
        print("crossing_zone polygon:")
        for p in d["corridor"]:
            print("  - [%s, %s]" % (p[0], p[1]))
        print("drop_points:")
        for dp in d["drops"]:
            print("  - id %d xyz %s gps lon%s lat%s %s/%s"
                  % (dp["id"], dp["xyz"], dp["gps"]["lon"], dp["gps"]["lat"],
                     dp["type_id"], dp["color"]))
        print("fences:")
        for f in d["fences"]:
            print("  lo %s hi %s" % (f["lo"], f["hi"]))
        print("geofence:")
        for p in d["geofence"]:
            print("  [%s, %s]" % (p[0], p[1]))
        print("takeoff polygon:")
        for p in d["takeoff_poly"]:
            print("  [%s, %s]" % (p[0], p[1]))
        return

    import yaml
    root = args.root or WS
    cfgd = os.path.join(root, "src", "zx2026_common", "config")
    if args.emit_xyz:
        sc_path = os.path.join(root, "tools", "gcs", "scene_topology_real.yaml")
        with open(sc_path, encoding="utf-8") as f:
            scene = yaml.safe_load(f)
        fr, _ = geo.frame_from_venue(yaml_load_venue(WS))
        for dp in scene["drop_points"]:
            g = dp["gps"]
            e, n = fr.to_enu(geo.dms2deg(g["lon"]), geo.dms2deg(g["lat"]))
            print("平台 %s gps→ENU：xyz: [%s, %s, %s]"
                  % (dp["id"], r2(e), r2(n), dp["xyz"][2]))
        return
    with open(os.path.join(cfgd, "scene_topology.yaml"), encoding="utf-8") as f:
        scene = yaml.safe_load(f)
    with open(os.path.join(cfgd, "competition_rules.yaml"), encoding="utf-8") as f:
        rules = yaml.safe_load(f)
    check_deployed(root, scene, rules)
    print("VENUE CHECK OK（scene/rules 与 GPS 派生数一致）")


if __name__ == "__main__":
    main()
