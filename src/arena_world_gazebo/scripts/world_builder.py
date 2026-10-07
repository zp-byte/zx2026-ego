#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world_gazebo::world_builder — 从 scene_topology 生成 Gazebo 11 世界。

复用 zx2026_common.scene 的确定性场景（同 seed=42）：树/灌木取 Scene 内 AABB 的
外接圆柱/盒，保证与 python 后端的碰撞模型和 A* 规划几何一致 —— 换物理引擎不换场景。
输出: arena_world_gazebo/worlds/forest_world.world
用法: python3 world_builder.py
"""
import math
import os
import sys

# ⚠ 生成器自持（2026-10-07 换场）：此前本文件是 defop_ws 的副本，_WS 硬编码指向
# defop_ws，故 ego 的表/Scene/mesh URI 实际全来自 defop_ws —— 改 ego 自己的表不起作用。
# 收归本仓后，ego 的场景与 defop/arena 线解耦，改动不外溢。
_WS = "/home/ubuntu/zx2026_ego_ws"
sys.path.insert(0, os.path.join(_WS, "src/zx2026_common/scripts"))
sys.path.insert(0, os.path.join(_WS, "src/arena_world_gazebo/scripts"))

from zx2026_common.scene import Scene

OUT = os.path.join(os.path.dirname(__file__), "..", "worlds", "forest_world.world")

# 杨树林 mesh（table 场景视觉；STL 由 tools/import_poplar_sdf.py 配套拷入）
MESH_URI = "file:///home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/meshes/poplar"
# 云杉林 mesh（DeFoP 原林资产；整树 dae，纹理随目录）
MESH_URI_SPRUCE = "file:///home/ubuntu/zx2026_ego_ws/src/arena_world_gazebo/meshes/spruce"

# 平台色板（唯一色表；权威源=scene.color_map/dp.color 的颜色名，此处只做名字→RGB）
PLATFORM_COLORS = {
    "red": (0.90, 0.20, 0.20),
    "blue": (0.20, 0.45, 0.90),
    "yellow": (0.90, 0.85, 0.10),
}
DRONE_COLORS = [
    (0.90, 0.30, 0.30), (0.30, 0.75, 0.95), (0.30, 0.85, 0.45),
    (0.95, 0.65, 0.15), (0.75, 0.45, 0.95), (0.95, 0.90, 0.30),
]


def mat(r, g, b, a=1.0):
    return ('<material><ambient>%.2f %.2f %.2f %.2f</ambient>'
            '<diffuse>%.2f %.2f %.2f %.2f</diffuse><specular>0.2 0.2 0.2 1</specular></material>'
            % (r, g, b, a, r, g, b, a))


def col_geo(geom):
    return '<collision name="col"><geometry>%s</geometry></collision>' % geom


def vis_geo(geom, c):
    return '<visual name="vis"><geometry>%s</geometry>%s</visual>' % (geom, c)


def vis_pose(name, geom, color, x, y, z, yaw=0.0):
    """带位姿的视觉（几何+材质），用于单 link 多视觉模型。"""
    return ('<visual name="%s"><pose>%.3f %.3f %.3f 0 0 %.3f</pose>'
            '<geometry>%s</geometry>%s</visual>'
            % (name, x, y, z, yaw, geom, mat(*color)))


def branch_links(ob):
    """树枝碰撞体（gz 树枝物理缺环补齐）：消费 Scene ob.branches（世界系
    线段+半径），逐位生成 SDF cylinder 碰撞+视觉。

    设计律与 python 后端一致：枝参与硬碰撞（物理+事件），球冠仍仅视觉。
    branches.enabled=false 时 Scene 不生枝（ob.branches=None）→ 本函数零输出，
    A/B 单旗天然成立（无需额外开关）。

    几何注意：Gazebo Classic 11（sdformat 1.7）无 capsule 几何（sdformat
    1.8+ 才有），用无帽 cylinder 近似——恰与 monitor 的 point_segment_dist
    线段判定同几何，端帽差 ≤ 枝径 2.2cm 可忽略。SDF cylinder 沿自身 Z 轴；
    姿态取 roll=0 / pitch=acos(dz) / yaw=atan2(dy,dx)（同胶囊朝向约定）。

    ⚠ 已知隐患（2026-10-01 发现，未修）：下面 `世界 − (cx,cy)` **没有反旋
    mesh_yaw**，模型整体再被 Gazebo 转一次 ⇒ 带 `mesh_yaw≠0` 的表场景里枝会
    双重旋转、落到离视觉枝 ~1.8m 的错位置。本函数当前只服务 procedural 旧路径
    （`mesh_yaw` 恒 0，故实际无害）且 `branches.enabled=false`，但**新写的
    真枝路径请用 `branch_collisions()`**，它做了 R(−yaw) 反旋。
    """
    links = []
    brs = getattr(ob, "branches", None) or []
    # model pose=(cx,cy,h/2)：世界坐标→model 系
    ox, oy, oz = ob.cx, ob.cy, ob.trunk_h / 2.0
    for k, (sx, sy, sz, ex, ey, ez, br) in enumerate(brs):
        dx, dy, dz = ex - sx, ey - sy, ez - sz
        ln = math.sqrt(dx * dx + dy * dy + dz * dz)
        if ln < 1e-9:
            continue
        ux, uy, uz = dx / ln, dy / ln, dz / ln
        pitch = math.acos(max(-1.0, min(1.0, uz)))
        yaw = math.atan2(uy, ux)
        mx, my, mz = (sx + ex) / 2.0 - ox, (sy + ey) / 2.0 - oy, (sz + ez) / 2.0 - oz
        cyl = '<cylinder><radius>%.3f</radius><length>%.3f</length></cylinder>' % (br, ln)
        links.append("""  <link name="branch_%d">
    <pose>%.3f %.3f %.3f 0 %.4f %.4f</pose>
    %s
    %s
  </link>""" % (k, mx, my, mz, pitch, yaw,
               col_geo(cyl), vis_geo(cyl, mat(0.45, 0.32, 0.18))))
    return links


def branch_collisions(ob, origin_z=0.0):
    """树的真枝碰撞体：消费 Scene ob.branches，发**只有碰撞、没有视觉**的
    `<collision>` 串（插进树模型自己的 `<link name="tree">` 里）。

    与 `branch_links()` 的分工：
      · `branch_links()` 是旧路径（细枝/独立 link，碰撞+视觉各一份）——保留不动；
      · 本函数服务 table 场景的真枝：杨树的枝**已经由 wood STL 渲染**，再发
        圆柱视觉就是双重渲染（同一根枝画两遍，还比 mesh 粗），故只发碰撞。

    姿态约定与 `branch_links()` 逐字相同（roll=0 / pitch=acos(dz) /
    yaw=atan2(dy,dx)，Gazebo Classic 11 无 capsule 用无帽 cylinder）。

    **为什么必须反旋（易错点，勿删此注释）**：`ob.branches` 是**世界系**线段，
    而写进树模型的 `<collision><pose>` 是**模型局部系**。模型整体还会被 Gazebo
    再按 `<pose>` 的 yaw 旋转一次，所以这里必须取 `R(-yaw)·(世界 − 树心)`；
    直接用 `世界 − 树心` 会让 Gazebo 转第二遍 ⇒ **双重旋转**，枝碰撞落到离
    视觉枝 1.8m 的错位置（实测 maxd=1.802m，靠本文件外的三口径对质脚本揪出）。
    cylinder 自身的 yaw 也必须在局部系里算，故先把两端点搬进局部系再求方向。

    **origin_z 为什么必须显式传**：`branch_links()` 里写死 `oz = trunk_h/2`
    是因为那边的模型 pose 把 z 放在干中（旧圆柱树模型）；杨树模型 pose z=0，
    原点在**地面**。两者不能共用同一个写死的基准，故此处由调用方给。
    """
    out = []
    brs = getattr(ob, "branches", None) or []
    th = float(getattr(ob, "mesh_yaw", 0.0) or 0.0)
    ct, st = math.cos(th), math.sin(th)

    def loc(p):
        """世界系 → 模型局部系：先平移，再 R(-yaw)。"""
        dx, dy, dz = p[0] - ob.cx, p[1] - ob.cy, p[2] - origin_z
        return (ct * dx + st * dy, -st * dx + ct * dy, dz)

    for k, (sx, sy, sz, ex, ey, ez, br) in enumerate(brs):
        a, b = loc((sx, sy, sz)), loc((ex, ey, ez))
        dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        ln = math.sqrt(dx * dx + dy * dy + dz * dz)
        if ln < 1e-9:
            continue
        ux, uy, uz = dx / ln, dy / ln, dz / ln
        pitch = math.acos(max(-1.0, min(1.0, uz)))
        yaw = math.atan2(uy, ux)
        mx, my, mz = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0
        out.append('<collision name="c_branch_%d"><pose>%.3f %.3f %.3f 0 %.4f '
                   '%.4f</pose><geometry><cylinder><radius>%.3f</radius>'
                   '<length>%.3f</length></cylinder></geometry></collision>'
                   % (k, mx, my, mz, pitch, yaw, br, ln))
    return "\n    ".join(out)


def tree_model_poplar(oid, ob):
    """杨树（table 场景）：wood STL 视觉（干+枝）+ 叶 STL 视觉 + 冠包络碰撞。

    口径沿革：
      · 10-01 只去碰撞冠（冠仅视觉）；
      · 10-02 上半天用户裁决「冠=物理存在」⇒ 干+真枝+冠三件套；
      · 10-02 晚用户裁决「冠物理、视觉都删掉，只保留树枝」⇒ 冠两路全删；
      · **10-07 用户裁决换场至 zx2026_ws（arena）场景，全恢复含叶视觉**
        （正式反转 10-02 裁决）：冠包络碰撞由表 meta `crown_collidable=true`
        开回（发 c_crown，且 `Scene._crown_z_range` 同步给出区间 = 四层同源），
        叶视觉在本函数拼回 v_leaf（`tree_<v>_leaves.stl`）；真枝由表缺
        `branch_file` 键而断源（不发 c_branch）。
      · 树体 = 干 c_trunk + 冠 c_crown + wood mesh + leaf mesh + 0 根真枝。
    杠杆语义（键保留为回滚/换场开关）：crown_collidable 控物理冠；叶视觉是
    本函数的直拼，不随任何键。
    ⚠ **叶 mesh 不是「纯观赏件」——它进了 Gazebo 真实相机传感器的渲染路径**
    （2026-10-07 受控实测定谳，见下）。两条相机链要分清：
      · **合成链**（`camera_render` / depth_sim，读 `Scene`）：Scene 里**没有叶的
        表示** ⇒ 看不到叶，速率与叶无关（实测有叶/无叶均 11.9 Hz，纹丝不动）；
      · **Gazebo 传感器链**（`drone_depth_sensor()` 的 D435 depth，本文件 :424）：
        **渲染真实几何** ⇒ 叶的面数直接吃它的帧率。
    ⚠⚠ **换场那次就是栽在这里**：原始叶 mesh 约 4100 面/棵 × 203 棵 ≈ 83 万面，
    把 ego 臂**独有**的 D435 深度相机从 2.28 Hz 打到 **0.595 Hz**，而
    `grid_map/odom_depth_timeout = 1.0 s` ⇒ 帧间隔 1.68 s ≫ 1.0 s ⇒
    `ego_replan_fsm` 每拍打 `Depth Lost! EMERGENCY_STOP`（单局 11966 条）⇒
    六机全程急停、零碰撞零落地。**arena 自己跑不受影响，因为它没有深度传感器。**
    ⇒ 叶 mesh 已抽稀到 **64 面/棵**（`tools/decimate_leaf_mesh.py`，顶点聚类），
    实测深度回到 **2.25 Hz**（≈ 无叶档 2.28 Hz 的 99%）。**改这两个 mesh 前先读
    该工具的 docstring，别再往深度相机里塞面。**
    """
    vi = ob.mesh_variant
    s = ob.mesh_scale or 1.0
    mesh = lambda part: ('<mesh><uri>%s/tree_%d_%s.stl</uri>'
                         '<scale>%.3f %.3f %.3f</scale></mesh>'
                         % (MESH_URI, vi, part, s, s, s))
    trunk_col = ('<collision name="c_trunk"><pose>0 0 %.3f 0 0 0</pose>'
                 '<geometry><cylinder><radius>%.3f</radius><length>%.3f'
                 '</length></cylinder></geometry></collision>'
                 % (ob.trunk_h / 2.0, ob.trunk_r, ob.trunk_h))
    # 冠包络圆柱：仅在表 meta 未关掉冠碰撞时发（= 旧表/云杉档口径）
    if getattr(ob, "crown_collidable", True):
        crown_h = ob.crown_h if ob.crown_h else 0.64 * ob.trunk_h / 0.78
        crown_col = ('<collision name="c_crown"><pose>0 0 %.3f 0 0 0</pose>'
                     '<geometry><cylinder><radius>%.3f</radius><length>%.3f'
                     '</length></cylinder></geometry></collision>'
                     % (ob.crown_z, ob.crown_r, crown_h))
    else:
        crown_col = ""
    # 真枝碰撞：模型 pose z=0（原点在地面），故 origin_z=0
    wood_vis = ('<visual name="v_wood"><geometry>%s</geometry>%s</visual>'
                % (mesh("wood"), mat(0.45, 0.31, 0.19)))
    # 冠视觉 v_leaf（tree_<v>_leaves.stl）：2026-10-07 换场恢复（反转 10-02 裁决）。
    leaf_vis = ('<visual name="v_leaf"><geometry>%s</geometry>%s</visual>'
                % (mesh("leaves"), mat(0.22, 0.47, 0.16)))
    body = "\n    ".join(x for x in (wood_vis, leaf_vis, trunk_col, crown_col,
                                     branch_collisions(ob, 0.0)) if x)
    return """<model name="tree_%d">
  <static>true</static>
  <pose>%.3f %.3f 0 0 0 %.3f</pose>
  <link name="tree">
    %s
  </link>
</model>""" % (oid, ob.cx, ob.cy, ob.mesh_yaw or 0.0, body)


def tree_model_spruce(oid, ob):
    """云杉（DeFoP picea_abies 林，table 场景 mesh_kind=spruce）：整树 dae 视觉
    + 树干/冠包络两圆柱碰撞。

    几何与树表同源（tools/import_spruce_world.py 从 meshes/spruce/
    spruce_variants.json 实测值 × 逐棵 scale 写出），dae 也按同一 scale 缩放
    → gz 物理 == py Scene == A* 占用图。

    与杨树的关键差别：云杉冠底中位 1.22m，正压巡航带（杨树是 2.81m，在带外）。
    故冠是真障碍、不是装饰 —— 视觉虽由 dae 承担，碰撞仍须显式给冠圆柱，
    与 scene.py._collides_crown 同几何。
    """
    vi = ob.mesh_variant
    s = ob.mesh_scale or 1.0
    vis = ('<visual name="v_tree"><geometry><mesh>'
           '<uri>%s/picea_abies_%d.dae</uri>'
           '<scale>%.4f %.4f %.4f</scale>'
           '</mesh></geometry></visual>'
           % (MESH_URI_SPRUCE, vi, s, s, s))
    trunk_col = ('<collision name="c_trunk"><pose>0 0 %.3f 0 0 0</pose>'
                 '<geometry><cylinder><radius>%.3f</radius><length>%.3f'
                 '</length></cylinder></geometry></collision>'
                 % (ob.trunk_h / 2.0, ob.trunk_r, ob.trunk_h))
    crown_h = ob.crown_h if ob.crown_h else 0.64 * ob.trunk_h / 0.78
    crown_col = ('<collision name="c_crown"><pose>0 0 %.3f 0 0 0</pose>'
                 '<geometry><cylinder><radius>%.3f</radius><length>%.3f'
                 '</length></cylinder></geometry></collision>'
                 % (ob.crown_z, ob.crown_r, crown_h))
    return """<model name="tree_%d">
  <static>true</static>
  <pose>%.3f %.3f 0 0 0 %.3f</pose>
  <link name="tree">
    %s
    %s
    %s
  </link>
</model>""" % (oid, ob.cx, ob.cy, ob.mesh_yaw or 0.0,
               vis, trunk_col, crown_col)


def tree_model(oid, ob):
    """圆柱树干（碰撞+视觉）+ 蓬松球形树冠（仅视觉）+ 树枝圆柱（碰撞+视觉，
    若 Scene 生枝）；几何取 ob 的 trunk/crown/branches 字段。"""
    cx = ob.cx
    cy = ob.cy
    r = ob.trunk_r
    h = ob.trunk_h
    cyl = '<cylinder><radius>%.2f</radius><length>%.2f</length></cylinder>' % (r, h)
    # 树冠：多个绿色球组成,更茂密可见（主冠半径=ob.crown_r，其余小球保留）
    crown_links = []
    crown_offsets = [
        (0.0, 0.0, h * 0.55, ob.crown_r),
        (0.55, 0.0, h * 0.75, 0.80),
        (-0.55, 0.15, h * 0.70, 0.80),
        (0.0, 0.55, h * 0.68, 0.78),
        (0.0, -0.50, h * 0.65, 0.75),
        (0.40, 0.35, h * 0.85, 0.65),
    ]
    green_shades = [
        (0.14, 0.48, 0.22),
        (0.18, 0.55, 0.26),
        (0.12, 0.44, 0.20),
    ]
    for i, (ox, oy, oz, cr) in enumerate(crown_offsets):
        color = green_shades[i % len(green_shades)]
        sph = '<sphere><radius>%.2f</radius></sphere>' % cr
        crown_links.append("""  <link name="crown_%d">
    <pose>%.2f %.2f %.2f 0 0 0</pose>
    %s
  </link>""" % (i, ox, oy, oz, vis_geo(sph, mat(*color))))
    return """<model name="tree_%d">
  <static>true</static>
  <pose>%.2f %.2f %.2f 0 0 0</pose>
  <link name="trunk">
    %s
    %s
  </link>
%s
%s
</model>""" % (oid, cx, cy, h / 2.0,
              col_geo(cyl), vis_geo(cyl, mat(0.45, 0.32, 0.18)),
              "\n".join(crown_links), "\n".join(branch_links(ob)))


def bush_model(oid, lo, hi):
    cx = (lo[0] + hi[0]) / 2.0
    cy = (lo[1] + hi[1]) / 2.0
    w = hi[0] - lo[0]
    d = hi[1] - lo[1]
    h = hi[2] - lo[2]
    box = '<box><size>%.2f %.2f %.2f</size></box>' % (w, d, h)
    return """<model name="bush_%d">
  <static>true</static>
  <pose>%.2f %.2f %.2f 0 0 0</pose>
  <link name="box">
    %s
    %s
  </link>
</model>""" % (oid, cx, cy, h / 2.0,
              col_geo(box), vis_geo(box, mat(0.20, 0.55, 0.28)))


def pad_model(i, x, y):
    cyl = '<cylinder><radius>0.3</radius><length>0.05</length></cylinder>'
    return """<model name="pad_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.025 0 0 0</pose>
  <link name="pad">
    %s
    %s
  </link>
</model>""" % (i, x, y, col_geo(cyl), vis_geo(cyl, mat(0.25, 0.55, 0.35)))


def drop_model(dp):
    """投送平台（规则口径：直径约 1.2m 的有色标识平台）：r=0.6 薄盘 h=0.05，
    色=dp.color（color_map 权威）；外缘白色描边环作 1.2m 口径标识。

    平台整体即碰撞体（薄盘 h=0.05），任何巡航/悬停高度层均不构成障碍。
    """
    r, g, b = PLATFORM_COLORS.get(dp.color, (0.90, 0.90, 0.90))
    disc = '<cylinder><radius>0.60</radius><length>0.05</length></cylinder>'
    ring = '<cylinder><radius>0.68</radius><length>0.02</length></cylinder>'
    return """<model name="drop_%d">
  <static>true</static>
  <pose>%.2f %.2f 0.0 0 0 0</pose>
  <link name="platform">
    %s
    <visual name="disc"><pose>0 0 0.025 0 0 0</pose><geometry>%s</geometry>%s</visual>
    <visual name="rim"><pose>0 0 0.01 0 0 0</pose><geometry>%s</geometry>%s</visual>
  </link>
</model>""" % (dp.id, dp.xyz[0], dp.xyz[1],
              col_geo(disc), disc, mat(r, g, b), ring, mat(0.95, 0.95, 0.95))


def drone_model(i, x, y):
    r, g, b = DRONE_COLORS[i % len(DRONE_COLORS)]
    # 碰撞保持紧凑盒（0.5×0.5×0.2），与 python 后端 drone_radius/避障几何一致，物理行为不变；
    # 视觉升级为四旋翼：机身 + 顶部盖板 + 4 机臂 + 4 电机 + 4 螺旋桨（均仅视觉，无碰撞）。
    col_box = '<box><size>0.5 0.5 0.2</size></box>'

    vis = []
    # 中心机架（机身，用各机颜色）
    vis.append(vis_pose("chassis", '<box><size>0.16 0.16 0.05</size></box>',
                        (r, g, b), 0, 0, 0))
    # 顶部盖板（飞控/电池仓，深色）
    vis.append(vis_pose("top", '<box><size>0.09 0.09 0.015</size></box>',
                        (0.16, 0.16, 0.18), 0, 0, 0.033))
    # 机头航向标记（+X 方向小红点，便于观察朝向）
    vis.append(vis_pose("nose", '<box><size>0.03 0.03 0.035</size></box>',
                        (0.95, 0.25, 0.20), 0.085, 0, 0.01))
    # 十字型四旋翼：4 机臂 + 电机 + 螺旋桨
    motor = '<cylinder><radius>0.032</radius><length>0.03</length></cylinder>'
    prop = '<cylinder><radius>0.10</radius><length>0.006</length></cylinder>'
    for k, (dx, dy) in enumerate([(1, 0), (-1, 0), (0, 1), (0, -1)]):
        arm = ('<box><size>0.24 0.024 0.016</size></box>' if dx != 0
               else '<box><size>0.024 0.24 0.016</size></box>')
        ax, ay = dx * 0.12, dy * 0.12   # 机臂中心
        tx, ty = dx * 0.24, dy * 0.24   # 电机/桨中心
        vis.append(vis_pose("arm%d" % k, arm, (0.24, 0.24, 0.26), ax, ay, 0.01))
        vis.append(vis_pose("motor%d" % k, motor, (0.33, 0.33, 0.35), tx, ty, 0.032))
        vis.append(vis_pose("prop%d" % k, prop, (0.82, 0.82, 0.84, 0.55), tx, ty, 0.062))

    return """<model name="drone_%d">
  <pose>%.2f %.2f 1.0 0 0 0</pose>
  <link name="base_link">
    <inertial>
      <mass>2.0</mass>
      <inertia><ixx>0.04833</ixx><iyy>0.04833</iyy><izz>0.08333</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    %s
%s
%s
  </link>
  <plugin name="drone_vel_%d" filename="libdrone_vel_plugin.so">
    <drone_id>%d</drone_id>
    <kp>3.0</kp>
    <g>9.81</g>
  </plugin>
</model>""" % (i, x, y, col_geo(col_box), "\n".join(vis),
               drone_depth_sensor(i), i, i)


def drone_depth_sensor(i):
    """D435 前视深度相机（ego 路线 2026-10-05，docs/EGO_DESIGN.md §三）。

    ⚠ **本传感器是「渲染成本传感器」，帧率被场景三角面数直接决定**
    （2026-10-07 受控实测，详见 `tree_model_poplar` docstring）：请求
    `update_rate=15.0`，实际帧率随场景几何走 —— 叶 mesh 4100 面/棵时只有
    0.595 Hz，抽稀到 64 面/棵后回到 2.25 Hz。而 `grid_map/odom_depth_timeout
    = 1.0 s`（grid_map.cpp:53）是硬门：帧间隔一旦越过 1.0 s，`ego_replan_fsm`
    每拍打 `Depth Lost! EMERGENCY_STOP`，全机队永久急停。
    ⇒ **往场景里加渲染几何前，先量本话题的 Hz。** 6 台机各一个深度相机，
    成本是叠加的。

    - 真加 Gazebo depth sensor；ROS 桥用 libgazebo_ros_openni_kinect.so
      （FUEL/ego-planner 官方 sim 同款，XTDrone 同参数表）。
      Phase 0.5 v5 定谳：libgazebo_ros_camera.so 不读 depthImage* 参数
      （strings 全空，depth 专用参数只在 depth_camera/kinect 两个 .so）——
      旧配置深度话题从未被创建，是零帧第一病根。
    - 装机体系原点平视（pose 全 0）：cam2body_（grid_map.cpp:95 硬编码，平移零）
      不破，plan_env 零改动。
      ⚠️ **2026-10-06 订正：原注释写「own visuals 最大前伸=机臂尖 0.12+桨 0.10=0.22
      < near clip 0.30 ⇒ 自遮挡免疫」——两处错**：(a) 把机臂**半长** 0.12 当成臂尖，
      桨盘中心其实在 (±0.24, ±0.24)（见 `_drone_visuals`：`tx,ty = dx*0.24, dy*0.24`），
      盘 r=0.10 ⇒ 外缘伸到 **0.44 m**；(b) **near clip 只裁掉比它「近」的几何，
      比它「远」的照渲染**——所以「小于 clip」≠「不可见」。实测：桨盘落在 87°
      半视场内的内侧象限正好在 **[0.30, 0.40] m**，每帧 186 px 被画成深度图回波
      ⇒ 经 grid_map 变成**距机心 0.303 m 的 3 个占据格**（体固定，两机逐字相同）。
      该幻影在膨胀 ≥0.3 时吞掉机心致规划器全程拒停。**修法在配置侧：
      `depth_mindist = 0.45` 把它滤掉**（见 src/VENDORED.md、docs/ego_blind_flight_case.md）。
      若要改本文件，正解是让自几何进不了视场（相机前移 / 桨不进视锥），
      但前者会破 cam2body_ 平移零的契约，故本轮不动。
    - 话题契约对齐旧 depth_sim：/drone_<i>/depth/image_raw +
      /drone_<i>/depth/camera_info；RGB 发 image_color 不订。
      注意 kinect 插件无点云开关，points 话题会发（无订阅者时 roscpp
      不序列化，实测成本见 Phase 0.5 验收）。
    - 输出 32FC1 米制：grid_map.cpp:994-997 原生 convertTo(CV_16UC1,
      k_depth_scaling_factor=1000)，与 fd250 sim 链语义逐字节一致。
    - 内参（grid_map 走 launch 参数，不订 camera_info）：256x144 hfov 1.5184
      ⇒ fx=fy=134.9 cx=128 cy=72（advanced_param_zx.xml 同帧装订）。
      Phase 0.5 v7 实测 424x240 全栈 2.3Hz < 5Hz 门 → 降 256x144（像素 -64%）。
    - 渲染率验收门 ≥5Hz（WSL 无头 gzserver，llvmpipe 梯子见设计文档 §八）。
    """
    return """    <sensor name="d435_depth" type="depth">
      <pose>0 0 0 0 0 0</pose>
      <update_rate>15.0</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>1.5184</horizontal_fov>
        <image><width>256</width><height>144</height><format>R8G8B8</format></image>
        <clip><near>0.30</near><far>10.0</far></clip>
      </camera>
      <plugin name="d435_depth_ros" filename="libgazebo_ros_openni_kinect.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>15.0</updateRate>
        <cameraName>drone_%d/depth</cameraName>
        <imageTopicName>image_color</imageTopicName>
        <cameraInfoTopicName>rgb_camera_info</cameraInfoTopicName>
        <depthImageTopicName>image_raw</depthImageTopicName>
        <depthImageCameraInfoTopicName>camera_info</depthImageCameraInfoTopicName>
        <pointCloudTopicName>points</pointCloudTopicName>
        <frameName>drone_%d/depth_optical</frameName>
        <pointCloudCutoff>0.30</pointCloudCutoff>
        <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
      </plugin>
    </sensor>""" % (i, i)


# ---------------------------------------------------------------------------
# 新场景元素
# ---------------------------------------------------------------------------
def thin_box_model(name, x, y, z, sx, sy, sz, color, alpha=1.0, collision=True):
    """通用扁平盒模型，用于场地、功能区、穿越区、路缘、围栏等。"""
    box = '<box><size>%.2f %.2f %.2f</size></box>' % (sx, sy, sz)
    m = mat(color[0], color[1], color[2], alpha)
    col = col_geo(box) if collision else ""
    return """<model name="%s">
  <static>true</static>
  <pose>%.2f %.2f %.2f 0 0 0</pose>
  <link name="link">
    %s
    %s
  </link>
</model>""" % (name, x, y, z + sz / 2.0, col, vis_geo(box, m))


def ground_plane_model(size_x, size_y, color=(0.42, 0.56, 0.32)):
    # 用薄盒代替 plane,材质显示更稳定,且避免与 court 地面 z-fighting
    return thin_box_model("ground_plane", 0, 0, -0.02, size_x, size_y, 0.02,
                          color, collision=True)


def court_models():
    """起降清理带：土面（x∈[-30,-24] 清理带，取代旧沥青球场）。"""
    models = []
    models.append(thin_box_model("clearing_dirt", -27, 0, 0.0, 6.2, 60.4, 0.02,
                                 (0.60, 0.52, 0.36)))
    return models


def takeoff_pad_model():
    """起降区：水泥垫 + 黄色标识带（pads x=-27.75~-24.75, y=±3.75）。"""
    models = []
    models.append(thin_box_model("takeoff_pad", -27, 0, 0.02, 5.6, 11.0, 0.02,
                                 (0.73, 0.71, 0.67)))
    models.append(thin_box_model("takeoff_band", -27, 0, 0.05, 5.6, 1.0, 0.02,
                                 (0.94, 0.75, 0.25)))
    return models


def crossing_zone_models():
    """穿越区：橙色透明面 + 棕色边框（林带 52×56，中心 (2,0)）。"""
    models = []
    models.append(thin_box_model("crossing_zone", 2, 0, 0.04, 52, 56, 0.02,
                                 (0.85, 0.48, 0.31), alpha=0.5))
    return models


def fence_models():
    """场地外围围栏（与 scene_topology.yaml static_obstacles fence AABB 对齐）：
    ±30 墙线、高 2.80（冠下体制真实侧墙；2.80 > 点云带上限 2.60）。"""
    models = []
    models.append(thin_box_model("fence_w", -30.15, 0, 0.0, 0.3, 60.9, 2.8,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_e",  30.15, 0, 0.0, 0.3, 60.9, 2.8,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_n", 0,  30.15, 0.0, 60.9, 0.3, 2.8,
                                 (0.31, 0.42, 0.27)))
    models.append(thin_box_model("fence_s", 0, -30.15, 0.0, 60.9, 0.3, 2.8,
                                 (0.31, 0.42, 0.27)))
    return models


def net_model(ob):
    """kind=net 垂直屏障网→绿色半透明薄板（2026-09-27 勘察视频定谳：林缘
    遮阳网顶高≈2-3m）。单一事实源=scene_topology.yaml static_obstacles；
    collision 开=接触判罚+LiDAR 可见（v1 规划层验证；真机 Mid360 对网
    结构性失明，盲态由装订禁行兜底，见 scene_topology net 段注）。"""
    lo, hi = ob.lo, ob.hi
    cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
    sx = max(hi[0] - lo[0], 0.05)
    sy = max(hi[1] - lo[1], 0.05)
    sz = hi[2] - lo[2]
    return thin_box_model("net_%d" % ob.id, cx, cy, lo[2], sx, sy, sz,
                          (0.20, 0.45, 0.22), alpha=0.45, collision=True)


# ---------------------------------------------------------------------------
# 主构建流程
# ---------------------------------------------------------------------------
def build():
    scene = Scene()
    v = scene.venue
    out = []
    out.append('<?xml version="1.0"?>')
    out.append('<sdf version="1.7">')
    out.append('<world name="zx2026_forest">')
    out.append("""  <physics type="ode">
    <max_step_size>0.002</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>500</real_time_update_rate>
  </physics>
  <gravity>0 0 -9.81</gravity>
  <include><uri>model://sun</uri></include>""")

    # 地面（按 venue size）
    out.append(ground_plane_model(v["size"][0], v["size"][1]))

    # 主场地元素
    out.extend(court_models())
    out.extend(takeoff_pad_model())
    out.extend(crossing_zone_models())
    out.extend(fence_models())

    # 垂直屏障网（static_obstacles kind=net；z 带外 A* 自动放行=过顶分界）
    n_net = 0
    for ob in scene.obstacles:
        if ob.kind == "net":
            out.append(net_model(ob))
            n_net += 1

    # 树/灌木（table 场景按表级 mesh_kind 分派：spruce=云杉 dae / 其余=杨树 STL；
    # 旧程序化场景=圆柱树）
    n_tree = n_bush = 0
    for ob in scene.obstacles:
        if ob.kind == "tree":
            if ob.mesh_kind == "spruce":
                out.append(tree_model_spruce(ob.id, ob))
            elif ob.mesh_variant is not None:
                out.append(tree_model_poplar(ob.id, ob))
            else:
                out.append(tree_model(ob.id, ob))
            n_tree += 1
        elif ob.kind == "bush":
            out.append(bush_model(ob.id, ob.lo, ob.hi))
            n_bush += 1

    # 起降点、投放点、无人机
    for i, (px, py) in enumerate(scene.get_pads()):
        out.append(pad_model(i, px, py))
    for dp in scene.drop_points:
        out.append(drop_model(dp))
    for i, (px, py) in enumerate(scene.get_pads()):
        out.append(drone_model(i, px, py))

    out.append("</world>")
    out.append("</sdf>")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("\n".join(out))
    print("wrote %s  (trees=%d bushes=%d nets=%d pads=%d drops=%d drones=%d)" %
          (OUT, n_tree, n_bush, n_net, len(scene.get_pads()),
           len(scene.drop_points), len(scene.get_pads())))


if __name__ == "__main__":
    build()
