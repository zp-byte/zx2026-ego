#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_cfg_preset.py paper|g1|show — DeFoP 参数档切换（固定速率 A/B 用）。

对照口径（单因子束）：论文原值 vs G1 修复档，**只动论文来源的三个旋钮**
（cmd_velocity / vel_max / sup 三件套）；我方自有机件 collision_settle_s 两档
恒为 3.0（③ 开启），保证对照只反映参数差、不掺杂机制差。

  档   cmd_velocity  vel_max  sup(min_dist/min_ratio/drone_radius)
  paper     1.0        2.0      1.00 / 0.05 / 0.30   <- evaluate.py:1085 调用点实参
  g1        1.3        1.5      1.50 / 0.02 / 0.32   <- filter_actions_by_grid_3 签名默认

按键正则替换（可反复重放，不依赖锚点唯一性）；yaml 与 default_cfg 双侧同帧。

两档**额外钉死** `sup_bearing_mode='off'`（④ 监督层几何口径，2026-10-01）：它不是
论文/G1 的对照变量（上游只有 off 一种口径），是本仓的实验开关；不钉死会留下
"档位已重置、监督口径还停在实验值"的混档。见文件末 STR_KEYS 与 sub_*_str。

⚠ 2026-10-01 体半径注记：本工具两档的 `drone_radius` 都是**历史档位原值**
（paper = 论文 0.4m 机架实参 0.30、g1 = 上游签名缺省 0.32），**都不是本 sim
机体的尺寸**。本 sim 机体 = world_builder 的 0.5×0.5×0.2 盒（面内外接 0.354），
判据（gazebo_collision_monitor._tree_hit）与 nav 膨胀基都取 0.35。
故：**本工具只用于复现论文/G1 对照，不是"修好"档**；跑完对照要回本仓口径，
须把 sup_drone_radius 写回 0.35——守卫 tools/defop_sup_selftest.py 会当场
FAIL 并指路。
"""
import io
import re
import sys

WS = '/home/ubuntu/zx2026_defop_ws/src/defop_nav'
YAML = WS + '/config/defop_nav.yaml'
PY = WS + '/scripts/defop_cpn_node.py'

PRESETS = {
    'paper': {'cmd_velocity': '1.0', 'vel_max': '2.0', 'sup_min_dist': '1.0',
              'sup_min_ratio': '0.05', 'sup_drone_radius': '0.30'},
    'g1': {'cmd_velocity': '1.3', 'vel_max': '1.5', 'sup_min_dist': '1.5',
           'sup_min_ratio': '0.02', 'sup_drone_radius': '0.32'},
}
SETTLE = '3.0'          # 两档恒定：③ 撞后镇定（自有机件，非论文参数）
# ④ 监督层几何口径：两档**都钉死 off**。它不是论文/G1 的对照变量（上游只有
# 一种口径 = off），而是本仓的实验开关；不钉死就会出现"档位已重置、监督口径
# 还停在实验值"的混档。切档语义 = 回到上游口径，然后由此处再显式开实验。
STR_KEYS = {'sup_bearing_mode': 'off'}
# ⑤ 死端超时/迟滞（2026-10-02 立案）：两档**都钉死 0.0**（上游两态 latch）。
# 同上，它不是论文/G1 的对照变量（上游 evaluate.py:309-326 无超时无迟滞），
# 是本仓实验开关；不钉死会留下"档位已重置、死端还在实验参数"的混档。
# 0.0 走**数值**通道（sub_line/sub_dict），与 STR_KEYS 分开。
ZERO_KEYS = ('deadend_max_s', 'deadend_exit_hyst_s')


def sub_line(text, key, val):
    """yaml 形态：`key: <num>`（保留行尾注释）。"""
    pat = re.compile(r'^(%s\s*:\s*)[0-9.]+' % re.escape(key), re.M)
    new, n = pat.subn(lambda m: m.group(1) + val, text)
    if n != 1:
        raise SystemExit('FAIL yaml %s: n=%d' % (key, n))
    return new


def sub_dict(text, key, val):
    """python dict 形态：`'key': <num>`。"""
    pat = re.compile(r"('%s':\s*)[0-9.]+" % re.escape(key))
    new, n = pat.subn(lambda m: m.group(1) + val, text)
    if n != 1:
        raise SystemExit('FAIL py %s: n=%d' % (key, n))
    return new


# ⚠ 非数值键必须走下面两条**独立**通道：上面的 [0-9.]+ 不认字符串，
#   把 sup_bearing_mode 塞进 sub_line 会 n=0 -> SystemExit（不是静默跳过）。
#   yaml 侧写成带引号的 'off'，否则 YAML 1.1 会把裸 `off` 解析成布尔 False。
def sub_line_str(text, key, val):
    """yaml 形态：`key: 'str'`（引号可选，回写时统一加单引号）。"""
    pat = re.compile(r"^(%s\s*:\s*)'?[A-Za-z_][A-Za-z0-9_]*'?"
                     % re.escape(key), re.M)
    new, n = pat.subn(lambda m: m.group(1) + "'" + val + "'", text)
    if n != 1:
        raise SystemExit('FAIL yaml %s: n=%d' % (key, n))
    return new


def sub_dict_str(text, key, val):
    """python dict 形态：`'key': 'str'`。"""
    pat = re.compile(r"('%s':\s*)'?[A-Za-z_][A-Za-z0-9_]*'?" % re.escape(key))
    new, n = pat.subn(lambda m: m.group(1) + "'" + val + "'", text)
    if n != 1:
        raise SystemExit('FAIL py %s: n=%d' % (key, n))
    return new


def apply(preset):
    vals = dict(PRESETS[preset])
    vals['collision_settle_s'] = SETTLE
    for k in ZERO_KEYS:
        vals[k] = '0.0'
    for path, sub, ssub in ((YAML, sub_line, sub_line_str),
                            (PY, sub_dict, sub_dict_str)):
        with io.open(path, encoding='utf-8') as f:
            s = f.read()
        for k, v in vals.items():
            s = sub(s, k, v)
        for k, v in STR_KEYS.items():
            s = ssub(s, k, v)
        with io.open(path, 'w', encoding='utf-8') as f:
            f.write(s)
    print('preset=%s applied -> %s + %s' % (preset, vals, STR_KEYS))


def show():
    with io.open(YAML, encoding='utf-8') as f:
        y = f.read()
    for k in ('cmd_velocity', 'vel_max', 'sup_min_dist', 'sup_min_ratio',
              'sup_drone_radius', 'collision_settle_s'):
        m = re.search(r'^%s\s*:\s*([0-9.]+)' % k, y, re.M)
        print('  yaml %-18s = %s' % (k, m.group(1) if m else '?'))
    for k in sorted(STR_KEYS):
        m = re.search(r"^%s\s*:\s*'?([A-Za-z_][A-Za-z0-9_]*)'?" % k, y, re.M)
        print('  yaml %-18s = %s' % (k, m.group(1) if m else '?'))
    for k in ZERO_KEYS:
        m = re.search(r'^%s\s*:\s*([0-9.]+)' % k, y, re.M)
        print('  yaml %-18s = %s  (两档恒 0.0)' % (k, m.group(1) if m else '?'))


if __name__ == '__main__':
    a = sys.argv[1] if len(sys.argv) > 1 else 'show'
    if a == 'show':
        show()
    elif a in PRESETS:
        apply(a)
        show()
    else:
        raise SystemExit('usage: paper|g1|show')
