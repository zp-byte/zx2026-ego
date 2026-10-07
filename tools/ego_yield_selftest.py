# -*- coding: utf-8 -*-
"""ego_bridge 恢复期让出执行器（yield_actuator）离线自检。

不复现 ROS 运行时：桩掉 rospy / 消息类型 / zx2026_common.config，用 importlib
加载**真源码** ego_bridge_node.py，驱动真的 _on_collision / _tick，读
pub_raw / pub_dbg 的**实参**。

修复的机制（2026-10-07 冒烟实证）：
  桥的 SETTLE_S 零速经 nav 的 _on_pen_raw 中继以 50Hz 上 /vel_cmd，与
  gazebo_collision_monitor 的 100Hz 弹开令逐拍对撞 → 逃逸脉冲被掐掉
  （d2 被令 2.0m/s×1.0s 只位移 0.10m，离撞点峰值 0.25m < 0.60m = 楔死）。
  修法 = 恢复窗内 pub_raw **整窗静默（含零）**，vel_cmd 让监控器独占。
  ⚠ 联合条件：dbg 心跳必须继续 —— nav 的 stall 看门狗（pen_stall_freeze_t
  =1.0s）会在心跳停摆时改发 Twist() 零速，换个写者继续掐弹开。窗长 2.5s >
  1.0s 阈值 ⇒ 不喂心跳必每窗触发。故 Y1 同时断言「pub_raw 零发布」与
  「pub_dbg 有发布」，缺一即修复失效。

断言口径：Y=行为（驱动真 _tick/_on_collision 读实参）/ C=配置路径 / D=真 YAML。
"""
import ast
import importlib.util
import sys
import types

WS = '/home/ubuntu/zx2026_ego_ws'
SRC = WS + '/src/ego_zx/scripts/ego_bridge_node.py'

# ---------------------------------------------------------------- 桩运行时
CLOCK = {'t': 1000.0}
PUBS = {}


class _T(object):
    def __init__(self, s):
        self._s = s

    def to_sec(self):
        return self._s


class _Vec(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist(object):
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()


class _Msg(object):
    def __init__(self, data=None):
        self.data = data


class PositionCommand(object):
    def __init__(self, vx, vy, vz, wz):
        self.velocity = _Vec()
        self.velocity.x, self.velocity.y, self.velocity.z = vx, vy, vz
        self.yaw_dot = wz


class FakePub(object):
    def __init__(self, name):
        self.resolved_name = name
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


rospy = types.ModuleType('rospy')
rospy.loginfo = lambda *a, **k: None
rospy.loginfo_throttle = lambda *a, **k: None
rospy.logwarn = lambda *a, **k: None
rospy.logwarn_throttle = lambda *a, **k: None
rospy.logerr = lambda *a, **k: None
rospy.get_time = lambda: CLOCK['t']
rospy.get_rostime = lambda: _T(CLOCK['t'])
rospy.get_param = lambda _n, d=None: d
rospy.init_node = lambda *a, **k: None
rospy.spin = lambda *a, **k: None
rospy.is_shutdown = lambda: False
rospy.Duration = lambda s: s
rospy.Timer = lambda *a, **k: None
rospy.Subscriber = lambda *a, **k: None


def _pub(name, _typ=None, **_kw):
    p = PUBS.get(name)
    if p is None:
        p = PUBS[name] = FakePub(name)
    return p


rospy.Publisher = _pub
sys.modules['rospy'] = rospy

for _mod, _names in (
        ('geometry_msgs.msg', ['Twist']),
        ('nav_msgs.msg', ['Odometry']),
        ('sensor_msgs.msg', ['Image']),
        ('quadrotor_msgs.msg', ['PositionCommand']),
        ('std_msgs.msg', ['Bool', 'String'])):
    _m = types.ModuleType(_mod)
    for _n in _names:
        setattr(_m, _n, {'Twist': Twist, 'PositionCommand': PositionCommand,
                         'Bool': _Msg, 'String': _Msg,
                         'Odometry': _Msg, 'Image': _Msg}[_n])
    sys.modules[_mod] = _m

# ---- 可注入的 zx2026_common.config ----
CFG = {'mode': 'ok', 'ret': {}}
_cfgmod = types.ModuleType('zx2026_common.config')


def _cfg_load(_name):
    if CFG['mode'] == 'raise':
        raise RuntimeError('injected cfg failure')
    return CFG['ret']


_cfgmod.load = _cfg_load
_pkg = types.ModuleType('zx2026_common')
_pkg.config = _cfgmod
sys.modules['zx2026_common'] = _pkg
sys.modules['zx2026_common.config'] = _cfgmod

_spec = importlib.util.spec_from_file_location('ego_bridge_node', SRC)
ego = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ego)

# ---------------------------------------------------------------- 断言框架
FAIL = []


def chk(name, cond, extra=''):
    print('  %-4s %s%s' % ('PASS' if cond else 'FAIL', name,
                           ('   [%s]' % extra) if extra else ''))
    if not cond:
        FAIL.append(name)


def reset(cfg_ret=None, cfg_raise=False):
    PUBS.clear()
    CFG['mode'] = 'raise' if cfg_raise else 'ok'
    CFG['ret'] = cfg_ret if cfg_ret is not None else {}


def mkbridge(cfg_ret=None, cfg_raise=False, t0=1000.0):
    reset(cfg_ret, cfg_raise)
    CLOCK['t'] = t0
    return ego.EgoBridge()


def raw(b):
    return PUBS['/drone_0/planning/cpn_vel_raw'].msgs


def dbg(b):
    return PUBS['/drone_0/defop/debug'].msgs


def fresh(b, dv=(1.6, 0.0, 0.5), wz=0.3):
    """刷活持笔旗 + 给一发新鲜 cmd（两样都要，否则走到别的分支）。"""
    b._on_active(_Msg(True))
    b._on_cmd(PositionCommand(dv[0], dv[1], dv[2], wz))


def rc(**kw):
    return {'collision': {'recovery': kw}}


def is_zero(tw):
    return (tw.linear.x == 0.0 and tw.linear.y == 0.0
            and tw.linear.z == 0.0 and tw.angular.z == 0.0)


def ast_no_raw_publish_in_yield():
    """Y-AST 强护栏：_tick 里 `if yielding:` 块内不得出现 pub_raw.publish。"""
    tree = ast.parse(open(SRC).read())
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        # 条件必须是裸名 yielding
        if not (isinstance(node.test, ast.Name) and node.test.id == 'yielding'):
            continue
        for sub in ast.walk(ast.Module(body=node.body, type_ignores=[])):
            if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute):
                if sub.func.attr == 'publish':
                    tgt = sub.func.value
                    if isinstance(tgt, ast.Attribute) and tgt.attr == 'pub_raw':
                        return False
        return True
    return False   # 没找到 if yielding: 块 = 结构已改，视为失败


print('=== ego_bridge yield_actuator 离线自检 ===')

# ---- C 配置路径 ----------------------------------------------------------
print('[C] 配置读取')
b = mkbridge(rc(yield_actuator=False))
chk('C1 非默认 false 读回 False', b.yield_actuator is False)

b = mkbridge({})                                  # 无 collision 键
chk('C2 缺键退默认 True', b.yield_actuator is True)

b = mkbridge(rc(yield_actuator='off'))            # YAML1.1 裸 on/off 字符串陷阱
chk('C3 字符串 "off" -> False（YAML1.1 陷阱）', b.yield_actuator is False)

b = mkbridge(rc(yield_actuator='true'))
chk('C4 字符串 "true" -> True', b.yield_actuator is True)

b = mkbridge(cfg_raise=True)
chk('C5 cfg.load 抛异常 -> 退默认 True 且**实例化不抛**（桥起不来=六机全冻）',
    b.yield_actuator is True)

# ---- Y 行为 --------------------------------------------------------------
print('[Y] 行为（驱动真 _tick）')

# Y1 恢复窗内 pub_raw 零发布 + pub_dbg 心跳继续（**联合条件**）
b = mkbridge(rc(yield_actuator=True))
fresh(b)
b._tick(None)
base_raw = len(raw(b))
chk('Y1a 基线：正常拍发 1 条 pub_raw', base_raw == 1, 'n=%d' % base_raw)
b._on_collision(_Msg(True))                        # 上升沿 -> 让出
CLOCK['t'] += 0.3
fresh(b)
b._tick(None)
chk('Y1b 让出窗内 pub_raw **零发布**（含零）',
    len(raw(b)) == base_raw, 'Δ=%d' % (len(raw(b)) - base_raw))
chk('Y1c 让出窗内 pub_dbg 心跳**仍在发**（nav 看门狗不被饿死）',
    len(dbg(b)) >= 2, 'dbg n=%d' % len(dbg(b)))

# Y2 下降沿交还
b._on_collision(_Msg(False))
CLOCK['t'] += 0.1
fresh(b)
b._tick(None)
chk('Y2 下降沿后交还：pub_raw 恢复发布',
    len(raw(b)) == base_raw + 1, 'n=%d' % len(raw(b)))
m = raw(b)[-1]
chk('Y2b 交还后发的是真 cmd（非零）',
    abs(m.linear.x - 1.6) < 1e-9 and abs(m.linear.z - 0.5) < 1e-9,
    '(%.2f,%.2f,%.2f)' % (m.linear.x, m.linear.y, m.linear.z))

# Y3 让出窗内 cmd 更新也不泄漏
b = mkbridge(rc(yield_actuator=True))
fresh(b)
b._on_collision(_Msg(True))
n0 = len(raw(b))
for _ in range(5):
    CLOCK['t'] += 0.02
    fresh(b)                                       # 每拍都有新鲜 cmd + 活旗
    b._tick(None)
chk('Y3 让出窗 5 拍**零泄漏**（cmd 再新鲜也不发）', len(raw(b)) == n0,
    'Δ=%d' % (len(raw(b)) - n0))

# Y4 边沿自同步：窗长与 bounce_dt/cooldown_dt 无关
b = mkbridge(rc(yield_actuator=True, bounce_dt=1.0, cooldown_dt=1.5))
fresh(b)
b._on_collision(_Msg(True))
n0 = len(raw(b))
for _ in range(50):                                # 推进 5.0s ≫ 2.5s 恢复窗
    CLOCK['t'] += 0.1
    fresh(b)
    b._tick(None)
chk('Y4 未收下降沿则一直让出（窗长=下降沿，不镜像配置）',
    len(raw(b)) == n0, 'Δ=%d @+5.0s' % (len(raw(b)) - n0))

# Y5 看门狗：超 YIELD_MAX_S 收回执行器（防监控器死致永久静默）
CLOCK['t'] = 1000.0 + ego.YIELD_MAX_S + 1.0
fresh(b)
b._tick(None)
chk('Y5 超 YIELD_MAX_S 收回执行器并恢复发布',
    len(raw(b)) == n0 + 1, 'YIELD_MAX_S=%.0f n=%d' % (ego.YIELD_MAX_S, len(raw(b))))

# Y6 关闭档 = 旧行为（SETTLE_S 内发**精确零**）
b = mkbridge(rc(yield_actuator=False))
fresh(b)
b._tick(None)
n0 = len(raw(b))
b._on_collision(_Msg(True))
CLOCK['t'] += 0.3
fresh(b)
b._tick(None)
chk('Y6 关闭档：窗内发精确零 Twist（逐字节旧行为）',
    len(raw(b)) == n0 + 1 and is_zero(raw(b)[-1]),
    'n=%d zero=%s' % (len(raw(b)) - n0, is_zero(raw(b)[-1]) if len(raw(b)) > n0 else 'n/a'))

# Y7 latched 首帧 True 的幻影上升沿（钉现状，防后人误当 bug 顺手改掉）
b = mkbridge(rc(yield_actuator=True))
chk('Y7 未收 False 直接喂 True（latched 幻影沿）-> 仍触发让出', b._col_prev is False)
b._on_collision(_Msg(True))
chk('Y7b 幻影沿让出成立（记档行为）', b._yield is True)

# Y8 恢复窗结束后 _yield 复位、可再次让出（重臂）
b = mkbridge(rc(yield_actuator=True))
b._on_collision(_Msg(True))
b._on_collision(_Msg(False))
chk('Y8a 下降沿清 _yield', b._yield is False)
b._on_collision(_Msg(True))
chk('Y8b 再碰撞可重新让出（重臂）', b._yield is True)

# ---- AST 强护栏 ----------------------------------------------------------
print('[A] 静态护栏')
chk('A1 _tick 的 `if yielding:` 块内无 pub_raw.publish（静默是硬约束）',
    ast_no_raw_publish_in_yield())

# ---- D 真 YAML -----------------------------------------------------------
print('[D] 真 YAML')
import yaml
_p = WS + '/src/zx2026_common/config/sim_settings.yaml'
_y = yaml.safe_load(open(_p))
_found = False
for _ln, _line in enumerate(open(_p), 1):
    if _line.strip().startswith('yield_actuator:'):
        _found = True
chk('D1 真 YAML 含 yield_actuator 键', _found)
_v = _y['collision']['recovery'].get('yield_actuator')
chk('D2 盘上默认档 = True 且类型为 bool（非字符串）',
    _v is True and isinstance(_v, bool), repr(_v))

print()
if FAIL:
    print('=== FAIL %d 条: %s ===' % (len(FAIL), ', '.join(FAIL)))
    sys.exit(1)
print('=== ALL PASS ===')
