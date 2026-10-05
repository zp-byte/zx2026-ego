#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""defop_deadend_selftest.py — 死端旋转超时 + 退出迟滞 + 强制放宽 的守卫。

背景（2026-10-02 立案，docs/deadend_timeout_case.md）
-----------------------------------------------------
上游 evaluate.py:309-326 的 deadend_action 是**无超时无迟滞的两态 latch**：
唯一出口是「min_score < deadend_low 且 len(safe)>0」，一旦该条长期不成立
（β 档掩膜把 safe 集变稀）就"进得去出不来"，无界原地旋转、零前进烧预算。
本仓实测相邻回合最短仅 0.6s（空↔非空抖动反复进出）。

本守卫的目标（**可失败**，不是"跑个报告"）
-----------------------------------------
[A] 反向注入：未修（max_s=0）时永久死端 ⇒ 永远旋转 —— 证明缺陷真实存在。
[B] 超时：max_s=4.0 时 3.9s 不得进状态 2、4.1s 必须进。
[C] 迟滞：低条件须**连续**满足 hyst_s 才退出；只满足一拍不得退出。
[D] 迟滞窗口内条件断裂 ⇒ 计时清零重计（同 arena_nav de_selftest 的 H2）。
[E] 状态 2 只在**高**条件（比进入档更宽）持续后交还常规；"仅达低条件"不得退出
    —— 这正是上游 evaluate.py:835 用 has_deadend_in_segment 想防的"模式抖动"。
[F] **防双实现**：`_deadend_action`（真机）与 `_shadow_de_tick`（影子）必须
    都调用同一个 `_deadend_step_logic`。任一被改成内联副本即 FAIL
    （本仓已在 _crown_z_range 上中过"两份实现、单改无效"的同族）。
[G] **新旧对拍**：把改动前的两态 latch 内联为 reference，对一组覆盖性输入序列
    断言 `max_s=hyst_s=0` 时逐拍输出一致 —— 「默认档逐字节旧行为」的机器证明。
[H] 降级档（min_score is None）：以"监督器是否全否决"为准，不得因本案改动而漂移。
[I] **步长钳位**：`plan_once` 在 nav 持笔/碰撞镇定窗不执行，其间隙不得整段
    充进死端状态时长（110544/00 d1 法证：replica `t_rot`=7.7s vs 真机 mode 线
    `rot_s`=0.72s）。cap=0 必须复现旧缺陷，cap>0 必须钳并置旗。

用法（无需 ROS：只从源码抽 `_deadend_step_logic` 纯函数，不 import 节点）
    cd ~/zx2026_defop_ws && python3 tools/defop_deadend_selftest.py
"""
import io
import os
import random
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# 源码路径可用 DEFOP_CPN_PY 覆盖 —— 供**反向注入**验证本守卫自身可失败
# （把 _deadend_action 改回内联旧实现，[F] 必须当场 FAIL）。
PY = os.environ.get('DEFOP_CPN_PY') or os.path.join(
    HERE, '..', 'src', 'defop_nav', 'scripts', 'defop_cpn_node.py')

fails = 0


def chk(label, got, want):
    global fails
    ok = (got == want)
    if not ok:
        fails += 1
    print('  %-58s %-4s got=%s want=%s' % (label, 'OK' if ok else 'FAIL',
                                           got, want))
    return ok


def _extract_func(src, name):
    m = re.search(r'^def %s\(.*?(?=^def |\Z)' % re.escape(name),
                  src, re.M | re.S)
    return m.group(0) if m else None


def _extract_method(src, name):
    m = re.search(r'^    def %s\(.*?(?=^    def |^class |\Z)'
                  % re.escape(name), src, re.M | re.S)
    return m.group(0) if m else None


# ---------------------------------------------------------------- 源码抽取
if not os.path.exists(PY):
    sys.stderr.write('找不到 %s\n' % PY)
    sys.exit(2)
SRC = io.open(PY, encoding='utf-8').read()

_f = _extract_func(SRC, '_deadend_step_logic')
if not _f:
    sys.stderr.write('抽取失败：_deadend_step_logic（模块级纯函数）\n')
    sys.exit(2)
# ③ 的两个辅助纯函数与状态机同批抽（状态机调用它们；漏抽即 NameError，
# 这本身就是"新增了未随行的依赖"的机器化探针）。
_fa = _extract_func(SRC, '_sup_bad')
_fb = _extract_func(SRC, '_sup_dwell')
for _n, _body in (('_sup_bad', _fa), ('_sup_dwell', _fb)):
    if not _body:
        sys.stderr.write('抽取失败：%s（③ 监督层条件的单一实现）\n' % _n)
        sys.exit(2)
_ns = {}
exec(compile(_f, PY, 'exec'), _ns)          # 只含纯算术，无 ROS/np 依赖
exec(compile(_fa, PY, 'exec'), _ns)
exec(compile(_fb, PY, 'exec'), _ns)
step_logic = _ns['_deadend_step_logic']
sup_bad = _ns['_sup_bad']
sup_dwell = _ns['_sup_dwell']

_c = _extract_func(SRC, '_clamp_dt')
if not _c:
    sys.stderr.write('抽取失败：_clamp_dt（模块级纯函数）\n')
    sys.exit(2)
_cns = {}
exec(compile(_c, PY, 'exec'), _cns)
clamp_dt = _cns['_clamp_dt']

HI, LO = 6.5, 5.0


def new_st():
    return {'t0': None, 'ok_t0': None}


def drive(seq, max_s, hyst_s, hi=HI, lo=LO):
    """按 (now, min_score, n_safe) 序列驱动，返回逐拍的 (step, action, forced)。"""
    st, out = new_st(), []
    step = 0
    for now, ms, ns in seq:
        step, act, forced = step_logic(step, ms, ns, now, st, hi, lo,
                                       max_s, hyst_s)
        out.append((step, act, forced))
    return out


def drive_v(seq, max_s, hyst_s, veto_hi, dwell, hi=HI, lo=LO):
    """同 drive，但序列是 (now, min_score, n_safe, veto_frac)。"""
    st, out = new_st(), []
    step = 0
    for now, ms, ns, vf in seq:
        step, act, forced = step_logic(step, ms, ns, now, st, hi, lo,
                                       max_s, hyst_s,
                                       veto_frac=vf, veto_hi=veto_hi,
                                       veto_dwell_s=dwell)
        out.append((step, act, forced))
    return out


def ref_old(seq, hi=HI, lo=LO):
    """**改动前**的两态 latch（evaluate.py:309-326 逐字转写），作对拍基准。

    返回逐拍的 (step, action, forced)；旧实现没有状态 2，forced 恒 False。
    """
    step, out = 0, []
    for _now, ms, ns in seq:
        hi_ok = (ns > 0) if ms is None else (ms < hi and ns > 0)
        lo_ok = (ns > 0) if ms is None else (ms < lo and ns > 0)
        if step == 0 and hi_ok:
            act = 'success'
        elif step == 1 and lo_ok:
            step, act = 0, 'success'
        elif step == 0:
            step, act = 1, 'stop'
        else:
            act = 'rotate'
        out.append((step, act, False))
    return out


DEAD = 9.0        # min_score 高 = 死端（>= deadend_high）
SAFE_HI = 1.0     # 高/低条件都成立
SAFE_LO = 5.5     # 只达低条件（5.0 < 5.5 < 6.5）：状态 2 的"不得退出"探针


def seq_dead(n, dt=0.1, t0=0.0):
    return [(t0 + i * dt, DEAD, 0) for i in range(n)]


def seq_hold(n, dt=0.1, t0=0.0, ms=DEAD, ns=0):
    return [(t0 + i * dt, ms, ns) for i in range(n)]


print('=' * 72)
print('defop 死端超时/迟滞守卫  (%s)' % os.path.relpath(PY, HERE))
print('=' * 72)

# ---------------------------------------------------------------- [A] 反向注入
print('\n[A] 反向注入：未修档（max_s=0）永久死端 => 无界旋转')
o = drive(seq_dead(200), max_s=0.0, hyst_s=0.0)
chk('max_s=0 时永不进状态 2（缺陷存在）', any(r[2] for r in o), False)
chk('200 拍后仍停在状态 1', o[-1][0], 1)
chk('末拍仍是 rotate（无界旋转）', o[-1][1], 'rotate')

# ---------------------------------------------------------------- [B] 超时
print('\n[B] 超时门限：max_s=4.0（hyst_s=0）')
o = drive(seq_dead(80), max_s=4.0, hyst_s=0.0)
by_t = {round(0.0 + i * 0.1, 1): r for i, r in enumerate(o)}
chk('t=3.9s 未进状态 2', by_t[3.9][2], False)
chk('t=3.9s 仍是 rotate', by_t[3.9][1], 'rotate')
chk('t=4.0s 必须进状态 2', by_t[4.0][2], True)
chk('t=4.0s action=forced', by_t[4.0][1], 'forced')
chk('t=6.0s 仍处状态 2（保持，不来回翻）', by_t[6.0][0], 2)
chk('首拍为 stop（进入即稳一拍）', o[0][1], 'stop')

# ---------------------------------------------------------------- [C] 迟滞
print('\n[C] 退出迟滞：hyst_s=2.5（max_s=0 隔离）')
o = drive([(0.0, DEAD, 0), (1.0, DEAD, 0), (10.0, SAFE_HI, 5),
           (10.1, SAFE_HI, 5), (12.4, SAFE_HI, 5), (12.5, SAFE_HI, 5)],
          max_s=0.0, hyst_s=2.5)
chk('低条件首拍（10.0）不得退出', o[2][0], 1)
chk('迟滞窗口内（10.1）不得退出', o[3][0], 1)
chk('差一点（12.4）不得退出', o[4][0], 1)
chk('满 2.5s（12.5）必须退出', o[5], (0, 'success', False))

# ---------------------------------------------------------------- [D] 断裂重计
print('\n[D] 迟滞窗口内条件断裂 => 计时清零重计')
o = drive([(0.0, DEAD, 0), (10.0, SAFE_HI, 5), (10.5, DEAD, 0),
           (11.0, SAFE_HI, 5), (13.4, SAFE_HI, 5), (13.5, SAFE_HI, 5)],
          max_s=0.0, hyst_s=2.5)
chk('断裂后重置起点：13.4 不得退出', o[4][0], 1)
chk('自重计满 2.5s：13.5 必须退出', o[5], (0, 'success', False))

# ---------------------------------------------------------------- [E] 状态2退出档
# 关键关系：deadend_low(5.0) < deadend_high(6.5) ⇒ lo_ok 是**更严**的条件。
# 分数 5.5 恰好"满足 hi(6.5) 但不满足 lo(5.0)"= 上游 :835 想防的那个夹缝。
print('\n[E] 状态 2 的退出档比状态 1 宽（上游 evaluate.py:835 的原意）')
# 先押进状态 1（一拍死端），再喂 5.5 —— 陷阱是"进了才出不来"（从状态 0 起
# 时 5.5 满足 hi ⇒ 直接 success，根本不会进）。
o = drive([(0.0, DEAD, 0)] + seq_hold(80, dt=0.1, t0=0.1, ms=SAFE_LO, ns=5),
          max_s=0.0, hyst_s=2.5)
chk('[E1] 已在状态1 + 仅达 hi 条件(5.5) => 永不出（β 陷阱）', o[-1][0], 1)
chk('[E1] 80 拍后仍是 rotate', o[-1][1], 'rotate')
o = drive([(0.0, DEAD, 0)] + seq_hold(80, dt=0.1, t0=0.1, ms=SAFE_LO, ns=5),
          max_s=4.0, hyst_s=2.5)
chk('[E2] t=4.0 超时 => 状态 2', o[40][0], 2)
chk('[E2] 4.1 起计迟滞，6.5 仍不得退出', o[65][0], 2)
chk('[E2] 满 2.5s（6.6）交还状态 0', o[66], (0, 'success', False))

# ---------------------------------------------------------------- [F] 防双实现
print('\n[F] 防双实现：真机与影子必须共用同一个 _deadend_step_logic')
m_real = _extract_method(SRC, '_deadend_action')
m_shadow = _extract_method(SRC, '_shadow_de_tick')
chk('抽到 _deadend_action', m_real is not None, True)
chk('抽到 _shadow_de_tick', m_shadow is not None, True)
chk('真机侧调用 _deadend_step_logic', bool(
    m_real and '_deadend_step_logic(' in m_real), True)
chk('影子侧调用 _deadend_step_logic', bool(
    m_shadow and '_deadend_step_logic(' in m_shadow), True)
# 反向：两侧都不得再出现**内联的阈值比较**（真机侧仍可读 cfg 取 hi/lo 传参，
# 但不许自己比）。这是"实现被复制回调用点"的机器化探针。
chk('真机侧无内联阈值比较（已抽走）', bool(
    m_real and 'min_score <' in m_real), False)
chk('影子侧无内联阈值比较（已抽走）', bool(
    m_shadow and 'min_score <' in m_shadow), False)

# ---------------------------------------------------------------- [G] 新旧对拍
print('\n[G] 新旧对拍：默认档（0.0/0.0）必须逐拍等于改动前两态 latch')
random.seed(20261002)
seq = []
t = 0.0
for _ in range(4000):
    t += 0.1
    r = random.random()
    if r < 0.45:
        seq.append((t, DEAD, 0))                    # 死端
    elif r < 0.70:
        seq.append((t, SAFE_HI, random.choice([1, 3, 9])))   # 全条件成立
    elif r < 0.85:
        seq.append((t, SAFE_LO, random.choice([1, 4])))      # 仅低条件
    else:
        seq.append((t, None, random.choice([0, 0, 2])))      # 降级档
new_out = drive(seq, max_s=0.0, hyst_s=0.0)
old_out = ref_old(seq)
mism = [i for i in range(len(seq)) if new_out[i] != old_out[i]]
chk('4000 拍逐拍一致（step/action/forced）', len(mism), 0)
if mism:
    i = mism[0]
    print('     首个失配 @%d seq=%s new=%s old=%s'
          % (i, seq[i], new_out[i], old_out[i]))

# ---------------------------------------------------------------- [H] 降级档
print('\n[H] 降级档（min_score is None）：全否决才算死端')
chk('无否决（ns>0）=> 直接 success', drive([(0.0, None, 4)], 0.0, 0.0)[0][1],
    'success')
chk('全否决（ns=0）=> stop', drive([(0.0, None, 0)], 0.0, 0.0)[0][1], 'stop')

# ---------------------------------------------------------------- [I] 步长钳位
print('\n[I] 步长钳位：跳过拍（nav 持笔/镇定窗）的间隙不得整段充进状态时长')
# 反向注入：cap=0 关闭钳位 ⇒ 5s 间隙原样计入（旧缺陷可复现）
chk('[I1] cap=0 不钳（旧行为可复现）', clamp_dt(5.0, 0.0), (5.0, False))
chk('[I2] cap=0.5 且 dt=0.05（正常拍）不钳', clamp_dt(0.05, 0.5), (0.05, False))
chk('[I3] cap=0.5 且 dt=0.5（恰在界）不钳', clamp_dt(0.5, 0.5), (0.5, False))
chk('[I4] cap=0.5 且 dt=5.0（5s 间隙）=> 钳到 0.5 并置旗',
    clamp_dt(5.0, 0.5), (0.5, True))
chk('[I5] cap=0.5 且 dt=0.500001 => 钳', clamp_dt(0.500001, 0.5), (0.5, True))

# ------------------------------------------------------- [J] ③ 监督层条件原语
print('\n[J] ③ 监督层条件的原语 _sup_bad（veto_hi<=0 或读数缺 => 整条关闭）')
chk('[J1] veto_hi=0.0 => 恒假（关闭）', sup_bad(0.99, 0.0), False)
chk('[J2] veto_hi=-1.0 => 恒假', sup_bad(0.99, -1.0), False)
chk('[J3] 读数缺（None）=> 假', sup_bad(None, 0.60), False)
chk('[J4] 恰在界（0.60, 门 0.60）=> 真（>= 非 >）', sup_bad(0.60, 0.60), True)
chk('[J5] 差一点（0.599, 门 0.60）=> 假', sup_bad(0.599, 0.60), False)
chk('[J6] 明显越线（0.688, 门 0.60）=> 真', sup_bad(0.688, 0.60), True)

# ---------------------------------------------------- [K] 门咬合（hi/lo **同改**）
print('\n[K] 门咬合：vf 越线 => hi/lo 同置假 => 判死端且**不因迟滞退出**')
# SAFE=1.0 < lo=5.0：min_score 一路"安全"，旧行为永不进死端。
SAFE = 1.0
o = drive_v([(0.0, SAFE, 4, 0.30), (0.1, SAFE, 4, 0.59),
             (0.2, SAFE, 4, 0.60), (0.3, SAFE, 4, 0.60)],
            max_s=0.0, hyst_s=0.0, veto_hi=0.60, dwell=0.0)
chk('[K1] vf=0.30（未越线）=> 常规 success', o[0][1], 'success')
chk('[K2] vf=0.59（差一点）=> 仍 success', o[1][1], 'success')
chk('[K3] vf=0.60（越线）=> 判死端 stop', o[2], (1, 'stop', False))
chk('[K4] 越线持续 => 留在死端 rotate', o[3], (1, 'rotate', False))
# 关键：hyst_s>0 且低条件本就满足，越线期间**不许**被迟滞放出去
o = drive_v([(0.0, DEAD, 0, 1.00), (1.0, SAFE, 8, 1.00),
             (5.0, SAFE, 8, 1.00)], max_s=0.0, hyst_s=1.0,
            veto_hi=0.60, dwell=0.0)
chk('[K5] 进死端后 vf 持续越线 + min_score 转安全 => 不得退出（hi/lo 同改）',
    o[2][0], 1)
chk('[K6] 同上：动作仍是 rotate', o[2][1], 'rotate')
# 反面：vf 松口后，迟滞正常放行（证明 [K5] 不是"再也出不来"）
o = drive_v([(0.0, DEAD, 0, 1.00), (1.0, SAFE, 8, 0.10),
             (2.1, SAFE, 8, 0.10)], max_s=0.0, hyst_s=1.0,
            veto_hi=0.60, dwell=0.0)
chk('[K7] vf 松口 + 低条件连续满足 1.0s => 正常退出',
    o[2], (0, 'success', False))

# ------------------------------------------------------------ [L] ③ 迟滞 dwell
print('\n[L] ③ 迟滞 dwell：越线须**连续** dwell 秒才咬合，断裂即清零重计')
o = drive_v([(0.0, SAFE, 4, 0.90), (0.1, SAFE, 4, 0.90),
             (0.2, SAFE, 4, 0.10), (0.3, SAFE, 4, 0.90),
             (0.4, SAFE, 4, 0.90), (0.5, SAFE, 4, 0.90),
             (0.6, SAFE, 4, 0.90)], max_s=0.0, hyst_s=0.0,
            veto_hi=0.60, dwell=0.30)
chk('[L1] 越线首拍（仅起表）=> 不咬合', o[0][1], 'success')
chk('[L2] 连续 0.1s（不足 0.30）=> 不咬合', o[1][1], 'success')
chk('[L3] 0.2s 断裂 => 计时清零，本拍仍不咬合', o[2][1], 'success')
chk('[L4] 0.3s 重启计时（尚不足 0.30）=> 不咬合', o[3][1], 'success')
chk('[L5] 0.4s（自 0.3 起才 0.1s）=> 仍不咬合', o[4][1], 'success')
chk('[L6] 0.5s（自 0.3 起 0.2s）=> 仍不咬合', o[5][1], 'success')
chk('[L7] 0.6s（自 0.3 起满 0.30）=> 咬合 stop', o[6], (1, 'stop', False))
chk('[L8] dwell<=0 时首拍即咬合（无迟滞）',
    drive_v([(0.0, SAFE, 4, 0.90)], 0.0, 0.0, 0.60, 0.0)[0][1], 'stop')
# 计时确实在**断裂处**清零（不是从序列头一直累）：0.0 起表 -> 0.2 断裂清零
# -> 若未清零，t=0.5 已满 0.30 就该咬合；[L6] 报 success 即证明清零发生过。
chk('[L9] vf 全程低于门 => 永不咬合（计时器不得自增）',
    any(r[1] != 'success' for r in drive_v(
        [(t / 10.0, SAFE, 4, 0.59) for t in range(20)], 0.0, 0.0, 0.60, 0.30)),
    False)

# ------------------------------------- [M] 默认档逐字节旧行为（含 vf 在场但门关）
print('\n[M] 默认档（veto_hi=0.0）：**即便逐拍喂入越线 vf** 也必须逐拍等于旧实现')
random.seed(20261003)
seq2, t = [], 0.0
for _ in range(4000):
    t += 0.1
    r = random.random()
    if r < 0.45:
        seq2.append((t, DEAD, 0, 1.0))                    # 死端 + vf 满
    elif r < 0.70:
        seq2.append((t, SAFE_HI, random.choice([1, 3, 9]), random.random()))
    elif r < 0.85:
        seq2.append((t, SAFE_LO, random.choice([1, 4]), 0.99))
    else:
        seq2.append((t, None, random.choice([0, 0, 2]), 0.75))
m_out = drive_v(seq2, max_s=0.0, hyst_s=0.0, veto_hi=0.0, dwell=0.0)
m_ref = ref_old([(a, b, c) for (a, b, c, _d) in seq2])
mism2 = [i for i in range(len(seq2)) if m_out[i] != m_ref[i]]
chk('[M] 4000 拍逐拍一致（门关，vf 全在也不许影响）', len(mism2), 0)
if mism2:
    i = mism2[0]
    print('     首个失配 @%d seq=%s new=%s old=%s'
          % (i, seq2[i], m_out[i], m_ref[i]))
# 反向注入：门一旦打开（veto_hi>0），上述含 vf 的序列**必须**与旧实现不同
m_on = drive_v(seq2, 0.0, 0.0, 0.60, 0.0)
n_diff = sum(1 for i in range(len(seq2)) if m_on[i] != m_ref[i])
chk('[M2] 同一序列门开到 0.60 后必须与旧实现不同（证明 [M] 非空转）',
    n_diff > 0, True)
print('     （门 0.60 下与旧实现不同的拍数 = %d / %d）' % (n_diff, len(seq2)))

print('\n' + '=' * 72)
if fails:
    print('FAIL %d 项 —— 死端状态机被改动，见上' % fails)
else:
    print('ALL PASS —— 超时/迟滞/强制放宽/③ 监督层门 与"默认档逐字节旧行为"均已钉死')
print('=' * 72)
sys.exit(1 if fails else 0)
