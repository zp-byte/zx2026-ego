#!/usr/bin/env bash
# ③ 治理的离线闸门（先于实弹）：exec bit + 编译 + 档位 + 四守卫 + 残留门。
# 用法：bash tools/sup33_gate.sh
cd /home/ubuntu/zx2026_defop_ws || exit 9

echo "=== 1) exec bit / mode ==="
chmod +x src/defop_nav/scripts/defop_cpn_node.py tools/sup33_window.py
chmod 644 tools/defop_ab_run.py
if git diff --summary | grep -q 'mode change'; then
  echo "  !! 仍有 mode change"; git diff --summary | grep 'mode change'
else
  echo "  ok 无 mode change"
fi

echo "=== 2) py_compile ==="
python3 -m py_compile \
  src/defop_nav/scripts/defop_cpn_node.py \
  tools/defop_ab_run.py tools/defop_deadend_selftest.py \
  tools/sup33_replay.py tools/sup33_gate_scan.py tools/sup33_decide.py \
  && echo "  ok" || { echo "  FAIL"; exit 1; }

echo "=== 3) 档位自检（yaml 权威值 vs A/B 预登记）==="
python3 - <<'PY'
import io, re, sys
s = io.open('src/defop_nav/config/defop_nav.yaml', encoding='utf-8').read()
want = {'sup_bearing_mode': "'off'", 'deadend_max_s': '4.0',
        'deadend_exit_hyst_s': '1.0', 'yaw_rate_spot_deg': '45.0',
        'sup_min_dist': '1.0', 'deadend_veto_frac': '0.0',
        'deadend_veto_dwell_s': '0.0'}
bad = 0
for k, v in want.items():
    m = re.search(r'^%s:\s*(\S+)' % re.escape(k), s, re.M)
    got = m.group(1) if m else '<missing>'
    ok = (got == v)
    bad += (not ok)
    print('  %-24s %-10s %s' % (k, got, 'ok' if ok else 'FAIL want=%s' % v))
sys.exit(1 if bad else 0)
PY
[ $? -ne 0 ] && { echo "  档位自检 FAIL"; exit 1; }

echo "=== 4) 回归守卫 ==="
for t in defop_deadend_selftest defop_sup_geo_selftest \
         defop_forensics_selftest defop_reactwin_selftest; do
  printf '  %-28s ' "$t"
  if python3 "tools/$t.py" > "/tmp/gt_$t.log" 2>&1; then
    echo PASS
  else
    echo "FAIL  (tail -20 /tmp/gt_$t.log)"; tail -20 "/tmp/gt_$t.log"; exit 1
  fi
done

echo "=== 5) 残留门（跑批前必须零 ROS 进程）==="
ps -eo pid,etime,args | grep -E 'rosmaster|roslaunch|gzserver|rosout|defop_' \
  | grep -v grep | grep -v sup33_gate || true
n=$(ps -eo args | grep -E 'rosmaster|roslaunch|gzserver|defop_' \
      | grep -v grep | grep -vc sup33_gate)
echo "  残留=$n"
[ "$n" != "0" ] && { echo "  残留门 FAIL"; exit 1; }
echo "闸门全过"
