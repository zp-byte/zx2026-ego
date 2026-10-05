#!/usr/bin/env bash
# 批 5(门关) vs 批 6(门开) 碰撞位置对质（同森林同 seed43 ⇒ 坐标可直接比）。
cd /home/ubuntu/zx2026_defop_ws || exit 9
for B in run_logs/defop_ab_20261002_135937 run_logs/defop_ab_20261002_143210; do
  tag=$(echo "$B" | sed 's/.*_//')
  case "$B" in *135937) tag="批5 门关";; *143210) tag="批6 门开";; esac
  echo "======== $tag ($B) ========"
  for d in "$B"/*/; do
    c=$(basename "$d"); c=${c%%_*}
    n=$(grep -c "COLLISION #" "$d"collisions.txt 2>/dev/null)
    n=${n:-0}
    echo "---- cell$c  碰撞=$n ----"
    if [ "$n" != "0" ]; then
      grep -o "drone [0-9] COLLISION #[0-9]* kind=[a-z]* at ([^)]*) dir=[^ ]*" \
           "$d"collisions.txt | sed 's/^/      /'
      grep -o "^[0-9.]*" "$d"collisions.txt | sed 's/^/      t=/'
    fi
  done
done
echo
echo "=== 批 6 逐格结果 ==="
python3 - <<'PY'
import csv
for f in ('results/b5_20261002.csv', 'results/b6_20261002.csv'):
    print('--', f)
    for r in csv.DictReader(open(f)):
        if r['arm'] != 'defop':
            continue
        print('   run%s verdict=%-5s score=%-4s landed=%-2s col=%-3s minclr=%-6s '
              'done=%-6s' % (r['run'], r['verdict'], r['score'], r['landed'],
                             r['col'], r['minclr'], r.get('done', r.get('t_done', ''))))
PY
