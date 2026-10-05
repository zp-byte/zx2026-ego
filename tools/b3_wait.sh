#!/usr/bin/env bash
# 守候批 3（β 翻 on）-> 与批 2 并排对拍 + 归因四件套（reactwin / forensics）。
cd /home/ubuntu/zx2026_defop_ws || exit 1
A=run_logs/defop_ab_20261002_121827          # 批 2（对照：shadow / 4.0 / 1.0）
B=run_logs/defop_ab_20261002_125427          # 批 3（on / 4.0 / 1.0）
while pgrep -f 'tools/defop_ab_run.py' >/dev/null 2>&1; do sleep 20; done
echo "=========== 批 3 收工 $(date +%H:%M:%S) ==========="
tail -8 /tmp/b3_console.log

echo
echo "=== 并排对拍（P4 余量按 verify.txt 的 DONE 时刻 − 最后移动时刻）==="
python3 - "$A" "$B" <<'PY'
import glob, json, os, re, sys

def cellstats(c):
    sc = {}
    p = os.path.join(c, "score.yaml")
    if os.path.exists(p):
        for ln in open(p):
            m = re.match(r"^(total|landed_n|correct_n|retired_n):\s*(\S+)", ln)
            if m:
                sc[m.group(1)] = m.group(2)
    col = 0
    p = os.path.join(c, "collisions.txt")
    if os.path.exists(p):
        col = sum(1 for _ in open(p, errors="ignore") if "COLLISION #" in _)
    de = fr = 0
    p = os.path.join(c, "rosout.log")
    if os.path.exists(p):
        for ln in open(p, errors="ignore"):
            if "DEADEND enter" in ln:
                de += 1
            elif "DEADEND forced-relax" in ln:
                fr += 1
    rot = forced = 0.0
    p = os.path.join(c, "probe.json")
    if os.path.exists(p):
        for v in json.load(open(p)).get("defop_debug", {}).values():
            rot += v.get("rot_s", 0.0)
            forced += v.get("forced_s", 0.0)
    mv = None
    p = os.path.join(c, "verify.txt")
    if os.path.exists(p):
        prev, t = {}, None
        for ln in open(p, errors="ignore"):
            m = re.match(r"\s+t=(\d+) drone(\d+)=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", ln)
            if m:
                tt = int(m.group(1)); d = int(m.group(2))
                xyz = tuple(float(m.group(i)) for i in (3, 4, 5))
                if d in prev and max(abs(a - b) for a, b in zip(xyz, prev[d])) > 0.05:
                    mv = tt
                prev[d] = xyz
            m = re.match(r"-- state -> DONE @ (\d+)s", ln)
            if m:
                t = int(m.group(1))
        margin = (t - mv) if (t is not None and mv is not None) else None
    else:
        margin = None
    return sc, col, de, fr, rot, forced, margin

rows = []
for B_ in sys.argv[1:]:
    for c in sorted(glob.glob(os.path.join(B_, "*"))):
        if os.path.isdir(c):
            rows.append((os.path.basename(B_), os.path.basename(c)) + cellstats(c))

print("  %-24s %-22s %5s %5s %5s %5s %5s %8s %8s %7s"
      % ("批", "格", "分", "land", "col", "DE", "FR", "rot_s", "forced_s", "P4余量"))
agg = {}
for b, c, sc, col, de, fr, rot, forced, mg in rows:
    print("  %-24s %-22s %5s %5s %5d %5d %5d %8.1f %8.1f %7s"
          % (b[-6:], c, sc.get("total", "-"), sc.get("landed_n", "-"),
             col, de, fr, rot, forced,
             "%.0fs" % mg if mg is not None else "-"))
    a = agg.setdefault(b[-6:], [0, 0, 0, 0, 0, 0.0, 0.0, []])
    a[0] += int(sc.get("total", 0)); a[1] += int(sc.get("landed_n", 0))
    a[2] += col; a[3] += de; a[4] += fr; a[5] += rot; a[6] += forced
    if mg is not None:
        a[7].append(mg)
print()
for k, a in agg.items():
    print("  [%s] 合计: 分=%d (均 %.1f)  landed=%d  col=%d  DE=%d  FR=%d  "
          "rot=%.1fs  forced=%.1fs  P4余量 min=%s"
          % (k, a[0], a[0] / 3.0, a[1], a[2], a[3], a[4], a[5], a[6],
             "%.0fs" % min(a[7]) if a[7] else "-"))
PY

echo
echo "=== 机制归因（reactwin，Tier 2 逐拍线应生效）==="
python3 tools/defop_reactwin.py "$A" "$B" 2>&1 | tail -40

echo
echo "=== 碰撞解剖（forensics，按 kind）==="
for c in "$B"/*/; do
  printf "  [%s] " "$(basename "$c")"
  python3 tools/defop_collision_forensics.py "$c" 2>/dev/null | grep '碰撞解剖' | sed 's/^ *//'
done

echo
echo "=== 强制放宽计数 ==="
grep -hc 'DEADEND forced-relax' "$B"/*/rosout.log 2>/dev/null | paste -sd+ | bc 2>/dev/null || echo 0
