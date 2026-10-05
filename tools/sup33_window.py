#!/usr/bin/env python3
"""③ 法证：把每起碰撞前 N 秒的逐拍线（Tier 2）摊开，看监督层当时眼中的世界。

行 = [sim_t, cod, col_prob, veto, safe, forced,
      dnear, dnear2, dmin, near_az, yaw_deg, vwx, vwy, n_indx1]
  cod 0=非死端 1=deadend_stop 2=deadend_rotate ; col_prob = CPN min_score
  dnear = 深度图里 < sup_min_dist(1.0m) 的有效像素数（0 ⇒ 监督器**没看见**）
  near_az = 这些近像素的平均方位(deg, 机体系) ; yaw_deg = 机体航向
  n_indx1 = 死端门第二条件 len(indx1)（>0 即"有安全基元"）
用法: python3 tools/sup33_window.py run_logs/<批> [-w 8]
"""
import json
import os
import re
import sys

COLL = re.compile(r"(\d+\.\d+).*drone (\d+) COLLISION #(\d+) (?:kind=(\w+) )?"
                  r"at \(([-\d.]+),([-\d.]+),([-\d.]+)\)")


def load_cells(base):
    for c in sorted(os.listdir(base)):
        p = os.path.join(base, c, "probe.json")
        if os.path.exists(p):
            yield c, json.load(open(p)).get("defop_debug", {})


def colls_of(cell):
    p = os.path.join(cell, "collisions.txt")
    out = []
    if os.path.exists(p):
        for ln in open(p, errors="ignore"):
            m = COLL.search(ln)
            if m:
                out.append((float(m.group(1)), int(m.group(2)),
                            m.group(4), float(m.group(5)), float(m.group(6))))
    return out


def main():
    base = sys.argv[1]
    win = 8.0
    if "-w" in sys.argv:
        win = float(sys.argv[sys.argv.index("-w") + 1])
    for cname, dbg in load_cells(base):
        cell = os.path.join(base, cname)
        cs = colls_of(cell)
        print("\n" + "=" * 96)
        print("[%s]  碰撞 %d 起   探针机数=%d" % (cname, len(cs), len(dbg)))
        if not cs:
            continue
        for (t, d, kind, x, y) in cs:
            tl = dbg.get(str(d), {}).get("tl") or dbg.get(d, {}).get("tl") or []
            seg = [r for r in tl if t - win <= r[0] <= t + 0.6]
            print("\n  -- drone%d  t=%.2f  kind=%s  (%.2f,%.2f)  窗口 %.0fs: %d 拍 --"
                  % (d, t, kind, x, y, win, len(seg)))
            print("     %8s %4s %8s %5s %5s %4s %6s %6s %6s %7s %7s %9s %6s  %s"
                  % ("t", "cod", "col_pr", "veto", "safe", "frcd",
                     "dnear", "dnr2", "dmin", "near_az", "yaw", "vw(x,y)",
                     "nix1", "备注"))
            for r in seg:
                st, cod, cp, veto, safe, forced = r[:6]
                dnear, dnr2, dmin, naz, yaw = (r[6:11] + [None] * 5)[:5]
                vwx, vwy, nix1 = (r[11:14] + [None] * 3)[:3]
                mark = ""
                if cod == 1:
                    mark = "死端-停"
                elif cod == 2:
                    mark = "死端-转"
                if forced:
                    mark += " 强制放宽"
                print("     %8.2f %4d %8s %5s %5s %4s %6s %6s %6s %7s %7s %9s %6s  %s"
                      % (st, cod, "-" if cp is None else "%.2f" % cp,
                         veto, safe, forced,
                         "-" if dnear is None else dnear,
                         "-" if dnr2 is None else dnr2,
                         "-" if dmin is None else "%.2f" % dmin,
                         "-" if naz is None else "%.1f" % naz,
                         "-" if yaw is None else "%.1f" % yaw,
                         "-" if vwx is None else "%.2f,%.2f" % (vwx, vwy),
                         "-" if nix1 is None else nix1, mark))
            if seg:
                # 撞前 1.5 s 的窗口统计
                near = [r for r in seg if r[0] <= t and r[0] >= t - 1.5]
                if near:
                    safes = [r[4] for r in near if r[4] is not None]
                    cps = [r[2] for r in near if r[2] is not None]
                    cods = [r[1] for r in near]
                    dns = [r[6] for r in near if len(r) > 6 and r[6] is not None]
                    dms = [r[8] for r in near if len(r) > 8 and r[8] is not None]
                    print("     >> 撞前1.5s: 拍数=%d  safe min/med/max=%s  "
                          "col_prob min=%s  死端拍=%d/%d  "
                          "dnear max=%s  dmin min=%s"
                          % (len(near),
                             "%d/%d/%d" % (min(safes), sorted(safes)[len(safes) // 2],
                                           max(safes)) if safes else "-",
                             "%.2f" % min(cps) if cps else "-",
                             sum(1 for v in cods if v), len(cods),
                             max(dns) if dns else "(无字段)",
                             "%.2f" % min(dms) if dms else "-"))


if __name__ == "__main__":
    main()
