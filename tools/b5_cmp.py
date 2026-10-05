#!/usr/bin/env python3
"""批 2/3/4/5 逐格速率与结果对比（判插桩是否拖性能）。"""
import csv
import glob
import os

os.chdir('/home/ubuntu/zx2026_defop_ws')
for tag in ('b2', 'b3', 'b4', 'b5', 'b6'):
    fs = sorted(glob.glob('results/%s_*.csv' % tag))
    print('=== %s (%s) ===' % (tag, ', '.join(os.path.basename(x) for x in fs)))
    for f in fs:
        for r in csv.DictReader(open(f)):
            if r['arm'] != 'defop':
                continue
            print('  run%s  %-5s score=%-4s landed=%-2s col=%-3s minclr=%-6s '
                  'vel_hz=%-5s lat_hz=%-5s rtf=%s'
                  % (r['run'], r['verdict'], r['score'], r['landed'], r['col'],
                     r['minclr'], r['vel_cmd_hz'], r['latent_hz'], r['rtf']))
