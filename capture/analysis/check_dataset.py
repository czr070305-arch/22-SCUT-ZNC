# -*- coding: utf-8 -*-
"""图集质量检查: 亮度分布 / 白线占比 / 撕裂检测"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import os
import sys

from PIL import Image

files = sorted(glob.glob(sys.argv[1] + '/*.png') if len(sys.argv) > 1 else ['dataset/*.png'])
if not files:
    print('没找到图片')
    sys.exit(1)

print('共 %d 张' % len(files))
print('%-20s %4s %4s %6s %7s %7s' % ('file', 'min', 'max', 'mean', 'white%', 'tear'))
print('-' * 58)

good = bad = 0
for f in files:
    im = Image.open(f).convert('L')
    w, h = im.size
    px = list(im.getdata())

    mn = min(px)
    mx = max(px)
    mean = sum(px) / len(px)
    white = sum(1 for v in px if v > 180) * 100.0 / len(px)

    # 逐行均值, 撕裂会让某行均值突变
    rowmean = []
    for y in range(h):
        row = px[y * w:(y + 1) * w]
        rowmean.append(sum(row) / w)
    diffs = [abs(rowmean[y + 1] - rowmean[y]) for y in range(h - 1)]
    diffs_sorted = sorted(diffs)
    med = diffs_sorted[len(diffs_sorted) // 2]
    tear = (max(diffs) / med) if med > 0.3 else (max(diffs) / 0.3)

    flag = ''
    if tear > 8:
        flag = ' <== 撕裂'
        bad += 1
    elif mx < 120:
        flag = ' <== 偏暗'
        bad += 1
    elif white < 0.3:
        flag = ' <== 几乎没白线'
        bad += 1
    else:
        good += 1

    print('%-20s %4d %4d %6.1f %7.2f %7.2f%s' % (
        os.path.basename(f)[-18:], mn, mx, mean, white, tear, flag))

print('-' * 58)
print('可用 %d 张, 有问题 %d 张' % (good, bad))
