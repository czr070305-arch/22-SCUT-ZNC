# -*- coding: utf-8 -*-
"""图集总览: 直方图 + 拼图 + 撕裂检测(行错位相关法)"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import os
import sys

from PIL import Image

outdir = sys.argv[1] if len(sys.argv) > 1 else 'dataset'
files = sorted(glob.glob(os.path.join(outdir, '*.png')))

# ---------- 1. 全图集直方图 ----------
hist = [0] * 256
for f in files:
    im = Image.open(f).convert('L')
    for v, n in enumerate(im.histogram()):
        hist[v] += n

total = sum(hist)
print('=== 全图集灰度直方图 (累计 %.0f 万像素) ===' % (total / 1e4))
# 打印关键区间
bins = [(0, 10), (10, 20), (20, 40), (40, 60), (60, 80), (80, 100),
        (100, 120), (120, 140), (140, 160), (160, 180), (180, 200), (200, 256)]
for lo, hi in bins:
    n = sum(hist[lo:hi])
    print('  %3d-%3d : %6.2f%%  %s' % (lo, hi - 1, n * 100.0 / total, '#' * int(n * 600.0 / total)))

# ---------- 2. 撕裂检测: 相邻行最佳水平偏移 ----------
def row_shift(a, b):
    """返回使 b 与 a 最匹配的水平偏移量"""
    w = len(a)
    best_s, best_d = 0, None
    for s in range(-12, 13, 2):
        d = 0
        n = 0
        for x in range(12, w - 12, 2):
            d += abs(a[x] - b[x + s])
            n += 1
        d /= n
        if best_d is None or d < best_d:
            best_d, best_s = d, s
    return best_s

print()
print('=== 撕裂检测 (相邻行水平错位) ===')
print('%-20s %6s %8s %8s' % ('file', '白线%', '最大错位', '判定'))
print('-' * 48)
usable = []
for f in files:
    im = Image.open(f).convert('L')
    w, h = im.size
    px = list(im.getdata())
    rows = [px[y * w:(y + 1) * w] for y in range(h)]

    # 白线占比: 用全图集的 P99.5 分位当阈值
    acc = 0
    thr = 255
    for v in range(255, -1, -1):
        acc += hist[v]
        if acc > total * 0.004:      # 最亮 0.4%
            thr = v
            break

    white = sum(1 for v in px if v >= thr) * 100.0 / len(px)
    maxshift = 0
    for y in range(10, h - 10, 3):
        s = row_shift(rows[y], rows[y + 1])
        if abs(s) > abs(maxshift):
            maxshift = s

    ok = abs(maxshift) < 6
    print('%-20s %5.2f%% %8d %8s' % (
        os.path.basename(f)[-18:], white, maxshift, 'OK' if ok else '撕裂'))
    if ok:
        usable.append(f)

print('-' * 48)
print('白线阈值取 P99.6 = %d' % thr)
print('可用 %d / %d' % (len(usable), len(files)))

# ---------- 3. 拼图 (4 列 x N 行, 每张放大 2 倍) ----------
if usable:
    SW, SH = 188 * 2, 120 * 2
    COLS = 4
    ROWS = (len(usable) + COLS - 1) // COLS
    sheet = Image.new('L', (COLS * SW, ROWS * SH), 40)
    for i, f in enumerate(usable):
        im = Image.open(f).convert('L').resize((SW, SH), Image.NEAREST)
        sheet.paste(im, ((i % COLS) * SW, (i // COLS) * SH))
    p = 'capture/_sheet_usable.png'
    sheet.save(p)
    print('拼图: %s  (%dx%d)' % (p, sheet.size[0], sheet.size[1]))
