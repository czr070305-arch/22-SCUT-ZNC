# -*- coding: utf-8 -*-
"""检查白线的实际灰度值, 以及各行的白线段分布"""
import sys

from PIL import Image

f = sys.argv[1]
im = Image.open(f).convert('L')
w, h = im.size
print('size', w, h)

px = list(im.getdata())
rows = [px[y * w:(y + 1) * w] for y in range(h)]

for y in [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110]:
    row = rows[y]
    mx = max(row)
    # 找 > 80 的连续段
    runs = []
    st = -1
    for x in range(w):
        if row[x] > 80:
            if st < 0:
                st = x
        else:
            if st >= 0:
                runs.append((st, x - 1, max(row[st:x])))
                st = -1
    if st >= 0:
        runs.append((st, w - 1, max(row[st:])))
    s = ' '.join('%d-%d(peak%d)' % r for r in runs[:6])
    print('y=%3d max=%3d  %s' % (y, mx, s if s else '无白段'))
