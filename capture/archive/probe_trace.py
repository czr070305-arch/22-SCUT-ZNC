# -*- coding: utf-8 -*-
"""逐行打印某张图的白线段, 定位爬线为什么断"""
import sys

import vision_sim as vs


def probe(f):
    r = vs.run_one(f)
    print('=' * 76)
    print(f, '  thre=%d rows=%d top=%d bottom=%d' % (r['thre'], r['rows'], r['top'], r['bottom']))
    print('=' * 76)
    print('%3s %5s %-58s %s' % ('y', 'rowW', '白线段(起-止)', '爬线中点'))
    for y in range(vs.V_H - 1, -1, -1):
        segs = ' '.join('%d-%d(w%d)' % (l, rr, rr - l + 1) for l, rr in r['runs'][y])
        mid = 'x=%d' % r['mid'][y] if r['mid'][y] >= 0 else ''
        mark = ''
        if y == r['bottom']:
            mark = '  <- 起点'
        if r['look'][0] >= 0 and y == r['look'][1]:
            mark += '  <- 前瞻(红)'
        print('%3d %5d %-58s %-8s%s' % (y, r['row_white'][y], segs[:58], mid, mark))


if __name__ == '__main__':
    for f in sys.argv[1:]:
        probe(f)
