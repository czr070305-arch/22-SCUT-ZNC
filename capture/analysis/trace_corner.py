# -*- coding: utf-8 -*-
"""追踪直角弯附近爬线的每一步，看拐角判断为什么没触发"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sys

from PIL import Image

import vision_sim as vs


def trace(f):
    im = Image.open(f).convert('L')
    px = list(im.getdata())
    frame = [px[y * vs.V_W:(y + 1) * vs.V_W] for y in range(vs.V_H)]
    thre = vs.img_threshold(frame)
    runs, rw = vs.binarize_runs(frame, thre)

    S = vs.State()
    vs.track_build(S, runs, rw, multi_start=True)

    print('=' * 88)
    print('%s  thre=%d rows=%d top=%d corner=%d bottom=%d' % (
        f.split('_')[-1], thre, S.path_rows, S.path_top, S.corner_dir, S.bottom_x))
    print('=' * 88)
    print('%3s %5s %-40s %6s %8s %8s' % ('y', 'rowW', '白线段', 'pred', 'tolerance', '结果'))
    print('-' * 88)

    start = S.bottom_x
    for y in range(vs.V_H - 1, max(0, S.path_top - 8), -1):
        segs = ' '.join('%d-%d(w%d)' % (l, r, r - l + 1) for l, r in runs[y])
        rws = rw[y]
        mark = ''

        if S.mid[y] >= 0:
            if y == S.path_top:
                mark = 'TOP x=%d' % S.mid[y]
            else:
                mark = 'mid=%d' % S.mid[y]
        if rws > vs.P['ROW_WHITE_MAX']:
            mark = '[盲区]'
        if y == vs.V_H - 1:
            mark += ' <- 底部'
        if S.corner_dir != 0 and y == S.path_top:
            mark += ' <- 拐角!'

        # 显示该行的宽段情况
        wide = [(l, r) for (l, r) in runs[y] if r - l + 1 > vs.P['RUN_W_MAX_NARROW']]
        wtag = ('  [宽段%dx%d]' % (wide[0][0], wide[0][1])) if wide else ''

        print('%3d %5d %-40s %30s %s' % (y, rws, segs[:40], mark, wtag))

    print()
    print('拐角相关参数: CORNER_MIN_ROWS=%d CORNER_W_MAX=%d CORNER_LOOK=%d NEAR_END=%d' % (
        vs.P['CORNER_MIN_ROWS'], vs.P['CORNER_W_MAX'],
        vs.P['CORNER_LOOK'], vs.P['CORNER_NEAR_END']))


if __name__ == '__main__':
    for a in sys.argv[1:]:
        trace(a)
