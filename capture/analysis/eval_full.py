# -*- coding: utf-8 -*-
"""用完整元素链（环岛 / 十字 / 岔路 / 直角）评估整个图集

判定优先级与 C 固件 elem_update() 一致：
    环岛 > 岔路 > 十字 > 直角
"""

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import sys

sys.path.insert(0, 'capture')

from PIL import Image

import labels
import vision_sim as vs
import island


def analyze(f):
    im = Image.open(f).convert('L')
    px = list(im.getdata())
    frame = [px[y * vs.V_W:(y + 1) * vs.V_W] for y in range(vs.V_H)]

    thre = vs.img_threshold(frame)
    runs, rw = vs.binarize_runs(frame, thre)

    S = vs.State()
    vs.track_build(S, runs, rw, multi_start=True)
    ok_steer = vs.track_steer(S)

    # 四个检测器
    isl = island.detect_island(S, runs, rw) if S.path_rows >= 20 else 0
    fork_rows, fork_side = vs.find_fork(S, runs) if S.path_rows >= 20 else (0, 0)
    cross_rows = vs.detect_cross(S, runs, rw) if S.path_rows >= 15 else 0
    cd = vs.post_detect_corner(S, runs) if S.path_rows >= 12 else 0

    # 优先级：环岛 > 岔路 > 十字 > 直角
    if isl != 0:
        letter = 'I'
        detail = 'isle(%d)' % isl
    elif fork_rows >= vs.P['FORK_MIN_ROWS']:
        letter = 'F'
        detail = 'fork(%d行)' % fork_rows
    elif cross_rows >= 2:
        letter = 'C'
        detail = 'cross(%d行)' % cross_rows
    elif cd > 0:
        letter = 'R'
        detail = 'corner+'
    elif cd < 0:
        letter = 'L'
        detail = 'corner-'
    else:
        letter = 'N'
        detail = ''

    # 宽度比
    prof = []
    for y in range(22, vs.V_H):
        if S.mid[y] < 0:
            continue
        for (l, r) in runs[y]:
            if l <= S.mid[y] <= r:
                prof.append(r - l + 1)
                break
    ratio = 0.0
    if len(prof) >= 20:
        prof.sort()
        med = prof[len(prof) // 2] or 2
        ratio = max(prof) / float(med)

    return dict(name=f.split('/')[-1], thre=thre, rows=S.path_rows,
                err=S.steer_err, letter=letter, detail=detail,
                isl=isl, fork=fork_rows, cross=cross_rows, corner=cd,
                ratio=ratio, valid=1 if ok_steer and S.path_rows >= vs.P['MIN_ROWS_STEER'] else 0)


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))
    print('=' * 84)
    print('完整元素链评估  (I环岛 > F岔路 > C十字 > L/R直角 > N普通)')
    print('=' * 84)
    print('%-16s %4s %5s %7s %6s %-5s %s' % ('图片', 'T', '行数', 'E', '宽度比', '元素', '依据'))
    print('-' * 84)

    from collections import Counter
    cnt = Counter()
    for f in files:
        r = analyze(f)
        cnt[r['letter']] += 1
        print('%-16s %4d %5d %7.1f %6.1f %-5s %s' % (
            r['name'][-14:-4], r['thre'], r['rows'], r['err'],
            r['ratio'], r['letter'], r['detail']))

    print('-' * 84)
    print('元素分布:', dict(cnt))
    print()

    # 可疑：爬线太短的
    print('=== 爬线异常（<25 行）===')
    for f in files:
        r = analyze(f)
        if r['rows'] < 25:
            print('  %s  行数=%d  T=%d' % (r['name'][-14:-4], r['rows'], r['thre']))


if __name__ == '__main__':
    main()
