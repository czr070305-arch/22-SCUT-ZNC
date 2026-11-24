# -*- coding: utf-8 -*-
"""拆解转向误差的各个分量: dx / slope项 / corner项"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import sys

import vision_sim as vs


def breakdown(f):
    r = vs.run_one(f)
    if not r['ok'] or r['rows'] < vs.P['MIN_ROWS_STEER']:
        return None
    S = vs.State()
    # 重跑拿内部状态
    from PIL import Image
    im = Image.open(f).convert('L')
    px = list(im.getdata())
    frame = [px[y * vs.V_W:(y + 1) * vs.V_W] for y in range(vs.V_H)]
    thre = vs.img_threshold(frame)
    runs, rw = vs.binarize_runs(frame, thre)
    vs.track_build(S, runs, rw, multi_start=vs.P.get('MULTI_START', True))

    y0 = -1
    for i in range(vs.V_H - 1, -1, -1):
        if S.mid[i] >= 0:
            y0 = i
            break
    if y0 < 0:
        return None

    look_y = max(0, y0 - vs.P['LOOKAHEAD'])
    look_x = -1
    for i in range(look_y, y0 + 1):
        if S.mid[i] >= 0:
            look_x = S.mid[i]
            look_y = i
            break
    if look_x < 0:
        look_x, look_y = S.mid[y0], y0

    if vs.UNDIST_ENABLE:
        dx = vs.undistort_x(float(look_x), float(look_y)) - vs.undistort_x(float(vs.V_W // 2), float(look_y))
    else:
        dx = float(look_x - vs.V_W // 2)

    yn = max(0, y0 - 14)
    yn_used = -1
    for i in range(yn, y0):
        if S.mid[i] >= 0:
            yn_used = i
            break
    near_slope = 0.0
    if yn_used >= 0 and y0 != yn_used:
        near_slope = float(S.mid[yn_used] - S.mid[y0]) / float(y0 - yn_used)

    return dict(
        name=r['name'][-20:], rows=S.path_rows, y0=y0,
        look_x=look_x, look_y=look_y,
        dx=dx, slope=near_slope, slope_term=near_slope * vs.P['ERR_SLOPE_W'],
        err_final=r['err'], corner=S.corner_dir,
        bottom=S.bottom_x,
    )


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))
    if len(sys.argv) > 1:
        files = [f for f in files if any(a in f for a in sys.argv[1:])]

    print('%-20s %4s %4s %6s %6s %7s %8s %7s %5s' % (
        'file', 'rows', 'y0', 'lookX', 'dx', 'slope', 'slope*12', 'err', 'cor'))
    print('-' * 82)
    for f in files:
        b = breakdown(f)
        if not b:
            print('%-20s (无效)' % f.split('_')[-1][:18])
            continue
        print('%-20s %4d %4d %6d %6.1f %7.2f %8.1f %7.1f %5d' % (
            b['name'], b['rows'], b['y0'], b['look_x'], b['dx'],
            b['slope'], b['slope_term'], b['err_final'], b['corner']))


if __name__ == '__main__':
    main()
