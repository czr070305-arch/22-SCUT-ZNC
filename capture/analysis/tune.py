# -*- coding: utf-8 -*-
"""参数网格搜索: 用人工标注找最优的 LOOKAHEAD / CORNER_BIAS / ERR_SLOPE_W 等"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import itertools
import sys

import labels
import vision_sim as vs

RIGHT, WRONG = 1, -1


def score_params(params, files):
    """返回 (方向正确率, 平均值绝对误差, 饱和数, 拐角命中, 拐角漏检)"""
    old = {k: vs.P[k] for k in params}
    for k, v in params.items():
        vs.P[k] = v
    try:
        dir_ok = dir_total = 0
        errs = []
        sat = 0
        c_ok = c_miss = 0
        for f in files:
            scene, want = labels.get(f)
            if scene.startswith('X'):
                continue
            r = vs.run_one(f)
            if r['rows'] < 20:
                continue
            e = r['err']
            errs.append(abs(e))
            if abs(e) > 105:
                sat += 1
            got = 1 if e > 8 else (-1 if e < -8 else 0)
            if want != 0:
                dir_total += 1
                if got == want:
                    dir_ok += 1
            if scene in ('CL', 'CR'):
                wc = -1 if scene == 'CL' else 1
                c1, c2 = r['corner'], r['corner_v2']
                if c1 == wc or c2 == wc:
                    c_ok += 1
                else:
                    c_miss += 1
        return (dir_ok, dir_total, errs, sat, c_ok, c_miss)
    finally:
        for k, v in old.items():
            vs.P[k] = v


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))

    base = dict(vs.P)
    print('=== 基线 ===')
    ok, tot, errs, sat, cob, cm = score_params({}, files)
    avg = sum(errs) / max(len(errs), 1)
    print('方向对 %d/%d, 平均|err|=%.1f, 饱和 %d, 拐角 %d/%d' % (ok, tot, avg, sat, cob, cob + cm))

    print()
    print('=== 单参数扫描 ===')
    grid = {
        'LOOKAHEAD': [20, 28, 36, 44, 52],
        'CORNER_BIAS': [10.0, 22.0, 35.0, 50.0, 70.0],
        'ERR_SLOPE_W': [4.0, 8.0, 12.0, 18.0, 24.0],
        'ERR_OFF_W': [0.8, 1.0, 1.4, 1.8],
        'CORNER_TAN_MAX': [0.10, 0.20, 0.35, 0.50],
        'MIN_ROWS_STEER': [6, 10, 16],
    }
    for k, vals in grid.items():
        print('--- %s ---' % k)
        for v in vals:
            ok, tot, errs, sat, cob, cm = score_params({k: v}, files)
            avg = sum(errs) / max(len(errs), 1)
            print('   %-16s=%-7s 方向 %2d/%2d  平均|err|=%5.1f  饱和 %2d  拐角 %d/%d' % (
                k, v, ok, tot, avg, sat, cob, cob + cm))


if __name__ == '__main__':
    main()
