# -*- coding: utf-8 -*-
"""用人工标注评估仿真器: 巡线是否走在白线上, 直角弯/分叉/环岛是否识别"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import sys

import labels
import vision_sim as vs

RIGHT, WRONG, LOST = '对', '错', '丢'


def err_sign(e):
    if e > 8:
        return 1
    if e < -8:
        return -1
    return 0


def main(verbose=True):
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))
    if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
        files = [f for f in files if any(a in f for a in sys.argv[1:])]

    # 统计
    line_ok = 0          # 巡线成功（爬够行数）
    on_track = 0         # 绿色线落在白线上
    corner_ok = 0
    corner_miss = 0
    corner_false = 0
    island_ok = 0
    island_false = 0
    total = 0
    valid_total = 0      # 排除撕裂图

    print('%-20s %4s %5s %4s %5s %6s %5s %5s  %s' % (
        'file', 'rows', 'err', '真值', '方向', '拐角真', 'v2拐', 'v2岛', '判定'))
    print('-' * 90)

    for f in files:
        scene, want_dir = labels.get(f)
        r = vs.run_one(f)
        total += 1

        is_tear = scene.startswith('X')
        if not is_tear:
            valid_total += 1

        got_dir = err_sign(r['err'])
        line_ok_flag = r['ok'] and r['rows'] >= 20
        if line_ok_flag:
            line_ok += 1

        # 巡线是否在白线上：看中点的像素亮度是否高于阈值
        if line_ok_flag:
            cnt = 0
            hit = 0
            for y in range(vs.V_H):
                if r['mid'][y] >= 0:
                    cnt += 1
                    if r['frame'][y][r['mid'][y]] > r['thre']:
                        hit += 1
            if cnt > 0 and hit * 100 / cnt >= 85:
                on_track += 1

        # 拐角判定（真值里 CL/CR）
        # 两条判据都算命中：
        #   1) 爬线内部的拐角检测（turn_build 里的宽段判断）-> corner
        #   2) 学长的横向扫描             -> corner_v2
        want_corner = 0
        if scene == 'CL':
            want_corner = -1
        elif scene == 'CR':
            want_corner = 1

        c1 = r['corner']          # 爬线内置
        c2 = r['corner_v2']       # 学长 v2e
        hit = (c1 == want_corner and want_corner != 0) or (c2 == want_corner and want_corner != 0)

        v2c = c1 if c1 != 0 else c2
        if want_corner != 0:
            if hit:
                corner_ok += 1
            else:
                corner_miss += 1
        else:
            if not is_tear and (c1 != 0 or c2 != 0):
                corner_false += 1

        # 环岛
        if scene == 'O':
            if r['island_v2'] != 0:
                island_ok += 1
        else:
            if r['island_v2'] != 0 and not is_tear:
                island_false += 1

        # 方向判定
        if is_tear:
            verdict = '(撕裂, 不评价)'
        elif want_corner != 0:
            verdict = '拐角%s' % ('对' if v2c == want_corner else '漏')
        elif want_dir != 0:
            verdict = '方向%s' % ('对' if got_dir == want_dir else '错')
        else:
            verdict = '直行%s' % ('对' if abs(r['err']) < 25 else '偏')

        if verbose:
            print('%-20s %4d %5.0f %4s %5d %6d %5d %5d  %s' % (
                r['name'][-18:], r['rows'], r['err'],
                scene, got_dir, want_corner, v2c, r['island_v2'], verdict))

    print('-' * 90)
    print('总计 %d 张, 其中有效(非撕裂) %d 张' % (total, valid_total))
    print('巡线爬够 20 行: %d/%d (%.0f%%)' % (line_ok, total, line_ok * 100.0 / total))
    print('绿线落在白线上: %d/%d (%.0f%%)' % (on_track, line_ok, on_track * 100.0 / max(line_ok, 1)))
    print('直角弯命中: %d, 漏检: %d' % (corner_ok, corner_miss))
    print('直角弯误报(非弯图判成弯): %d' % corner_false)
    print('环岛命中: %d, 误报: %d' % (island_ok, island_false))


if __name__ == '__main__':
    main()
