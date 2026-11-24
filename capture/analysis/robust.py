# -*- coding: utf-8 -*-
"""光照鲁棒性测试

房间和赛道的最大差异就是光照。这里把图集按不同方式模拟变形，
看阈值 / 巡线行数 / 转向误差是否还稳定。

变形方式（模拟实车可能遇到的偏差）:
    B±N : 整体亮度加减 N
    G<n> : gamma 校正 <n>（<1 变亮，>1 变暗）
    N<n> : 叠加 ±n 的均匀噪声
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import sys

from PIL import Image

import labels
import vision_sim as vs
import island


def transform(frame, kind, arg):
    out = []
    if kind == 'B':
        for row in frame:
            out.append([max(0, min(255, int(v) + arg)) for v in row])
    elif kind == 'G':
        lut = [max(0, min(255, int(255.0 * ((i / 255.0) ** arg)))) for i in range(256)]
        for row in frame:
            out.append([lut[v] for v in row])
    elif kind == 'N':
        # 用位置做伪随机，保证每次结果一样
        for y, row in enumerate(frame):
            out.append([max(0, min(255, v + (((y * 37 + x * 91) % (2 * arg + 1)) - arg)))
                        for x, v in enumerate(row)])
    return out


def evaluate(files, kind=None, arg=0):
    ok_rows = 0
    total = 0
    thres = []
    errs = []
    letters = []
    for f in files:
        scene, _ = labels.get(f)
        if scene.startswith('X'):
            continue
        im = Image.open(f).convert('L')
        px = list(im.getdata())
        frame = [px[y * vs.V_W:(y + 1) * vs.V_W] for y in range(vs.V_H)]

        if kind:
            frame = transform(frame, kind, arg)

        thre = vs.img_threshold(frame)
        runs, rw = vs.binarize_runs(frame, thre)
        S = vs.State()
        vs.track_build(S, runs, rw, multi_start=True)
        vs.track_steer(S)

        if S.path_rows < 20:
            total += 1
            thres.append(thre)
            letters.append('.')
            continue

        isl = island.detect_island(S, runs, rw)
        cor = S.corner_dir
        letter = 'I' if isl else ('R' if cor > 0 else ('L' if cor < 0 else 'N'))

        ok_rows += S.path_rows
        total += 1
        thres.append(thre)
        errs.append(abs(S.steer_err))
        letters.append(letter)
    return dict(
        n=total,
        avg_rows=ok_rows / float(total),
        thre_min=min(thres) if thres else 0,
        thre_max=max(thres) if thres else 0,
        avg_err=(sum(errs) / len(errs)) if errs else 0,
        letters=''.join(letters),
    )


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))
    base = evaluate(files)
    print('%-12s %5s %6s %8s %8s   %s' % ('条件', '张数', '平均行', '阈值范围', '平均|E|', '元素序列'))
    print('-' * 78)

    def show(name, r):
        same = '  一致' if r['letters'] == base['letters'] else '  <<< 变了'
        print('%-12s %5d %6.1f %3d~%-4d %8.1f   %s%s' % (
            name, r['n'], r['avg_rows'], r['thre_min'], r['thre_max'],
            r['avg_err'], r['letters'], same))

    show('原始', base)
    for n in (-20, -10, +10, +20, +30):
        show('亮度%+d' % n, evaluate(files, 'B', n))
    for g in (0.7, 0.85, 1.2, 1.4):
        show('gamma%.2f' % g, evaluate(files, 'G', g))
    for n in (5, 10, 20):
        show('噪声±%d' % n, evaluate(files, 'N', n))


if __name__ == '__main__':
    sys.path.insert(0, 'capture')
    main()
