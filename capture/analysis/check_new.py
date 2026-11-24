# -*- coding: utf-8 -*-
"""撕裂检测 v3

v1/v2 都失败的原因: 都依赖"某一行的差值特别大"。
但实际撕裂常常是【多段错位】：画面被切成 3~5 条横带，
每条带内部连续，带与带之间基线不同。这时每行差值都不算特别突出，
峰值被摊平了。

v3 换判据：检测"基线段"
  1. 算每行在【鱼眼内部】的均值 m[y]
  2. 撕裂处 m[y] 会突然跳到另一个基准，之后保持平稳
  3. 所以找 m[y] 的"分段常数"结构：相邻行差值大的点就是段边界
  4. 如果段边界 >= 1 个 且 各段均值差 > 阈值 -> 撕裂

这个方法对"多段错位"和"单缝"都有效，且不受画面内容（白线/散热器）干扰，
因为白线在单行里只占少数像素，不影响整行均值。
"""

import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import os
import sys

from PIL import Image

W, H = 188, 120
Y_LO, Y_HI = 30, 92          # 鱼眼内部，避免圆边界
X_LO, X_HI = 40, 148         # 中间带，避开圆的左右边缘


def analyze(path, verbose=False):
    im = Image.open(path).convert('L')
    px = list(im.getdata())
    rows = [px[y * W:(y + 1) * W] for y in range(H)]

    mean = []
    for y in range(Y_LO, Y_HI):
        row = rows[y]
        mean.append(sum(row[X_LO:X_HI]) / float(X_HI - X_LO))

    # 相邻行均值差
    d = [abs(mean[i + 1] - mean[i]) for i in range(len(mean) - 1)]

    # 段边界：差值 > max(3.5, 3 * 中位差)
    ds = sorted(d)
    med = ds[len(ds) // 2] if ds else 0.0
    thr = max(3.5, med * 3.0 + 1.0)

    bounds = [i for i, v in enumerate(d) if v > thr]

    # 各段均值
    segs = []
    prev = 0
    for b in bounds:
        segs.append((Y_LO + prev, Y_LO + b, sum(mean[prev:b + 1]) / float(b - prev + 1)))
        prev = b + 1
    segs.append((Y_LO + prev, Y_HI - 1, sum(mean[prev:]) / float(len(mean) - prev)))

    # 段间均值差
    max_jump = 0.0
    for i in range(len(segs) - 1):
        max_jump = max(max_jump, abs(segs[i][2] - segs[i + 1][2]))

    # 判定：有段边界 且 段间均值差明显
    torn = (len(bounds) >= 1) and (max_jump > 4.0)

    if verbose:
        print('  段数=%d, 边界=%s, 段间最大差=%.1f, 阈值=%.1f' % (
            len(segs), bounds, max_jump, thr))

    return dict(name=os.path.basename(path), segs=len(segs), bounds=bounds,
                jump=max_jump, torn=torn, mean=mean)


def main():
    pat = sys.argv[1] if len(sys.argv) > 1 else os.path.join(_ROOT, 'dataset', '20261119_*.png')
    files = sorted(glob.glob(pat))
    print('%-18s %5s %5s %8s  %s' % ('图片', '段数', '边界', '段间差', '结论'))
    print('-' * 50)
    bad = []
    for f in files:
        r = analyze(f)
        tag = '撕裂' if r['torn'] else 'ok'
        if r['torn']:
            bad.append(r['name'])
        print('%-18s %5d %5d %8.1f  %s' % (
            r['name'][-14:], r['segs'], len(r['bounds']), r['jump'], tag))
    print('-' * 50)
    print('共 %d, 撕裂 %d, 可用 %d' % (len(files), len(bad), len(files) - len(bad)))
    if bad:
        print('撕裂的:')
        for n in bad:
            print('  ' + n)


if __name__ == '__main__':
    main()
