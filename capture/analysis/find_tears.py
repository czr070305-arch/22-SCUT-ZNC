# -*- coding: utf-8 -*-
"""严格的撕裂检测

撕裂的定义：DVP 采集中途被切帧，画面上半部分和下半部分来自不同时刻，
表现为某一行处出现巨大的横向错位或亮度突变。

多指标交叉验证：
  1. row_jump   相邻两行的平均绝对差的最大值（撕裂处会很大）
  2. shift      相邻两行的最佳水平错位（撕裂处非零）
  3. band       是否存在大段纯黑带（DVP 停帧的典型特征）
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


def analyze(path):
    im = Image.open(path).convert('L')
    px = list(im.getdata())
    rows = [px[y * W:(y + 1) * W] for y in range(H)]

    # 1) 相邻行平均绝对差
    jumps = []
    for y in range(H - 1):
        a, b = rows[y], rows[y + 1]
        d = sum(abs(a[x] - b[x]) for x in range(W)) / float(W)
        jumps.append(d)
    jumps_sorted = sorted(jumps)
    med_jump = jumps_sorted[len(jumps_sorted) // 2]
    max_jump = max(jumps)
    jump_ratio = max_jump / med_jump if med_jump > 0.5 else max_jump / 0.5

    # 2) 最佳水平错位（只在大跳变处检查）
    worst_y = jumps.index(max_jump)
    def best_shift(a, b):
        best_s, best_d = 0, None
        for s in range(-14, 15):
            d, n = 0, 0
            for x in range(16, W - 16):
                d += abs(a[x] - b[x + s])
                n += 1
            d /= n
            if best_d is None or d < best_d:
                best_d, best_s = d, s
        return best_s
    shift = best_shift(rows[worst_y], rows[worst_y + 1])

    # 3) 纯黑带（整行均值 < 8 的最长连续段）
    run_rows = [sum(r) / float(W) for r in rows]
    longest_black = cur = 0
    for m in run_rows:
        if m < 8.0:
            cur += 1
            longest_black = max(longest_black, cur)
        else:
            cur = 0

    # 判定
    torn = (jump_ratio > 6.0) or (abs(shift) >= 8) or (longest_black >= 6)

    return dict(
        name=os.path.basename(path),
        jump_ratio=jump_ratio,
        max_jump=max_jump,
        med_jump=med_jump,
        worst_y=worst_y,
        shift=shift,
        black=longest_black,
        torn=torn,
    )


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))
    print('%-22s %7s %8s %6s %7s %5s  %s' % (
        '图片', '跳变比', '最大跳变', '位置', '错位', '黑带', '结论'))
    print('-' * 74)

    keep, drop = [], []
    for f in files:
        r = analyze(f)
        tag = '撕裂' if r['torn'] else 'ok'
        why = []
        if r['jump_ratio'] > 6.0:
            why.append('跳变%.1fx' % r['jump_ratio'])
        if abs(r['shift']) >= 8:
            why.append('错位%d' % r['shift'])
        if r['black'] >= 6:
            why.append('黑带%d行' % r['black'])
        print('%-22s %7.1f %8.1f %6d %7d %5d  %-4s %s' % (
            r['name'][-18:], r['jump_ratio'], r['max_jump'], r['worst_y'],
            r['shift'], r['black'], tag, ' '.join(why)))
        (drop if r['torn'] else keep).append(r['name'])

    print('-' * 74)
    print('保留 %d 张，删除 %d 张' % (len(keep), len(drop)))
    print()
    print('=== 要删除的 ===')
    for n in drop:
        print('  ' + n)
    print()
    print('=== 要保留的 ===')
    for n in keep:
        print('  ' + n)


if __name__ == '__main__':
    main()
