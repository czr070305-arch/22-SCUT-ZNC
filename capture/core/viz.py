# -*- coding: utf-8 -*-
"""把仿真结果画到图上: 白线段(灰) / 爬线结果(绿) / 前瞻点(红) / 起点(蓝)"""
import glob
import os
import sys

from PIL import Image, ImageDraw

import vision_sim as vs

SCALE = 3


def draw(path, bias=0):
    r = vs.run_one(path, bias)
    im = Image.open(path).convert('RGB')
    im = im.resize((vs.V_W * SCALE, vs.V_H * SCALE), Image.NEAREST)
    d = ImageDraw.Draw(im)

    # 白线段（灰框）
    for y in range(vs.V_H):
        for (l, rr) in r['runs'][y]:
            d.rectangle([l * SCALE, y * SCALE, rr * SCALE + SCALE - 1, y * SCALE + SCALE - 1],
                        outline=(120, 120, 120))

    # 爬线中点（绿） + 边沿（青）
    for y in range(vs.V_H):
        if r['mid'][y] >= 0:
            x = r['mid'][y]
            d.rectangle([x * SCALE, y * SCALE, x * SCALE + SCALE - 1, y * SCALE + SCALE - 1],
                        fill=(0, 255, 0))
        if r['edge_l'][y] >= 0:
            d.point([r['edge_l'][y] * SCALE, y * SCALE], fill=(0, 200, 200))
            d.point([r['edge_r'][y] * SCALE, y * SCALE], fill=(0, 200, 200))

    # 前瞻点（红）
    if r['look'][0] >= 0:
        lx, ly = r['look']
        d.ellipse([lx * SCALE - 5, ly * SCALE - 5, lx * SCALE + 5, ly * SCALE + 5], fill=(255, 0, 0))
        # 中线基准（黄线）
        d.line([vs.V_W // 2 * SCALE, ly * SCALE, lx * SCALE, ly * SCALE], fill=(255, 255, 0))

    # 起点（蓝）
    if r['bottom'] >= 0:
        d.ellipse([r['bottom'] * SCALE - 4, (vs.V_H - 1) * SCALE - 4,
                   r['bottom'] * SCALE + 4, (vs.V_H - 1) * SCALE + 4], fill=(0, 100, 255))

    return im, r


def main():
    files = sorted(glob.glob('dataset/*.png'))
    if len(sys.argv) > 1:
        files = [f for f in files if any(a in f for a in sys.argv[1:])]
    else:
        files = files

    imgs = []
    infos = []
    for f in files:
        im, r = draw(f)
        imgs.append(im)
        infos.append('%s rows=%d err=%.0f' % (os.path.basename(f)[-14:-4], r['rows'], r['err']))

    COLS = 4
    SW, SH = vs.V_W * SCALE, vs.V_H * SCALE
    ROWS = (len(imgs) + COLS - 1) // COLS
    sheet = Image.new('RGB', (COLS * SW, ROWS * SH), (30, 30, 30))
    for i, im in enumerate(imgs):
        sheet.paste(im, ((i % COLS) * SW, (i // COLS) * SH))
    sheet.save('capture/_sim_result.png')
    print('saved capture/_sim_result.png', sheet.size)
    for i, s in enumerate(infos):
        print('  [%2d] row%d col%d  %s' % (i + 1, i // COLS + 1, i % COLS + 1, s))


if __name__ == '__main__':
    main()
