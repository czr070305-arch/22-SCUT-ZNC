# -*- coding: utf-8 -*-
"""把二值化后的图像放大输出, 看清白线段在哪"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import sys

from PIL import Image

import vision_sim as vs

SCALE = 5


def main(f, tag=''):
    r = vs.run_one(f)
    px = [0] * (vs.V_W * vs.V_H)
    for y in range(vs.V_H):
        for (l, rr) in r['runs'][y]:
            for x in range(l, rr + 1):
                px[y * vs.V_W + x] = 255     # 参加到 run 的白段 -> 亮
    im = Image.frombytes('L', (vs.V_W, vs.V_H), bytes(px))
    im = im.resize((vs.V_W * SCALE, vs.V_H * SCALE), Image.NEAREST)
    out = 'capture/_bina_%s.png' % (tag or f.split('_')[-1].split('.')[0])
    im.save(out)
    print(out, 'thre=%d rows=%d' % (r['thre'], r['rows']))


if __name__ == '__main__':
    args = sys.argv[1:]
    tag = ''
    if args and not args[0].endswith('.png'):
        tag = args[0]
        args = args[1:]
    for f in args:
        main(f, tag)
