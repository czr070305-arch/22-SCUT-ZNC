# -*- coding: utf-8 -*-
"""生成实车对拍表

用途：把一张图集图片在 PC 上跑出的结果列出来，
你在板上用 CAPTURE/RAW 页看到的现象应该对得上。
任何一项对不上，问题就在那一层，不用瞎猜。
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import glob
import sys

sys.path.insert(0, 'capture')

import labels
import vision_sim as vs
import island

# 与 C 固件保持一致的参数
vs.P['MULTI_START'] = True


def two_pass_thre(frame):
    """跟 C 版 img_threshold() 逐行等价"""
    hi = 0
    for row in frame:
        for v in row:
            if v > hi:
                hi = v
    dark_cut = hi * 15 // 100

    hist = [0] * 256
    total = 0
    lo, hi2 = 255, 0
    for row in frame:
        for v in row:
            if v <= dark_cut:
                continue
            hist[v] += 1
            total += 1
            if v < lo:
                lo = v
            if v > hi2:
                hi2 = v
    if total < 64:
        return vs.P['THRE_MIN']
    t = vs.otsu_threshold(hist, total) + vs.P['THRE_MARGIN']
    floor = lo + (hi2 - lo) * 2 // 10
    if t < floor:
        t = floor
    return max(vs.P['THRE_MIN'], min(vs.P['THRE_MAX'], t))


def run(f):
    from PIL import Image
    im = Image.open(f).convert('L')
    px = list(im.getdata())
    frame = [px[y * vs.V_W:(y + 1) * vs.V_W] for y in range(vs.V_H)]

    thre = two_pass_thre(frame)
    runs, rw = vs.binarize_runs(frame, thre)
    S = vs.State()
    vs.track_build(S, runs, rw, multi_start=True)
    vs.track_steer(S)

    isl = island.detect_island(S, runs, rw) if S.path_rows >= 20 else 0
    cor = S.corner_dir

    if isl != 0:
        letter = 'I'
    elif S.path_rows < 20:
        letter = 'N'
    elif cor > 0:
        letter = 'R'
    elif cor < 0:
        letter = 'L'
    else:
        letter = 'N'

    return dict(name=f.split('/')[-1], thre=thre, rows=S.path_rows,
                err=S.steer_err, letter=letter, isl=isl, cor=cor,
                valid=1 if S.path_rows >= vs.P['MIN_ROWS_STEER'] else 0)


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))
    print('=' * 78)
    print('实车对拍表  (TRACK 页应显示的值)')
    print('=' * 78)
    print('%-20s %4s %5s %7s %4s %s' % ('图片', 'T', '行数', 'E', '元素', '说明'))
    print('-' * 78)
    for f in files:
        scene, _ = labels.get(f)
        r = run(f)
        note = ''
        if scene.startswith('X'):
            note = '(撕裂图, 仅供参考)'
        elif r['rows'] < 20:
            note = '爬线失败 -> E 冻结, 元素 N'
        print('%-20s %4d %5d %7.1f   %s   %s' % (
            r['name'][-18:], r['thre'], r['rows'], r['err'], r['letter'], note))

    print()
    print('=' * 78)
    print('怎么用')
    print('=' * 78)
    print('''
1. 把车摆成跟图集一样的角度（正对白线）
2. 看 TRACK 页第 2 行 T 后面的数字 = 阈值
     PC 算出 60~83 之间。如果你板上是 120+ 或 20-，说明
     曝光/镜头跟采集图集时不一样，先调回再往下看。
3. 看第 1 行 E 后面的数字 = 转向误差
     对不上时先看第 2 行末尾的 +/-：
       + = 视觉认为线有效（err_valid=1）
       - = 视觉认为丢线（E 是上一帧的残值）
4. 看第 2 行末尾的字母
     N 普通 / I 环岛 / L 左直角 / R 右直角 / C 十字 / F 岔路
''')


if __name__ == '__main__':
    main()
