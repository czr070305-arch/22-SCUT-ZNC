# -*- coding: utf-8 -*-
"""图集完整对照表（最终版，与 C 固件一致）

判定链（和固件 vision_task.c 的顺序一致）:
    1. 爬线 track_build
    2. 转向 track_steer
    3. 直角弯后处理 post_detect_corner
    4. 环岛 v2e_detect_island（优先级最高）

元素字母优先级（和 task_track.c 的显示逻辑一致）:
    I 环岛 > F 岔路 > C 十字 > L/R 直角 > N 普通
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

WANT = {'S': 'N', 'O': 'I', 'F': 'N', 'CL': 'L', 'CR': 'R', 'L': 'L', 'R': 'R'}
SCENE_CN = {'S': '直道', 'O': '环岛', 'F': '分叉', 'CL': '左直角', 'CR': '右直角',
            'L': '左弯', 'R': '右弯', 'X': '撕裂', '?': '未知'}


def analyze(f):
    im = Image.open(f).convert('L')
    px = list(im.getdata())
    frame = [px[y * vs.V_W:(y + 1) * vs.V_W] for y in range(vs.V_H)]

    thre = vs.img_threshold(frame)
    runs, rw = vs.binarize_runs(frame, thre)

    S = vs.State()
    vs.track_build(S, runs, rw, multi_start=vs.P.get('MULTI_START', True))
    ok_steer = vs.track_steer(S)

    # 直角弯后处理
    cd = vs.post_detect_corner(S, runs) if S.path_rows >= vs.P['CORNER_MIN_ROWS'] else 0
    # 环岛
    isl = island.detect_island(S, runs, rw) if S.path_rows >= 20 else 0

    valid = 1 if (S.path_rows >= vs.P['MIN_ROWS_STEER'] and ok_steer) else 0

    # 字母优先级：环岛 > 直角
    if isl != 0:
        letter = 'I'
    elif cd > 0:
        letter = 'R'
    elif cd < 0:
        letter = 'L'
    else:
        letter = 'N'

    # 绿灯落线率
    hit = cnt = 0
    for y in range(vs.V_H):
        if S.mid[y] >= 0:
            cnt += 1
            if frame[y][S.mid[y]] > thre:
                hit += 1
    on_line = (hit * 100 // cnt) if cnt else 0

    # 宽度比（诊断用）
    ratio = 0.0
    prof = []
    for y in range(vs.P.get('TOP_CUT', 22), vs.V_H):
        if S.mid[y] < 0:
            continue
        for (l, r) in runs[y]:
            if l <= S.mid[y] <= r:
                prof.append((y, r - l + 1))
                break
    if len(prof) >= 20:
        prof.sort()
        low = sorted(w for (_, w) in prof[len(prof) * 4 // 10:])
        med = low[len(low) // 2] if low else 2
        maxw = max(w for (_, w) in prof)
        ratio = maxw / float(med or 2)

    return dict(
        name=f.split('/')[-1], thre=thre, rows=S.path_rows, err=S.steer_err,
        letter=letter, valid=valid, island=isl, corner=cd,
        on_line=on_line, ratio=ratio, bottom=S.bottom_x, top=S.path_top,
    )


def main():
    files = sorted(glob.glob(os.path.join(_ROOT, 'dataset', '*.png')))

    print('=' * 104)
    print('图集完整对照表  ——  当前算法（与 C 固件同一套参数与判定顺序）')
    print('=' * 104)
    print('%-15s %-5s %-4s %4s %4s %7s %4s %4s %6s %-4s %s' % (
        '图片', '场景', '期望', 'T', '行数', 'E', '有效', '落线', '宽度比', '元素', '判定'))
    print('-' * 104)

    stat_ok = stat_n = stat_skip = 0
    by_scene = {}

    for f in files:
        scene, _ = labels.get(f)
        r = analyze(f)
        want = WANT.get(scene, '-')
        is_tear = scene.startswith('X')

        if is_tear:
            verdict = '跳过(撕裂)'
            stat_skip += 1
        elif r['rows'] < 20:
            verdict = '巡线失败'
            stat_n += 1
        elif want == '-':
            verdict = '—'
        elif r['letter'] == want:
            verdict = '对'
            stat_ok += 1
            stat_n += 1
        else:
            verdict = '错'
            stat_n += 1

        if not is_tear:
            s = by_scene.setdefault(scene, [0, 0])
            s[1] += 1
            if verdict == '对':
                s[0] += 1

        print('%-15s %-5s %-4s %4d %4d %7.1f %4s %3d%% %5.1fx %-4s %s' % (
            r['name'][-13:], SCENE_CN.get(scene, scene), want,
            r['thre'], r['rows'], r['err'],
            '+' if r['valid'] else '-', r['on_line'], r['ratio'],
            r['letter'], verdict))

    print('-' * 104)
    print()
    print('分场景汇总（不含撕裂图）')
    print('-' * 42)
    print('%-10s %8s %8s %8s' % ('场景', '张数', '正确', '正确率'))
    for sc in ['S', 'O', 'F', 'CL', 'CR']:
        if sc not in by_scene:
            continue
        ok, n = by_scene[sc]
        print('%-10s %8d %8d %7d%%' % (SCENE_CN.get(sc, sc), n, ok,
                                       ok * 100 // n if n else 0))
    print('-' * 42)
    print()
    print('总计 %d 张，撕裂 %d 张，有效 %d 张' % (
        len(files), stat_skip, stat_skip + stat_n))
    print('识别正确 %d / %d = %.0f%%' % (
        stat_ok, stat_n, stat_ok * 100.0 / max(stat_n, 1)))


if __name__ == '__main__':
    main()
