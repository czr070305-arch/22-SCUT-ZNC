# -*- coding: utf-8 -*-
"""环岛识别: 基于"双白段间距发散" + "一侧持续存在"

2025 赛道只有一个六边形环岛，进入时画面特征是：
  - 参考线（车所在的那条）继续往上
  - 旁边出现第二条线（岛的边界）
  - 两条线的间距随 y 上升逐渐拉大（发散）

而直角弯只有一条线（横臂），分叉的两条线是对称分开的。
"""

import glob
import sys

import labels
import vision_sim as vs

# 画面顶部这么多行是房间杂物（桌沿/路由器/窗帘），不参与元素判断
ISLAND_TOP_CUT = 22      # 顶部裁切: 房间杂物区
ISLAND_MIN_Y = 40        # 双段行至少要有一条出现在这行以下（否则是房间杂物）


def detect_island(S, runs, row_white):
    """环岛识别 v5（与 C 固件 v2e_detect_island 一致）

    关键判据:
      1. 双段行必须【连续】（相邻行差 <= 2），且连续段 >= 5 行
      2. 连续段内间距从下往上单调变大（发散比 >= 1.25）
      3. 第二条线稳定在同一侧（>= 70%）
    """
    if S.path_rows < 20 or S.path_top < 0:
        return 0

    low = [S.mid[y] for y in range(int(vs.V_H * 0.45), vs.V_H) if S.mid[y] >= 0]
    if len(low) < 6:
        return 0
    low.sort()
    ref = low[len(low) // 2]

    row_sep = [0] * vs.V_H
    row_side = [0] * vs.V_H

    for y in range(ISLAND_TOP_CUT, vs.V_H - 4):
        if row_white[y] > vs.P['ROW_WHITE_MAX']:
            continue
        cand = []
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < vs.P['RUN_W_MIN'] or w > vs.P['RUN_W_MAX_NARROW']:
                continue
            cand.append((abs(0.5 * (l + r) - ref), 0.5 * (l + r)))
        if len(cand) < 2:
            continue
        cand.sort()
        d1, c1 = cand[0]
        d2, c2 = cand[1]
        if d1 > 22:
            continue
        sep = abs(c2 - c1)
        if sep < 4 or sep > 80:
            continue
        row_sep[y] = sep
        row_side[y] = 1 if c2 > c1 else -1

    # 找最长连续段（允许跳 1 行）
    best_lo, best_len = -1, 0
    cur_lo, cur_len = -1, 0
    for y in range(ISLAND_TOP_CUT, vs.V_H - 4):
        if row_sep[y] > 0:
            if cur_len == 0:
                cur_lo = y
            cur_len += 1
            if cur_len > best_len:
                best_len = cur_len
                best_lo = cur_lo
        else:
            if y + 1 < vs.V_H - 4 and row_sep[y + 1] > 0:
                continue
            cur_len = 0

    if best_len < 15:
        return 0

    seg = [y for y in range(best_lo, min(best_lo + best_len + 2, vs.V_H - 4))
           if row_sep[y] > 0]
    n = len(seg)
    if n < 15:
        return 0

    # 最大间距上限（真环岛 31~45，十字/误报 59~79）
    if max(row_sep[y] for y in seg) > 55:
        return 0

    k = max(1, n // 3)
    top_sep = sum(row_sep[y] for y in seg[:k])
    bot_sep = sum(row_sep[y] for y in seg[-k:])
    if bot_sep < 1:
        return 0
    if top_sep * 100 < bot_sep * 125:
        return 0

    n_right = sum(1 for y in seg if row_side[y] > 0)
    if n_right * 10 >= n * 7:
        return 1
    if (n - n_right) * 10 >= n * 7:
        return -1
    return 0


def main():
    files = sorted(glob.glob('dataset/*.png'))
    print('%-18s %5s %5s %6s %8s  %s' % ('file', 'rows', 'top', '真值', '我判定', '说明'))
    print('-' * 70)
    hit = miss = false_pos = 0
    for f in files:
        scene, _ = labels.get(f)
        r = vs.run_one(f)
        if r['rows'] < 20:
            continue
        # 重算 runs（run_one 有返回）
        isl = detect_island_pub(r)
        want = 1 if scene == 'O' else 0
        if scene == 'O':
            if isl != 0:
                hit += 1
            else:
                miss += 1
        elif isl != 0 and not scene.startswith('X'):
            false_pos += 1
        mark = ''
        if scene == 'O':
            mark = '  <- 环岛' + (' 命中' if isl else ' 漏检')
        elif isl != 0 and not scene.startswith('X'):
            mark = '  <- 误报'
        print('%-18s %5d %5d %6s %8d%s' % (r['name'][-14:], r['rows'], r['top'],
                                           scene, isl, mark))
    print('-' * 70)
    print('环岛命中 %d, 漏检 %d, 误报 %d' % (hit, miss, false_pos))


def detect_island_pub(r):
    """从 run_one 的结果重建 State 再检测"""
    S = vs.State()
    S.mid = r['mid']
    S.edge_l = r['edge_l']
    S.edge_r = r['edge_r']
    S.path_top = r['top']
    S.path_rows = r['rows']
    S.bottom_x = r['bottom']
    return detect_island(S, r['runs'], r['row_white'])


if __name__ == '__main__':
    main()
