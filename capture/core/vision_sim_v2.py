# -*- coding: utf-8 -*-
"""
vision_sim_v2.py —— 逐行对照当前固件（feat/vision-team-merge）重写的离线仿真

与 vision_sim.py（旧版，对应 11-19 固件）的区别：
  新增   start_chain      起点连续性，避免选到底部车头阴影
  新增   CFG_TRACK_TOP    远端裁剪，避开赛道外白板
  新增   two_runs_above   区分折角 vs 岔路/环岛合流
  新增   corner_from_path_end  横臂只剩窄段时的折角兜底
  新增   s_corner_hold    折角方向记忆
  新增   fit_bands        近/中/远三段最小二乘 + 航向项
  新增   element 的 probe_rows / find_fork / find_island / count_sep_run
  新增   state_update     状态机（只输出 speed_pct / state）

用法:
    python capture/vision_sim_v2.py [图集目录]
"""
import os
import sys
import glob
import math

from PIL import Image

V_W, V_H = 188, 120

# ============================================================================
# 参数 —— 与 code/vision/vision_cfg.h 逐项对应
# ============================================================================
CFG = dict(
    THRE_MARGIN=8, THRE_MIN=30, THRE_MAX=196, THRE_DARK_PCT=12,
    RUN_W_MIN=3, RUN_W_MAX_NARROW=38, RUN_W_MAX_WIDE=150,
    CROSS_W=60, ROW_WHITE_MAX=96,
    START_ROWS=9, START_TOL=36,
    TOL_BASE=7.0, TOL_GROW=0.75, TOL_MAX=26.0,
    EXTRAP_ROWS=9, BLIND_ROWS=24, SLOPE_SMOOTH=0.72,
    LOOKAHEAD=44, ERR_OFF_W=1.0, ERR_SLOPE_W=4.0,
    CORNER_ENABLE=0, CORNER_BIAS=8.0,
    CORNER_MIN_ROWS=12, CORNER_W_MAX=110, CORNER_NEAR_END=8,
    RUN_EDGE_TOL=6, CORNER_LOOK=40, CORNER_MIN_Y=10,
    MIN_ROWS_STEER=10,
    FORK_REF_TOL=13, FORK_MIN_ROWS=12,
    BRANCH_MIN_D=11, BRANCH_MAX_D=42, DIVERGE_PX=4,
    ISLE_MIN_ROWS=12, ISLE_SEP_MIN=8, ISLE_SEP_MAX=60,
    ISLE_LAST_SEP=16, ISLAND_EXIT_ROWS=36,
    CROSS_ARM_MIN=18, ISLAND_FORCE=14, ELEM_CD=22,
    ISLAND_SIDE=0, ISLAND_MIN_FRAME=10, ISLAND_MAX_FRAME=260,
    ISLAND_QUIET=5,
    UNDIST_ENABLE=1,
    TRACK_TOP=24,
    START_CHAIN_ROWS=8, START_CHAIN_TOL=12.0, START_CHAIN_MIN=3,
    CORNER_FORK_ROWS=7, CORNER_FORK_SEP=8, CORNER_END_MIN=8,
    CORNER_HOLD=3, ELEM_EXTRA_ROWS=20,
    ISLE_CONFIRM=2, ISLE_FAST_ROWS=12, ISLE_ENTRY_ROWS=10,
    HEAD_ROWS=14,
    STATE_MAX_CORNER=30, STATE_MAX_CROSS=40, STATE_MAX_ISLAND=250,
    SPD_CORNER=70, SPD_CROSS=100, SPD_ISLAND=60,
)

# 畸变（学长 Perspective.c，非 OpenCV 顺序）
CAM_FX, CAM_FY = 148.404692, 151.874202
CAM_CX, CAM_CY = 96.738556, 50.592531
CAM_MOVE_X, CAM_MOVE_Y = 0.0, -9.0
K1, K2, K3 = -1.101773, 1.298787, -0.684578
P1, P2 = -0.006304, 0.004262


# ============================================================================
# 畸变
# ============================================================================
def distort_pixel(u, v):
    x = (u - CAM_MOVE_X - CAM_CX) / CAM_FX
    y = (v - CAM_MOVE_Y - CAM_CY) / CAM_FY
    r2 = x * x + y * y
    k = 1.0 + K1 * r2 + K2 * r2 * r2 + K3 * r2 * r2 * r2
    dx = x * k + 2 * P1 * x * y + P2 * (r2 + 2 * x * x)
    dy = y * k + 2 * P2 * x * y + P1 * (r2 + 2 * y * y)
    return CAM_CX + CAM_FX * dx, CAM_CY + CAM_FY * dy


def undistort_pixel(xd, yd, nmax=16):
    """复刻 C 的 vision_undistort_pixel（16 次迭代）"""
    uu, vv = xd, yd
    for _ in range(nmax):
        px, py = distort_pixel(uu, vv)
        du, dv = xd - px, yd - py
        uu += du
        vv += dv
        if abs(uu) > 1e4 or abs(vv) > 1e4 or uu != uu or vv != vv:
            return xd, yd, 0
        if abs(du) < 0.02 and abs(dv) < 0.02:
            return uu, vv, 1
    if abs(uu) > 1e4 or uu != uu:
        uu = xd
    if abs(vv) > 1e4 or vv != vv:
        vv = yd
    return uu, vv, 0


def undistort_x(xd, yd):
    u, _, _ = undistort_pixel(xd, yd)
    return u


# ============================================================================
# 二值化 + 提段（复刻 vision_imgproc.c 的两遍法）
# ============================================================================
def img_threshold(frame, bias=0):
    hi = 0
    for row in frame:
        m = max(row)
        if m > hi:
            hi = m
    dark_cut = (hi * CFG['THRE_DARK_PCT']) // 100

    hist = [0] * 256
    total = 0
    for row in frame:
        for v in row:
            if v <= dark_cut:
                continue
            hist[v] += 1
            total += 1
    if total < 64:
        return CFG['THRE_MIN']

    sum_all = sum(i * hist[i] for i in range(256))
    w_b = 0
    sum_b = 0
    best_var = -1.0
    best_t = CFG['THRE_MIN']
    for t in range(256):
        w_b += hist[t]
        if w_b == 0:
            continue
        w_f = total - w_b
        if w_f == 0:
            break
        sum_b += t * hist[t]
        m_b = sum_b / w_b
        m_f = (sum_all - sum_b) / w_f
        var = w_b * w_f * (m_b - m_f) ** 2
        if var > best_var:
            best_var = var
            best_t = t

    thre = best_t + CFG['THRE_MARGIN'] + bias
    if thre < CFG['THRE_MIN']:
        thre = CFG['THRE_MIN']
    if thre > CFG['THRE_MAX']:
        thre = CFG['THRE_MAX']
    return thre


def binarize_runs(frame, thre):
    """3x3 去噪 + 逐行提段（复刻 img_binarize_runs）"""
    runs = [[] for _ in range(V_H)]
    row_white = [0] * V_H

    b = [[1 if v > thre else 0 for v in row] for row in frame]
    for y in range(V_H):
        cur = b[y]
        up = b[y - 1] if y > 0 else [0] * V_W
        dn = b[y + 1] if y + 1 < V_H else [0] * V_W
        out = [0] * V_W
        for x in range(V_W):
            if not cur[x]:
                continue
            lf = cur[x - 1] if x > 0 else 0
            rt = cur[x + 1] if x + 1 < V_W else 0
            out[x] = 1 if ((up[x] and dn[x]) or (lf and rt)) else 0

        n = 0
        st = -1
        for x in range(V_W + 1):
            white = (x < V_W) and (out[x] != 0)
            if white:
                if st < 0:
                    st = x
            elif st >= 0:
                end = x - 1
                w = end - st + 1
                if w >= CFG['RUN_W_MIN']:
                    if n > 0 and st - runs[y][n - 1][1] <= 2:
                        runs[y][n - 1] = (runs[y][n - 1][0], end)
                    elif n < 8:
                        runs[y].append((st, end))
                        n += 1
                st = -1
        row_white[y] = sum(r - l + 1 for l, r in runs[y])
    return runs, row_white


# ============================================================================
# 全局状态（对应 C 的 g_*）
# ============================================================================
class G:
    def __init__(self):
        self.mid = [-1] * V_H
        self.edge_l = [-1] * V_H
        self.edge_r = [-1] * V_H
        self.path_top = -1
        self.path_top_x = 0
        self.path_slope = 0.0
        self.path_rows = 0
        self.bottom_x = -1
        self.corner_dir = 0
        self.corner_far_x = 0
        self.look_x = -1
        self.look_y = -1
        self.steer_err = 0.0
        self.lost = 0
        self.lost_cnt = 0
        self.prev_bottom = -1

        self.elem = 0
        self.elem_side = 0
        self.prefer_side = 0
        self.island_on = 0
        self.island_side = 0
        self.fork_rows = 0
        self.isle_rows = 0
        self.cross_rows = 0

        self.state = 0
        self.state_frames = 0
        self.speed_pct = 100

        self.fit_rows = [0] * 3
        self.fit_x = [V_W // 2] * 3
        self.fit_k = [0.0] * 3
        self.fit_y = [0] * 3

        self.c_cd = 0
        self.isle_quiet = 0
        self.island_frames = 0
        self.island_force = 0
        self.isle_hit = 0
        self.corner_hold = 0
        self.corner_dir_keep = 0
        self.corner_x_keep = 0
        self.row_white = [0] * V_H
        self.sep = [0] * V_H
        self.br = [0] * V_H

    def reset_corner_hold(self):
        self.corner_hold = 0
        self.corner_dir_keep = 0
        self.corner_x_keep = 0


def iabs(v):
    return -v if v < 0 else v


def v_sign(v):
    return 1 if v > 0 else (-1 if v < 0 else 0)


def find_run_near(runs, y, pred, tol, narrow=True):
    best = -1
    best_d = 1e6
    for i, (l, r) in enumerate(runs[y]):
        w = r - l + 1
        if w < CFG['RUN_W_MIN']:
            continue
        if narrow:
            if w > CFG['RUN_W_MAX_NARROW']:
                continue
        elif w > CFG['RUN_W_MAX_WIDE']:
            continue
        c = 0.5 * (l + r)
        d = abs(c - pred)
        if d <= tol and d < best_d:
            best_d = d
            best = i
    return best


def find_run_prefer(runs, y, pred, side):
    best = -1
    best_d = -1
    best_c = -1
    for i, (l, r) in enumerate(runs[y]):
        w = r - l + 1
        if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
            continue
        c = (l + r) // 2
        d = c - int(pred)
        if d * side <= 0 or iabs(d) < 4 or iabs(d) > 34:
            continue
        if iabs(d) > best_d:
            best_d = iabs(d)
            best = i
            best_c = c
    if best >= 0 and best_c >= 0 and y + 1 < V_H:
        if find_run_near(runs, y + 1, float(best_c), 6.0, True) < 0:
            best = -1
    return best


# ============================================================================
# 爬线（复刻 track.c）
# ============================================================================
def start_chain(g, runs, y, c):
    cnt = 0
    miss = 0
    for k in range(1, CFG['START_CHAIN_ROWS'] + 1):
        yy = y - k
        if yy < 0:
            break
        if g.row_white[yy] > CFG['ROW_WHITE_MAX']:
            cnt += 1
            miss = 0
            continue
        if find_run_near(runs, yy, float(c), CFG['START_CHAIN_TOL'], True) >= 0:
            cnt += 1
            miss = 0
        else:
            miss += 1
            if miss >= 2:
                break
    return cnt


def two_runs_above(g, runs, y, pred):
    for k in range(2, CFG['CORNER_FORK_ROWS'] + 1):
        yy = y - k
        if yy < 0:
            break
        if g.row_white[yy] > CFG['ROW_WHITE_MAX']:
            continue
        n = 0
        l = -1
        r = -1
        for (ll, rr) in runs[yy]:
            w = rr - ll + 1
            c = (ll + rr) // 2
            if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
                continue
            if iabs(c - int(pred)) > CFG['BRANCH_MAX_D'] + 10:
                continue
            if l < 0:
                l = c
            else:
                r = c
            n += 1
            if n >= 2:
                break
        if n >= 2 and iabs(r - l) >= CFG['CORNER_FORK_SEP']:
            return 1
    return 0


def side_runs_below(g, runs, y0, side):
    cnt = 0
    for k in range(1, 11):
        y = y0 + k
        if y >= V_H:
            break
        m = g.mid[y] if g.mid[y] >= 0 else g.path_top_x
        for (l, r) in runs[y]:
            w = r - l + 1
            c = (l + r) // 2
            d = (c - m) * side
            if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
                continue
            if CFG['CORNER_END_MIN'] <= d <= CFG['CORNER_LOOK'] + 8:
                cnt += 1
                break
    return cnt


def corner_from_path_end(g, runs):
    if g.corner_dir != 0 or g.path_top < 0:
        return
    if g.path_top > V_H - 30:
        return
    if g.elem != 0 or g.fork_rows >= CFG['FORK_MIN_ROWS'] or g.isle_rows > 0:
        return

    pos_rows = neg_rows = 0
    pos_d = neg_d = 0
    for y in range(g.path_top - 10, g.path_top + 5):
        if y < 0 or y >= V_H:
            continue
        m = g.mid[y] if g.mid[y] >= 0 else g.path_top_x
        for (l, r) in runs[y]:
            w = r - l + 1
            c = (l + r) // 2
            d = c - m
            if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
                continue
            if iabs(d) < CFG['CORNER_END_MIN'] or iabs(d) > CFG['CORNER_LOOK'] + 8:
                continue
            if d > 0:
                pos_rows += 1
                pos_d = max(pos_d, d)
            else:
                neg_rows += 1
                neg_d = max(neg_d, -d)

    if pos_rows >= 3 and pos_rows >= neg_rows:
        side = 1
    elif neg_rows >= 3:
        side = -1
    else:
        return
    if side_runs_below(g, runs, g.path_top, side) >= 4:
        return

    g.corner_dir = side
    g.corner_far_x = g.path_top_x + g.corner_dir * CFG['CORNER_LOOK']
    g.corner_far_x = max(2, min(V_W - 3, g.corner_far_x))
    g.corner_hold = CFG['CORNER_HOLD']
    g.corner_dir_keep = g.corner_dir
    g.corner_x_keep = g.corner_far_x


def track_build(g, runs, prefer_side=0):
    for y in range(V_H):
        g.mid[y] = -1
        g.edge_l[y] = -1
        g.edge_r[y] = -1
    g.path_top = -1
    g.path_top_x = 0
    g.path_slope = 0.0
    g.path_rows = 0
    g.bottom_x = -1
    g.corner_dir = 0
    g.corner_far_x = 0

    if g.corner_hold > 0:
        g.corner_dir = g.corner_dir_keep
        g.corner_far_x = g.corner_x_keep
        g.corner_hold -= 1

    # ---- 1) 起点：底部窗口内挑 start_chain 最长的 ----
    ref = g.prev_bottom if g.prev_bottom >= 0 else V_W // 2
    y0 = V_H - 1
    ymin = max(0, y0 - CFG['START_ROWS'])
    best_y = -1
    best_i = -1
    best_chain = -1
    best_d = 1e6

    for y in range(y0, ymin - 1, -1):
        if g.row_white[y] > CFG['ROW_WHITE_MAX']:
            continue
        for i, (l, r) in enumerate(runs[y]):
            w = r - l + 1
            if w < CFG['RUN_W_MIN'] or w > CFG['CROSS_W']:
                continue
            c = 0.5 * (l + r)
            d = abs(c - ref)
            if d > CFG['START_TOL']:
                continue
            chain = start_chain(g, runs, y, int(c))
            if chain > best_chain or (chain == best_chain and d < best_d):
                best_chain = chain
                best_d = d
                best_y = y
                best_i = i

    if best_y < 0:
        return 0

    l, r = runs[best_y][best_i]
    cur = 0.5 * (l + r)
    slope = 0.0
    start_y = best_y
    g.bottom_x = int(cur)
    g.mid[best_y] = int(cur)
    g.edge_l[best_y] = l
    g.edge_r[best_y] = r
    g.path_top = best_y
    g.path_top_x = int(cur)
    g.path_rows = 1
    g.prev_bottom = int(cur)

    rows_since = 0
    blind_since = 0

    # ---- 2) 逐行往上 ----
    for y in range(start_y - 1, CFG['TRACK_TOP'] - 1, -1):
        pred = cur + slope
        tol = min(CFG['TOL_BASE'] + CFG['TOL_GROW'] * rows_since, CFG['TOL_MAX'])

        # 盲行
        if g.row_white[y] > CFG['ROW_WHITE_MAX']:
            cand = -1
            for i, (l, r) in enumerate(runs[y]):
                wi = r - l + 1
                if wi < CFG['RUN_W_MIN'] or wi > CFG['RUN_W_MAX_WIDE']:
                    continue
                if (runs[y][i][0] - CFG['CORNER_NEAR_END'] <= pred
                        <= runs[y][i][1] + CFG['CORNER_NEAR_END']):
                    cand = i
                    break
            if (cand >= 0 and g.elem != 1
                    and g.path_rows >= CFG['CORNER_MIN_ROWS']
                    and y >= CFG['CORNER_MIN_Y']
                    and g.fork_rows < CFG['FORK_MIN_ROWS']):
                l, r = runs[y][cand]
                dl = iabs(l - int(cur))
                dr = iabs(r - int(cur))
                one_side = ((dl <= CFG['CORNER_NEAR_END'] and dr > CFG['CORNER_NEAR_END'] * 2) or
                            (dr <= CFG['CORNER_NEAR_END'] and dl > CFG['CORNER_NEAR_END'] * 2))
                above_ok = 0
                cont = 0
                for k in range(1, 9):
                    if y - k < 0:
                        break
                    if g.row_white[y - k] > CFG['ROW_WHITE_MAX']:
                        cont = 0
                        continue
                    if find_run_near(runs, y - k, pred, CFG['TOL_MAX'] + 6.0, True) >= 0:
                        cont += 1
                        if cont >= 2:
                            above_ok = 1
                            break
                    else:
                        cont = 0

                if one_side and not above_ok and g.isle_rows == 0 and not two_runs_above(g, runs, y, pred):
                    far = l if dl > dr else r
                    if far > int(cur):
                        far = min(far, int(cur) + CFG['CORNER_LOOK'])
                    else:
                        far = max(far, int(cur) - CFG['CORNER_LOOK'])
                    g.corner_dir = 1 if far > int(cur) else -1
                    g.corner_far_x = far
                    g.corner_hold = CFG['CORNER_HOLD']
                    g.corner_dir_keep = g.corner_dir
                    g.corner_x_keep = g.corner_far_x
                    g.mid[y] = far
                    g.edge_l[y] = l
                    g.edge_r[y] = r
                    g.path_top = y
                    g.path_top_x = far
                    g.path_rows += 1
                    break

            blind_since += 1
            rows_since += 1
            slope *= 0.8
            if blind_since > CFG['BLIND_ROWS'] or rows_since > CFG['EXTRAP_ROWS'] + CFG['BLIND_ROWS']:
                break
            continue

        blind_since = 0
        idx = find_run_near(runs, y, pred, tol, True)

        if prefer_side != 0 and len(runs[y]) > 1:
            alt = find_run_prefer(runs, y, pred, prefer_side)
            if alt >= 0:
                idx = alt

        if idx >= 0:
            l, r = runs[y][idx]
            c = 0.5 * (l + r)
            step = float(rows_since + 1)
            inst = (c - cur) / step
            slope = CFG['SLOPE_SMOOTH'] * slope + (1.0 - CFG['SLOPE_SMOOTH']) * inst
            cur = c
            g.mid[y] = int(c)
            g.edge_l[y] = l
            g.edge_r[y] = r
            g.path_top = y
            g.path_top_x = int(c)
            g.path_rows += 1
            rows_since = 0
            continue

        # 宽段折角
        widx = -1
        for i, (l, r) in enumerate(runs[y]):
            wi = r - l + 1
            if wi < CFG['RUN_W_MIN'] or wi > CFG['RUN_W_MAX_WIDE']:
                continue
            if l - CFG['RUN_EDGE_TOL'] <= pred <= r + CFG['RUN_EDGE_TOL']:
                widx = i
                break

        if widx >= 0:
            l, r = runs[y][widx]
            w = r - l + 1
            if w > CFG['RUN_W_MAX_NARROW']:
                above_ok = 0
                dl = iabs(l - int(cur))
                dr = iabs(r - int(cur))
                one_side = ((dl <= CFG['CORNER_NEAR_END'] and dr > CFG['CORNER_NEAR_END']) or
                            (dr <= CFG['CORNER_NEAR_END'] and dl > CFG['CORNER_NEAR_END']))
                for k in range(1, 5):
                    if y - k < 0:
                        break
                    if g.row_white[y - k] > CFG['ROW_WHITE_MAX']:
                        continue
                    if find_run_near(runs, y - k, pred, tol + 12.0, True) >= 0:
                        above_ok = 1
                        break
                if (not above_ok and g.path_rows >= CFG['CORNER_MIN_ROWS']
                        and g.elem != 1 and g.isle_rows == 0
                        and g.fork_rows < CFG['FORK_MIN_ROWS']
                        and not two_runs_above(g, runs, y, pred)
                        and (w <= CFG['CORNER_W_MAX'] or one_side)):
                    far = l if iabs(l - int(cur)) > iabs(r - int(cur)) else r
                    if far > int(cur):
                        far = min(far, int(cur) + CFG['CORNER_LOOK'])
                    else:
                        far = max(far, int(cur) - CFG['CORNER_LOOK'])
                    g.corner_dir = 1 if far > int(cur) else -1
                    g.corner_far_x = far
                    g.corner_hold = CFG['CORNER_HOLD']
                    g.corner_dir_keep = g.corner_dir
                    g.corner_x_keep = g.corner_far_x
                    g.mid[y] = far
                    g.edge_l[y] = l
                    g.edge_r[y] = r
                    g.path_top = y
                    g.path_top_x = far
                    g.path_rows += 1
                    break

        rows_since += 1
        if rows_since > CFG['EXTRAP_ROWS']:
            break
        cur += slope

    corner_from_path_end(g, runs)
    g.path_slope = slope
    return 1


def track_ref_x(g, y):
    if y < 0:
        y = 0
    if y >= V_H:
        y = V_H - 1
    if g.mid[y] >= 0:
        return g.mid[y]
    if g.path_top >= 0 and y < g.path_top:
        x = g.path_top_x + int(g.path_slope * (g.path_top - y))
        return max(2, min(V_W - 3, x))
    for i in range(y + 1, V_H):
        if g.mid[i] >= 0:
            return g.mid[i]
    if g.bottom_x >= 0:
        return g.bottom_x
    return V_W // 2


def track_steer(g):
    y0 = -1
    for i in range(V_H - 1, -1, -1):
        if g.mid[i] >= 0:
            y0 = i
            break
    if y0 < 0:
        g.look_x = -1
        g.look_y = -1
        return

    if g.path_rows < CFG['MIN_ROWS_STEER'] and g.corner_dir == 0:
        g.look_x = -1
        g.look_y = -1
        return

    look_y = max(0, y0 - CFG['LOOKAHEAD'])
    look_x = -1
    for i in range(look_y, y0 + 1):
        if g.mid[i] >= 0:
            look_x = g.mid[i]
            look_y = i
            break
    if look_x < 0:
        look_x = g.mid[y0]
        look_y = y0

    if CFG['CORNER_ENABLE'] and g.corner_dir != 0:
        look_x = g.corner_far_x
        look_y = g.path_top

    if CFG['UNDIST_ENABLE']:
        dx = undistort_x(float(look_x), float(look_y)) - undistort_x(V_W / 2.0, float(look_y))
    else:
        dx = float(look_x - V_W // 2)
    err = dx * CFG['ERR_OFF_W']

    yn = max(0, y0 - 14)
    yn_used = -1
    for i in range(yn, y0):
        if g.mid[i] >= 0:
            yn_used = i
            break
    if yn_used >= 0:
        near_slope = (g.mid[yn_used] - g.mid[y0]) / float(y0 - yn_used)
        err += near_slope * CFG['ERR_SLOPE_W']

    if CFG['CORNER_ENABLE'] and g.corner_dir != 0:
        err += g.corner_dir * CFG['CORNER_BIAS']

    # NaN / Inf 清洗
    if err != err:
        g.look_x = -1
        g.look_y = -1
        g.lost = 1
        return
    if err > 1e6:
        err = 110.0
    if err < -1e6:
        err = -110.0
    err = max(-110.0, min(110.0, err))

    g.steer_err = err
    g.look_x = look_x
    g.look_y = look_y


# ============================================================================
# 元素识别（复刻 element.c）
# ============================================================================
def path_x_at(g, y):
    if y < 0:
        y = 0
    if y >= V_H:
        y = V_H - 1
    if g.mid[y] >= 0:
        return g.mid[y]
    ya = yb = -1
    for i in range(y + 1, V_H):
        if g.mid[i] >= 0:
            yb = i
            break
    for i in range(y - 1, -1, -1):
        if g.mid[i] >= 0:
            ya = i
            break
    if yb >= 0 and ya >= 0:
        xb = g.mid[yb]
        xa = g.mid[ya]
        return xb + (xa - xb) * (yb - y) // (yb - ya)
    if yb >= 0:
        return g.mid[yb]
    if ya >= 0:
        return g.mid[ya]
    return track_ref_x(g, y)


def probe_rows(g, runs):
    for y in range(V_H):
        g.sep[y] = 0
        g.br[y] = 0
    if g.path_top < 0:
        return
    y_lo = max(g.path_top - CFG['ELEM_EXTRA_ROWS'], CFG['TRACK_TOP'])

    for y in range(V_H - 3, y_lo - 1, -1):
        if len(runs[y]) < 2:
            continue
        ref = path_x_at(g, y)
        near_d = 999
        narrow = 0
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
                continue
            narrow += 1
            c = (l + r) // 2
            near_d = min(near_d, iabs(c - ref))
        if narrow < 2:
            continue

        # 环岛：参考点两侧各有一段
        cl = cr = -1
        dl = dr = 9999
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
                continue
            c = (l + r) // 2
            if c < ref - 2:
                if ref - c < dl:
                    dl = ref - c
                    cl = c
            elif c > ref + 2:
                if c - ref < dr:
                    dr = c - ref
                    cr = c
        if cl >= 0 and cr >= 0:
            sep = cr - cl
            if CFG['ISLE_SEP_MIN'] <= sep <= CFG['ISLE_SEP_MAX']:
                g.sep[y] = sep

        # 支线
        if near_d <= CFG['FORK_REF_TOL']:
            bd = bs = 0
            for (l, r) in runs[y]:
                w = r - l + 1
                if w < CFG['RUN_W_MIN'] or w > CFG['RUN_W_MAX_NARROW']:
                    continue
                c = (l + r) // 2
                d = c - ref
                ad = iabs(d)
                if ad < CFG['BRANCH_MIN_D'] or ad > CFG['BRANCH_MAX_D']:
                    continue
                if ad > bd:
                    bd = ad
                    bs = v_sign(d)
            if bd > 0:
                g.br[y] = bs * bd


def count_sep_run(g):
    if g.path_top < 0:
        return 0
    y_lo = max(g.path_top - CFG['ELEM_EXTRA_ROWS'], CFG['TRACK_TOP'])
    run = best = 0
    for y in range(V_H - 3, y_lo - 1, -1):
        if g.sep[y] > 0:
            run += 1
            best = max(best, run)
        else:
            run = 0
    return best


def find_fork(g):
    active = 0
    rows = 0
    first_d = max_d = 0
    best = 0
    best_side = 0
    for y in range(V_H - 3, 1, -1):
        if g.br[y] != 0:
            if not active:
                active = 1
                rows = 0
                first_d = g.br[y]
                max_d = g.br[y]
            rows += 1
            if iabs(g.br[y]) > iabs(max_d):
                max_d = g.br[y]
        elif active:
            if (rows >= CFG['FORK_MIN_ROWS']
                    and iabs(max_d) >= iabs(first_d) + CFG['DIVERGE_PX']
                    and rows > best):
                best = rows
                best_side = v_sign(max_d)
            active = 0
    if (active and rows >= CFG['FORK_MIN_ROWS']
            and iabs(max_d) >= iabs(first_d) + CFG['DIVERGE_PX'] and rows > best):
        best = rows
        best_side = v_sign(max_d)
    return best, best_side


def find_island(g):
    active = 0
    rows = 0
    start_sep = max_sep = tail_sep = 0
    converged = 0
    best = 0
    best_y0 = -1
    side_out = 0

    for y in range(V_H - 3, 1, -1):
        if g.sep[y] > 0:
            if not active:
                active = 1
                rows = 0
                start_sep = g.sep[y]
                max_sep = g.sep[y]
                converged = 0
                best_y0 = y
            rows += 1
            max_sep = max(max_sep, g.sep[y])
            tail_sep = g.sep[y]
            if max_sep >= CFG['ISLE_LAST_SEP'] and tail_sep <= max_sep // 2:
                converged = 1
        elif active:
            if (rows >= CFG['ISLE_MIN_ROWS'] and converged
                    and max_sep >= CFG['ISLE_LAST_SEP']
                    and max_sep >= start_sep + CFG['DIVERGE_PX'] and rows > best):
                best = rows
                best_y0 = y + rows
            active = 0
    if (active and rows >= CFG['ISLE_MIN_ROWS'] and converged
            and max_sep >= CFG['ISLE_LAST_SEP']
            and max_sep >= start_sep + CFG['DIVERGE_PX'] and rows > best):
        best = rows
        best_y0 = y + rows

    if best > 0 and best_y0 >= 0:
        ref = track_ref_x(g, best_y0 + 3)
        for y2 in range(best_y0, max(best_y0 - 9, -1), -1):
            if y2 < 0:
                break
            if g.mid[y2] >= 0:
                side_out = v_sign(g.mid[y2] - ref)
                break
    return best, side_out


def detect_cross(g, runs):
    if g.path_top < 0:
        return 0
    y_lo = max(g.path_top - CFG['ELEM_EXTRA_ROWS'], CFG['TRACK_TOP'])
    rows = 0
    for y in range(y_lo, V_H - 5):
        ref = path_x_at(g, y)
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < CFG['CROSS_W']:
                continue
            if (ref - l) < CFG['CROSS_ARM_MIN'] or (r - ref) < CFG['CROSS_ARM_MIN']:
                continue
            has_above = any(0 <= y - k < V_H and g.mid[y - k] >= 0 for k in range(1, 7))
            has_below = any(0 <= y + k < V_H and g.mid[y + k] >= 0 for k in range(1, 7))
            if has_above and has_below:
                rows += 1
    return rows


def island_enter(g, side):
    g.island_on = 1
    if CFG['ISLAND_SIDE'] != 0:
        g.island_side = CFG['ISLAND_SIDE']
    elif side != 0:
        g.island_side = side
    else:
        g.island_side = 1
    g.island_frames = 0
    g.isle_quiet = 0
    g.island_force = CFG['ISLAND_FORCE']
    g.elem = 3
    g.elem_side = g.island_side
    g.prefer_side = g.island_side


def elem_update(g, runs):
    probe_rows(g, runs)
    fork_rows, fork_side = find_fork(g)
    isle_rows, isle_side = find_island(g)
    sep_run = count_sep_run(g)
    if sep_run > isle_rows:
        isle_rows = sep_run

    g.fork_rows = fork_rows
    g.isle_rows = isle_rows
    g.cross_rows = detect_cross(g, runs)

    if g.c_cd > 0:
        g.c_cd -= 1

    if isle_rows >= CFG['ISLE_MIN_ROWS']:
        g.isle_hit = min(g.isle_hit + 1, 1000)
    else:
        g.isle_hit = 0

    g.elem = 0
    g.elem_side = 0

    # 已在环岛里
    if g.island_on:
        g.elem = 3
        g.elem_side = g.island_side
        g.island_frames += 1
        if g.island_force > 0:
            g.island_force -= 1
            g.prefer_side = g.island_side
        else:
            g.prefer_side = 0

        if isle_rows == 0 and fork_rows <= 1 and g.path_rows >= CFG['ISLAND_EXIT_ROWS']:
            g.isle_quiet += 1
        else:
            g.isle_quiet = 0

        if ((g.isle_quiet >= CFG['ISLAND_QUIET'] and g.island_frames >= CFG['ISLAND_MIN_FRAME'])
                or g.lost_cnt >= 10
                or g.island_frames > CFG['ISLAND_MAX_FRAME']):
            g.island_on = 0
            g.prefer_side = 0
            g.island_side = 0
            g.island_frames = 0
            g.isle_quiet = 0
            g.island_force = 0
            g.c_cd = CFG['ELEM_CD']
        return

    # 环岛入口
    if (isle_rows >= CFG['ISLE_MIN_ROWS'] and g.c_cd == 0
            and (g.isle_hit >= CFG['ISLE_CONFIRM'] or isle_rows >= CFG['ISLE_FAST_ROWS'])):
        island_enter(g, isle_side)
        return

    # 分叉不做环岛入口（实测：直角弯的另一条腿会误报）

    # 十字
    if g.cross_rows >= 2:
        g.elem = 1
        g.elem_side = 0
        g.prefer_side = 0
        return

    g.elem = 0
    g.elem_side = 0
    g.prefer_side = 0


def state_update(g):
    if g.island_on:
        st = 4
    elif g.elem == 1:
        st = 2
    elif g.corner_dir != 0:
        st = 1
    else:
        st = 0

    if st != g.state:
        g.state = st
        g.state_frames = 0
    elif g.state_frames < 100000:
        g.state_frames += 1

    if ((g.state == 1 and g.state_frames > CFG['STATE_MAX_CORNER'])
            or (g.state == 2 and g.state_frames > CFG['STATE_MAX_CROSS'])
            or (g.state == 4 and g.state_frames > CFG['STATE_MAX_ISLAND'])):
        g.state = 0
        g.state_frames = 0
        g.island_on = 0
        g.island_side = 0
        g.prefer_side = 0
        g.c_cd = CFG['ELEM_CD']

    if g.state == 1:
        g.speed_pct = CFG['SPD_CORNER']
    elif g.state == 2:
        g.speed_pct = CFG['SPD_CROSS']
    elif g.state == 4:
        g.speed_pct = CFG['SPD_ISLAND']
    else:
        g.speed_pct = 100


# ============================================================================
# 单帧处理
# ============================================================================
def process_frame(g, frame, bias=0):
    thre = img_threshold(frame, bias)
    runs, row_white = binarize_runs(frame, thre)
    g.row_white = row_white

    ok = track_build(g, runs, prefer_side=g.prefer_side)
    if ok:
        track_steer(g)
        elem_update(g, runs)
        state_update(g)
        g.lost = 0
        g.lost_cnt = 0
    else:
        g.lost = 1
        g.lost_cnt += 1

    return dict(thre=thre, ok=bool(ok), runs=runs, row_white=row_white)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    root = args[0] if args else 'dataset'
    files = sorted(glob.glob(os.path.join(root, '*.png')))
    if not files:
        print("没找到图片:", root)
        return

    g = G()
    print("帧  文件                       行数  阈值 err      look    元素 状态 速度")
    print("-" * 88)
    errs = []
    for idx, f in enumerate(files):
        im = Image.open(f).convert('L')
        w, h = im.size
        if (w, h) != (V_W, V_H):
            print("  跳过尺寸不符:", f, im.size)
            continue
        px = list(im.getdata())
        frame = [px[y * V_W:(y + 1) * V_W] for y in range(V_H)]

        r = process_frame(g, frame)
        elem_name = {0: '-', 1: '十', 3: '岛'}.get(g.elem, '?')
        print("%3d %-26s %4d %4d %7.2f %4d,%3d %s %4d %4d"
              % (idx, os.path.basename(f)[:26], g.path_rows, r['thre'],
                 g.steer_err, g.look_x, g.look_y, elem_name, g.state, g.speed_pct))
        if r['ok']:
            errs.append(g.steer_err)

    print("-" * 88)
    print("可跟踪帧 %d/%d" % (len(errs), len(files)))
    if errs:
        print("err 范围 %.2f ~ %.2f   均值 %.2f"
              % (min(errs), max(errs), sum(errs) / len(errs)))


if __name__ == '__main__':
    main()
