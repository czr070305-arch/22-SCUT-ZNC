# -*- coding: utf-8 -*-
"""
vision_sim.py —— 视觉算法离线仿真器

把 code/vision/ 里的 C 算法逐函数翻译成 Python，在 dataset/*.png 上跑，
输出线拟合质量 / 转向误差 / 元素判定，用来在没有实车的情况下调参。
"""

import glob
import math
import os
import sys

from PIL import Image

V_W, V_H = 188, 120

# ============================================================================
# 参数表 —— 跟 vision_cfg.h 一一对应，改这里就是改固件
# ============================================================================
P = dict(
    # 阈值
    THRE_MARGIN=8,
    THRE_MIN=30,
    THRE_MAX=196,
    THRE_DARK_SKIP=12,
    THRE_DARK_PCT=12,
    THRE_FLOOR_RATIO_NUM=2,      # floor = lo + (hi-lo)*2/10
    THRE_FLOOR_RATIO_DEN=10,
    # 白线段
    RUN_W_MIN=3,
    RUN_W_MAX_NARROW=38,
    RUN_W_MAX_WIDE=150,
    CROSS_W=50,
    CROSS_ARM_MIN=18,
    ROW_WHITE_MAX=130,
    # 爬线
    START_ROWS=20,
    START_TOL=55,
    TOL_BASE=7,
    TOL_GROW=0.75,
    TOL_MAX=26,
    EXTRAP_ROWS=9,
    TRACK_TOP=8,          # 爬线上限行(换镜头: 24->8)
    BLIND_ROWS=24,
    SLOPE_SMOOTH=0.72,
    # 转向
    LOOKAHEAD=26,
    MULTI_START=True,      # C 固件里是"每个候选起点都试"，默认开启
    ERR_OFF_W=1.0,
    ERR_SLOPE_W=2.0,
    CORNER_BIAS=25.0,
    CORNER_MIN_ROWS=12,
    CORNER_W_MAX=110,
    CORNER_NEAR_END=8,
    RUN_EDGE_TOL=6,
    CORNER_LOOK=40,
    CORNER_TAN_MAX=0.20,   # 横臂爬升/跨度比上限, 0.20≈11.3°, 超了算斜臂不是直角弯
    CORNER_MIN_Y=10,
    MIN_ROWS_STEER=10,
    # 元素（只到 fork 为止）
    FORK_REF_TOL=13,
    FORK_MIN_ROWS=12,
    BRANCH_MIN_D=11,
    BRANCH_MAX_D=42,
    DIVERGE_PX=4,
    ISLE_SEP_MIN=8,
    ISLE_SEP_MAX=60,
    ELEM_TOP_Y=0,
    # 学长元素识别
    V2E_RATIO_THRES=150,
    V2E_BLACK_SUM=60,
    V2E_LANE_MIN_W=3,
    V2E_LANE_MAX_W=30,
    V2E_HORI_DX=4,
    V2E_HALF_SEARCH_W=15,
    V2E_CIRCLE_RAYS=5,
    V2E_CIRCLE_DX_START=10,
    V2E_CIRCLE_DX_STEP=5,
    V2E_CIRCLE_SLOPE_MIN=2,
)

# 畸变参数（学长标定，注意 distCoeffs 不是 OpenCV 顺序！）
# 学长 Perspective.c:
#   distCoeffs[5] = {-1.101773, 1.298787, -0.006304, 0.004262, -0.684578}
#   k1 = [0], k2 = [1], p1 = [2], p2 = [3], k3 = [4]
CAM_FX, CAM_FY = 148.404692, 151.874202
CAM_CX, CAM_CY = 96.738556, 50.592531
CAM_MOVE_X, CAM_MOVE_Y = 0.0, -9.0
K1, K2, K3 = -1.101773, 1.298787, -0.684578
P1, P2 = -0.006304, 0.004262

UNDIST_ENABLE = False   # 2026-11-24 换无畸变镜头


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


def undistort_x(xd, yd):
    """去畸变取 x 坐标。发散时退回原始值，绝不让 nan/inf 流进控制环。"""
    uu, vv = xd, yd
    for _ in range(8):
        px, py = distort_pixel(uu, vv)
        du, dv = xd - px, yd - py
        # 迭代跑飞就放弃，用原值
        if not (-1e4 < uu < 1e4 and -1e4 < vv < 1e4):
            return xd
        uu += du
        vv += dv
        if abs(du) < 0.02 and abs(dv) < 0.02:
            break
    if not (-1e4 < uu < 1e4):
        return xd
    return uu


# ============================================================================
# 阈值（大津法）
# ============================================================================
def otsu_threshold(hist, total):
    sum_all = sum(i * h for i, h in enumerate(hist))
    w_bg = 0
    sum_bg = 0
    best_t = 0
    best_var = -1
    for i in range(256):
        w_bg += hist[i]
        if w_bg == 0:
            continue
        w_fg = total - w_bg
        if w_fg == 0:
            break
        sum_bg += i * hist[i]
        m_bg = sum_bg / w_bg
        m_fg = (sum_all - sum_bg) / w_fg
        diff = m_bg - m_fg
        var = w_bg * w_fg * diff * diff
        if var > best_var:
            best_var = var
            best_t = i
    return best_t


def img_threshold(frame, bias=0):
    # 两遍法: 第一遍求全图最大亮度, 按相对比例定黑角裁切线
    hi0 = 0
    for row in frame:
        for v in row:
            if v > hi0:
                hi0 = v
    dark_cut = hi0 * P['THRE_DARK_PCT'] // 100

    hist = [0] * 256
    total = 0
    lo, hi = 255, 0
    for row in frame:
        for v in row:
            if v <= dark_cut:
                continue
            hist[v] += 1
            total += 1
            if v < lo:
                lo = v
            if v > hi:
                hi = v
    if total < 64:
        return P['THRE_MIN']
    thr = otsu_threshold(hist, total) + P['THRE_MARGIN'] + bias
    floor_thre = lo + (hi - lo) * P['THRE_FLOOR_RATIO_NUM'] // P['THRE_FLOOR_RATIO_DEN']
    if thr < floor_thre:
        thr = floor_thre
    return max(P['THRE_MIN'], min(P['THRE_MAX'], thr))


# ============================================================================
# 二值化 + 3x3 去噪 + 行白段
# ============================================================================
def binarize_runs(frame, thre):
    runs = [[] for _ in range(V_H)]     # [(l, r), ...]
    row_white = [0] * V_H

    def b(row):
        return [1 if v > thre else 0 for v in row]

    prev = [0] * V_W
    cur = b(frame[0]) if V_H > 0 else [0] * V_W
    nxt = b(frame[1]) if V_H > 1 else [0] * V_W

    for y in range(V_H):
        out = [0] * V_W
        for x in range(V_W):
            if not cur[x]:
                continue
            up = prev[x]
            dn = nxt[x]
            lf = cur[x - 1] if x > 0 else 0
            rt = cur[x + 1] if x < V_W - 1 else 0
            out[x] = 255 if ((up and dn) or (lf and rt)) else 0

        # 行白段
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
                if w >= P['RUN_W_MIN']:
                    if n > 0 and st - runs[y][n - 1][1] <= 2:
                        runs[y][n - 1] = (runs[y][n - 1][0], end)
                    elif n < 8:
                        runs[y].append((st, end))
                        n += 1
                st = -1
        row_white[y] = sum(r - l + 1 for l, r in runs[y])

        prev = cur
        cur = nxt
        nxt = b(frame[y + 2]) if y + 2 < V_H else [0] * V_W

    return runs, row_white


# ============================================================================
# 爬线
# ============================================================================
class State:
    def __init__(self):
        self.reset()

    def reset(self):
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
        self.prev_bottom = -1
        self.corner_open_deg = 0


def find_run_near(runs, y, pred, tol, narrow):
    best = -1
    best_d = 1e6
    wmax = P['RUN_W_MAX_NARROW'] if narrow else P['RUN_W_MAX_WIDE']
    for i, (l, r) in enumerate(runs[y]):
        w = r - l + 1
        if w < P['RUN_W_MIN'] or w > wmax:
            continue
        c = 0.5 * (l + r)
        d = abs(c - pred)
        if d <= tol and d < best_d:
            best_d = d
            best = i
    return best


def track_build(S, runs, row_white, prefer_side=0, multi_start=False):
    for y in range(V_H):
        S.mid[y] = -1
        S.edge_l[y] = -1
        S.edge_r[y] = -1
    S.path_top = -1
    S.path_top_x = 0
    S.path_slope = 0.0
    S.path_rows = 0
    S.bottom_x = -1
    S.corner_dir = 0
    S.corner_far_x = 0

    # ---- 起点候选 ----
    # 两条信息同时用:
    #   1) 上一帧底点 prev_bottom 作引力中心（连续性，防止左右跳）
    #   2) 如果没有历史，用"最佳竖直候选"作中心（见下）
    ref = S.prev_bottom if S.prev_bottom >= 0 else V_W // 2
    ymin = max(0, V_H - 1 - P['START_ROWS'])

    # 收集底部窗口内所有合法的 (y, idx)
    raw = []
    for y in range(V_H - 1, ymin - 1, -1):
        if row_white[y] > P['ROW_WHITE_MAX']:
            continue
        for i, (l, r) in enumerate(runs[y]):
            w = r - l + 1
            if w < P['RUN_W_MIN'] or w > P['CROSS_W']:
                continue
            raw.append((y, i, 0.5 * (l + r), w))

    if S.prev_bottom < 0 and raw:
        # 无历史: 选"最像赛道线"的那条。
        # 赛道线特征: 窄（w 小）、在画面中下部、上下连续。
        # 用"该 x 位置在底部窗口出现了多少行"当连续度打分。
        def streak(cx):
            n = 0
            for (yy, ii, c, w) in raw:
                if abs(c - cx) <= P['START_TOL'] / 2:
                    n += 1
            return n

        # 只在最靠底部的 5 行里取候选，避免被上方的杂物吸引
        base = [(y, i, c, w) for (y, i, c, w) in raw if y >= V_H - 5]
        if base:
            # 打分: 连续行数多者优先; 平手取窄的
            scored = [(streak(c), -w, c, y, i) for (y, i, c, w) in base]
            scored.sort(reverse=True)
            ref = int(scored[0][2])

    cands = []
    for (y, i, c, w) in raw:
        if abs(c - ref) <= P['START_TOL']:
            cands.append((y, i))

    if not cands:
        return False

    if multi_start:
        # 每个候选起点都试一次，选爬得最长的那条。
        #
        # 平手时选"起点更靠底部"很重要：起点越靠近车，越早看到白线，
        # 也就越不容易被底部那几行杂散（车头反光、机身）带到错误位置。
        # 但起点不能低于实际赛道线的底部——那些行根本没有白线。
        # 所以规则是：先比行数，平手比起点 y。
        best_score = -1
        best_y0 = -1
        best_state = None
        for (y0, i0) in cands:
            T = State()
            T.prev_bottom = S.prev_bottom
            _climb(T, runs, row_white, y0, i0, prefer_side)
            score = T.path_rows
            if score > best_score or (score == best_score and y0 > best_y0):
                best_score = score
                best_y0 = y0
                best_state = T
        S.__dict__.update(best_state.__dict__)
        return S.path_rows > 0
    else:
        y0, i0 = cands[0]
        _climb(S, runs, row_white, y0, i0, prefer_side)
        return True


def _climb(S, runs, row_white, start_y, start_idx, prefer_side):
    """从 (start_y, start_idx) 起点向上爬线，结果写回 S"""
    cur = 0.5 * (runs[start_y][start_idx][0] + runs[start_y][start_idx][1])
    slope = 0.0
    rows_since = 0
    blind_since = 0

    S.bottom_x = int(cur)
    S.mid[start_y] = int(cur)
    S.edge_l[start_y], S.edge_r[start_y] = runs[start_y][start_idx]
    S.path_top = start_y
    S.path_top_x = int(cur)
    S.path_rows = 1
    S.prev_bottom = int(cur)

    corner_hit = False
    _top = P.get('TRACK_TOP', 0)
    for y in range(start_y - 1, _top - 1, -1):
        pred = cur + slope
        tol = min(P['TOL_BASE'] + P['TOL_GROW'] * rows_since, P['TOL_MAX'])

        if corner_hit:
            break

        if row_white[y] > P['ROW_WHITE_MAX']:
            blind_since += 1
            rows_since += 1
            slope *= 0.8
            if blind_since > P['BLIND_ROWS'] or rows_since > P['EXTRAP_ROWS'] + P['BLIND_ROWS']:
                break
            continue
        blind_since = 0

        # ---- 0) 先判拐角：宽带跨越 pred 且上方没延续 ----
        #
        # 必须排在窄段匹配之前。因为直角弯的横线往往同时满足
        # "有窄段落在 pred 附近"（横线两端的小段），先匹配窄段
        # 就会顺着横线爬过去，把它当成赛道线。
        corner_here = False
        for i, (l, r) in enumerate(runs[y]):
            w = r - l + 1
            if w <= P['RUN_W_MAX_NARROW'] or w > P['RUN_W_MAX_WIDE']:
                continue
            if not (l - P['RUN_EDGE_TOL'] <= pred <= r + P['RUN_EDGE_TOL']):
                continue
            above_ok = False
            for k in range(1, 5):
                if y - k < 0:
                    break
                if row_white[y - k] > P['ROW_WHITE_MAX']:
                    continue
                if find_run_near(runs, y - k, pred, tol + 12.0, True) >= 0:
                    above_ok = True
                    break
            if above_ok or S.path_rows < P['CORNER_MIN_ROWS']:
                continue
            dl = abs(l - int(cur))
            dr = abs(r - int(cur))
            one_side = ((dl <= P['CORNER_NEAR_END'] and dr > P['CORNER_NEAR_END']) or
                        (dr <= P['CORNER_NEAR_END'] and dl > P['CORNER_NEAR_END']))
            if w > P['CORNER_W_MAX'] and not one_side:
                continue
            far = l if dl > dr else r

            # 拐角开口角: 竖线方向 vs 横臂方向
            #
            # 直角弯的横臂几乎水平（dy≈0），开口角接近 90°
            # 分叉/环岛的进入角度 120~150°，横臂有明显的纵向爬升
            #
            #   ang = atan2(|dy|, |dx|)  单位: 度
            #     0°  = 完全水平（L 形直角弯）
            #    30°  = 30° 上翘（120° 拐角）
            #    60°  = 60° 上翘（分叉的斜线）
            #
            # 外积 > 0.20 -> 角 > 11.3° -> 不是直角弯
            dx_far = far - int(cur)
            dy_far = y - S.path_top
            if dx_far != 0:
                tan_a = abs(dy_far) / abs(dx_far)
                if tan_a > P['CORNER_TAN_MAX']:
                    # 角度太大，这是分叉/环岛的斜臂，不是直角弯
                    S.corner_open_deg = int(57.29578 * math.atan(tan_a))
                    continue

            if far > cur:
                far = min(far, int(cur) + P['CORNER_LOOK'])
            else:
                far = max(far, int(cur) - P['CORNER_LOOK'])
            if far == int(cur):
                continue                      # 没有方向信息，别乱报
            S.corner_dir = 1 if far > cur else -1
            S.corner_far_x = int(far)
            S.mid[y] = int(far)
            S.edge_l[y], S.edge_r[y] = l, r
            S.path_top = y
            S.path_top_x = int(far)
            S.path_rows += 1
            corner_here = True
            break
        if corner_here:
            break

        idx = find_run_near(runs, y, pred, tol, True)
        if idx >= 0:
            l, r = runs[y][idx]
            c = 0.5 * (l + r)
            step = rows_since + 1
            inst = (c - cur) / step
            slope = P['SLOPE_SMOOTH'] * slope + (1.0 - P['SLOPE_SMOOTH']) * inst
            cur = c
            S.mid[y] = int(c)
            S.edge_l[y], S.edge_r[y] = l, r
            S.path_top = y
            S.path_top_x = int(c)
            S.path_rows += 1
            rows_since = 0
            continue

        # 没找到窄段 -> 试宽段（非拐角的横向结构）
        widx = -1
        for i, (l, r) in enumerate(runs[y]):
            w = r - l + 1
            if w < P['RUN_W_MIN'] or w > P['RUN_W_MAX_WIDE']:
                continue
            if (l - P['RUN_EDGE_TOL']) <= pred <= (r + P['RUN_EDGE_TOL']):
                widx = i
                break

        if widx >= 0:
            l, r = runs[y][widx]
            c = 0.5 * (l + r)
            step = rows_since + 1
            inst = (c - cur) / step
            slope = P['SLOPE_SMOOTH'] * slope + (1.0 - P['SLOPE_SMOOTH']) * inst
            cur = c
            S.mid[y] = int(c)
            S.edge_l[y], S.edge_r[y] = l, r
            S.path_top = y
            S.path_top_x = int(c)
            S.path_rows += 1
            rows_since = 0
            continue

        rows_since += 1
        if rows_since > P['EXTRAP_ROWS']:
            break
        cur += slope

    S.path_slope = slope
    return True


def track_ref_x(S, y):
    if y < 0:
        y = 0
    if y >= V_H:
        y = V_H - 1
    if S.mid[y] >= 0:
        return S.mid[y]
    if S.path_top >= 0 and y < S.path_top:
        x = S.path_top_x + int(S.path_slope * (S.path_top - y))
        return max(2, min(V_W - 3, x))
    for i in range(y + 1, V_H):
        if S.mid[i] >= 0:
            return S.mid[i]
    if S.bottom_x >= 0:
        return S.bottom_x
    return V_W // 2


def track_steer(S):
    y0 = -1
    for i in range(V_H - 1, -1, -1):
        if S.mid[i] >= 0:
            y0 = i
            break
    if y0 < 0 or S.path_rows < P['MIN_ROWS_STEER']:
        S.look_x = -1
        S.look_y = -1
        return False

    look_y = max(0, y0 - P['LOOKAHEAD'])
    look_x = -1
    for i in range(look_y, y0 + 1):
        if S.mid[i] >= 0:
            look_x = S.mid[i]
            look_y = i
            break
    if look_x < 0:
        look_x = S.mid[y0]
        look_y = y0

    if S.corner_dir != 0:
        look_x = S.corner_far_x
        look_y = S.path_top

    if UNDIST_ENABLE:
        dx = undistort_x(float(look_x), float(look_y)) - undistort_x(float(V_W // 2), float(look_y))
    else:
        dx = float(look_x - V_W // 2)
    err = dx * P['ERR_OFF_W']

    yn = max(0, y0 - 14)
    yn_used = -1
    for i in range(yn, y0):
        if S.mid[i] >= 0:
            yn_used = i
            break
    if yn_used >= 0 and y0 != yn_used:
        near_slope = float(S.mid[yn_used] - S.mid[y0]) / float(y0 - yn_used)
        err += near_slope * P['ERR_SLOPE_W']

    if S.corner_dir != 0:
        err += S.corner_dir * P['CORNER_BIAS']

    err = max(-110.0, min(110.0, err))
    S.steer_err = err
    S.look_x = look_x
    S.look_y = look_y
    return True


# ============================================================================
# 跑一张图
# ============================================================================
def run_one(path, bias=0, verbose=False):
    im = Image.open(path).convert('L')
    w, h = im.size
    assert (w, h) == (V_W, V_H), 'size %s != %dx%d' % ((w, h), V_W, V_H)
    px = list(im.getdata())
    frame = [px[y * V_W:(y + 1) * V_W] for y in range(V_H)]

    thre = img_threshold(frame, bias)
    runs, row_white = binarize_runs(frame, thre)

    S = State()
    ok = track_build(S, runs, row_white, multi_start=P.get('MULTI_START', True))
    if ok:
        track_steer(S)

    # 学长元素识别（跑在原始灰度图上，不用二值图）
    corner_v2 = 0
    island_v2 = 0
    corner_post = 0
    if ok:
        corner_post = post_detect_corner(S, runs)
        corner_v2 = v2e_detect_corner(frame, S)
        island_v2 = v2e_detect_island(frame, S)

    return dict(
        path=path,
        name=os.path.basename(path),
        thre=thre,
        ok=ok,
        rows=S.path_rows,
        top=S.path_top,
        err=S.steer_err,
        look=(S.look_x, S.look_y),
        corner=S.corner_dir,
        corner_v2=corner_v2,
        corner_post=corner_post,
        island_v2=island_v2,
        mid=S.mid,
        edge_l=S.edge_l,
        edge_r=S.edge_r,
        runs=runs,
        row_white=row_white,
        bottom=S.bottom_x,
        frame=frame,
    )



# ============================================================================
# 学长元素识别 v2e（归一化梯度 + 连续性跟踪 + 射线投票）
# ============================================================================
V2_NONE, V2_LEFT, V2_RIGHT, V2_TOP, V2_BOTTOM, V2_BORDER, V2_SEARCHED = 0, 1, 2, 3, 4, 5, 6


def _in_range(x, y):
    return (y >= 2) and (y <= V_H - 4) and (x >= 2) and (x <= V_W - 3)


def v2e_edge_y(frame, x, y):
    if not _in_range(x, y):
        return V2_BORDER
    up_px = frame[y - 2][x]
    dn_px = frame[y + 2][x]
    if up_px + dn_px < P['V2E_BLACK_SUM']:
        return V2_SEARCHED
    denom = up_px + dn_px
    gy = ((dn_px - up_px) << 10) // denom
    if gy > P['V2E_RATIO_THRES']:
        return V2_TOP
    if gy < -P['V2E_RATIO_THRES']:
        return V2_BOTTOM
    return V2_SEARCHED


def v2e_hscan(frame, S, dir_, multi_start=True):
    """返回 (valid, trend)。dir_=-1 向左，+1 向右。"""
    if S.path_rows < 3 or S.path_top < 0:
        return 0, 0
    y0 = S.path_top
    x0 = S.path_top_x
    if x0 < 0:
        return 0, 0

    HALF = P['V2E_HALF_SEARCH_W']
    lo = max(2, y0 - HALF)
    hi = min(V_H - 4, y0 + HALF)
    prev_t = prev_b = -1
    seen = False
    for yy in range(lo, hi):
        v = v2e_edge_y(frame, x0, yy)
        if v in (V2_BORDER, V2_SEARCHED):
            continue
        if not seen and v == V2_TOP:
            prev_t = yy
            seen = True
        elif seen and v == V2_BOTTOM:
            prev_b = yy
            if P['V2E_LANE_MIN_W'] <= (prev_b - prev_t) <= P['V2E_LANE_MAX_W']:
                break
            prev_t = -1
            seen = False
            prev_b = -1
    if prev_t < 0 or prev_b < 0:
        return 0, 0

    first_y = (prev_t + prev_b) >> 1
    last_y = first_y
    valid = 1
    DX = P['V2E_HORI_DX']

    for k in range(1, (V_W // 2) // DX + 1):
        x = x0 + dir_ * DX * k
        if x < 3 or x > V_W - 4:
            break
        span = HALF + k
        t = b = -1
        t_err = b_err = 0x7FFFFFFF

        lo = max(2, prev_t - span)
        hi = min(V_H - 4, prev_t + span)
        for yy in range(lo, hi):
            if v2e_edge_y(frame, x, yy) == V2_TOP:
                ad = abs(yy - prev_t)
                if ad < t_err:
                    t_err, t = ad, yy

        lo = max(2, prev_b - span)
        hi = min(V_H - 4, prev_b + span)
        for yy in range(lo, hi):
            if v2e_edge_y(frame, x, yy) == V2_BOTTOM:
                ad = abs(yy - prev_b)
                if ad < b_err:
                    b_err, b = ad, yy

        if t < 0 or b < 0:
            if valid > 0:
                break
            continue
        prev_t, prev_b = t, b
        last_y = (t + b) >> 1
        valid += 1

    trend = 0
    if valid >= 3 and first_y >= 0 and last_y >= 0:
        d = last_y - first_y
        if abs(d) >= valid:
            trend = 1 if d > 0 else -1
    return valid, trend


def v2e_detect_corner(frame, S):
    n_l, trend_l = v2e_hscan(frame, S, -1)
    n_r, trend_r = v2e_hscan(frame, S, +1)
    if trend_l != 0 and trend_r != 0:
        return 0
    if trend_l != 0 and n_l >= 4:
        return -1 if trend_l < 0 else 0
    if trend_r != 0 and n_r >= 4:
        return 1 if trend_r < 0 else 0
    return 0


def v2e_ray_y(frame, x, start_y):
    if x < 3 or x > V_W - 4:
        return -1
    if start_y > V_H - 4:
        start_y = V_H - 4
    in_white = False
    top_y = -1
    for y in range(start_y, 3, -1):
        v = v2e_edge_y(frame, x, y)
        if v == V2_BORDER:
            return -1
        if v == V2_SEARCHED:
            continue
        if not in_white:
            if v == V2_TOP:
                in_white = True
                top_y = y
            elif v == V2_BOTTOM:
                return -1
        else:
            if v == V2_BOTTOM:
                w = top_y - y
                if w < P['V2E_LANE_MIN_W'] or w > P['V2E_LANE_MAX_W']:
                    in_white = False
                    top_y = -1
                else:
                    return top_y
    return -1


def v2e_check_trend(grp):
    vals = [g for g in grp if g >= 0]
    if len(vals) < 3:
        return 0
    span = vals[-1] - vals[0]
    n = len(vals) - 1
    if span > 0 and span >= P['V2E_CIRCLE_SLOPE_MIN'] * n and span >= 3:
        return 1
    if span < 0 and -span >= P['V2E_CIRCLE_SLOPE_MIN'] * n and -span >= 3:
        return -1
    return 0


def v2e_detect_island(frame, S):
    if S.path_rows < 5 or S.path_top < 0:
        return 0
    y0 = min(V_H - 4, S.path_top + 10)
    x0 = S.path_top_x
    if x0 < 0:
        return 0

    N = P['V2E_CIRCLE_RAYS']
    lgrp, rgrp = [], []
    for i in range(N):
        off = P['V2E_CIRCLE_DX_START'] + P['V2E_CIRCLE_DX_STEP'] * i
        lgrp.append(v2e_ray_y(frame, x0 - off, y0))
        rgrp.append(v2e_ray_y(frame, x0 + off, y0))

    l_valid = sum(1 for v in lgrp if v > 0)
    r_valid = sum(1 for v in rgrp if v > 0)

    if l_valid > 2 and r_valid > 2:
        return 0

    l_tr = v2e_check_trend(lgrp)
    r_tr = v2e_check_trend(rgrp)
    if l_valid > 1 and l_tr != 0:
        return -1
    if r_valid > 1 and r_tr != 0:
        return 1
    return 0



# ============================================================================
# 直角弯后处理（宽度剖面法）—— 与 C 的 post_detect_corner() 等价
# ============================================================================
def post_detect_corner(S, runs):
    """返回 -1 左直角 / +1 右直角 / 0 无"""
    TOP_CUT = P.get('TOP_CUT', 22)
    if S.path_rows < P['CORNER_MIN_ROWS']:
        return 0

    prof = []
    for y in range(TOP_CUT, V_H):
        m = S.mid[y]
        if m < 0:
            continue
        for (l, r) in runs[y]:
            if l <= m <= r:
                prof.append((y, r - l + 1, l, r))
                break
    if len(prof) < 20:
        return 0

    prof.sort()
    widest = max(prof, key=lambda t: t[1])
    if widest[1] < 20:
        return 0

    n = len(prof)
    low = sorted(w for (_, w, _, _) in prof[n * 4 // 10:])
    med = low[len(low) // 2] if low else 2
    if med < 2:
        med = 2

    if widest[1] * 10 // med < P.get('CORNER_RATIO_X10', 25):
        return 0

    wy, ww, wl, wr = widest
    ref = -1
    for (y, w, l, r) in prof:
        if y > wy + 3:
            ref = (l + r) // 2
            break
    if ref < 0:
        ref = V_W // 2

    dl = abs(wl - ref)
    dr = abs(wr - ref)
    far = wl if dl > dr else wr
    if far == ref:
        return 0
    return 1 if far > ref else -1



# ============================================================================
# 十字 / 岔路（与 C 的 vision_element.c 对应）
# ============================================================================
def detect_cross(S, runs, row_white):
    """横向长白贯穿中线 -> 十字。返回满足条件的行数。"""
    rows = 0
    for y in range(P['ELEM_TOP_Y'], V_H - 5):
        ref = track_ref_x(S, y)
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < P['CROSS_W']:
                continue
            if (ref - l) < P['CROSS_ARM_MIN'] or (r - ref) < P['CROSS_ARM_MIN']:
                continue
            above = any(S.mid[y - k] >= 0 for k in range(1, 7) if y - k >= 0)
            below = any(S.mid[y + k] >= 0 for k in range(1, 7) if y + k < V_H)
            if above and below:
                rows += 1
    return rows


def find_fork(S, runs):
    """支线越离越远 -> 岔路。返回 (行数, 方向)。"""
    br = [0] * V_H
    for y in range(V_H - 3, P['ELEM_TOP_Y'] - 1, -1):
        ref = track_ref_x(S, y)
        near_d = 999
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < P['RUN_W_MIN'] or w > P['RUN_W_MAX_NARROW']:
                continue
            c = 0.5 * (l + r)
            near_d = min(near_d, abs(c - ref))
        if near_d > P['FORK_REF_TOL']:
            continue
        best = 0
        bside = 0
        for (l, r) in runs[y]:
            w = r - l + 1
            if w < P['RUN_W_MIN'] or w > P['RUN_W_MAX_NARROW']:
                continue
            c = 0.5 * (l + r)
            d = c - ref
            if P['BRANCH_MIN_D'] <= abs(d) <= P['BRANCH_MAX_D'] and abs(d) > best:
                best = abs(d)
                bside = 1 if d > 0 else -1
        br[y] = bside * best

    active = 0
    rows = 0
    first_d = 0
    max_d = 0
    best_rows = 0
    best_side = 0
    for y in range(V_H - 3, P['ELEM_TOP_Y'] - 1, -1):
        if br[y] != 0:
            if not active:
                active = 1
                rows = 0
                first_d = br[y]
                max_d = br[y]
            rows += 1
            if abs(br[y]) > abs(max_d):
                max_d = br[y]
        elif active:
            if rows >= P['FORK_MIN_ROWS'] and abs(max_d) >= abs(first_d) + P['DIVERGE_PX'] and rows > best_rows:
                best_rows = rows
                best_side = 1 if max_d > 0 else -1
            active = 0
    if active and rows >= P['FORK_MIN_ROWS'] and abs(max_d) >= abs(first_d) + P['DIVERGE_PX'] and rows > best_rows:
        best_rows = rows
        best_side = 1 if max_d > 0 else -1
    return best_rows, best_side


def main():
    files = sorted(glob.glob('dataset/*.png'))
    if len(sys.argv) > 1:
        files = [f for f in files if any(a in f for a in sys.argv[1:])]

    print('%-22s %5s %5s %5s %7s %5s %5s %5s' % (
        'file', 'thre', 'rows', 'top', 'err', 'cor', 'v2cor', 'v2isl'))
    print('-' * 74)
    stats = []
    for f in files:
        r = run_one(f)
        print('%-22s %5d %5d %5d %7.1f %5d %5d %5d' % (
            r['name'][-20:], r['thre'], r['rows'], r['top'], r['err'],
            r['corner'], r['corner_v2'], r['island_v2']))
        stats.append(r)

    print('-' * 70)
    rows = [r['rows'] for r in stats]
    errs = [abs(r['err']) for r in stats if r['ok']]
    print('平均行数 %.1f (最少 %d) / 成功 %d/%d' % (
        sum(rows) / len(rows), min(rows), sum(1 for r in stats if r['ok']), len(stats)))
    if errs:
        print('平均|err| %.1f, 最大 %.1f' % (sum(errs) / len(errs), max(errs)))
    print('阈值范围 %d ~ %d' % (min(r['thre'] for r in stats), max(r['thre'] for r in stats)))


if __name__ == '__main__':
    main()
