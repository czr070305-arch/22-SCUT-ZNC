#include "v2.h"

/*
 * v2_track.c —— 纵向爬线 + 横向扫描 (学长算法)
 *
 * 与 vision/track.c 的根本差异:
 *
 *   vision/track.c:  每行找"白线段的中心", 逐行追踪一个 x
 *                    缺点是遇到拐点/横条, "段"的形态一变就断
 *
 *   v2_track.c  :    每行找"左边缘和右边缘", 配对成一条有宽度的线
 *                    中线 = (l + r) / 2, 宽度 = r - l
 *                    宽度异常就能识别出拐点/横条/干扰
 *
 * 宽度这个信息是关键 —— 原算法丢掉了它, 所以分不清
 * "拐弯的白线" 和 "盖住路的白块"。
 */

/* ---------- 中间结果 (v2.h 里声明, 调试可视化用) ---------- */
int v2_l_edge_x[V2_H];
int v2_r_edge_x[V2_H];
int v2_center_x[V2_H];
int v2_center_end_row;
int v2_center_cnt;
int v2_corner_dir;
int v2_island_side;

/* 内部: 上一帧的底部起点, 跨帧先验 */
static int s_last_start_x = V2_W / 2;

/* 爬到的顶部斜率, 供元素判定外推用 */
static float s_slope = 0.0f;

/* ==========================================================================
 * 工具
 * ========================================================================== */

int v2_iabs(int v)
{
    return (v < 0) ? -v : v;
}

int v2_imax(int a, int b) { return (a > b) ? a : b; }
int v2_imin(int a, int b) { return (a < b) ? a : b; }

/* ==========================================================================
 * 横向: 在某一行里找白线的左右边沿
 *
 * 以 (prev_l, prev_r) 为参考, 在各自 ±half_w 范围内找最近的边沿点。
 *
 * 为什么用"最近"而不是"全行扫描":
 *   一行里可能有别的白线(岔路支线、赛道外的白色), 全行扫描会抓错。
 *   用上一行的边沿做先验, 只会跟住"自己这条线"。
 * ========================================================================== */
void v2_scan_row(int y, int *lx, int *rx, int prev_l, int prev_r, int last_center, int half_w)
{
    int lo, hi, i;
    int best_l = *lx;
    int best_r = *rx;
    int l_err = 0x7FFFFFFF;
    int r_err = 0x7FFFFFFF;
    uint8_t val;

    if ((prev_l < 0) && (prev_r < 0)) {
        return;             /* 没有先验, 放弃这一行 */
    }

    /* 一边丢失时, 用另一边外推 */
    if (prev_l < 0) { prev_l = prev_r - 8; }
    if (prev_r < 0) { prev_r = prev_l + 8; }

    lo = v2_imax(prev_l - half_w, 2);
    hi = v2_imin(prev_r + half_w, V2_W - 3);

    for (i = lo; i < hi; i++) {
        int d;

        val = v2_edge_x(i, y);
        if (val == (uint8_t)V2_LEFT) {
            d = v2_iabs(i - last_center);
            if (d < l_err) { l_err = d; best_l = i; }
        } else if (val == (uint8_t)V2_RIGHT) {
            d = v2_iabs(i - last_center);
            if (d < r_err) { r_err = d; best_r = i; }
        }
    }

    *lx = best_l;
    *rx = best_r;
}

/* ==========================================================================
 * 纵向: 从底部往上逐行爬
 *
 * 返回爬到的最顶行; 中心线写进 v2_center_x[]
 * ========================================================================== */
int v2_climb(int start_y)
{
    int y, i;
    int lost = 0;           /* 连续丢行数, 用于外推 */
    int stop = 0;           /* 连续失败数, 用于终止 */
    int last_dx = 0;
    int cnt = 0;

    v2_center_end_row = start_y;
    s_slope = 0.0f;

    if ((v2_l_edge_x[start_y] < 0) || (v2_r_edge_x[start_y] < 0)) {
        return start_y;
    }
    v2_center_x[start_y] = (v2_l_edge_x[start_y] + v2_r_edge_x[start_y]) >> 1;
    cnt = 1;

    for (y = start_y - V2_VERT_DY; y > 2; y -= V2_VERT_DY) {
        int ref = y + V2_VERT_DY * (lost + 1);
        int losc_l = v2_l_edge_x[ref];
        int rsc_r  = v2_r_edge_x[ref];
        int rsc_c  = v2_center_x[ref];
        int l = -1, r = -1;
        int width, mid;

        if (ref >= V2_H) { ref = V2_H - 1; losc_l = v2_l_edge_x[ref]; rsc_r = v2_r_edge_x[ref]; rsc_c = v2_center_x[ref]; }

        v2_scan_row(y, &l, &r, losc_l, rsc_r, rsc_c, V2_HALF_SEARCH_W);

        if ((l < 0) || (r < 0)) {
            /* 两个边沿没找齐: 沿用上一行, 计入丢失 */
            v2_l_edge_x[y] = losc_l;
            v2_r_edge_x[y] = rsc_r;
            lost++;
            stop++;
            if (lost > V2_LOST_MAX_Y) { break; }
            continue;
        }

        width = r - l;

        /* 宽度异常: 太窄是噪声, 太宽是压到了别的东西(横条/白块) */
        if ((width < V2_LANE_MIN_W) || (width > V2_LANE_MAX_W)) {
            v2_l_edge_x[y] = losc_l;
            v2_r_edge_x[y] = rsc_r;
            lost++;
            stop++;
            if (stop > V2_LOST_MAX_Y) { break; }
            continue;
        }

        /* 正常: 记录 */
        mid = (l + r) >> 1;
        v2_l_edge_x[y] = l;
        v2_r_edge_x[y] = r;
        v2_center_x[y] = mid;
        v2_center_end_row = y;
        cnt++;

        last_dx = mid - rsc_c;
        lost = 0;
        if (stop > 0) { stop--; }
    }

    /* 用顶部若干行的位移估斜率(每行 x 变化量) */
    if (cnt >= 3) {
        int top = v2_center_end_row;
        int bot = start_y;
        int span = bot - top;

        if (span > 0) {
            s_slope = (float)(v2_center_x[top] - v2_center_x[bot]) / (float)span;
        }
    }

    (void)i;
    (void)last_dx;
    return v2_center_end_row;
}

/* ==========================================================================
 * 找起点
 *
 * 从画面底部往上找几行, 在上一帧起点的邻域里找到一条完整的白线。
 * 用上一次的起点做先验, 避免抓到旁边的白线(岔路/赛道外)。
 * ========================================================================== */
int v2_find_start(int *out_y)
{
    int y;
    int lo = v2_imax(s_last_start_x - V2_START_TOL, 20);
    int hi = v2_imin(s_last_start_x + V2_START_TOL, V2_W - 20);

    for (y = V2_H - 4; y > V2_H - 4 - V2_START_SEARCH_ROWS; y--) {
        int i;
        int in_white = 0;
        int l = -1, r = -1;
        int best_l = -1, best_r = -1;
        int best_d = 0x7FFFFFFF;

        if (y < 3) { break; }

        for (i = lo; i < hi; i++) {
            uint8_t v = v2_edge_x(i, y);

            if ((v == (uint8_t)V2_BORDER) || (v == (uint8_t)V2_SEARCHED)) {
                if (in_white && (l >= 0) && (r < 0)) {
                    /* 白线在这一行结束, 检查宽度 */
                    int w = i - l;
                    if ((w >= V2_LANE_MIN_W) && (w <= V2_LANE_MAX_W)) {
                        int mid = (l + i) >> 1;
                        int d = v2_iabs(mid - s_last_start_x);
                        if (d < best_d) {
                            best_d = d;
                            best_l = l;
                            best_r = i;
                        }
                    }
                    in_white = 0;
                    l = -1;
                }
                continue;
            }

            if (!in_white && (v == (uint8_t)V2_LEFT)) {
                l = i;
                in_white = 1;
            } else if (in_white && (v == (uint8_t)V2_RIGHT)) {
                r = i;
                /* 一对边沿找齐, 宽度检查在外面做 */
                {
                    int w = r - l;
                    if ((w >= V2_LANE_MIN_W) && (w <= V2_LANE_MAX_W)) {
                        int mid = (l + r) >> 1;
                        int d = v2_iabs(mid - s_last_start_x);
                        if (d < best_d) {
                            best_d = d;
                            best_l = l;
                            best_r = r;
                        }
                    }
                }
                in_white = 0;
                l = -1;
                r = -1;
            } else if (in_white && (l >= 0) && (i - l > V2_LANE_MAX_W)) {
                /* 太宽了, 不是线, 重来 */
                in_white = 0;
                l = -1;
            }
        }

        if (best_l >= 0) {
            v2_l_edge_x[y] = best_l;
            v2_r_edge_x[y] = best_r;
            v2_center_x[y] = (best_l + best_r) >> 1;
            s_last_start_x = v2_center_x[y];
            *out_y = y;
            return 1;
        }
    }
    return 0;
}
