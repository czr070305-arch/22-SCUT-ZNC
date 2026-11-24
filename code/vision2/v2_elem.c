#include "v2.h"

/*
 * v2_elem.c —— 直角弯(横向扫描) + 环岛(射线投票)
 *
 * 这是本模块相对原 vision/ 最关键的改进。
 *
 * ---------------------------------------------------------------------------
 * 一、直角弯: 为什么原方案识别不了
 *
 *   原 vision/ 用"某一行出现宽白段, 且只朝一侧延伸"来判折角。
 *   问题: 判据依赖单个宽段的宽度阈值(CFG_RUN_W_MAX_NARROW=38)。
 *        拐点处横向条很宽(超过 38)就被排除, 判据直接失效;
 *        宽度刚好卡在临界值时又会时有时无, 表现为字母乱跳。
 *
 *   学长方案: 从中心点往左右逐列横向扫描, 每列找白线的"上下边沿",
 *            算这一列的白色区间中心 y。
 *            如果 y 随 x 单调变化, 就是一条斜向的白线 —— 折角。
 *
 *   本质区别: 前者看"一段白的形状", 后者看"一条轨迹的走向"。
 *            轨迹法不受拐点粗细影响。
 *
 * ---------------------------------------------------------------------------
 * 二、环岛: 为什么原方案会误判
 *
 *   原 vision/ 用"间距涨上去又落回来"(梭形)判环岛。
 *   问题: 必须看到椭圆的顶部合拢点, 而那个位置在画面最上方几行。
 *        把 CFG_ELEM_TOP_Y 设成 18 之后, 环岛就再也识别不出来了。
 *
 *   学长方案: 以跟踪点为原点, 左右各打 5 条竖射线,
 *            看某一侧是否连续多条射线都能找到白线, 且 y 单调变化。
 *            不需要看到顶部, 只看"旁边有没有弧线经过"。
 */

/* ==========================================================================
 * 一、横向扫描: 直角弯
 * ========================================================================== */

/*
 * 从爬到的最顶行出发, 往 dir 方向逐列扫描, 每列找白线的上下边沿。
 *
 * 返回: 有效列数。趋势写进 *out_trend (-1 递减 / +1 递增 / 0 无)
 *
 * 判据: 各列的白色中心 y 应该单调变化。
 *   往左扫时 y 递减 = 白线往左上走 = 左折角
 */
static int v2_hscan(int dir, int *out_trend)
{
    int x, k;
    int prev_t = -1, prev_b = -1;
    int valid = 0;
    int first_y = -1, last_y = -1;
    int trend;
    int y0, x0;

    *out_trend = 0;

    if (v2_center_cnt < 3) { return 0; }

    y0 = v2_center_end_row;
    x0 = v2_center_x[y0];
    if (x0 < 0) { return 0; }

    for (k = 1; k <= (V2_W / 2) / V2_HORI_DX; k++) {
        int t = -1, b = -1;
        int span, lo, hi, yy, seen;

        x = x0 + dir * V2_HORI_DX * k;
        if ((x < 3) || (x > V2_W - 4)) { break; }

        /* 越往两边, 白线越可能偏离 y0, 搜索范围随之放宽 */
        span = V2_HALF_SEARCH_W + k * 2;
        lo = v2_imax(y0 - span, 2);
        hi = v2_imin(y0 + span, V2_H - 4);
        seen = 0;

        for (yy = lo; yy < hi; yy++) {
            uint8_t v = v2_edge_y(x, yy);

            if ((v == (uint8_t)V2_BORDER) || (v == (uint8_t)V2_SEARCHED)) {
                continue;
            }
            if (!seen && (v == (uint8_t)V2_TOP)) {
                t = yy;
                seen = 1;
            } else if (seen && (v == (uint8_t)V2_BOTTOM)) {
                b = yy;
                if (((b - t) >= V2_LANE_MIN_W) && ((b - t) <= V2_LANE_MAX_W)) {
                    break;          /* 找到合理的白线区间 */
                }
                t = -1;
                seen = 0;
                b = -1;
            }
        }

        if ((t < 0) || (b < 0)) {
            /* 这一列没找到: 允许少量缺失, 但连续缺就停 */
            if (valid > 0) { break; }
            continue;
        }

        /* 与前一条射线比, 跳太远说明不是同一条线 */
        if (prev_t >= 0) {
            int dmid = ((t + b) >> 1) - ((prev_t + prev_b) >> 1);
            if (v2_iabs(dmid) > V2_HALF_SEARCH_W * 2) { break; }
        }

        prev_t = t;
        prev_b = b;
        if (first_y < 0) { first_y = (t + b) >> 1; }
        last_y = (t + b) >> 1;
        valid++;
    }

    if ((valid >= 3) && (first_y >= 0) && (last_y >= 0)) {
        trend = last_y - first_y;
        if (v2_iabs(trend) >= valid) {
            *out_trend = (trend > 0) ? 1 : -1;
        }
    }

    return valid;
}

/*
 * 判定折角
 *
 * 往左扫和往右扫各试一次, 谁的"趋势"更明确谁就是折角。
 * 两边都有趋势时(十字/岔路)不判折角, 交给元素识别。
 */
void v2_detect_corner(void)
{
    int trend_l = 0, trend_r = 0;
    int n_l, n_r;

    v2_corner_dir = 0;

    n_l = v2_hscan(-1, &trend_l);
    n_r = v2_hscan( 1, &trend_r);

    /* 两边都有明显趋势 -> 十字或岔路, 不当折角 */
    if ((trend_l != 0) && (trend_r != 0)) { return; }

    if ((trend_l != 0) && (n_l >= 4)) {
        /* 往左扫 y 递减 = 白线往左上走 = 左折角 */
        if (trend_l < 0) { v2_corner_dir = -1; }
    } else if ((trend_r != 0) && (n_r >= 4)) {
        if (trend_r < 0) { v2_corner_dir = 1; }
    }
}

/* ==========================================================================
 * 二、环岛: 射线投票
 * ========================================================================== */

/*
 * 从 (x, start_y) 往下打一条竖射线, 看能否穿过一条白线。
 *
 * 返回: 白线上边沿的 y; 找不到返回 -1
 */
static int v2_ray_y(int x, int start_y)
{
    int y;
    int in_white = 0;
    int top_y = -1;

    if ((x < 3) || (x > V2_W - 4)) { return -1; }

    for (y = start_y; y > 3; y--) {
        uint8_t v = v2_edge_y(x, y);

        if (v == (uint8_t)V2_BORDER) { return -1; }
        if (v == (uint8_t)V2_SEARCHED) { continue; }

        if (!in_white) {
            if (v == (uint8_t)V2_TOP) {
                in_white = 1;
                top_y = y;
            } else if (v == (uint8_t)V2_BOTTOM) {
                /* 一上来就是下边沿, 说明起点已在白线里, 不算 */
                return -1;
            }
        } else {
            if (v == (uint8_t)V2_BOTTOM) {
                int w = top_y - y;
                if (w < V2_LANE_MIN_W) {
                    in_white = 0;       /* 太窄, 噪声 */
                    top_y = -1;
                } else if (w > V2_LANE_MAX_W) {
                    in_white = 0;       /* 太宽, 不是线 */
                    top_y = -1;
                } else {
                    return top_y;       /* 找到一条完整的白线 */
                }
            }
        }
    }
    return -1;
}

/*
 * 检查一组射线找到的 y 值是否有单调趋势。
 *   返回 +1 递增 / -1 递减 / 0 无趋势
 */
static int v2_check_trend(const int *grp, int n)
{
    int i;
    int first = -1, last = -1;
    int valid = 0;
    int span;

    for (i = 0; i < n; i++) {
        if (grp[i] < 0) { continue; }
        if (first < 0) { first = grp[i]; }
        last = grp[i];
        valid++;
    }

    if (valid < 3) { return 0; }

    span = last - first;

    if ((span > 0) && (span >= V2_CIRCLE_SLOPE_MIN * (valid - 1)) && (span >= 3)) {
        return 1;
    }
    if ((span < 0) && (-span >= V2_CIRCLE_SLOPE_MIN * (valid - 1)) && (-span >= 3)) {
        return -1;
    }
    return 0;
}

/*
 * 环岛检测 (单帧)
 *   返回: -1 左环岛 / +1 右环岛 / 0 无
 */
int v2_detect_island(void)
{
    int lgrp[V2_CIRCLE_RAYS];
    int rgrp[V2_CIRCLE_RAYS];
    int l_valid = 0, r_valid = 0;
    int l_trend, r_trend;
    int i;
    int x0, y0;

    if (v2_center_cnt < 5) { return 0; }

    y0 = v2_center_end_row + 10;        /* 从中心稍下方开始往下打 */
    if (y0 > V2_H - 4) { y0 = V2_H - 4; }

    x0 = v2_center_x[v2_center_end_row];
    if (x0 < 0) { return 0; }

    for (i = 0; i < V2_CIRCLE_RAYS; i++) {
        int off = V2_CIRCLE_DX_START + V2_CIRCLE_DX_STEP * i;

        lgrp[i] = v2_ray_y(x0 - off, y0);
        rgrp[i] = v2_ray_y(x0 + off, y0);

        if (lgrp[i] > 0) { l_valid++; }
        if (rgrp[i] > 0) { r_valid++; }
    }

    /* 两侧都多 -> 十字横臂, 不是环岛 */
    if ((l_valid > 2) && (r_valid > 2)) { return 0; }

    l_trend = v2_check_trend(lgrp, V2_CIRCLE_RAYS);
    r_trend = v2_check_trend(rgrp, V2_CIRCLE_RAYS);

    if ((l_valid > 1) && (l_trend != 0)) { return -1; }
    if ((r_valid > 1) && (r_trend != 0)) { return 1; }

    return 0;
}
