#include "vision.h"
#include "v2_elem.h"

/*
 * v2_elem.c —— 学长的元素识别算法 (直角弯 + 环岛)
 *
 * ==========================================================================
 * 移植说明
 * ==========================================================================
 *
 * 本文件只做元素识别, 不做二值化、不做爬线。
 * 它的输入是:
 *   1) mt9v03x_image     摄像头原始灰度 (现场算归一化梯度, 不依赖二值图)
 *   2) g_mid_line[]      第一版 vision_track.c 爬出来的中线
 *   3) g_edge_l/r[]      第一版爬出来的左右边沿
 *   4) g_path_top/rows   第一版爬到的顶部行和行数
 *
 * 输出:
 *   g_corner_dir         直角弯方向 (-1 左 / +1 右 / 0 无)
 *   v2_island_detect()   环岛方向 (-1 左 / +1 右 / 0 无)
 *
 * 为什么不依赖二值图:
 *   归一化梯度 = (右 - 左) * 1024 / (右 + 左), 是比值。
 *   曝光变化、蓝布反光、鱼眼黑角都不会让这个比值漂移。
 *   所以这套判据比"大津阈值 + 切段"稳得多。
 *
 * ==========================================================================
 * 一、直角弯: 为什么第一版的判据不可靠
 * ==========================================================================
 *
 *   第一版用"某一行出现宽白段, 且只朝一侧延伸"判折角。
 *   问题: 依赖单个宽段的宽度阈值 (CFG_RUN_W_MAX_NARROW = 38)。
 *        拐点处横向条很宽(超过 38)就被排除, 判据直接失效;
 *        宽度刚好卡在临界值时又时有时无, 表现为字母乱跳。
 *
 *   学长方案: 从爬到的最顶行出发, 往左右逐列横向扫描,
 *            每列找白线的"上下边沿", 算这一列白色区间的中心 y。
 *            如果中心 y 随 x 单调变化, 就是一条斜向的白线 = 折角。
 *
 *   本质区别: 前者看"一段白的形状", 后者看"一条轨迹的走向"。
 *            轨迹法不受拐点粗细影响。
 *
 * ==========================================================================
 * 二、环岛: 为什么第一版会误判
 * ==========================================================================
 *
 *   第一版用"两条线的间距涨上去又落回来"(梭形)判环岛。
 *   问题: 必须看到椭圆顶部合拢点, 而那个位置在画面最上方几行。
 *        CFG_ELEM_TOP_Y 一限制, 环岛就再也识别不出来。
 *
 *   学长方案: 以跟踪点为原点, 左右各打 5 条竖射线,
 *            看某一侧是否连续多条射线都能找到白线, 且 y 单调变化。
 *            不需要看到顶部, 只看"旁边有没有弧线经过"。
 */

/* ==========================================================================
 * 边缘判定 (现场从原始灰度算, 不建缓存)
 * ========================================================================== */

/* 越界判断: 要读 ±2, 所以留 2 像素余量 */
#define V2E_IN_RANGE(x, y)   (((y) >= 2) && ((y) <= (V_H) - 4) && \
                              ((x) >= 2) && ((x) <= (V_W) - 3))

/*
 * 垂直方向的边沿: 在一列里沿 y 判断
 *
 * 注: 这里只有垂直边沿(AT_Y), 没有水平边沿(AT_X)。
 *     因为本模块的两个判据都是从"列"的角度看白线:
 *       直角弯横向扫描 -> 每列找白线的上下边沿
 *       环岛射线      -> 每条射线也是一列, 找上下边沿
 *     第一版的爬线已经在做"行"方向的工作, 不需要重复。
 *   返回 V2_TOP    暗 -> 亮 (往下变白 = 白线上边沿)
 *        V2_BOTTOM 亮 -> 暗 (白线下边沿)
 */
static uint8 v2e_edge_y(int x, int y)
{
    int up_px, down_px;
    int gy, denom;

    if (!V2E_IN_RANGE(x, y)) { return (uint8)V2_BORDER; }

    up_px   = mt9v03x_image[y - 2][x];
    down_px = mt9v03x_image[y + 2][x];

    if ((up_px + down_px) < V2E_BLACK_SUM) { return (uint8)V2_SEARCHED; }

    denom = up_px + down_px;
    gy = ((down_px - up_px) << 10) / denom;

    if (gy >  V2E_RATIO_THRES) { return (uint8)V2_TOP;    }
    if (gy < -V2E_RATIO_THRES) { return (uint8)V2_BOTTOM; }
    return (uint8)V2_SEARCHED;
}

/* ==========================================================================
 * 一、直角弯: 横向扫描
 * ========================================================================== */

/*
 * 从爬到的最顶行出发, 往 dir 方向逐列扫描, 每列找白线的上下边沿。
 *
 * 返回: 有效列数。趋势写进 *out_trend (-1 递减 / +1 递增 / 0 无)
 *
 * 用 g_mid_line / g_edge_l / g_edge_r 定位起点, 但每列的上下边沿
 * 是现场从原始灰度算的, 不依赖二值图。
 */
static int v2e_hscan(int dir, int *out_trend)
{
    int k;
    int prev_t = -1, prev_b = -1;
    int valid = 0;
    int first_y = -1, last_y = -1;
    int trend;
    int y0, x0;

    *out_trend = 0;

    if (g_path_rows < 3) { return 0; }
    if (g_path_top < 0)  { return 0; }

    /* 从路径顶端出发: 那里正是线开始拐的位置。
     * 先用这一列的第一条白线做种子。 */
    y0 = g_path_top;
    x0 = g_path_top_x;
    if (x0 < 0) { return 0; }

    /* ---- 种子列: 从 y0 附近找第一对上下边沿 ---- */
    {
        int lo = (y0 - V2E_HALF_SEARCH_W > 2) ? (y0 - V2E_HALF_SEARCH_W) : 2;
        int hi = (y0 + V2E_HALF_SEARCH_W < V_H - 4) ? (y0 + V2E_HALF_SEARCH_W) : (V_H - 4);
        int yy, seen = 0;

        for (yy = lo; yy < hi; yy++) {
            uint8 v = v2e_edge_y(x0, yy);

            if ((v == (uint8)V2_BORDER) || (v == (uint8)V2_SEARCHED)) { continue; }
            if (!seen && (v == (uint8)V2_TOP)) {
                prev_t = yy;
                seen = 1;
            } else if (seen && (v == (uint8)V2_BOTTOM)) {
                prev_b = yy;
                if (((prev_b - prev_t) >= V2E_LANE_MIN_W) &&
                    ((prev_b - prev_t) <= V2E_LANE_MAX_W)) {
                    break;      /* 找到合理的种子 */
                }
                prev_t = -1;
                seen = 0;
                prev_b = -1;
            }
        }

        if ((prev_t < 0) || (prev_b < 0)) { return 0; }
        first_y = (prev_t + prev_b) >> 1;
        last_y  = first_y;
        valid   = 1;
    }

    /* ---- 逐列往 dir 方向跟 ----
     *
     * 关键改动: 不再是"读整列找第一对边沿", 而是以上一列的
     * 上下边沿为中心, 各自在 ±HALF 内找最近的边沿。
     *
     * 为什么必须这样:
     *   直角弯拐角处白线在竖直方向变得很长(横着的胶带),
     *   一列的 BOTTOM-TOP 会远超 LANE_MAX_W。
     *   如果按"整列找第一对"就会把这个宽区间丢掉,
     *   轨迹直接断掉, 折角判不出来。
     *   连续性跟踪只关心"上边沿往上走了多少", 不受宽度影响。
     */
    for (k = 1; k <= (V_W / 2) / V2E_HORI_DX; k++) {
        int x = x0 + dir * V2E_HORI_DX * k;
        int t = -1, b = -1;
        int t_err = 0x7FFFFFFF, b_err = 0x7FFFFFFF;
        int lo, hi, yy;
        int up_span, dn_span;

        if ((x < 3) || (x > V_W - 4)) { break; }

        /* 搜索范围: 以上一列的上下边沿为中心。越往两边放宽一点 */
        up_span = V2E_HALF_SEARCH_W + k;
        dn_span = V2E_HALF_SEARCH_W + k;

        lo = (prev_t - up_span > 2) ? (prev_t - up_span) : 2;
        hi = (prev_t + up_span < V_H - 4) ? (prev_t + up_span) : (V_H - 4);
        for (yy = lo; yy < hi; yy++) {
            if (v2e_edge_y(x, yy) == (uint8)V2_TOP) {
                int d = yy - prev_t;
                int ad = (d < 0) ? -d : d;
                if (ad < t_err) { t_err = ad; t = yy; }
            }
        }

        lo = (prev_b - dn_span > 2) ? (prev_b - dn_span) : 2;
        hi = (prev_b + dn_span < V_H - 4) ? (prev_b + dn_span) : (V_H - 4);
        for (yy = lo; yy < hi; yy++) {
            if (v2e_edge_y(x, yy) == (uint8)V2_BOTTOM) {
                int d = yy - prev_b;
                int ad = (d < 0) ? -d : d;
                if (ad < b_err) { b_err = ad; b = yy; }
            }
        }

        if ((t < 0) || (b < 0)) {
            /* 这一列没跟上: 允许少量缺失, 但连续缺就停 */
            if (valid > 0) { break; }
            continue;
        }

        prev_t = t;
        prev_b = b;
        last_y = (t + b) >> 1;
        valid++;
    }

    if ((valid >= 3) && (first_y >= 0) && (last_y >= 0)) {
        trend = last_y - first_y;
        if (((trend < 0) ? -trend : trend) >= valid) {
            *out_trend = (trend > 0) ? 1 : -1;
        }
    }

    return valid;
}

/*
 * 判定折角 (直角弯)
 *
 * 往左扫和往右扫各试一次, 谁的"趋势"更明确谁就是折角。
 * 两边都有趋势时(十字/岔路)不判折角, 交给元素识别。
 *
 * 结果写进 g_corner_dir, 供 track_steer() 加偏置用。
 */
void v2e_detect_corner(void)
{
    int trend_l = 0, trend_r = 0;
    int n_l, n_r;

    g_corner_dir = 0;

    n_l = v2e_hscan(-1, &trend_l);
    n_r = v2e_hscan( 1, &trend_r);

    /* 两边都有明显趋势 -> 十字或岔路, 不当折角 */
    if ((trend_l != 0) && (trend_r != 0)) { return; }

    /* 往左扫时 y 递减 = 白线往左上走 = 左折角 */
    if ((trend_l != 0) && (n_l >= 4)) {
        if (trend_l < 0) { g_corner_dir = -1; }
    } else if ((trend_r != 0) && (n_r >= 4)) {
        if (trend_r < 0) { g_corner_dir = 1; }
    }
}

/* ==========================================================================
 * 二、环岛: 射线投票
 * ========================================================================== */

/*
 * 从 (x, start_y) 往下打一条竖射线, 看能否穿过一条白线。
 * 返回: 白线上边沿的 y; 找不到返回 -1
 */
static int v2e_ray_y(int x, int start_y)
{
    int y;
    int in_white = 0;
    int top_y = -1;

    if ((x < 3) || (x > V_W - 4)) { return -1; }
    if (start_y > V_H - 4) { start_y = V_H - 4; }

    for (y = start_y; y > 3; y--) {
        uint8 v = v2e_edge_y(x, y);

        if (v == (uint8)V2_BORDER) { return -1; }
        if (v == (uint8)V2_SEARCHED) { continue; }

        if (!in_white) {
            if (v == (uint8)V2_TOP) {
                in_white = 1;
                top_y = y;
            } else if (v == (uint8)V2_BOTTOM) {
                /* 一上来就是下边沿, 说明起点已在白线里, 不算 */
                return -1;
            }
        } else {
            if (v == (uint8)V2_BOTTOM) {
                int w = top_y - y;
                if (w < V2E_LANE_MIN_W) {
                    in_white = 0;       /* 太窄, 噪声 */
                    top_y = -1;
                } else if (w > V2E_LANE_MAX_W) {
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
static int v2e_check_trend(const int *grp, int n)
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

    if ((span > 0) && (span >= V2E_CIRCLE_SLOPE_MIN * (valid - 1)) && (span >= 3)) {
        return 1;
    }
    if ((span < 0) && (-span >= V2E_CIRCLE_SLOPE_MIN * (valid - 1)) && (-span >= 3)) {
        return -1;
    }
    return 0;
}

/*
 * 环岛检测 (单帧)
 *   返回: -1 左环岛 / +1 右环岛 / 0 无
 *
 * 以第一版爬线的最顶行为原点, 左右各打 5 条竖射线。
 * 某侧有效射线 >= 3 且有单调趋势 -> 那一侧有环岛入口的弧线。
 * 两侧都有 -> 是十字横臂, 不判环岛。
 */
int v2e_detect_island(void)
{
    int lgrp[V2E_CIRCLE_RAYS];
    int rgrp[V2E_CIRCLE_RAYS];
    int l_valid = 0, r_valid = 0;
    int l_trend, r_trend;
    int i;
    int x0, y0;

    if (g_path_rows < 5) { return 0; }
    if (g_path_top < 0)  { return 0; }

    /* 从中心稍下方开始往下打: 那里是"旁边有没有弧线"最明显的位置 */
    y0 = g_path_top + 10;
    if (y0 > V_H - 4) { y0 = V_H - 4; }

    x0 = g_path_top_x;
    if (x0 < 0) { return 0; }

    for (i = 0; i < V2E_CIRCLE_RAYS; i++) {
        int off = V2E_CIRCLE_DX_START + V2E_CIRCLE_DX_STEP * i;

        lgrp[i] = v2e_ray_y(x0 - off, y0);
        rgrp[i] = v2e_ray_y(x0 + off, y0);

        if (lgrp[i] > 0) { l_valid++; }
        if (rgrp[i] > 0) { r_valid++; }
    }

    /* 两侧都多 -> 十字横臂, 不是环岛 */
    if ((l_valid > 2) && (r_valid > 2)) { return 0; }

    l_trend = v2e_check_trend(lgrp, V2E_CIRCLE_RAYS);
    r_trend = v2e_check_trend(rgrp, V2E_CIRCLE_RAYS);

    if ((l_valid > 1) && (l_trend != 0)) { return -1; }
    if ((r_valid > 1) && (r_trend != 0)) { return 1; }

    return 0;
}
