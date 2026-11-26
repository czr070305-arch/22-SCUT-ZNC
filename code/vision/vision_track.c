/* ==========================================================================
 *  track.c —— 单线"爬线"跟踪 + 前瞻转向误差
 *  --------------------------------------------------------------------------
 *  核心思想：单线赛道没有"左右边线"可言，所以不做双边线，而是
 *  从画面最底下一行开始，逐行往上找"一段白"，每行只保留一个跟踪点。
 *  这样得到一条中心路径（g_mid_line），几何意义就是车前方引导线的走向。
 *
 *  三个必须解决的问题：
 *   1) 连续弯 / 大折线：白线一行一行地移动，靠"预测 + 搜索窗"跟住它。
 *      斜率用一阶滤波，搜索窗随丢线行数慢慢张开。
 *   2) 十字的横臂：那一行是大面积白（或超长白段），它会诱惑跟踪跟着横走。
 *      做法是整行直接判为"盲行"跳过，横穿过去后上一行的白线还在原位，
 *      于是自然直行——不需要额外记忆。
 *   3) >90° 折线：折角处会出现一段"近水平的白线"（宽段），此时按几何
 *      把折角远端当作前瞻点，车就会朝弯里打方向，而不是傻直行。
 * ========================================================================== */
#include "vision.h"
#include "algorithm/tune.h"

int   g_mid_line[V_H];
int   g_edge_l[V_H];
int   g_edge_r[V_H];
int   g_path_top = -1;
int   g_path_top_x = 0;
float g_path_slope = 0.0f;
int   g_path_rows = 0;
int   g_bottom_x = -1;
int   g_corner_dir = 0;
int   g_corner_far_x = 0;
int   g_look_x = -1;
int   g_look_y = -1;
float g_steer_err = 0.0f;
int   g_lost = 0;
int   g_lost_cnt = 0;

/* 分段拟合结果（2026-11-20） */

static int s_prev_bottom = -1;          /* 上一帧底部跟踪点（跨帧先验） */
static int s_row_white[V_H];            /* 每行白像素总量 */
static int s_corner_hold = 0;           /* 折角方向还要保持几帧 */
static int s_corner_dir_keep = 0;       /* 保持的折角方向 */
static int s_corner_x_keep = 0;         /* 保持的折角远端 x */
static int s_corner_fresh = 0;          /* 本帧是不是有新判定（区别于记忆恢复） */
static int s_corner_cfm_dir = 0;        /* 连续确认：方向 */
static int s_corner_cfm_cnt = 0;        /* 连续确认：已连续多少帧 */
#if (CFG_CROSS_LOCK)
static int s_cross_lock_x   = -1;       /* 十字直行锁定的 x */
static int s_cross_lock_cnt = 0;        /* 已经锁了多少帧 */
static int s_last_clean_x   = -1;       /* 最近一次"非十字"帧的底部 x */
#endif
static int s_cross_after   = 0;         /* 十字过后还要【禁止判折角】多少帧 */


/* 小工具：整数绝对值 */
static int iabs(int v)
{
    return (v < 0) ? -v : v;
}

/* --------------------------------------------------------------------------
 * 整行白像素统计 + 盲行判定
 *
 * 盲行 = 这一行白像素太多（横臂 / 大面积反光 / 过曝）。
 * 它的特征是"一大片"，而不是"一条线"，所以不能拿它当引导线走。
 * -------------------------------------------------------------------------- */
static void calc_row_white(void)
{
    int i, y;

    /* 车上不存整幅二值图：把该行各白段的宽度加起来，结果等价。 */
    for (y = 0; y < V_H; y++) {
        int total = 0;

        for (i = 0; i < g_run_num[y]; i++) {
            total += (int)g_runs[y][i].r - (int)g_runs[y][i].l + 1;
        }
        s_row_white[y] = total;
    }
}

/* --------------------------------------------------------------------------
 * 在一行里找"离 pred 最近"的一段白
 *   pred   : 预测的 x（浮点）
 *   tol    : 允许的偏差
 *   narrow : 1 只要窄段；0 宽段也算（宽段由调用者另行解释）
 * 返回段下标，找不到返回 -1。
 * -------------------------------------------------------------------------- */
static int find_run_near(int y, float pred, float tol, int narrow)
{
    int i;
    int best = -1;
    float best_d = 1.0e6f;

    for (i = 0; i < g_run_num[y]; i++) {
        int w = g_runs[y][i].r - g_runs[y][i].l + 1;
        float c;
        float d;

        if (w < CFG_RUN_W_MIN) {
            continue;
        }
        if (narrow) {
            if (w > CFG_RUN_W_MAX_NARROW) {
                continue;
            }
        } else if (w > CFG_RUN_W_MAX_WIDE) {
            continue;
        }
        c = 0.5f * (float)(g_runs[y][i].l + g_runs[y][i].r);
        d = (c > pred) ? (c - pred) : (pred - c);
        if ((d <= tol) && (d < best_d)) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

/* 在偏好侧里挑一段"确实是另一条支线"的段（用于岔路 / 环岛选边） */
static int find_run_prefer(int y, float pred, int side)
{
    int i;
    int best = -1;
    int best_d = 9999;              /* 记为"最近"，初值给大 */
    int best_c = -1;

    for (i = 0; i < g_run_num[y]; i++) {
        int w = g_runs[y][i].r - g_runs[y][i].l + 1;
        int c;
        int d;

        if (w < CFG_RUN_W_MIN) {
            continue;
        }
        /* 环岛里咬的是环的外弧，比普通支线宽得多：放宽上限，
         * 否则入口那条弧永远被滤掉，车就"进不去"。 */
        if (w > ((g_island_on != 0) ? CFG_ISLE_PREFER_WMAX : CFG_RUN_W_MAX_NARROW)) {
            continue;
        }
        c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
        d = c - (int)pred;
        if (d * side <= 0) {
            continue;                       /* 不在偏好侧 */
        }
        if (iabs(d) < 4) {
            continue;                       /* 太近，还是原来那条线 */
        }
        if (iabs(d) > ((g_island_on != 0) ? CFG_ISLE_PREFER_DMAX : 34)) {
            continue;                       /* 太远，不是本路口的支线 */
        }
        /* 以前取最远的一段（4~34px 里挑最大），环岛入口一眼就切到
         * 环对岸那条边上，跟踪点横跨 30px，车就横着穿岛。
         * 取"离主线最近"的那一段——那才是从主线分出来的岔口边。 */
        /* ★ 横臂端头过滤：这段窄白是不是"挂在横臂一端的竖壁"？
         *   同一行如果有比 CFG_CROSS_W 更宽的段，而且本段就贴在它边上，
         *   那就是横臂收边，不是支线。只在十字附近启用。 */
#if (CFG_ARM_END_FILTER)
        if (g_cross_rows >= 1) {
            int jj;
            int arm_end = 0;

            for (jj = 0; jj < g_run_num[y]; jj++) {
                int wj = g_runs[y][jj].r - g_runs[y][jj].l + 1;

                if (wj <= CFG_CROSS_W) {
                    continue;
                }
                if ((iabs(g_runs[y][jj].l - g_runs[y][i].r) <= CFG_ARM_END_GAP) ||
                    (iabs(g_runs[y][jj].r - g_runs[y][i].l) <= CFG_ARM_END_GAP)) {
                    arm_end = 1;
                    break;
                }
            }
            if (arm_end) {
                continue;               /* 横臂端头：不当支线 */
            }
        }
#endif
        if (iabs(d) < best_d) {
            best_d = iabs(d);
            best = i;
            best_c = c;
        }
    }
    /* ★ 2026-11-21：切边前先确认"这条支线在下一行也还在"。
     *   只在一行里出现的第二段白，多半是反光/另一段赛道的边，
     *   直接切过去会把跟踪点甩丢（实测第 13/19 帧就是这么丢的）。 */
    if ((best >= 0) && (best_c >= 0) && ((y + 1) < V_H)) {
        /* 环岛里咬的是宽弧，这里也必须按"宽段"去找，
         * 否则上面刚放宽、这里又把宽段滤掉，等于没改。 */
        int narrow2 = (g_island_on != 0) ? 0 : 1;

        if (find_run_near(y + 1, (float)best_c, 6.0f, narrow2) < 0) {
            best = -1;
        }
    }
    return best;
}

/* --------------------------------------------------------------------------
 * 起点连续性：从 (y,c) 往上数，有几行还能接上同一段线
 *
 * ★ 2026-11-20 新增。原来起点只按"离上一帧底部点最近"选，而画面最底部
 *   常常是车头阴影/布边反光（82 帧里 11 帧踩到），选中后跟 1~3 行就断，
 *   整帧丢线。现在把"能连续往上走"作为第一判据，远近只用来打破平局。
 * -------------------------------------------------------------------------- */
static int start_chain(int y, int c)
{
    int k;
    int cnt = 0;
    int miss = 0;

    for (k = 1; k <= CFG_START_CHAIN_ROWS; k++) {
        int yy = y - k;

        if (yy < 0) {
            break;
        }
        if (s_row_white[yy] > CFG_ROW_WHITE_MAX) {
            cnt++;                          /* 盲行（横臂/反光）不算断 */
            miss = 0;
            continue;
        }
        if (find_run_near(yy, (float)c, (float)CFG_START_CHAIN_TOL, 1) >= 0) {
            cnt++;
            miss = 0;
        } else if (++miss >= 2) {
            break;                          /* 连续两行接不上才算断 */
        }
    }
    return cnt;
}

/* --------------------------------------------------------------------------
 * 宽白上方是否还有"两段分开的窄线"
 *
 * ★ 2026-11-20 新增。岔路的分叉口、环岛远处的合流点，在二值图上都是
 *   一段宽白，和 90° 折角长得一模一样。区别只有一处：
 *     折角   —— 横臂上方是空的（线在这里拐走了）；
 *     岔路/环岛 —— 横臂上方还有两条分开的线（正是它们汇成了这段宽白）。
 *   不加这条，车会在每个岔路口/环岛合流处按"折角"往一边搓方向。
 * -------------------------------------------------------------------------- */
static int two_runs_above(int y, float pred)
{
    int k, i;

    for (k = 2; k <= CFG_CORNER_FORK_ROWS; k++) {
        int yy = y - k;
        int n = 0;
        int l = -1;
        int r = -1;

        if (yy < 0) {
            break;
        }
        if (s_row_white[yy] > CFG_ROW_WHITE_MAX) {
            continue;
        }
        for (i = 0; i < g_run_num[yy]; i++) {
            int w = g_runs[yy][i].r - g_runs[yy][i].l + 1;
            int c = (g_runs[yy][i].l + g_runs[yy][i].r) / 2;

            if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                continue;
            }
            if (iabs(c - (int)pred) > (CFG_BRANCH_MAX_D + 10)) {
                continue;
            }
            if (l < 0) {
                l = c;
            } else {
                r = c;
            }
            n++;
            if (n >= 2) {
                break;
            }
        }
        if ((n >= 2) && (iabs(r - l) >= CFG_CORNER_FORK_SEP)) {
            return 1;
        }
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * 路径在"画面远端"断掉时，判断这是不是一次直角/大折角
 *
 * ★ 2026-11-20 新增。原来的折角判据要求"一段宽白（横臂）包住预测点"，
 *   但实测（图集第 28 帧这类）横臂在鱼眼边缘常常只剩一条 3~5px 的窄段，
 *   和主线断成两截——宽段判据完全看不到它，于是车直冲过弯。
 *
 *   判据改成：路径已经爬到最上面几行（或早早断掉），而"最后几行"里
 *   在主线旁边（8~48px）另有一段窄白，且那个方向的上方【没有】第二条线
 *   （有就说明是岔路口，不是折角）→ 线在这里拐走了，按折角处理。
 * -------------------------------------------------------------------------- */
/* 某个方向上、路径下方是否"一直并排着另一条线"。
 * 岔路的支线会一路并排（>=4 行）；折角的横臂只在转角附近露 2~4 行。 */
static int side_runs_below(int y0, int side)
{
    int k, i, cnt = 0;

    for (k = 1; k <= 10; k++) {
        int y = y0 + k;
        int m;

        if (y >= V_H) {
            break;
        }
        m = (g_mid_line[y] >= 0) ? g_mid_line[y] : g_path_top_x;
        for (i = 0; i < g_run_num[y]; i++) {
            int w = g_runs[y][i].r - g_runs[y][i].l + 1;
            int c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
            int d = (c - m) * side;

            if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                continue;
            }
            if ((d >= CFG_CORNER_END_MIN) && (d <= (CFG_CORNER_LOOK + 8))) {
                cnt++;
                break;
            }
        }
    }
    return cnt;
}

static void corner_from_path_end(void)
{
    int y, i;
    int pos_rows = 0;
    int neg_rows = 0;
    int pos_d = 0;
    int neg_d = 0;
    int side;

    if ((g_corner_dir != 0) || (g_path_top < 0)) {
        return;
    }
    /* 只在"路径已经走到远端"时才判：要么顶到裁剪线，要么中途断掉 */
    if (g_path_top > (V_H - 30)) {
        return;
    }
    /* 已经判成岔路/十字/环岛的帧，不要再按折角处理（两个机制会互相打架） */
    if ((g_elem != 0) || (g_fork_rows >= CFG_FORK_MIN_ROWS) || (g_isle_rows > 0) ||
        (s_cross_after > 0)) {
        return;
    }

    /* 折角横臂可能落在路径顶部的【上方】（线在顶部拐走）也可能在【下方】
     * （横臂把爬线挡住了），两边都要看。 */
    for (y = g_path_top - 10; (y <= g_path_top + 4) && (y < V_H); y++) {
        int m;

        if (y < 0) {
            continue;
        }
        /* 整行大面积白 = 横臂/反光行：这里的"窄段"是横臂被贴纸缝切碎的，
         * 不是折角的另一条腿。不跳过的话十字横臂会被判成折角。 */
        if (s_row_white[y] > CFG_ROW_WHITE_MAX) {
            continue;
        }
        m = (g_mid_line[y] >= 0) ? g_mid_line[y] : g_path_top_x;

        for (i = 0; i < g_run_num[y]; i++) {
            int w = g_runs[y][i].r - g_runs[y][i].l + 1;
            int c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
            int d = c - m;

            if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                continue;
            }
            if ((iabs(d) < CFG_CORNER_END_MIN) || (iabs(d) > (CFG_CORNER_LOOK + 8))) {
                continue;
            }
            if (d > 0) {
                pos_rows++;
                if (d > pos_d) {
                    pos_d = d;
                }
            } else {
                neg_rows++;
                if (-d > neg_d) {
                    neg_d = -d;
                }
            }
        }
    }
    /* 要求"同一侧连续两行以上"——单行多半是噪点或另一段赛道的边 */
    if ((pos_rows >= 3) && (pos_rows >= neg_rows)) {
        side = 1;
    } else if (neg_rows >= 3) {
        side = -1;
    } else {
        return;
    }
    /* 支线一直并排 = 岔路，不是折角 */
    if (side_runs_below(g_path_top, side) >= 4) {
        return;
    }

    g_corner_dir = side;
    s_corner_fresh = 1;
    g_corner_far_x = g_path_top_x + g_corner_dir * CFG_CORNER_LOOK;
    if (g_corner_far_x < 2) {
        g_corner_far_x = 2;
    }
    if (g_corner_far_x > V_W - 3) {
        g_corner_far_x = V_W - 3;
    }
    s_corner_hold = CFG_CORNER_HOLD;
    s_corner_dir_keep = g_corner_dir;
    s_corner_x_keep = g_corner_far_x;
}

/* --------------------------------------------------------------------------
 * 爬线主函数
 *
 *  prefer_side: 0 中立；-1 优先走左侧支线；+1 优先走右侧支线。
 *  只有"同一行确实有两段线"时偏好侧才起作用，所以正常巡线不受影响。
 * -------------------------------------------------------------------------- */
/////////////////////////  Paired edge tracking  //////////////////////////

#if CFG_VISION_PAIRED_EDGE
// ══════════════════════════════════════════════════════════════════
// * Match both boundaries against the last measured pair.
// * Return the selected run index, or -1 if no candidate fits.
// ══════════════════════════════════════════════════════════════════
static int find_run_paired(int y, int anchor_y, float pred, float tol, float cur)
{
    int i;
    int best = -1;
    float best_score = 1.0e6f;
    float shift = pred - cur;

    for (i = 0; i < g_run_num[y]; i++) {
        int left = g_runs[y][i].l;
        int right = g_runs[y][i].r;
        int width = right - left + 1;
        float center = 0.5f * (float)(left + right);
        float center_delta = center - pred;
        float left_delta = (float)(left - g_edge_l[anchor_y]) - shift;
        float right_delta = (float)(right - g_edge_r[anchor_y]) - shift;
        float score;

        if ((width < CFG_RUN_W_MIN) || (width > CFG_RUN_W_MAX_NARROW) ||
            (center_delta < -tol) || (center_delta > tol)) {
            continue;
        }
#if (CFG_ROW_STEP_MAX > 0)
        if (iabs((left + right) / 2 - (int)cur) > CFG_ROW_STEP_MAX) {
            continue;
        }
#endif
        score = left_delta * left_delta + right_delta * right_delta;
        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}
#endif

/////////////////////////  Track construction  //////////////////////////

int track_build(int prefer_side)
{
    int y, i, k, idx, start_y;
    int switched;                       /* 本行是不是刚"换线"到偏好支线上 */
    float cur = 0.0f;
    float slope = 0.0f;
    int rows_since = 0;
    int blind_since = 0;

    /* ---- 清空本帧结果 ---- */
    for (y = 0; y < V_H; y++) {
        g_mid_line[y] = -1;
        g_edge_l[y] = -1;
        g_edge_r[y] = -1;
    }
    g_path_top = -1;
    g_path_top_x = 0;
    g_path_slope = 0.0f;
    g_path_rows = 0;
    g_bottom_x = -1;
    g_corner_dir = 0;
    g_corner_far_x = 0;

    if (s_cross_after > 0) {
        s_cross_after--;                 /* 十字刚过去：这段时间不许判折角 */
    }

    s_corner_fresh = 0;                  /* 本帧是否有新折角判定，下面各判定点会置 1 */

    /* ★ 2026-11-20：折角记忆。折角往往只有 1~2 帧能看见（车一转头，
     *   横臂就出画了），没有记忆的话"看见折角→下一帧没了→车回正"，
     *   过弯一半就摆回来。 */
    if (s_corner_hold > 0) {
        g_corner_dir = s_corner_dir_keep;
        g_corner_far_x = s_corner_x_keep;
        s_corner_hold--;
    }

    calc_row_white();

    /* ---- 1) 起点：底部若干行里，挑"能连续往上走"的那一段 ----
     * ★ 2026-11-20 改：第一判据是"往上还能接多少行"（start_chain），
     *   离上一帧底部点远近只用来打破平局。见 start_chain 的注释。 */
    {
        int ref = (s_prev_bottom >= 0) ? s_prev_bottom : (V_W / 2);
        int y0 = V_H - 1;
        int ymin = V_H - 1 - CFG_START_ROWS;
        int best_y = -1;
        int best_i = -1;
        int best_chain = -1;
        float best_d = 1.0e6f;

        start_y = -1;
        if (ymin < 0) {
            ymin = 0;
        }
        for (y = y0; y >= ymin; y--) {
            if (s_row_white[y] > CFG_ROW_WHITE_MAX) {
                continue;                   /* 盲行不能当起点 */
            }
            for (i = 0; i < g_run_num[y]; i++) {
                int w = g_runs[y][i].r - g_runs[y][i].l + 1;
                float c;
                float d;
                int chain;

                if ((w < CFG_RUN_W_MIN) || (w > CFG_CROSS_W)) {
                    continue;
                }
                c = 0.5f * (float)(g_runs[y][i].l + g_runs[y][i].r);
                d = (c > (float)ref) ? (c - (float)ref) : ((float)ref - c);
                if (d > (float)CFG_START_TOL) {
                    continue;
                }
                chain = start_chain(y, (int)c);
                if ((chain > best_chain) ||
                    ((chain == best_chain) && (d < best_d))) {
                    best_chain = chain;
                    best_d = d;
                    best_y = y;
                    best_i = i;
                }
            }
        }
        if (best_y >= 0) {
            cur = 0.5f * (float)(g_runs[best_y][best_i].l + g_runs[best_y][best_i].r);
            start_y = best_y;
            slope = 0.0f;
            g_bottom_x = (int)cur;
            g_mid_line[best_y] = (int)cur;
            g_edge_l[best_y] = g_runs[best_y][best_i].l;
            g_edge_r[best_y] = g_runs[best_y][best_i].r;
            g_path_top = best_y;
            g_path_top_x = (int)cur;
            g_path_rows = 1;
            s_prev_bottom = (int)cur;
        }
    }

    if (start_y < 0) {
        return 0;                           /* 本帧没跟到线 */
    }

#if (CFG_CROSS_LOCK)
    /* ---- 十字直行锁 ----
     * g_elem / g_cross_rows 是【上一帧】elem_update() 的结论（差一帧，够用）。
     * 判到十字就不再爬线：把中线整条画成竖线，err 于是只剩"线在正前方"，
     * 车直着穿过去。离开十字（g_elem 归 0 / 横臂出画）自动放开。 */
    {
        /* 环岛状态里也允许"强十字"直行：十字横臂 >=3 行才认，
         * 这样环岛入口那两行过渡不会被误当成十字。 */
        int cross_now = !g_exit_active && ((g_elem == 1) || (g_cross_rows >= 2)) &&
                        ((g_island_on == 0) || (g_cross_rows >= 3));

        if (cross_now && (s_cross_lock_cnt == 0)) {
            /* 用"进十字之前最后一帧"的底部点，而不是这一帧的 ——
             * 这一帧的起点很可能已经踩到横臂上了。 */
            s_cross_lock_x = s_last_clean_x;
            if ((s_cross_lock_x < 2) || (s_cross_lock_x > (V_W - 3))) {
                s_cross_lock_x = g_bottom_x;
            }
            if ((s_cross_lock_x < 2) || (s_cross_lock_x > (V_W - 3))) {
                s_cross_lock_x = V_W / 2;
            }
        }
        if (cross_now) {
            /* ★ 十字锁生效 = 这条路直走，之前判出来的折角一律作废。
             *   不清的话折角记忆（s_corner_hold）会在 track_steer 里
             *   把 look_x 抢回折角远端，十字里 err 会从 +1 变成 -64。 */
            g_corner_dir = 0;
            s_corner_hold = 0;
            s_cross_after = CFG_CROSS_AFTER;   /* 十字后保持直行若干帧 */
            if (s_cross_lock_cnt < CFG_CROSS_LOCK_MAX) {
                s_cross_lock_cnt++;
            } else {
                cross_now = 0;              /* 锁太久：放开，防止真丢线时卡死 */
                s_cross_lock_cnt = 0;
            }
        } else {
            s_cross_lock_cnt = 0;
        }

        if (cross_now) {
            int ly;

            for (ly = 0; ly <= start_y; ly++) {
                g_mid_line[ly] = s_cross_lock_x;
                g_edge_l[ly] = -1;
                g_edge_r[ly] = -1;
            }
            g_path_top = 0;
            g_path_top_x = s_cross_lock_x;
            g_path_rows = start_y + 1;
            g_path_slope = 0.0f;
            s_prev_bottom = s_cross_lock_x;
            return 1;
        }

        s_last_clean_x = g_bottom_x;        /* 记住最后一个"干净"的底部点 */
    }
#endif

    /* ★ 十字期间不许"咬边"（prefer_side）。
     * 环岛状态会把 prefer_side 设成岛侧，而且环岛的宽度上限现在是 90px，
     * 于是十字横臂正好落进"可咬"的范围 —— 跟踪点当场跳到横臂上，
     * 路径只剩二三十行（实测 12.bmp：P 从 119 掉到 28），err 直接崩。
     * 十字横臂不是"另一条支线"，这里一律不切。 */
    if (g_cross_rows >= 2) {
        prefer_side = 0;
    }

    /* ---- 2) 往上逐行跟 ---- */
#if (CFG_TRACK_TOP > 0)
    for (y = start_y - 1; y >= CFG_TRACK_TOP; y--) {
    /* ★ 只爬到 CFG_TRACK_TOP：y<34 是赛道外的白板/白格纹，
       逐帧实测那里有 ~51 条能骗过"窄段"筛选的假线段，裁掉它远端才不乱跳。 */
#else
    for (y = start_y - 1; y >= 0; y--) {
#endif
        float pred = cur + slope;
        float tol = (float)CFG_TOL_BASE + (float)CFG_TOL_GROW * (float)rows_since;
        switched = 0;

        if (tol > (float)CFG_TOL_MAX) {
            tol = (float)CFG_TOL_MAX;
        }

        /* 盲行（十字横臂 / 大面积反光）：整行不可信。
         * ★ 这里【不】沿斜率外推横向位置，而是原地直穿——横臂就在眼前，
         *   引导线只是被它遮住，位置其实没变；同时让斜率逐渐衰减，
         *   免得拿着弯道的斜率把跟踪点甩到画面外面。 */
        if (s_row_white[y] > CFG_ROW_WHITE_MAX) {
            /* ★ 盲行之前先看一眼：这行的大块白是不是"折角横臂"。
             *   横臂只朝【一侧】延伸（当前点贴着它的一端）= 线在这里拐走了，
             *   那是转向信号，不是盲区；直穿过去会让 r 虚高、e 却只剩几点。
             *   十字横臂是贯通两侧的，当前点离两端都远，不会误判。 */
            int cand = -1;

            for (i = 0; i < g_run_num[y]; i++) {
                int wi = g_runs[y][i].r - g_runs[y][i].l + 1;

                if ((wi < CFG_RUN_W_MIN) || (wi > CFG_RUN_W_MAX_WIDE)) {
                    continue;
                }
                if ((pred >= (float)(g_runs[y][i].l - CFG_CORNER_NEAR_END)) &&
                    (pred <= (float)(g_runs[y][i].r + CFG_CORNER_NEAR_END))) {
                    cand = i;
                    break;
                }
            }
            if ((cand >= 0) && (s_cross_after == 0) && (g_elem != 1) && (g_cross_rows < 2) &&
                (g_path_rows >= CFG_CORNER_MIN_ROWS) &&
                (y >= CFG_CORNER_MIN_Y) && (g_fork_rows < CFG_FORK_MIN_ROWS)) {
                int dl = iabs(g_runs[y][cand].l - (int)cur);
                int dr = iabs(g_runs[y][cand].r - (int)cur);
                int one_side = (int)(((dl <= CFG_CORNER_NEAR_END) && (dr > CFG_CORNER_NEAR_END * 2)) ||
                                     ((dr <= CFG_CORNER_NEAR_END) && (dl > CFG_CORNER_NEAR_END * 2)));
                int above_ok = 0;
                int cont = 0;

                /* 横臂上方若还【连续】有窄段接着，说明线没在这里拐走
                 * （十字的横臂就是这种：横臂在上面，竖线继续往上）。
                 * 要求连续 2 行——横臂上边缘常残留一两行碎白，单行不算。 */
                for (k = 1; k <= 8; k++) {
                    if ((y - k) < 0) {
                        break;
                    }
                    if (s_row_white[y - k] > CFG_ROW_WHITE_MAX) {
                        cont = 0;
                        continue;
                    }
                    if (find_run_near(y - k, pred, (float)CFG_TOL_MAX + 6.0f, 1) >= 0) {
                        cont++;
                        if (cont >= 2) {
                            above_ok = 1;
                            break;
                        }
                    } else {
                        cont = 0;
                    }
                }

                /* ★ 岔路分叉口 / 环岛合流点也是"一段宽白"：上方还有两段线。
                 *   折角的上方是空的。不区分的话，车会在每个岔路口按折角搓方向。 */
                if (one_side && (!above_ok) && (g_isle_rows == 0) &&
                    (!two_runs_above(y, pred))) {
                    int far = (dl > dr) ? g_runs[y][cand].l : g_runs[y][cand].r;

                    if (far > (int)cur) {
                        if ((far - (int)cur) > CFG_CORNER_LOOK) {
                            far = (int)cur + CFG_CORNER_LOOK;
                        }
                    } else if (((int)cur - far) > CFG_CORNER_LOOK) {
                        far = (int)cur - CFG_CORNER_LOOK;
                    }
                    g_corner_dir = (far > (int)cur) ? 1 : -1;
                    s_corner_fresh = 1;
                    g_corner_far_x = far;
                    s_corner_hold = CFG_CORNER_HOLD;
                    s_corner_dir_keep = g_corner_dir;
                    s_corner_x_keep = g_corner_far_x;
                    g_mid_line[y] = far;
                    g_edge_l[y] = g_runs[y][cand].l;
                    g_edge_r[y] = g_runs[y][cand].r;
                    g_path_top = y;
                    g_path_top_x = far;
                    g_path_rows++;
                    break;
                }
            }

            blind_since++;
            rows_since++;
            slope *= 0.8f;
            if ((blind_since > CFG_BLIND_ROWS) ||
                (rows_since > CFG_EXTRAP_ROWS + CFG_BLIND_ROWS)) {
                break;
            }
            continue;
        }
        blind_since = 0;

#if CFG_VISION_PAIRED_EDGE
        if ((g_path_top >= 0) && (g_edge_l[g_path_top] >= 0) &&
            (g_edge_r[g_path_top] >= 0)) {
            idx = find_run_paired(y, g_path_top, pred, tol, cur);
        } else {
            idx = find_run_near(y, pred, tol, 1);
        }
#else
        idx = find_run_near(y, pred, tol, 1);
#endif

#if (CFG_ROW_STEP_MAX > 0)
        /* 行间跳变硬限制：离上一行跟踪点太远的段一律不认。
         * find_run_near 只保证"离预测点 <= tol"，可预测点本身可能已经被
         * 脏斜率带偏；十字横臂就是趁这个口子把跟踪点整条拽走的。 */
        if (idx >= 0) {
            int c_int = (g_runs[y][idx].l + g_runs[y][idx].r) / 2;

            if (iabs(c_int - (int)cur) > CFG_ROW_STEP_MAX) {
                idx = -1;
            }
        }
#endif

        /* 偏好侧（岔路 / 环岛选边）：只要出现第二段线，就切到那一段去 */
        if ((prefer_side != 0) && (g_run_num[y] > 1)) {
            int alt = find_run_prefer(y, pred, prefer_side);

            if (alt >= 0) {
                idx = alt;
                switched = 1;           /* 换线了：这一行的位移不是斜率 */
            }
        }

        if (idx >= 0) {
            float c = 0.5f * (float)(g_runs[y][idx].l + g_runs[y][idx].r);
            float step = (float)(rows_since + 1);
            float inst = (c - cur) / step;

            /* 换线那一行，位移是"横移"不是斜率（可能一下 30px）。
             * 照样喂进斜率滤波，下一行预测点就飞出去，爬线当场断掉。 */
            if (switched) {
                slope = 0.0f;
            } else {
                slope = CFG_SLOPE_SMOOTH * slope + (1.0f - CFG_SLOPE_SMOOTH) * inst;
            }
            cur = c;

            g_mid_line[y] = (int)c;
            g_edge_l[y] = g_runs[y][idx].l;
            g_edge_r[y] = g_runs[y][idx].r;
            g_path_top = y;
            g_path_top_x = (int)c;
            g_path_rows++;
            rows_since = 0;
            continue;
        }

        /* 窄段没找到——先怀疑是"折角"：本行有一段包含预测点的宽白 */
        {
            int widx = -1;

            /* ★ 这里不能用"段中心离预测点多远"来判，因为折角处横臂会把
             *   整段的中心拽到一边（中心可能在 40px 外），但预测点其实就
             *   落在段里。正确判据是"这一段是否包含预测点"。 */
            for (i = 0; i < g_run_num[y]; i++) {
                int wi = g_runs[y][i].r - g_runs[y][i].l + 1;

                if ((wi < CFG_RUN_W_MIN) || (wi > CFG_RUN_W_MAX_WIDE)) {
                    continue;
                }
                if ((pred >= (float)(g_runs[y][i].l - CFG_RUN_EDGE_TOL)) &&
                    (pred <= (float)(g_runs[y][i].r + CFG_RUN_EDGE_TOL))) {
                    widx = i;
                    break;
                }
            }

            if (widx >= 0) {
                int w = g_runs[y][widx].r - g_runs[y][widx].l + 1;

                if (w > CFG_RUN_W_MAX_NARROW) {
                    int above_ok = 0;
                    int one_side = 0;
                    int dl = iabs(g_runs[y][widx].l - (int)cur);
                    int dr = iabs(g_runs[y][widx].r - (int)cur);
                    int far;

                    /* ★ 宽段只朝【一侧】延伸 = 当前点贴着它的一端，另一端伸得很远。
                     *   L 形折角就是这样（线到这里拐走，只剩一条胳膊）；
                     *   十字横臂是贯通两侧的，两端都离当前点远，所以不算。
                     *   不加这条时，超过 72px 的长折角一律不认，车会直冲过弯。 */
                    if (((dl <= CFG_CORNER_NEAR_END) && (dr > CFG_CORNER_NEAR_END)) ||
                        ((dr <= CFG_CORNER_NEAR_END) && (dl > CFG_CORNER_NEAR_END))) {
                        one_side = 1;
                    }

                    /* 上方若还有窄段接着，那这只是"斜着看变宽"，不是折角。
                     * 横臂常常 5~8 行厚，只看 4 行会一直在臂内打转，
                     * 把"上方还有竖线"这条证据漏掉，十字就被当成折角。 */
                    for (k = 1; k <= 8; k++) {
                        if ((y - k) < 0) {
                            break;
                        }
                        if (s_row_white[y - k] > CFG_ROW_WHITE_MAX) {
                            continue;
                        }
                        if (find_run_near(y - k, pred, tol + 12.0f, 1) >= 0) {
                            above_ok = 1;
                            break;
                        }
                    }
                    /* ★ 只在【近端】(y >= CFG_CORNER_MIN_Y) 判折角：
                     *   远端看到横臂就判 -> 车还没到角就提前切弯。
                     *   近端判时不再用“上方还有线”否决 —— 连续直角/十字旁边
                     *   本来就有下一段赛道在上方，一否决就只能按直线外推 -> 直行丢线。 */
                    if (((!above_ok) || (y >= CFG_CORNER_MIN_Y)) &&
                        (s_cross_after == 0) && (g_path_rows >= CFG_CORNER_MIN_ROWS) &&
                        (y >= CFG_CORNER_MIN_Y) &&
                        (g_elem != 1) && (g_cross_rows < 2) && (g_isle_rows == 0) &&
                        (g_fork_rows < CFG_FORK_MIN_ROWS) &&
                        ((!two_runs_above(y, pred)) || (y >= CFG_CORNER_MIN_Y)) &&
                        ((w <= CFG_CORNER_W_MAX) || one_side)) {
                        /* 折角：取离当前点更远的一端当作"线往哪边走"，
                         * 但只取前方 CFG_CORNER_LOOK 像素处——直接拿最远端
                         * 会让误差一路打满限幅，车在每个折角都搓方向。 */
                        if (iabs(g_runs[y][widx].l - (int)cur) >
                            iabs(g_runs[y][widx].r - (int)cur)) {
                            far = g_runs[y][widx].l;
                        } else {
                            far = g_runs[y][widx].r;
                        }
                        if (far > (int)cur) {
                            if ((far - (int)cur) > CFG_CORNER_LOOK) {
                                far = (int)cur + CFG_CORNER_LOOK;
                            }
                        } else if (((int)cur - far) > CFG_CORNER_LOOK) {
                            far = (int)cur - CFG_CORNER_LOOK;
                        }
                        g_corner_dir = (far > (int)cur) ? 1 : -1;
                    s_corner_fresh = 1;
                        g_corner_far_x = far;
                        s_corner_hold = CFG_CORNER_HOLD;
                        s_corner_dir_keep = g_corner_dir;
                        s_corner_x_keep = g_corner_far_x;
                        g_mid_line[y] = far;
                        g_edge_l[y] = g_runs[y][widx].l;
                        g_edge_r[y] = g_runs[y][widx].r;
                        g_path_top = y;
                        g_path_top_x = far;
                        g_path_rows++;
                        break;                  /* 到折角为止，上面交给前瞻去指 */
                    }
                }
            }
        }

        /* 真丢线：按斜率外推，超过上限就收手 */
        rows_since++;
        if (rows_since > CFG_EXTRAP_ROWS) {
            break;
        }
        cur += slope;
    }

    /* ★ 2026-11-20：宽段判据漏掉的直角（横臂只剩窄段）在这里补一刀 */
    corner_from_path_end();

    /* ============================================================
     * 折角「连续多帧确认」门控（2026-11-24）
     * ------------------------------------------------------------
     * 上面三处判定都是单帧直接写 g_corner_dir，误判代价很大
     * （前瞻点被搬到噪点上 -> 车猛打方向）。这里统一把关：
     *
     *   s_corner_fresh = 1  -> 本帧有新判定，累加计数器
     *   s_corner_fresh = 0  -> 只是记忆恢复，沿用已确认的状态
     *
     * 没攒够 CFG_CORNER_CONFIRM_TRACK 帧时：
     *   g_corner_dir 清零，【并且 s_corner_hold 也清零】。
     *   不清记忆的话，单帧噪声会被 CFG_CORNER_HOLD 放大 8 帧，
     *   下一帧门控又从记忆里读到同一个方向，白白累加。
     * ============================================================ */
    if (g_corner_dir != 0) {
        if (s_corner_fresh) {
            if (g_corner_dir == s_corner_cfm_dir) {
                if (s_corner_cfm_cnt < 1000) {
                    s_corner_cfm_cnt++;
                }
            } else {
                s_corner_cfm_dir = g_corner_dir;
                s_corner_cfm_cnt = 1;
            }
        }
        if (s_corner_cfm_cnt < CFG_CORNER_CONFIRM_TRACK) {
            g_corner_dir   = 0;
            g_corner_far_x = 0;
            s_corner_hold  = 0;      /* 关键：连记忆一起清 */
            s_corner_cfm_dir = 0;
            s_corner_cfm_cnt = 0;
        }
    } else {
        s_corner_cfm_dir = 0;
        s_corner_cfm_cnt = 0;
    }

    g_path_slope = slope;
    return 1;
}

/* --------------------------------------------------------------------------
 * 某行的"参考 x"：给元素判定用
 *   有跟踪点就用跟踪点；路径上方的行按顶部斜率外推；再不行就退回底部点。
 * -------------------------------------------------------------------------- */
int track_ref_x(int y)
{
    int i;

    if (y < 0) {
        y = 0;
    }
    if (y >= V_H) {
        y = V_H - 1;
    }
    if (g_mid_line[y] >= 0) {
        return g_mid_line[y];
    }
    if ((g_path_top >= 0) && (y < g_path_top)) {
        int x = g_path_top_x + (int)(g_path_slope * (float)(g_path_top - y));

        if (x < 2) {
            x = 2;
        }
        if (x > V_W - 3) {
            x = V_W - 3;
        }
        return x;
    }
    for (i = y + 1; i < V_H; i++) {
        if (g_mid_line[i] >= 0) {
            return g_mid_line[i];
        }
    }
    if (g_bottom_x >= 0) {
        return g_bottom_x;
    }
    return V_W / 2;
}

/* --------------------------------------------------------------------------
 * 转向误差
 *
 *  取"从底部往上 CFG_LOOKAHEAD 行"处的路径点作为前瞻点，误差取它相对
 *  车轴（画面中心线）的横向偏差；再加近端斜率做阻尼，避免来回摆。
 *
 *  为什么不用 atan2：误差最终是给舵机/差速的，横向像素偏差与角度在
 *  小角度下近似成正比，直接用像素量既省算力又便于标定。
 * -------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------
 * err 一阶低通滤波状态
 *
 *   y(n) = y(n-1) + a * (x(n) - y(n-1))
 *   a = 1 - ERR_LP/100       （ERR_LP 在 TRACK 页，0~95）
 *
 *   ERR_LP =  0  -> a = 1.00  完全关闭，直接用当帧值
 *   ERR_LP = 70  -> a = 0.30  时间常数约 3.3 帧 = 66ms @50fps（默认）
 *   ERR_LP = 90  -> a = 0.10  时间常数 10 帧 = 200ms，很钝
 *
 * 为什么需要:
 *   视觉 err 是单帧值。某一帧爬线偶发跳变（中线断一下、跳到别的
 *   白段上）会原封不动进 20ms 的控制环，车就"顿一下"。低通把单帧
 *   尖峰摊到几帧里，车身跟得平顺。
 *
 * 为什么不影响弯道:
 *   弯道里 err 是持续大值，低通只削瞬态、不改稳态，
 *   稳态输出等于输入，所以弯道转向力度不变。
 *
 * s_err_lp_ok = 0 表示上一帧没有有效值（丢线 / 路太短），
 * 本帧恢复时直接取新值，避免拿很久以前的旧状态当起点。
 * -------------------------------------------------------------------------- */
static float s_err_lp    = 0.0f;
static uint8 s_err_lp_ok = 0;

void track_steer(void)
{
    int i;
    int y0 = -1;
    int look_y;
    int look_x;
    float dx;
    float err;
    float near_slope = 0.0f;

    for (i = V_H - 1; i >= 0; i--) {
        if (g_mid_line[i] >= 0) {
            y0 = i;
            break;
        }
    }
    if (y0 < 0) {
        /* 整帧无路径：保留上一帧误差，丢线由上层计数 */
        g_look_x = -1;
        g_look_y = -1;
        s_err_lp_ok = 0;
        return;
    }

#if (CFG_CROSS_LOCK)
    /* ---- 十字：只认"进十字时那条线"的位置，其它一概不算 ----
     * 用户要的其实就一句话：十字就直着走。
     * 原来这里会把 折角 / 斜率阻尼 / 前瞻 全算一遍，任何一个环节被横臂
     * 干扰（折角把 look_x 拉到 52、或锁定点偏一点），err 就变成 ±64，
     * 车当场打死方向盘。十字期间直接跳过全部环节。 */
    if ((s_cross_lock_cnt > 0) && (s_cross_lock_x >= 0)) {
        float ce = (float)(s_cross_lock_x - (V_W / 2)) * CFG_ERR_OFF_W;

        s_err_lp = ce;
        s_err_lp_ok = 1;
        g_steer_err = ce;
        g_look_x = s_cross_lock_x;
        g_look_y = 0;
        return;
    }
#endif
    /* 路径太短 = 近处被横臂/反光挡住，这时候算出来的前瞻点没有意义。
     * 原来【已判折角】可以绕过这条，于是横臂一挡（实测只剩 33 行）
     * 照样按折角算，err 直接 -64。现在一律保持上一帧方向。 */
    if ((g_path_rows < CFG_MIN_ROWS_STEER) && (g_corner_dir == 0)) {
        /* 只跟到几行：多半是路口的大块白 / 反光把近处挡住了。
         * 这时算出来的前瞻点没有意义，宁可保持上一帧方向，也别乱打。
         * ★ 但如果已经判出折角，折角本身就是"往哪边走"的结论，要继续用。 */
        g_look_x = -1;
        g_look_y = -1;
        s_err_lp_ok = 0;
        return;
    }

    /* 前瞻点：优先取 "y0 - CFG_LOOKAHEAD" 附近最近的有效行 */
    look_y = y0 - (int)tune_get(TUNE_LOOKAHEAD);   /* TRACK 页 LOOK */
    if (look_y < 0) {
        look_y = 0;
    }
    look_x = -1;
    for (i = look_y; i <= y0; i++) {
        if (g_mid_line[i] >= 0) {
            look_x = g_mid_line[i];
            look_y = i;
            break;
        }
    }
    if (look_x < 0) {
        look_x = g_mid_line[y0];
        look_y = y0;
    }

    /* 折角修正：既然已经判定这是折角（线明确往哪边拐），前瞻就直接用折角方向，
     * 不再受"路径够不够长"的限制——否则前瞻会落在还没拐的那段竖直线上，
     * 算出来的偏差只有几点，车照样冲过去。 */
#if (CFG_CORNER_ENABLE)
    if (g_corner_dir != 0) {
        look_x = g_corner_far_x;
        look_y = g_path_top;
    }
#endif

    /* 【移植补丁】畸变补偿（CFG_UNDIST_ENABLE）
     *
     * 这颗镜头是明显桶形畸变（k1=-1.10）：同一条直线的横向像素间隔，
     * 在画面顶部（远处）会被"压缩"，于是同一个物理偏角，远处算出来的
     * dx 偏小 —— 车在远处弯道打得不够。
     *
     * 做法：把"前瞻点"和"画面中心点"在同一行上各自反畸变，再求差。
     * 两点在同一条畸变行上，中心点自身的偏差被抵消，只留下"该行横向
     * 缩放比"的修正，不引入常值偏置。每帧只算 2 个点，开销可忽略。
     *
     * ★ 队友 11-21 版把这一段丢了，性能会比你 11-19 版差。此处补回。
     * ------------------------------------------------------------------ */
#if (CFG_UNDIST_ENABLE)
    dx = vision_undistort_x((float)look_x, (float)look_y)
       - vision_undistort_x((float)(V_W / 2), (float)look_y);
#else
    dx = (float)(look_x - (V_W / 2));
#endif
    err = dx * CFG_ERR_OFF_W;

    /* 近端斜率阻尼：底部往回 14 行内的平均斜率（px / 行） */
    {
        int yn = y0 - 14;
        int yn_used = -1;

        if (yn < 0) {
            yn = 0;
        }
        for (i = yn; i < y0; i++) {
            if (g_mid_line[i] >= 0) {
                yn_used = i;
                break;
            }
        }
        /* 斜率项的两个保护：
         *
         * 1) 分母不能太小
         *    上面的循环找"往上 14 行内第一个有效行"。如果第 y0-14 行
         *    恰好断了一帧，它会往车头方向缩，分母从 14 变成 5、3、甚至 1。
         *    分母 = 1 时，爬线抖 1 像素就让 near_slope 变成 1.0，
         *    err 直接跳 SLOPE 那么多。这就是"偶尔抽一下"的来源。
         *    所以分母不足 CFG_SLOPE_MIN_SPAN 行时，干脆不用斜率。
         *
         * 2) 直线段不用斜率
         *    直线的真实斜率是 0，斜率项贡献的全是爬线噪声。
         *    但弯道入口恰恰是"横向偏差还小、斜率已经大"的时刻，
         *    一刀切会丢掉最该提前的那几帧。
         *    所以按横向偏差 |dx| 渐变：
         *      |dx| <= GATE_LO  -> 完全不用（直线）
         *      |dx| >= GATE_HI  -> 全量使用（弯道）
         *      中间线性过渡
         */
        if ((yn_used >= 0) && ((y0 - yn_used) >= CFG_SLOPE_MIN_SPAN)) {
            float adx = (dx < 0.0f) ? -dx : dx;
            float gate;

            if (adx <= CFG_SLOPE_GATE_LO) {
                gate = 0.0f;
            } else if (adx >= CFG_SLOPE_GATE_HI) {
                gate = 1.0f;
            } else {
                gate = (adx - CFG_SLOPE_GATE_LO) /
                       (CFG_SLOPE_GATE_HI - CFG_SLOPE_GATE_LO);
            }

            if (gate > 0.0f) {
                near_slope = (float)(g_mid_line[yn_used] - g_mid_line[y0]) /
                             (float)(y0 - yn_used);
                err += near_slope * tune_get(TUNE_SLOPE_W) * gate;
            }
        }
    }

    /* 折角时额外往弯里推一点，补偿"单点前瞻"看不出 90° 以上的折 */
#if (CFG_CORNER_ENABLE)
    if (g_corner_dir != 0) {
        err += (float)g_corner_dir * CFG_CORNER_BIAS;
    }
#endif

    /* ---- 环岛"往弯里推" ----
     * 仿真那边是靠前馈差速（FF_ISLAND = 20%）把车压进环的，车上没有
     * 前馈通道，这里折算成误差偏置直接加进 err：KP 在 2.5~7 之间时，
     * 60px 大约相当于 base 的 15~20% 差速，正好够把车拐进环。
     * 只在环岛状态里加，一判出岛立刻归零。 */
    if (g_island_on && !g_exit_active) {
        err += (float)g_island_side * CFG_ISLE_ERR_PUSH;
    }

    /* ------------------------------------------------------------------
     * NaN / Inf 清洗  ★必须在限幅之前★
     *
     * 反畸变的迭代反解在极端像素位置会发散, 产生 NaN 或 Inf.
     * 而 NaN 与任何数比较都为 false, 所以:
     *     if (err > 110.0f)  err = 110.0f;    <- NaN 在这里被放过
     *     if (err < -110.0f) err = -110.0f;   <- 这里也被放过
     * NaN 就这么一路传到控制层的 PID 和 chassis_set_target,
     * 屏幕上会看到 err 变成 214748364 (INT32_MAX), 电机乱转.
     *
     * 自比较是判断 NaN 的标准做法, 不用引入 isnan()/<math.h>.
     * ★ 队友 11-21 版删掉了这段，补回。
     * ------------------------------------------------------------------ */
    if (!(err == err)) {
        /* NaN: 保留上一帧的误差, 当作本帧无效 */
        g_look_x = -1;
        g_look_y = -1;
        g_lost = 1;
        s_err_lp_ok = 0;
        return;
    }
    if (err > 1.0e6f)  { err =  110.0f; }   /* +Inf 或荒谬大值 */
    if (err < -1.0e6f) { err = -110.0f; }   /* -Inf 或荒谬小值 */

    /* 限幅：别让一个脏点把方向打到极限 */
    if (err > 110.0f) {
        err = 110.0f;
    }
    if (err < -110.0f) {
        err = -110.0f;
    }

    /* ---- err 一阶低通（TRACK 页 ERR_LP 可调，0=关闭）----
     * 放在限幅之后：输入已经有界，滤波输出自然有界，不用重复钳位。
     * 只削弱瞬态，不改稳态 —— 弯道转向力度不受影响。 */
    {
        float a = 1.0f - tune_get(TUNE_ERR_LP);

        if ((!s_err_lp_ok) || (a >= 0.999f))
        {
            s_err_lp = err;         /* 首次 / 丢线刚恢复 / 关闭滤波 */
        }
        else
        {
            if (a < 0.05f)
            {
                a = 0.05f;          /* 保留最低响应，防止完全不跟 */
            }
            s_err_lp += a * (err - s_err_lp);
        }
        s_err_lp_ok = 1;
        err = s_err_lp;
    }

    g_steer_err = err;
    g_look_x = look_x;
    g_look_y = look_y;
}
