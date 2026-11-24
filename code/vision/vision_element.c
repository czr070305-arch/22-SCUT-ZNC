/* ==========================================================================
 *  element.c —— 十字 / 岔路 / 环岛 识别，以及"两圈分别走两条路线"的记忆
 *  --------------------------------------------------------------------------
 *  单线赛道元素的本质都是"一行里出现了不止一段白"：
 *
 *     ┌──────────────────────────────────────────────────────────────┐
 *     │ 一条主线 + 一条往旁边分出去、越往上越远   → 岔路              │
 *     │ 两条线把画面夹成"梭形"：先分开、又合回去 → 环岛（闭合的圈）    │
 *     │ 一段横臂、横穿路径两侧、上下都还有线      → 十字              │
 *     └──────────────────────────────────────────────────────────────┘
 *
 *  ★ 两个关键判据（都是踩过坑才加上的）：
 *   1) 岔路要"越往上越分开"。折线（>90° 的弯）也会出现"两段白"，但那是
 *      来路和去路，越往上越靠拢——只看"有两段"会把手里的每个折线当成岔路。
 *   2) 环岛要"分开了再合回来"。单看某一行，环岛入口和岔路长得一模一样；
 *      区别在于环岛是闭合的，两条边到顶又并成一条，而岔路一直分着走。
 *      所以环岛靠"梭形"轮廓识别，不靠单行。
 * ========================================================================== */
#include "vision.h"

int g_elem = 0;
int g_elem_side = 0;
int g_prefer_side = 0;
int g_island_on = 0;
int g_island_side = 0;
int g_lap = 1;
int g_fork_pass = 0;
int g_fork_use_branch = 0;
int g_fork_rows = 0;
int g_isle_rows = 0;
int g_cross_rows = 0;

/* ★ 2026-11-20：状态机对外量（控制层直接用） */
int g_state = 0;
int g_state_frames = 0;
int g_speed_pct = 100;

static int s_cd = 0;                /* 元素冷却（帧） */

/* 直角弯判据的去抖计数（2026-11-21 加）
 * 直道上 g_corner_dir 会零星误报，一帧误报就把速度砍到 70%，
 * 表现为"前后一顿一顿"。必须连续 N 帧同方向才认。 */
static int s_corner_dir_last = 0;   /* 上一帧的 g_corner_dir */
static int s_corner_frames   = 0;   /* 连续同方向的帧数 */
static int s_isle_quiet = 0;        /* 环岛里双线消失帧数 */
static int s_island_frames = 0;     /* 在环岛里的帧数 */
static int s_island_force = 0;      /* 进岛后还需要"强制咬边"的帧数 */
static int s_isle_hit = 0;
static int s_isle_candidate_side = 0;
static int s_fork_hit = 0;
static int s_isle_cool = 0;          /* 出岛后的冷却帧数 */          /* 分叉形态连续命中帧数（环岛入口去抖） */          /* 环岛连续命中帧数（去抖） */

/* 逐行的形态缓存：s_sep[y]=两线间距；s_br[y]=支线相对参考点的偏移（0=无） */
static int s_sep[V_H];
static int s_br[V_H];

static int path_x_at(int y);         /* 定义在后面（插值参考点） */

static int v_abs(int v)
{
    return (v < 0) ? -v : v;
}

static int v_sign(int v)
{
    if (v > 0) {
        return 1;
    }
    if (v < 0) {
        return -1;
    }
    return 0;
}

void elem_reset(void)
{
    g_elem = 0;
    g_elem_side = 0;
    g_prefer_side = 0;
    g_island_on = 0;
    g_island_side = 0;
    g_lap = 1;
    g_fork_pass = 0;
    g_fork_use_branch = 0;
    g_fork_rows = 0;
    g_isle_rows = 0;
    g_cross_rows = 0;
    s_cd = 0;
    s_isle_quiet = 0;
    s_island_frames = 0;
    s_island_force = 0;
    s_isle_hit = 0;
    s_isle_candidate_side = 0;
    s_fork_hit = 0;
    g_state = 0;
    g_state_frames = 0;
    g_speed_pct = 100;
}

/* --------------------------------------------------------------------------
 * 十字：横臂 = 一段特别长的白，而且它被"路径"横穿（路径两侧各伸出很长）。
 *       只在一侧伸出的是折角（去路），不是十字。
 * -------------------------------------------------------------------------- */
static int detect_cross(void)
{
    int y, i, rows = 0;
    int y_lo;

    if (g_path_top < 0) {
        return 0;
    }
    y_lo = g_path_top - CFG_ELEM_EXTRA_ROWS;
    if (y_lo < CFG_TRACK_TOP) {
        y_lo = CFG_TRACK_TOP;
    }

    for (y = y_lo; y < V_H - 5; y++) {
        int ref = path_x_at(y);

        for (i = 0; i < g_run_num[y]; i++) {
            int w = g_runs[y][i].r - g_runs[y][i].l + 1;

            if (w < CFG_CROSS_W) {
                continue;
            }
            if (((ref - g_runs[y][i].l) < CFG_CROSS_ARM_MIN) ||
                ((g_runs[y][i].r - ref) < CFG_CROSS_ARM_MIN)) {
                continue;
            }
            {
                int k;
                int has_above = 0;
                int has_below = 0;

                for (k = 1; k <= 6; k++) {
                    if ((y - k >= 0) && (g_mid_line[y - k] >= 0)) {
                        has_above = 1;
                    }
                    if ((y + k < V_H) && (g_mid_line[y + k] >= 0)) {
                        has_below = 1;
                    }
                }
                if (has_above && has_below) {
                    rows++;
                }
            }
        }
    }
    return rows;
}

/* --------------------------------------------------------------------------
 * 某行的"参考 x"：优先用跟踪点；没有就【线性插值】上下两行，
 * 而不是拿顶部斜率一路外推。
 *
 * ★ 2026-11-20：外推在"路径只剩几行"时会把参考点甩到画面外几十像素，
 *   于是顶部的白板/格纹随便一段都能落进 11~42px 的"支线"窗口里，
 *   假岔路、假环岛基本都是这么来的。
 * -------------------------------------------------------------------------- */
static int path_x_at(int y)
{
    int i;
    int ya = -1;
    int yb = -1;

    if (y < 0) {
        y = 0;
    }
    if (y >= V_H) {
        y = V_H - 1;
    }
    if (g_mid_line[y] >= 0) {
        return g_mid_line[y];
    }
    for (i = y + 1; i < V_H; i++) {
        if (g_mid_line[i] >= 0) {
            yb = i;
            break;
        }
    }
    for (i = y - 1; i >= 0; i--) {
        if (g_mid_line[i] >= 0) {
            ya = i;
            break;
        }
    }
    if ((yb >= 0) && (ya >= 0)) {
        int xb = g_mid_line[yb];
        int xa = g_mid_line[ya];

        return xb + (xa - xb) * (yb - y) / (yb - ya);
    }
    if (yb >= 0) {
        return g_mid_line[yb];
    }
    if (ya >= 0) {
        return g_mid_line[ya];
    }
    return track_ref_x(y);
}

/* --------------------------------------------------------------------------
 * 第一遍：把每一行的"形态"量化
 *   s_sep[y] = 最近两段窄白之间的间距（只有一段白时为 0）
 *   s_br[y]  = 支线相对参考点的偏移（无支线时为 0）
 * 单线的赛道上，绝大多数行 s_sep=0、s_br=0；只有元素处会出现非零。
 * -------------------------------------------------------------------------- */
static void probe_rows(void)
{
    int y, i;
    int y_lo;

    for (y = 0; y < V_H; y++) {
        s_sep[y] = 0;
        s_br[y] = 0;
    }
    if (g_path_top < 0) {
        return;                             /* 本帧没路径：不判元素，免得对着白板判 */
    }

    /* ★ 只探"路径顶部再往上 CFG_ELEM_EXTRA_ROWS 行"这一带，且不低于
     *   CFG_TRACK_TOP。再往上没有跟踪点，参考位置只能外推，而画面顶部
     *   是赛道外的白板/格纹——假岔路、假环岛绝大多数来自那几行。 */
    y_lo = g_path_top - CFG_ELEM_EXTRA_ROWS;
    if (y_lo < CFG_TRACK_TOP) {
        y_lo = CFG_TRACK_TOP;
    }

    for (y = V_H - 3; y >= y_lo; y--) {
        int ref;
        int near_d = 999;
        int narrow = 0;

        if (g_run_num[y] < 2) {
            continue;
        }
        ref = path_x_at(y);

        for (i = 0; i < g_run_num[y]; i++) {
            int w = g_runs[y][i].r - g_runs[y][i].l + 1;
            int c;
            int ad;

            if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                continue;
            }
            narrow++;
            c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
            ad = v_abs(c - ref);
            if (ad < near_d) {
                near_d = ad;
            }
        }
        if (narrow < 2) {
            continue;
        }

        /* ★ 2026-11-20 改：环岛的判据是"参考点【两侧】各有一段线"，
         *   而不是"最近的两段线间距"。
         *   环岛入口：主线在中间，岛的近边在一侧、远边在另一侧——
         *   实测第 58 帧就是 3 段（58 / 101 / 148），旧判据取"最近两段"
         *   得到 101 和 58，参考点自己把一侧占掉了，间距永远算不出来，
         *   所以环岛经常"没进去"。 */
        {
            int cl = -1;
            int cr = -1;
            int dl = 9999;
            int dr = 9999;

            for (i = 0; i < g_run_num[y]; i++) {
                int w = g_runs[y][i].r - g_runs[y][i].l + 1;
                int c;

                if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                    continue;
                }
                c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
                if (c < (ref - 2)) {
                    if ((ref - c) < dl) {
                        dl = ref - c;
                        cl = c;
                    }
                } else if (c > (ref + 2)) {
                    if ((c - ref) < dr) {
                        dr = c - ref;
                        cr = c;
                    }
                }
            }
            if ((cl >= 0) && (cr >= 0)) {
                int sep = cr - cl;

                if ((sep >= CFG_ISLE_SEP_MIN) && (sep <= CFG_ISLE_SEP_MAX)) {
                    s_sep[y] = sep;
                }
            }
        }

        /* 支线：主线接得上（距参考点近），另有一段在 11~42 px 之外 */
        if (near_d <= CFG_FORK_REF_TOL) {
            int bd = 0;
            int bs = 0;

            for (i = 0; i < g_run_num[y]; i++) {
                int w = g_runs[y][i].r - g_runs[y][i].l + 1;
                int c;
                int d;
                int ad;

                if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                    continue;
                }
                c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
                d = c - ref;
                ad = v_abs(d);
                if ((ad < CFG_BRANCH_MIN_D) || (ad > CFG_BRANCH_MAX_D)) {
                    continue;
                }
                if (ad > bd) {
                    bd = ad;
                    bs = v_sign(d);
                }
            }
            if (bd > 0) {
                s_br[y] = bs * bd;
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * 环岛入口：参考点【两侧都有线】连续多少行
 *
 * ★ 2026-11-20 新增。环岛和岔路在入口都是"主线分叉"，唯一稳定的区别是：
 *   环岛是中间一条主线、两侧各有一条边（三条线同时出现）；
 *   岔路是主线 + 一条支线（两段线，在参考点同侧）。
 *   所以环岛的入口判据用"两侧都有线"，不用等远处的合流（那个往往在车
 *   开过入口之后才进画面）。
 * -------------------------------------------------------------------------- */
static int count_sep_run(void)
{
    int y;
    int y_lo;
    int run = 0;
    int best = 0;

    if (g_path_top < 0) {
        return 0;
    }
    y_lo = g_path_top - CFG_ELEM_EXTRA_ROWS;
    if (y_lo < CFG_TRACK_TOP) {
        y_lo = CFG_TRACK_TOP;
    }
    for (y = V_H - 3; y >= y_lo; y--) {
        if (s_sep[y] > 0) {
            run++;
            if (run > best) {
                best = run;
            }
        } else {
            run = 0;
        }
    }
    return best;
}

/* --------------------------------------------------------------------------
 * 岔路：支线连续存在、且越往上越远（滑移 ≥ CFG_DIVERGE_PX）。
 * -------------------------------------------------------------------------- */
static int find_fork(int *side_out)
{
    int y;
    int active = 0;
    int rows = 0;
    int first_d = 0;
    int max_d = 0;
    int sum_d = 0;          /* 2026-11-24 加：整段 s_br 累加，用于定方向 */
    int best = 0;
    int best_side = 0;

    for (y = V_H - 3; y >= 2; y--) {
        if (s_br[y] != 0) {
            if (!active) {
                active = 1;
                rows = 0;
                first_d = s_br[y];
                max_d = s_br[y];
                sum_d = s_br[y];
            }
            rows++;
            if (v_abs(s_br[y]) > v_abs(max_d)) {
                max_d = s_br[y];
            }
            sum_d += s_br[y];
        } else if (active) {
            if ((rows >= CFG_FORK_MIN_ROWS) &&
                (v_abs(max_d) >= v_abs(first_d) + CFG_DIVERGE_PX) &&
                (rows > best)) {
                best = rows;
                best_side = v_sign(sum_d);   /* 2026-11-24 改：整段累加，避免单行噪声翻方向 */
            }
            active = 0;
        }
    }
    if (active && (rows >= CFG_FORK_MIN_ROWS) &&
        (v_abs(max_d) >= v_abs(first_d) + CFG_DIVERGE_PX) && (rows > best)) {
        best = rows;
        best_side = v_sign(sum_d);   /* 2026-11-24 改：整段累加，避免单行噪声翻方向 */
    }
    *side_out = best_side;
    return best;
}

/* --------------------------------------------------------------------------
 * 环岛：看"两条线的间距"轮廓。闭合的圈必然是
 *         间距从很小 → 变大（岛的宽度）→ 又缩回很小（两条边在远处并回）
 *   而岔路是 间距一直变大，或者一直分着走到画面外。
 *   所以判据 = 行数够多 + 最大间距够大 + 间距确实"涨过又落回"。
 * -------------------------------------------------------------------------- */
static int find_island(int *side_out)
{
    int y;
    int active = 0;
    int rows = 0;
    int start_sep = 0;
    int max_sep = 0;
    int tail_sep = 0;
    int converged = 0;
    int best = 0;
    int best_y0 = -1;

    *side_out = 0;

    for (y = V_H - 3; y >= 2; y--) {
        if (s_sep[y] > 0) {
            if (!active) {
                active = 1;
                rows = 0;
                start_sep = s_sep[y];
                max_sep = s_sep[y];
                converged = 0;
                best_y0 = y;
            }
            rows++;
            if (s_sep[y] > max_sep) {
                max_sep = s_sep[y];
            }
            tail_sep = s_sep[y];
            /* 峰值之后又收窄：这是圈在"合回去"*/
            if ((max_sep >= CFG_ISLE_LAST_SEP) &&
                (tail_sep <= (max_sep / 2))) {
                converged = 1;
            }
        } else if (active) {
            if ((rows >= CFG_ISLE_MIN_ROWS) && converged &&
                (max_sep >= CFG_ISLE_LAST_SEP) &&
                (max_sep >= start_sep + CFG_DIVERGE_PX) && (rows > best)) {
                best = rows;
                best_y0 = y + rows;
            }
            active = 0;
        }
    }
    if (active && (rows >= CFG_ISLE_MIN_ROWS) && converged &&
        (max_sep >= CFG_ISLE_LAST_SEP) &&
        (max_sep >= start_sep + CFG_DIVERGE_PX) && (rows > best)) {
        best = rows;
        best_y0 = y + rows;
    }

    /* 进岛方向：越过"老顶点"之后，跟踪点往哪边偏，就走哪一边。
     * 这样车已经选好的那条边被"确认"，不会被强制偏好拽到对岸去横穿岛。 */
    if ((best > 0) && (best_y0 >= 0)) {
        int ref = track_ref_x(best_y0 + 3);
        int y2;

        for (y2 = best_y0; (y2 >= best_y0 - 8) && (y2 >= 0); y2--) {
            if (g_mid_line[y2] >= 0) {
                *side_out = v_sign(g_mid_line[y2] - ref);
                break;
            }
        }
    }
    return best;
}

/* 统一的方向判定（2026-11-24 新增）
 *
 * 为什么需要：
 *   入口1（find_island）用 v_sign(g_mid_line[y] - ref) 算方向，
 *   入口2（find_fork）用 v_sign(sum_d)（s_br 累加）。
 *   两者在同一个环岛能判出相反的方向 ——
 *   实测 isle_test5：11 次入口2 触发全判 R，2 次入口1 全判 L。
 *
 * 算法：优先看"跟踪路径本身弯向哪边"（最可靠）。
 *   1) 取路径顶部往上 8 行里的 g_mid_line
 *   2) 和底部跟踪点 g_bottom_x 比较
 *   3) 平均偏移超过 CFG_ISLE_SIDE_MIN_PX 才算数
 *   4) 路径没弯时退回传入的 fallback
 */
static int unified_island_side(int fallback)
{
    int y;
    int ref;
    int dx = 0;
    int cnt = 0;

    if ((g_path_top < 0) || (g_bottom_x < 0)) {
        return fallback;
    }
    ref = g_bottom_x;

    for (y = g_path_top; (y >= 0) && (y >= (g_path_top - 8)); y--) {
        if (g_mid_line[y] >= 0) {
            dx += (g_mid_line[y] - ref);
            cnt++;
        }
    }

    if (cnt == 0) {
        return fallback;
    }
    dx = dx / cnt;

    if ((dx < CFG_ISLE_SIDE_MIN_PX) && (dx > -CFG_ISLE_SIDE_MIN_PX)) {
        return fallback;
    }
    return (dx > 0) ? 1 : -1;
}

static void island_enter(int side)
{
    /* 2026-11-24: 统一重算方向，优先用路径弯曲方向。
     * 原来直接信 find_island / find_fork 的方向，两者符号可能相反。 */
    side = unified_island_side(side);
    if (side == 0) { side = 1; }    /* 兜底：都判不出时默认右转 */
    g_island_on = 1;
    /* side 的取法：配置里强制（±1）就用配置，否则用"车已经走上的那一边" */
    if (CFG_ISLAND_SIDE != 0) {
        g_island_side = CFG_ISLAND_SIDE;
    } else if (side != 0) {
        g_island_side = side;
    } else {
        g_island_side = 1;
    }
    s_island_frames = 0;
    s_isle_quiet = 0;
    s_island_force = CFG_ISLAND_FORCE;
    g_elem = 3;
    g_elem_side = g_island_side;
    g_prefer_side = g_island_side;
}

static void elem_update_core(void)
{
    int fork_rows;
    int isle_rows;
    int fork_side = 0;
    int isle_side = 0;

    probe_rows();
    fork_rows = find_fork(&fork_side);
    isle_rows = find_island(&isle_side);
    {
        int sep_run = count_sep_run();

        /* ★ 环岛入口：两侧都有线（三条线）连续够多行，就算环岛，
         *   不等"梭形收敛"。这是"有些环岛没进去"的直接修法。 */
        if (sep_run > isle_rows) {
            isle_rows = sep_run;
        }
    }
    g_fork_rows = fork_rows;
    g_isle_rows = isle_rows;
    g_cross_rows = detect_cross();

    if (s_cd > 0) {
        s_cd--;
    }
    if (s_isle_cool > 0) {
        s_isle_cool--;
    }

    /* 元素去抖：单帧误判会让 g_prefer_side 突然打向一边（车抽一下），
     * 所以要求连续几帧都判出来。 */
    if ((g_cross_rows < 2) && (fork_rows >= CFG_ISLE_ENTRY_ROWS)) {
        if (s_fork_hit < 1000) {
            s_fork_hit++;
        }
    } else {
        s_fork_hit = 0;
    }
    // 十字帧中断环岛确认，方向变化时从当前帧重新累计
    if ((g_cross_rows < 2) && (isle_rows >= CFG_ISLE_MIN_ROWS)) {
        if (isle_side != s_isle_candidate_side) {
            s_isle_hit = 0;
            s_isle_candidate_side = isle_side;
        }
        if (s_isle_hit < CFG_ISLE_CONFIRM) {
            s_isle_hit++;
        }
    } else {
        s_isle_hit = 0;
        s_isle_candidate_side = 0;
    }

    g_elem = 0;
    g_elem_side = 0;

    /* ====================== 已在环岛里 ====================== */
    if (g_island_on) {
        g_elem = 3;
        g_elem_side = g_island_side;
        s_island_frames++;

        /* 进岛初期"强制咬边"，把车拉进岛；之后交给连续性跟踪，
         * 免得把岛对岸那条线当成自己的线（那样会横穿岛，罚 30 s）。 */
        /* 整个环岛过程都保持"咬边"。以前只在头十几帧强制，之后放手，
         * 车会骑回主线直着穿过去。配合 find_run_prefer 已改成取最近支线，
         * 全程咬边只是贴着当前这条边，不会去够对岸。 */
        g_prefer_side = g_island_side;
        if (s_island_force > 0) {
            s_island_force--;
        }

        /* 出岛：圈的轮廓消失、并且路径已经稳定地看得很远 */
        if ((isle_rows == 0) && (fork_rows <= 1) &&
            (g_path_rows >= CFG_ISLAND_EXIT_ROWS)) {
            s_isle_quiet++;
        } else {
            s_isle_quiet = 0;
        }

        if (((s_isle_quiet >= CFG_ISLAND_QUIET) &&
             (s_island_frames >= CFG_ISLAND_MIN_FRAME)) ||
            (g_lost_cnt >= 10) ||
            /* 判出十字 = 环岛结构已经没了；多半是上一次误判，不放出来会一直咬边 */
            ((g_cross_rows >= 2) && (s_island_frames >= CFG_ISLAND_MIN_FRAME)) ||
            (s_island_frames > CFG_ISLAND_MAX_FRAME)) {
            g_island_on = 0;
            g_prefer_side = 0;
            g_island_side = 0;
            s_island_frames = 0;
            s_isle_quiet = 0;
            s_island_force = 0;
            s_cd = CFG_ELEM_CD;
            s_isle_cool = CFG_ISLE_COOLDOWN;     /* 出岛冷却，防止连锁误判 */
        }
        return;
    }

    /* ====================== 环岛入口 ====================== */
    /* 十字横臂会被拆成"参考点两侧各一段"，凑够行数就误判环岛；
     * 本帧已经判出十字时一律不许进环岛。 */
    if ((isle_rows >= CFG_ISLE_MIN_ROWS) && (g_cross_rows < 2) && (s_cd == 0) &&
        (s_isle_cool == 0) &&
        (s_isle_hit >= CFG_ISLE_CONFIRM)) {
        island_enter(isle_side);
        return;
    }

    /* ====================== 环岛入口（分叉形态） ======================
     * 预赛没有岔路：一条主线 + 一条越分越开的支线，就是环岛的环口。
     * 这是 VS 仿真里验证过的入口判据（fork_rows >= CFG_ISLE_ENTRY_ROWS），
     * 82 帧上环岛能进、误判 0 帧。回来后车上反而把这条通路删了，
     * 只留 isle_rows（三条线），而实测那个判据在实车上最多只有 11 行，
     * 永远到不了 12 —— 这就是"环岛进不去"的直接原因。
     *
     * 必须先排除十字（g_cross_rows < 2）：十字横臂在竖直主线两侧
     * 也会表现成"主线 + 旁边一条线"，不排除就会把每个十字当环岛，
     * 进去以后 prefer_side 一直咬边，就是"卡在十字里出不来"。
     * ------------------------------------------------------------------ */
    if ((CFG_NO_FORK != 0) && (g_cross_rows < 2) && (s_cd == 0) && (g_island_on == 0) &&
        (s_isle_cool == 0) &&
        (fork_rows >= CFG_ISLE_ENTRY_ROWS) && (s_fork_hit >= CFG_FORK_CONFIRM)) {
        island_enter(fork_side);
        return;
    }

    /* ====================== 十字 ====================== */
    if (g_cross_rows >= 2) {
        g_elem = 1;
        g_elem_side = 0;
        g_prefer_side = 0;
        return;
    }

    /* ====================== 普通赛道 ====================== */
    g_elem = 0;
    g_elem_side = 0;
    g_prefer_side = 0;
}

/* ==========================================================================
 *  状态机（2026-11-20 新增）
 *  --------------------------------------------------------------------------
 *  把"检测结果"翻译成控制层能直接用的三样东西：
 *      g_state      当前处在哪种元素里
 *      g_speed_pct  建议速度（元素里必须减速：2 m/s 下差速打满也转不过来）
 *
 *  每个状态都有超时：元素判错时最怕"卡在里面出不来"，超时强制回普通。
 * ========================================================================== */
static void state_update(void)
{
    int st    = 0;
    int target;

    /* ---- 直角弯判据去抖 ----
     *
     * 实测（直道 84s，CRUIS=1000）：
     *   cdir 有 169 帧非零（+1:145, -1:24），全部是误报；
     *   每次误报都把 speed_pct 从 100 砍到 70，
     *   造成 48 次 ±300 RPM 的两轮同向跳变 ="前后顿"。
     *
     * 真直角弯会连续几十帧成立，零星误报只有 1~2 帧，
     * 所以要求"连续 CFG_CORNER_CONFIRM 帧同方向"。 */
    if (g_corner_dir != 0) {
        if (g_corner_dir == s_corner_dir_last) {
            if (s_corner_frames < 1000) {
                s_corner_frames ++;
            }
        } else {
            s_corner_dir_last = g_corner_dir;
            s_corner_frames   = 1;
        }
    } else {
        s_corner_dir_last = 0;
        s_corner_frames   = 0;
    }

    if (g_island_on) {
        st = 4;
    } else if (g_elem == 1) {
        st = 2;
    } else if (s_corner_frames >= CFG_CORNER_CONFIRM) {
        /* 连续多帧确认才进直角弯状态 */
        st = 1;
    }

    if (st != g_state) {
        g_state = st;
        g_state_frames = 0;
    } else if (g_state_frames < 100000) {
        g_state_frames++;
    }

    /* 超时兜底：卡住就强制回普通状态 */
    if (((g_state == 1) && (g_state_frames > CFG_STATE_MAX_CORNER)) ||
        ((g_state == 2) && (g_state_frames > CFG_STATE_MAX_CROSS)) ||
        ((g_state == 4) && (g_state_frames > CFG_STATE_MAX_ISLAND))) {
        g_state = 0;
        g_state_frames = 0;
        g_island_on = 0;
        g_island_side = 0;
        g_prefer_side = 0;
        s_cd = CFG_ELEM_CD;
    }

    /* 元素里必须减速：2 m/s 下差速打满也转不过来。
     * 具体系数见 vision_cfg.h 的 CFG_SPD_*。 */
    switch (g_state) {
    case 1:  target = CFG_SPD_CORNER; break;
    case 2:  target = CFG_SPD_CROSS;  break;
    case 4:  target = CFG_SPD_ISLAND; break;
    default: target = 100;            break;
    }

    /* ---- 速度建议渐变 ----
     *
     * 即使 st 偶尔抖一下，也不让 speed_pct 一步跳 30%。
     * 每帧最多变 CFG_SPD_SLEW 个百分点，100->70 需要 15 帧 = 300ms。
     * 双轮同向目标突变会让速度环跟不上，表现就是"顿一下"；
     * 渐变让速度环有时间跟随。 */
#if (CFG_SPD_SLEW > 0)
    if (target > g_speed_pct) {
        g_speed_pct += CFG_SPD_SLEW;
        if (g_speed_pct > target) {
            g_speed_pct = target;
        }
    } else if (target < g_speed_pct) {
        g_speed_pct -= CFG_SPD_SLEW;
        if (g_speed_pct < target) {
            g_speed_pct = target;
        }
    }
#else
    g_speed_pct = target;
#endif
}

void elem_update(void)
{
    elem_update_core();
    state_update();
}
