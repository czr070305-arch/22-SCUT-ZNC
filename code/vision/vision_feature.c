/* ==========================================================================
 *  vision_feature.c --- 逐行特征提取
 *
 *  从 vision_element.c 拆出来的一层（2026-11-25）。
 *  这一层只做一件事：看图，把每一行白段的几何关系翻译成两个数组，
 *  不做任何元素判定。
 *
 *    s_sep[y]  同一条路上两段白之间的黑缝宽度（只有一段白时为 0）
 *    s_br[y]   支线相对参考线的偏移（无支线时为 0）
 *
 *  为什么单独拆：
 *    这两个函数是整个元素识别的数据源，占了 vision_element.c 一半体积，
 *    而且与状态机完全无关。拆开之后改元素判据不会碰到特征提取。
 * ========================================================================== */
#include "vision.h"
#include "vision_element.h"

/* 逐行特征缓存（原先定义在 vision_element.c） */
int s_sep[V_H];
int s_br[V_H];

/* --------------------------------------------------------------------------
 * 第一遍：把每一行的"形态"量化
 *   s_sep[y] = 最近两段窄白之间的间距（只有一段白时为 0）
 *   s_br[y]  = 支线相对参考点的偏移（无支线时为 0）
 * 单线的赛道上，绝大多数行 s_sep=0、s_br=0；只有元素处会出现非零。
 * -------------------------------------------------------------------------- */
void feat_probe_rows(void)
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
        ref = feat_path_x_at(y);

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
 * 某行的"参考 x"：优先用跟踪点；没有就【线性插值】上下两行，
 * 而不是拿顶部斜率一路外推。
 *
 * ★ 2026-11-20：外推在"路径只剩几行"时会把参考点甩到画面外几十像素，
 *   于是顶部的白板/格纹随便一段都能落进 11~42px 的"支线"窗口里，
 *   假岔路、假环岛基本都是这么来的。
 * -------------------------------------------------------------------------- */
int feat_path_x_at(int y)
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

int v_abs(int v)
{
    return (v < 0) ? -v : v;
}

int v_sign(int v)
{
    if (v > 0) {
        return 1;
    }
    if (v < 0) {
        return -1;
    }
    return 0;
}
