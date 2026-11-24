# -*- coding: utf-8 -*-
"""重写 vision_element.c

改动:
  1. 删除 find_fork() / probe_rows() / s_sep / s_br —— 岔路是遗留逻辑，
     2025 赛道没有 Y 形分岔，只会误报环岛入口和钝角弯
  2. 环岛判定一律走 v2e_detect_island()（v2_elem.c，已改成 v5 算法）
  3. elem_update() 简化为: 环岛 > 十字
"""

OUT = 'code/vision/vision_element.c'

NEW = r'''/* ==========================================================================
 *  vision_element.c —— 元素识别（十字 / 环岛）
 *
 * 2026-11-19 重构:
 *   - 删除岔路(find_fork)。2025 缩微光电赛道没有 Y 形分岔，
 *     而 find_fork 的判据"支线越离越远"会被环岛入口和 120/135/150 度
 *     钝角弯触发，全是误报。钝角弯本来就能靠爬线跟过去，不需要单独判。
 *   - 环岛判定走 v2_elem.c 的 v2e_detect_island()（双白段间距发散）。
 *   - 保留十字检测。
 * ========================================================================== */
#include "vision.h"
#include "v2_elem.h"

int g_elem = 0;
int g_elem_side = 0;
int g_prefer_side = 0;
int g_island_on = 0;
int g_island_side = 0;
int g_lap = 1;
int g_fork_pass = 0;
int g_fork_use_branch = 0;
int g_fork_rows = 0;        /* 保留变量名，恒为 0，其他文件还在引用 */
int g_isle_rows = 0;
int g_cross_rows = 0;

static int s_cd = 0;                /* 元素冷却帧数 */
static int s_isle_quiet = 0;        /* 双线消失的连续帧数 */
static int s_island_frames = 0;     /* 在环岛里的帧数 */
static int s_island_force = 0;      /* 还需要"强制咬边"的帧数 */

static int v_abs(int v)
{
    return (v < 0) ? -v : v;
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
}

/* --------------------------------------------------------------------------
 * 十字：横向长白贯穿中线
 *   一条特别长的白段（>= CFG_CROSS_W）横穿参考线，且上下都有赛道线
 * -------------------------------------------------------------------------- */
static int detect_cross(void)
{
    int y, i, rows = 0;

    for (y = CFG_ELEM_TOP_Y; y < V_H - 5; y++) {
        int ref = track_ref_x(y);

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
 * 元素主入口
 *
 * 环岛用锁存状态机:
 *   触发一次 -> g_island_on = 1, 前 CFG_ISLAND_FORCE 帧强制咬内侧
 *   之后交给爬线自己跟
 *   双线连续 CFG_ISLAND_QUIET 帧消失 且 路径够长 -> 退出
 *
 * 宁可多触发一次，不可漏检: 漏检 = 车冲进环岛不转，直接出界。
 * -------------------------------------------------------------------------- */
void elem_update(void)
{
    int isle_rows = 0;
    int isle_side = 0;

    isle_side = v2e_detect_island();
    isle_rows = (isle_side != 0) ? CFG_ISLE_MIN_ROWS : 0;

    g_isle_rows = isle_rows;
    g_fork_rows = 0;                        /* 岔路已废弃 */
    g_cross_rows = detect_cross();

    if (s_cd > 0) {
        s_cd--;
    }

    g_elem = 0;
    g_elem_side = 0;

    /* ====================== 环岛进行中 ====================== */
    if (g_island_on) {
        g_elem = 3;
        g_elem_side = g_island_side;
        s_island_frames++;

        /* 前几帧强制咬内侧。
         * 咬太久会把车拽到岛的外边界上，之后交给爬线自己跟。 */
        if (s_island_force > 0) {
            s_island_force--;
            g_prefer_side = g_island_side;
        } else {
            g_prefer_side = 0;
        }

        if ((isle_rows == 0) && (g_path_rows >= CFG_ISLAND_EXIT_ROWS)) {
            s_isle_quiet++;
        } else {
            s_isle_quiet = 0;
        }

        if (((s_isle_quiet >= CFG_ISLAND_QUIET) &&
             (s_island_frames >= CFG_ISLAND_MIN_FRAME)) ||
            (g_lost_cnt >= 10) ||
            (s_island_frames > CFG_ISLAND_MAX_FRAME)) {
            g_island_on = 0;
            g_prefer_side = 0;
            g_island_side = 0;
            s_island_frames = 0;
            s_isle_quiet = 0;
            s_island_force = 0;
            s_cd = CFG_ELEM_CD;
        }
        return;
    }

    /* ====================== 环岛触发 ====================== */
    if ((isle_rows >= CFG_ISLE_MIN_ROWS) && (s_cd == 0)) {
        g_island_on = 1;

        if (CFG_ISLAND_SIDE != 0) {
            g_island_side = CFG_ISLAND_SIDE;
        } else if (isle_side != 0) {
            g_island_side = isle_side;
        } else {
            g_island_side = 1;
        }

        s_island_frames = 0;
        s_isle_quiet = 0;
        s_island_force = CFG_ISLAND_FORCE;
        g_elem = 3;
        g_elem_side = g_island_side;
        g_prefer_side = g_island_side;
        return;
    }

    /* ====================== 十字 ====================== */
    if ((g_cross_rows >= 2) && (g_corner_dir == 0)) {
        g_elem = 1;
        g_elem_side = 0;
        g_prefer_side = 0;
        return;
    }

    /* ====================== 普通 ====================== */
    g_elem = 0;
    g_elem_side = 0;
    g_prefer_side = 0;
}
'''

blob = NEW.encode('gbk')
assert len(blob) > 3000
open(OUT, 'wb').write(blob)
print('vision_element.c 重写完成: %d 字节' % len(blob))
