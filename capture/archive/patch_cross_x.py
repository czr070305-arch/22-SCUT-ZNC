# -*- coding: utf-8 -*-
"""在 vision_element.c 里加入 X 形十字检测，并在 elem_update 中调用"""

P = 'code/vision/vision_element.c'
src = open(P, 'rb').read().decode('gbk')
NL = '\r\n'
lines = src.split(NL)

X_FUNC = '''/* --------------------------------------------------------------------------
 * X 形十字：两条斜线交叉
 *
 * 2025 赛道上除了 "+" 形十字，还有两条斜线交叉的 X 形（实测 seq035/036/045）。
 *
 * 判据（与环岛正好相反）:
 *   环岛: 双线间距从下往上【发散】，且连续段很长（>= 15 行）
 *   X 形: 双线间距从下往上【收敛】，连续段较短（5~14 行）
 *
 * 实测数据:
 *   X  形 seq035: 7 行  top=21 bot=7   ratio=2.90
 *   X  形 seq036: 9 行  top=52 bot=35  ratio=1.50
 *   X  形 seq045: 14 行 top=58 bot=31  ratio=1.89
 *   环岛 seq002: 29 行  top=25 bot=11  ratio=2.23
 *   环岛 seq003: 48 行  top=32 bot=15  ratio=2.19
 * 行数 15 是这两类之间最干净的分界。
 * -------------------------------------------------------------------------- */
static int detect_cross_x(void)
{
    static int row_sep[V_H];
    int n_mid = 0;
    int ref;
    int y, i, j, v;
    int best_lo = -1, best_len = 0;
    int cur_lo = -1, cur_len = 0;
    int n, k, seg_lo, seg_hi;
    int top_sep = 0, bot_sep = 0;

    if (g_path_rows < 15) { return 0; }

    /* 参考线 = 中下部中点中位数 */
    {
        int sorted[V_H];

        for (y = (V_H * 45) / 100; y < V_H; y++) {
            if (g_mid_line[y] >= 0) {
                sorted[n_mid++] = g_mid_line[y];
            }
        }
        if (n_mid < 6) { return 0; }
        for (i = 1; i < n_mid; i++) {
            v = sorted[i];
            for (j = i - 1; (j >= 0) && (sorted[j] > v); j--) {
                sorted[j + 1] = sorted[j];
            }
            sorted[j + 1] = v;
        }
        ref = sorted[n_mid / 2];
    }

    /* 逐行找最近的两段窄白段 */
    for (y = 0; y < V_H; y++) { row_sep[y] = 0; }

    for (y = CFG_TOP_CUT; y < V_H - 4; y++) {
        int d1 = 9999, c1 = 0;
        int d2 = 9999, c2 = 0;
        int rw = 0;

        for (i = 0; i < g_run_num[y]; i++) {
            rw += g_runs[y][i].r - g_runs[y][i].l + 1;
        }
        if (rw > CFG_ROW_WHITE_MAX) { continue; }

        for (i = 0; i < g_run_num[y]; i++) {
            int w = g_runs[y][i].r - g_runs[y][i].l + 1;
            int c, d;

            if ((w < CFG_RUN_W_MIN) || (w > CFG_RUN_W_MAX_NARROW)) {
                continue;
            }
            c = (g_runs[y][i].l + g_runs[y][i].r) / 2;
            d = iabs(c - ref);
            if (d < d1) {
                d2 = d1; c2 = c1;
                d1 = d;  c1 = c;
            } else if (d < d2) {
                d2 = d;  c2 = c;
            }
        }
        if (d1 > 30) { continue; }

        {
            int sep = iabs(c2 - c1);

            if ((sep < CFG_CROSS_X_SEP_MIN) || (sep > CFG_CROSS_X_SEP_MAX)) {
                continue;
            }
            row_sep[y] = sep;
        }
    }

    /* 找最长连续段 */
    for (y = CFG_TOP_CUT; y < V_H - 4; y++) {
        if (row_sep[y] > 0) {
            if (cur_len == 0) { cur_lo = y; }
            cur_len++;
            if (cur_len > best_len) {
                best_len = cur_len;
                best_lo  = cur_lo;
            }
        } else {
            if ((y + 1 < V_H - 4) && (row_sep[y + 1] > 0)) { continue; }
            cur_len = 0;
        }
    }

    /* X 形的关键：连续段短。环岛 >= 15 行，X 形 5~14 行 */
    if (best_len < 5)              { return 0; }
    if (best_len >= CFG_CROSS_X_MAX_RUN) { return 0; }

    seg_lo = best_lo;
    seg_hi = best_lo + best_len;
    if (seg_hi > V_H - 4) { seg_hi = V_H - 4; }

    n = 0;
    for (y = seg_lo; y <= seg_hi; y++) {
        if (row_sep[y] > 0) { n++; }
    }
    if (n < 5) { return 0; }

    k = n / 3;
    if (k < 1) { k = 1; }

    {
        int cnt = 0;

        for (y = seg_lo; (y <= seg_hi) && (cnt < k); y++) {
            if (row_sep[y] > 0) { top_sep += row_sep[y]; cnt++; }
        }
        cnt = 0;
        for (y = seg_hi; (y >= seg_lo) && (cnt < k); y--) {
            if (row_sep[y] > 0) { bot_sep += row_sep[y]; cnt++; }
        }
    }

    if (bot_sep < 1) { return 0; }

    /* 收敛：顶部间距 / 底部间距 >= 1.3 */
    if (top_sep * 100 < bot_sep * 130) { return 0; }

    return n;
}
'''

# 在 detect_cross 之前插入
anchor = 'static int detect_cross(void)'
idx = None
for i, l in enumerate(lines):
    if l.startswith(anchor):
        idx = i
        break
if idx is None:
    raise SystemExit('anchor 未找到')

# 往前找块注释起点
ins = idx
while ins > 0:
    prev = lines[ins - 1].strip()
    if prev.startswith('*') or prev.startswith('/*'):
        ins -= 1
    else:
        break

lines[ins:ins] = X_FUNC.split('\n') + ['']

# elem_update 里加调用
src2 = NL.join(lines)
old = '''    g_isle_rows = isle_rows;
    g_fork_rows = 0;                        /* 岔路已废弃 */
    g_cross_rows = detect_cross();'''
new = '''    g_isle_rows = isle_rows;
    g_fork_rows = 0;                        /* 岔路已废弃 */
    /* 十字两条路: X 形（斜线交叉）优先，再试 + 形（横贯长白） */
    g_cross_rows = detect_cross_x();
    if (g_cross_rows == 0) {
        g_cross_rows = detect_cross();
    }'''
if old not in src2:
    raise SystemExit('elem_update 锚点未找到')
src2 = src2.replace(old, new, 1)

blob = src2.encode('gbk')
assert len(blob) > 4000
open(P, 'wb').write(blob)
print('vision_element.c 已加入 X 形十字: %d 字节' % len(blob))
