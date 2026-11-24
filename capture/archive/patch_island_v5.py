# -*- coding: utf-8 -*-
"""把 v2e_detect_island() 换成 v5 算法

v5 相比旧版的三处改动:
  1. 扫描范围放宽到 V_H-4（原来只扫到 path_top+15，
     环岛斜臂在画面更下方时会漏检，seq003 就是这样漏掉的）
  2. 双段行必须【连续】（相邻行差 <= 2），且连续段 >= 5 行
     —— 钝角弯+十字的复合场景（seq035）双段行是断开的
  3. 参考线、发散比、左右占比的判据保持原样
"""

OUT = 'code/vision/v2_elem.c'

NEW_FUNC = r'''/*
 * 环岛识别：双白段间距发散（v5）
 *   返回 -1 左环岛 / +1 右环岛 / 0 无
 *
 * 2025 缩微光电赛道只有一个六边形环岛。进入时画面特征:
 *   车所在的赛道线继续往上，旁边出现岛的边界线，
 *   两条线的间距从下往上【持续变大】。
 *
 * 与其它元素的区别:
 *   直角弯  只有一条线（横臂），没有第二段
 *   钝角弯  第二条线出现几行就消失，间距不单调
 *   十字    两条线会【交叉】，间距先收窄再张开
 *   房间杂物（窗帘杆/桌沿/路由器）间距忽大忽小，没有趋势
 *
 * 实测标注（2026-11-19 人工确认）:
 *   seq002 / seq003 / seq013 是环岛
 *   seq035（钝角弯+十字）/ seq061 / seq063 不是
 *
 * 关键判据:
 *   1. 双段行必须【连续】（相邻行差 <= 2），且连续段 >= 5 行。
 *      钝角弯的"第二条线"往往断断续续，凑不出连续段。
 *   2. 连续段内，间距从下往上单调变大（发散比 >= 1.25）。
 *   3. 第二条线要稳定在同一侧（>= 70%）。
 *
 * y 坐标方向: y 越小 = 画面越靠上 = 离车越远。
 * 所以【顶部几行间距 应该大于 底部几行】。
 */
int v2e_detect_island(void)
{
    int y;
    int n_mid = 0;
    int ref;
    int i, j, v;

    int row_sep[V_H];       /* 该行的双段间距, 0 = 该行没有双段 */
    int row_side[V_H];      /* 第二条线在哪一侧 */

    int best_lo = -1;
    int best_len = 0;
    int cur_lo = -1;
    int cur_len = 0;

    int n;
    int k;
    int top_sep = 0;
    int bot_sep = 0;
    int n_right = 0;
    int n_left;

    if (g_path_rows < 20) { return 0; }
    if (g_path_top < 0)   { return 0; }

    /* ---- 1) 参考线 = 图像中下部爬线中点的中位数 ---- */
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

    /* ---- 2) 逐行找"离参考线最近的两段窄白段" ---- */
    for (y = 0; y < V_H; y++) {
        row_sep[y]  = 0;
        row_side[y] = 0;
    }

    /* 扫描范围放宽到画面底部:
     * 环岛的斜臂可能出现在爬线顶部【下方】很远的位置。 */
    for (y = V2E_TOP_CUT; y < V_H - 4; y++) {
        int d1 = 9999, c1 = 0;
        int d2 = 9999, c2 = 0;
        int rw = 0;

        /* 满行（十字/横向长白）不能当双段判。
         * s_row_white 是 vision_track.c 的 static，这里访问不到，
         * 就地累加 g_runs 的宽度。 */
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
            d = v2e_iabs(c - ref);
            if (d < d1) {
                d2 = d1; c2 = c1;
                d1 = d;  c1 = c;
            } else if (d < d2) {
                d2 = d;  c2 = c;
            }
        }
        if (d1 > V2E_ISLE_REF_TOL) { continue; }

        {
            int sep = v2e_iabs(c2 - c1);

            if ((sep < V2E_ISLE_SEP_MIN) || (sep > V2E_ISLE_SEP_MAX)) {
                continue;
            }
            row_sep[y]  = sep;
            row_side[y] = (c2 > c1) ? 1 : -1;
        }
    }

    /* ---- 3) 找最长的【连续】双段行段 ----
     * 允许跳 1 行（相邻行差 <= 2），因为二值化偶尔会在一两行上断开。
     * 但跳更多就说明读的是两个不同物体，不能算。 */
    for (y = V2E_TOP_CUT; y < V_H - 4; y++) {
        if (row_sep[y] > 0) {
            if (cur_len == 0) { cur_lo = y; }
            cur_len++;
            if (cur_len > best_len) {
                best_len = cur_len;
                best_lo  = cur_lo;
            }
        } else {
            /* 允许跳 1 行: 这一行没有，但下一行有，就不重置 */
            if ((y + 1 < V_H - 4) && (row_sep[y + 1] > 0)) {
                continue;
            }
            cur_len = 0;
        }
    }

    if (best_len < V2E_ISLE_MIN_ROWS) { return 0; }

    /* ---- 4) 连续段内统计发散比和左右占比 ---- */
    n = 0;
    for (y = best_lo; (y < best_lo + best_len + 2) && (y < V_H - 4); y++) {
        if (row_sep[y] > 0) { n++; }
    }
    if (n < V2E_ISLE_MIN_ROWS) { return 0; }

    k = n / 3;
    if (k < 1) { k = 1; }

    {
        int cnt = 0;

        /* 顶部 k 行（y 小 = 远） */
        for (y = best_lo; (y < V_H - 4) && (cnt < k); y++) {
            if (row_sep[y] > 0) {
                top_sep += row_sep[y];
                cnt++;
            }
        }
        /* 底部 k 行 */
        cnt = 0;
        for (y = best_lo + best_len + 1; (y >= best_lo) && (cnt < k); y--) {
            if ((y < V_H - 4) && (row_sep[y] > 0)) {
                bot_sep += row_sep[y];
                cnt++;
            }
        }
    }

    if (bot_sep < 1) { return 0; }
    /* top/bot >= 1.25，用整数比较避免浮点 */
    if (top_sep * 100 < bot_sep * 125) { return 0; }

    /* ---- 5) 第二条线必须稳定在同一侧 ---- */
    for (y = best_lo; (y < best_lo + best_len + 2) && (y < V_H - 4); y++) {
        if (row_sep[y] > 0) {
            if (row_side[y] > 0) { n_right++; }
        }
    }
    n_left = n - n_right;

    if (n_right * 10 >= n * 7) { return 1; }
    if (n_left  * 10 >= n * 7) { return -1; }
    return 0;
}
'''

src = open(OUT, 'rb').read().decode('gbk')
lines = src.split('\r\n')

# 找函数起止
start = None
for i, l in enumerate(lines):
    if l.startswith('int v2e_detect_island(void)'):
        start = i
        break
if start is None:
    raise SystemExit('没找到 v2e_detect_island')

# 往前吃掉块注释
cstart = start
while cstart > 0:
    prev = lines[cstart - 1].strip()
    if prev.startswith('*') or prev.startswith('/*'):
        cstart -= 1
    else:
        break

end = None
for i in range(start, len(lines)):
    if lines[i] == '}':
        end = i
        break
if end is None:
    raise SystemExit('没找到函数结尾')

print('替换 %d ~ %d 行' % (cstart + 1, end + 1))

body = '\r\n'.join(lines[:cstart] + NEW_FUNC.split('\n') + lines[end + 1:])
blob = body.encode('gbk')
assert len(blob) > 5000
open(OUT, 'wb').write(blob)
print('写入成功: %d 字节' % len(blob))
