# -*- coding: utf-8 -*-
"""安全替换 v2_elem.c 的 v2e_detect_island()

写法要点（上次把文件写成 0 字节的教训）:
  1. 先构造好完整的字节内容
  2. 编码成功之后再 open(..., 'wb')
  3. 写之前校验非空
"""

OUT = 'code/vision/v2_elem.c'

NEW_FUNC = r'''/*
 * 环岛识别：双白段间距发散
 *   返回 -1 左环岛 / +1 右环岛 / 0 无
 *
 * 2025 缩微光电赛道只有一个六边形环岛，进入时画面特征是：
 *   车所在的赛道线继续往上，
 *   旁边出现第二条线（岛的外边界），
 *   两条线的间距从下往上持续变大。
 *
 * 与其它元素的区别：
 *   直角弯  只有一条线（横臂），没有第二段
 *   分叉   两条线对称分开，间距不单调发散
 *   房间杂物（窗帘杆/桌沿/路由器）间距忽大忽小，没有趋势
 *
 * 原版（20 届学长的射线投票）在本赛道全部漏检且误报 3 张，
 * 因为那套参数是给别的前瞻距离和别的高度标定的。
 *
 * y 坐标方向：y 越小 = 画面越靠上 = 离车越远，
 * 所以【顶部几行间距 应该大于 底部几行】。
 */
int v2e_detect_island(void)
{
    int y;
    int n = 0;
    int ys[V_H];
    int seps[V_H];
    int side[V_H];
    int sorted[V_H];
    int n_mid = 0;
    int ref;
    int y_hi;
    int k;
    int top_sep = 0;
    int bot_sep = 0;
    int n_right = 0;
    int n_left;
    int i, j, v;

    if (g_path_rows < 20) { return 0; }
    if (g_path_top < 0)   { return 0; }

    /* ---- 1) 参考线 = 图像中下部爬线中点的中位数 ---- */
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

    /* ---- 2) 逐行找离参考线最近的两段窄白段 ---- */
    y_hi = g_path_top + 15;
    if (y_hi > V_H - 4) { y_hi = V_H - 4; }

    for (y = V2E_TOP_CUT; y <= y_hi; y++) {
        int d1 = 9999, c1 = 0;
        int d2 = 9999, c2 = 0;
        int rw = 0;
        int ww;

        /* 满行（十字/横向长白）不能当双段判。
         * s_row_white 是 vision_track.c 的 static，这里访问不到，
         * 就地累加 g_runs 的宽度。 */
        for (ww = 0; ww < g_run_num[y]; ww++) {
            rw += g_runs[y][ww].r - g_runs[y][ww].l + 1;
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
            ys[n]   = y;
            seps[n] = sep;
            side[n] = (c2 > c1) ? 1 : -1;
            n++;
            if (n >= V_H) { break; }
        }
    }

    if (n < V2E_ISLE_MIN_ROWS) { return 0; }

    /* ---- 3) 双段行的中位高度必须在画面中下部 ----
     * 顶部全是房间杂物（窗帘杆/桌沿/路由器），环岛边界一定在跑道上。 */
    for (i = 1; i < n; i++) {
        v = ys[i];
        for (j = i - 1; (j >= 0) && (ys[j] > v); j--) {
            ys[j + 1] = ys[j];
        }
        ys[j + 1] = v;
    }
    if (ys[n / 2] < V2E_ISLE_MIN_Y) { return 0; }

    /* ---- 4) 间距必须发散：上面 > 下面 ---- */
    k = n / 3;
    if (k < 1) { k = 1; }
    for (i = 0; i < k; i++) {
        top_sep += seps[i];
        bot_sep += seps[n - 1 - i];
    }
    if (bot_sep < 1) { return 0; }
    /* top/bot >= 1.25，用整数比较避免浮点 */
    if (top_sep * 100 < bot_sep * 125) { return 0; }

    /* ---- 5) 第二条线必须稳定在同一侧 ---- */
    for (i = 0; i < n; i++) {
        if (side[i] > 0) { n_right++; }
    }
    n_left = n - n_right;

    if (n_right * 10 >= n * 7) { return 1; }
    if (n_left  * 10 >= n * 7) { return -1; }
    return 0;
}
'''

# ---------- 读取 ----------
src = open(OUT, 'rb').read().decode('gbk')
lines = src.split('\r\n')

# ---------- 定位函数（含前面的块注释） ----------
func_start = None
for i, l in enumerate(lines):
    if l.startswith('int v2e_detect_island(void)'):
        func_start = i
        break
if func_start is None:
    raise SystemExit('没找到 v2e_detect_island')

# 往前吃掉块注释
cstart = func_start
while cstart > 0:
    prev = lines[cstart - 1].strip()
    if prev.startswith('*') or prev.startswith('/*'):
        cstart -= 1
    else:
        break

# 往后找函数结尾的 '}'
func_end = None
for i in range(func_start, len(lines)):
    if lines[i] == '}':
        func_end = i
        break
if func_end is None:
    raise SystemExit('没找到函数结尾')

print('替换区间: 第 %d ~ %d 行' % (cstart + 1, func_end + 1))

# ---------- 先编码，成功了再写 ----------
new_lines = NEW_FUNC.split('\n')
new_body = '\r\n'.join(lines[:cstart] + new_lines + lines[func_end + 1:])
try:
    blob = new_body.encode('gbk')
except UnicodeEncodeError as e:
    raise SystemExit('GBK 编码失败（未写入）: %s' % e)

if len(blob) < 5000:
    raise SystemExit('生成的内容太小 (%d 字节)，放弃写入' % len(blob))

open(OUT, 'wb').write(blob)
print('写入成功: %d 字节' % len(blob))
print('v2e_iabs 未定义?', 'static int v2e_iabs' not in new_body)
