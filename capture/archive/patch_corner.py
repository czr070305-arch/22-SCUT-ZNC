# -*- coding: utf-8 -*-
"""在 vision_track.c 里加入直角弯后处理

做法（按教训）:
  先构造完整字节 -> 编码成功 -> 校验长度 -> 才 open('wb')
"""

OUT = 'code/vision/vision_track.c'

POST_FUNC = r'''
/* --------------------------------------------------------------------------
 * 直角弯后处理：宽度剖面法
 *
 * 为什么不用"逐行判宽段":
 *   直角弯的横臂在鱼眼边缘会被压窄（seq006 组最大只有 w48），
 *   而且横臂上方往往还有东西（赛道线的另一段 / 房间杂物），
 *   逐行的 "above_ok" 判据会被这两件事同时干扰。
 *
 * 改用【整条爬线的宽度剖面】:
 *   1. 取每一行覆盖中线的那段的宽度
 *   2. 参考线宽 = 下部 60% 的中位数（那里一定是正常赛道线）
 *   3. 找最宽的那一行
 *   4. 最宽 / 参考 >= 2.5 就是直角弯的横臂
 *   5. 方向 = 最宽行里离参考线更远的那一端
 *
 * 实测宽度比（capture/report.py 的数据）:
 *   左直角 6.0x   右直角 8.1x
 *   直道   1.1~1.6x   分叉 1.2~1.5x   环岛 1.6~1.8x
 * 2.5 这个门槛把两类清清楚楚分开了。
 *
 * 画面顶部 CFG_TOP_CUT 行必须跳过：那里是房间杂物
 * （窗帘杆 / 桌沿 / 路由器），会把宽度比虚抬到 4x 以上，
 * 在直道图上误报成直角弯。
 * -------------------------------------------------------------------------- */
static void post_detect_corner(void)
{
    int y, i;
    int n = 0;
    int med;
    int ratio_x10;
    int wide_y = -1, wide_w = 0, wide_l = 0, wide_r = 0;
    int ref = -1;
    int dl, dr, far;
    int cnt[V_H];

    if (g_path_rows < CFG_CORNER_MIN_ROWS) { return; }

    /* ---- 1) 收集宽度剖面（跳过顶部杂物区） ---- */
    for (y = CFG_TOP_CUT; (y < V_H) && (n < V_H); y++) {
        int w = 0, l = 0, r = -1;

        if (g_mid_line[y] < 0) { continue; }
        for (i = 0; i < g_run_num[y]; i++) {
            if ((g_runs[y][i].l <= g_mid_line[y]) &&
                (g_mid_line[y] <= g_runs[y][i].r)) {
                l = g_runs[y][i].l;
                r = g_runs[y][i].r;
                w = r - l + 1;
                break;
            }
        }
        if (w <= 0) { continue; }
        cnt[n] = y;

        if (w > wide_w) {
            wide_w = w;
            wide_y = y;
            wide_l = l;
            wide_r = r;
        }
        n++;
    }
    if (n < 20) { return; }
    if (wide_w < 20) { return; }

    /* ---- 2) 参考线宽 = 后 60% 的中位数 ----
     * cnt[] 按 y 升序（从顶到底），后 60% 就是画面下部。 */
    {
        int lo = n * 4 / 10;
        int m = n - lo;
        int j, k;
        int tmp[(V_H)];

        if (m < 5) { return; }
        for (j = 0; j < m; j++) {
            int yy = cnt[lo + j];
            int w2 = 0;
            for (i = 0; i < g_run_num[yy]; i++) {
                if ((g_runs[yy][i].l <= g_mid_line[yy]) &&
                    (g_mid_line[yy] <= g_runs[yy][i].r)) {
                    w2 = g_runs[yy][i].r - g_runs[yy][i].l + 1;
                    break;
                }
            }
            tmp[j] = w2;
        }
        /* 插入排序 */
        for (j = 1; j < m; j++) {
            int v = tmp[j];
            for (k = j - 1; (k >= 0) && (tmp[k] > v); k--) {
                tmp[k + 1] = tmp[k];
            }
            tmp[k + 1] = v;
        }
        med = tmp[m / 2];
    }
    if (med < 2) { med = 2; }

    /* ---- 3) 宽度比门槛（用整数比较避免浮点） ---- */
    ratio_x10 = wide_w * 10 / med;
    if (ratio_x10 < CFG_CORNER_RATIO_X10) { return; }

    /* ---- 4) 方向：最宽行【下方】的中线作参考 ---- */
    for (y = wide_y + 3; y < V_H; y++) {
        if (g_mid_line[y] >= 0) {
            ref = g_mid_line[y];
            break;
        }
    }
    if (ref < 0) { ref = V_W / 2; }

    dl = iabs(wide_l - ref);
    dr = iabs(wide_r - ref);
    far = (dl > dr) ? wide_l : wide_r;
    if (far == ref) { return; }

    g_corner_dir = (far > ref) ? 1 : -1;
    g_corner_far_x = far;
}
'''

src = open(OUT, 'rb').read().decode('gbk')
lines = src.split('\r\n')

# 在 track_build 的 v2e_detect_corner() 调用之前插入函数定义
# 函数必须是文件级的，所以插在 track_build 定义之前
anchor = None
for i, l in enumerate(lines):
    if l.startswith('int track_build(int prefer_side)'):
        anchor = i
        break
if anchor is None:
    raise SystemExit('没找到 track_build')

# 往前找，插到它前面的注释块之前
ins = anchor
while ins > 0 and (lines[ins - 1].strip().startswith('*') or
                   lines[ins - 1].strip().startswith('/*') or
                   lines[ins - 1].strip() == ''):
    ins -= 1

new_lines = POST_FUNC.split('\n')

# 替换 v2e_detect_corner() 调用为 后处理 + v2e
src2 = '\r\n'.join(lines[:ins] + new_lines + lines[ins:])
src2 = src2.replace('    v2e_detect_corner();',
                    '    post_detect_corner();       /* 宽度剖面判直角弯 */\r\n'
                    '    v2e_detect_corner();', 1)

blob = src2.encode('gbk')
assert len(blob) > 20000, '内容异常小: %d' % len(blob)
open(OUT, 'wb').write(blob)
print('写入成功: %d 字节' % len(blob))
