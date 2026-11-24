# -*- coding: utf-8 -*-
"""步骤 1: post_detect_corner 回退到"宽度剖面 v1"
（即用户说"直角弯识别还行，但十字会被识别成 L/R"时的版本）

那一版的特征:
  - 宽度剖面法：遍历爬线覆盖的行，取最宽段 / 中位宽度
  - 门槛 CFG_CORNER_RATIO_X10 = 25  (2.5 倍)
  - 没有十字过滤（ARM_MIN）
  - 没有 v3 的几何判据（TOP_MIN / WIDE_MIN）
"""

P = 'code/vision/vision_track.c'

OLD_V1 = '''/* --------------------------------------------------------------------------
 * 直角弯检测：宽度剖面法
 *
 * 做法:
 *   1. 收集每一行"覆盖中线的那段的宽度"
 *   2. 参考线宽 = 下部 60% 的中位数（那里一定是正常赛道线）
 *   3. 找最宽的那一行
 *   4. 最宽 / 参考 >= CFG_CORNER_RATIO_X10/10 就是直角弯的横臂
 *   5. 方向 = 最宽行里离参考线更远的那一端
 *
 * 实测宽度比:
 *   左直角 6.0x   右直角 8.1x
 *   直道   1.1~1.6x  分叉 1.2~1.5x   环岛 1.6~1.8x
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
    static int cnt[V_H];

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

    /* ---- 2) 参考线宽 = 后 60% 的中位数 ---- */
    {
        int lo = n * 4 / 10;
        int m = n - lo;
        int j, k;
        static int tmp[V_H];

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

    /* ---- 3) 宽度比门槛 ---- */
    ratio_x10 = wide_w * 10 / med;
    if (ratio_x10 < CFG_CORNER_RATIO_X10) { return; }

    /* ---- 4) 方向: 最宽行下方的中线作参考 ---- */
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

src = open(P, 'rb').read().decode('gbk')
NL = '\r\n'
lines = src.split(NL)

start = None
for i, l in enumerate(lines):
    if l.startswith('static void post_detect_corner(void)'):
        start = i
        break
if start is None:
    raise SystemExit('没找到 post_detect_corner')

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

lines[cstart:end + 1] = OLD_V1.split('\n')

blob = NL.join(lines).encode('gbk')
assert len(blob) > 20000
open(P, 'wb').write(blob)
print('回退完成: %d 字节' % len(blob))
