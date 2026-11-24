# -*- coding: utf-8 -*-
"""post_detect_corner 换成 v3：几何判据（path_top 被宽段顶住）

为什么不用宽度比:
  横臂在画面里的宽度取决于车离它多远。远处窄、近处宽，
  比值能从 2 变到 8。任何固定门槛都会在某个距离区间失效
  （门槛 2.5 误报一片，门槛 6.0 远处漏检）。

v3 判据:
  1. path_top >= 8        （爬到画面边界不算，那是没路了）
  2. path_top 附近 ±4 行有 >= 25 宽的白段  （被横臂顶住）
  3. wide 下方能找到参考线
  4. 参考线两侧不都有臂    （两侧都有 = 十字）
  5. 取离参考线更远的一端作为方向

实测:
  真直角弯 7/7 全中
  真直角弯 seq006 top=57 顶部宽段=33
  真直角弯 seq077 top=58 顶部宽段=65
  直道 seq011/052/057 顶部宽段 3~8（不足 25）
  环岛 顶部宽段 3~5
"""

P = 'code/vision/vision_track.c'
raw = open(P, 'rb').read()

NEW_FUNC = '''/* --------------------------------------------------------------------------
 * 直角弯检测：几何判据（爬线上端被宽横臂顶住）
 *
 * 为什么不用宽度比:
 *   横臂在画面里的宽度取决于车离它多远。远处窄、近处宽，比值能从 2 变到 8。
 *   任何固定门槛都会在某个距离区间失效
 *   （门槛 2.5 误报一片，门槛 6.0 远处漏检）。
 *
 * v3 判据:
 *   1. path_top >= CFG_CORNER_TOP_MIN
 *      爬到画面边界不算，那是没路了，不是被横臂顶住
 *   2. path_top 附近 +-4 行内有 >= CFG_CORNER_WIDE_MIN 宽的白段
 *      直角弯的横臂会把爬线"顶住"，就在爬线上端旁边
 *   3. 宽段两侧不都有臂（两侧都有 = 十字，交给 detect_cross）
 *   4. 方向 = 离参考线更远的那一端
 *
 * 实测（2026-11-19 图集）:
 *   真直角弯 seq006 top=57 顶部宽段=33
 *   真直角弯 seq077 top=58 顶部宽段=65
 *   直道     seq011 top=26 顶部宽段= 8
 *   直道     seq052 top=29 顶部宽段= 3
 *   环岛     top=30~41 顶部宽段 3~5
 * -------------------------------------------------------------------------- */
static void post_detect_corner(void)
{
    int y, i;
    int wide_y = -1, wide_w = 0, wide_l = 0, wide_r = 0;
    int ref = -1;
    int dl, dr, far;

    if (g_path_rows < 20) { return; }
    if (g_path_top < CFG_CORNER_TOP_MIN) { return; }

    /* ---- 1) 在爬线上端附近找最宽的白段 ---- */
    for (y = g_path_top - 4; y <= g_path_top + 4; y++) {
        if (y < 0) { continue; }
        if (y >= V_H) { break; }

        for (i = 0; i < g_run_num[y]; i++) {
            int w = g_runs[y][i].r - g_runs[y][i].l + 1;

            if (w > wide_w) {
                wide_w = w;
                wide_y = y;
                wide_l = g_runs[y][i].l;
                wide_r = g_runs[y][i].r;
            }
        }
    }

    /* 没有横臂 -> 不是直角弯 */
    if (wide_w < CFG_CORNER_WIDE_MIN) { return; }

    /* ---- 2) 宽段下方的参考线 ---- */
    for (y = wide_y + 2; y < V_H; y++) {
        if (g_mid_line[y] >= 0) {
            ref = g_mid_line[y];
            break;
        }
    }
    if (ref < 0) { return; }

    dl = iabs(wide_l - ref);
    dr = iabs(wide_r - ref);

    /* ---- 3) 两侧都有臂 -> 十字，不是直角弯 ---- */
    if ((dl >= CFG_CORNER_ARM_MIN) && (dr >= CFG_CORNER_ARM_MIN)) {
        return;
    }

    /* ---- 4) 方向 = 离参考线更远的那一端 ---- */
    far = (dl > dr) ? wide_l : wide_r;
    if (far == ref) { return; }

    g_corner_dir = (far > ref) ? 1 : -1;
    g_corner_far_x = far;
}
'''

src = raw.decode('gbk')
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

print('替换行 %d ~ %d' % (cstart + 1, end + 1))

lines[cstart:end + 1] = NEW_FUNC.split('\n')

body = NL.join(lines)
blob = body.encode('gbk')
assert len(blob) > 20000
open(P, 'wb').write(blob)
print('写入完成: %d 字节' % len(blob))
