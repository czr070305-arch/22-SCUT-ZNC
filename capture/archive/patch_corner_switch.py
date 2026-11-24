# -*- coding: utf-8 -*-
"""步骤 2: 加 CFG_CORNER_ENABLE 开关 + 修复重复注释"""

# ---------- 1) vision_cfg.h: 加开关，比值改回 25 ----------
P = 'code/vision/vision_cfg.h'
src = open(P, 'rb').read().decode('gbk')
NL = '\r\n'
lines = src.split(NL)
out = []

for l in lines:
    if l.startswith('#define CFG_CORNER_RATIO_X10'):
        out.append('#define CFG_CORNER_RATIO_X10 (25)       /* 2.5 倍 */')
        continue
    if l.startswith('#define CFG_CORNER_BIAS'):
        continue        # 稍后统一在开关块里重写
    out.append(l)

# 找 CFG_CORNER_MIN_ROWS 那行，在它前面插入总开关块
idx = None
for i, l in enumerate(out):
    if l.startswith('#define CFG_CORNER_MIN_ROWS'):
        idx = i
        break
if idx is None:
    raise SystemExit('没找到 CFG_CORNER_MIN_ROWS')

switch_block = '''/* ---------- 直角弯总开关 ----------
 *
 * 0 = 关闭。拐角判定结果【不影响转向】:
 *     前瞻点仍然取中线，err 不加偏置。
 *     爬线照样能跟 60~90 度的弯，只是前瞻点不会提前跳到横臂远端。
 *
 * 1 = 开启。判出直角弯时:
 *     前瞻点被搬到横臂远端 (g_corner_far_x)
 *     err 额外加 +-CFG_CORNER_BIAS
 *
 * 为什么要这个开关:
 *   直角弯判据在实车上的误报还没压干净。误报时前瞻点会跳到噪点上，
 *   导致车猛打方向。关掉之后至少能保证基本跑直。
 *   元素字母仍然显示，方便观察判定结果。 */
#define CFG_CORNER_ENABLE    (1)

/* 直角弯额外转向偏置
 * 22 时误报会猛打方向，8 影响较小 */
#define CFG_CORNER_BIAS      (8.0f)

'''.replace('\n', '\r\n')

out[idx:idx] = switch_block.split(NL)

blob = NL.join(out).encode('gbk')
assert len(blob) > 5000
open(P, 'wb').write(blob)
print('vision_cfg.h: 加了 CFG_CORNER_ENABLE，比值回 25，BIAS=8')


# ---------- 2) vision_track.c: track_steer 里加开关 ----------
P2 = 'code/vision/vision_track.c'
raw2 = open(P2, 'rb').read()

# 2.1 前瞻点搬移
old1 = (b'    if (g_corner_dir != 0) {\r\n'
        b'        look_x = g_corner_far_x;\r\n'
        b'        look_y = g_path_top;\r\n'
        b'    }')
new1 = (b'#if (CFG_CORNER_ENABLE)\r\n'
        b'    if (g_corner_dir != 0) {\r\n'
        b'        look_x = g_corner_far_x;\r\n'
        b'        look_y = g_path_top;\r\n'
        b'    }\r\n'
        b'#endif')
if old1 not in raw2:
    raise SystemExit('没找到前瞻点搬移代码')
raw2 = raw2.replace(old1, new1, 1)

# 2.2 err 偏置
old2 = (b'    if (g_corner_dir != 0) {\r\n'
        b'        err += (float)g_corner_dir * CFG_CORNER_BIAS;\r\n'
        b'    }')
new2 = (b'#if (CFG_CORNER_ENABLE)\r\n'
        b'    if (g_corner_dir != 0) {\r\n'
        b'        err += (float)g_corner_dir * CFG_CORNER_BIAS;\r\n'
        b'    }\r\n'
        b'#endif')
if old2 not in raw2:
    raise SystemExit('没找到 err 偏置代码')
raw2 = raw2.replace(old2, new2, 1)

open(P2, 'wb').write(raw2)
print('vision_track.c: track_steer 加了 CFG_CORNER_ENABLE 开关')
