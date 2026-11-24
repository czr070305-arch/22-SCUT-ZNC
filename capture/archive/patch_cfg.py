# -*- coding: utf-8 -*-
OUT = 'code/vision/vision_cfg.h'

ADD = '''/* 直角弯后处理：宽度剖面法
 *   最宽白段 / 参考线宽  >= 此倍数 -> 直角弯
 *   实测: 左直角 6.0x   右直角 8.1x
 *         直道 1.1~1.6x  分叉 1.2~1.5x  环岛 1.6~1.8x */
#define CFG_CORNER_RATIO_X10 (25)       /* 2.5 倍 */

/* 直角弯判断时跳过的顶部行数
 * 画面顶部是房间杂物(窗帘杆/桌沿/路由器)，
 * 不剔除会把直道误报成直角弯(实测 4.2x) */
#define CFG_TOP_CUT          (22)
'''

raw = open(OUT, 'rb').read().decode('gbk')
lines = raw.split('\r\n')

# 找 CFG_CORNER_TAN_MAX 那一行，在它后面插入
idx = None
for i, l in enumerate(lines):
    if l.startswith('#define CFG_CORNER_TAN_MAX'):
        idx = i
        break
if idx is None:
    raise SystemExit('没找到 CFG_CORNER_TAN_MAX')

if 'CFG_CORNER_RATIO_X10' in raw:
    raise SystemExit('宏已存在')

lines[idx + 1:idx + 1] = [''] + ADD.split('\n')
body = '\r\n'.join(lines)
blob = body.encode('gbk')
assert len(blob) > 5000
open(OUT, 'wb').write(blob)
print('写入成功: %d 字节' % len(blob))
