# -*- coding: utf-8 -*-
"""直角弯参数收紧：门槛 2.5 -> 6.0，BIAS 22 -> 8"""

P = 'code/vision/vision_cfg.h'
src = open(P, 'rb').read().decode('gbk')
NL = '\r\n'
lines = src.split(NL)

changed = []

for i, l in enumerate(lines):
    if l.startswith('#define CFG_CORNER_BIAS'):
        lines[i] = ('#define CFG_CORNER_BIAS      (8.0f)'
                    '     /* 直角弯额外转向偏置。'
                    '实测误报会持续多帧，22 太猛，降到 8 */')
        changed.append('CFG_CORNER_BIAS 22 -> 8')

    elif l.startswith('#define CFG_CORNER_RATIO_X10'):
        lines[i] = ('#define CFG_CORNER_RATIO_X10 (60)'
                    '       /* 6.0 倍。实测真直角弯 6.0/8.1，'
                    '误报 2.6~4.5 */')
        changed.append('CFG_CORNER_RATIO_X10 25(2.5) -> 60(6.0)')

body = NL.join(lines)
blob = body.encode('gbk')
assert len(blob) > 5000
open(P, 'wb').write(blob)
print('已改:')
for c in changed:
    print('  ' + c)
