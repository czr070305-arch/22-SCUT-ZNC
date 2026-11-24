# -*- coding: utf-8 -*-
"""控制层参数调整：解决弯道内轮停转导致的抽搐"""

P = 'code/algorithm/ctrl.h'
src = open(P, 'rb').read().decode('gbk')
NL = '\r\n'
lines = src.split(NL)

changed = []

for i, l in enumerate(lines):
    # ① 内轮最低转速 0 -> 480
    if l.startswith('#define CTRL_INNER_WHEEL_MIN_RPM'):
        lines[i] = '#define CTRL_INNER_WHEEL_MIN_RPM    (480.0f)'
        changed.append('CTRL_INNER_WHEEL_MIN_RPM: 0 -> 480')
        continue

    # ② 转向输出上限 600 -> 300
    if l.startswith('#define CTRL_TURN_OUT_MAX'):
        lines[i] = '#define CTRL_TURN_OUT_MAX       (300.0f)'
        changed.append('CTRL_TURN_OUT_MAX: 600 -> 300')
        continue

    # ③ 加阻尼 KD 0 -> 8
    if l.startswith('#define CTRL_TURN_KD'):
        lines[i] = '#define CTRL_TURN_KD            (8.0f)'
        changed.append('CTRL_TURN_KD: 0 -> 8')
        continue

body = NL.join(lines)
blob = body.encode('gbk')
assert len(blob) > 3000
open(P, 'wb').write(blob)

print('已改:')
for c in changed:
    print('  ' + c)
