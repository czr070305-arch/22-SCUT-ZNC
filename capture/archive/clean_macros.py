# -*- coding: utf-8 -*-
"""删除 v2_elem.h 里失效的宏（旧射线投票算法留下的）"""

p = 'code/vision/v2_elem.h'
raw = open(p, 'rb').read()
src = raw.decode('gbk')
lines = src.split('\r\n')

DEAD = ['V2E_CIRCLE_DX_START', 'V2E_CIRCLE_DX_STEP', 'V2E_CIRCLE_RAYS',
        'V2E_CIRCLE_SLOPE_MIN', 'V2E_ISLE_MIN_ROWS', 'V2E_ISLE_MIN_Y']

out = []
skip_block_comment = False
i = 0
removed = []
while i < len(lines):
    line = lines[i]

    # 如果这一行定义了要删的宏
    hit = None
    for d in DEAD:
        if line.startswith('#define ' + d):
            hit = d
            break

    if hit is not None:
        removed.append(hit)
        # 往前删除紧邻的块注释（如果还在 out 里）
        while out:
            prev = out[-1].strip()
            if prev.startswith('*') or prev.startswith('/*'):
                out.pop()
            else:
                break
        # 删除后面的空行
        i += 1
        while i < len(lines) and lines[i].strip() == '':
            i += 1
        continue

    out.append(line)
    i += 1

blob = '\r\n'.join(out).encode('gbk')
assert len(blob) > 800
open(p, 'wb').write(blob)
print('删除 %d 个宏: %s' % (len(removed), removed))
print('新文件 %d 字节' % len(blob))
