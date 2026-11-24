# -*- coding: utf-8 -*-
p = 'code/vision/v2_elem.h'
raw = open(p, 'rb').read()

if b'V2E_ISLE_MIN_RUN' in raw:
    print('已存在，跳过')
else:
    anchor = b'#define V2E_ISLE_MIN_ROWS    (5)'
    if anchor not in raw:
        raise SystemExit('anchor 未找到')

    add_text = '''/* 环岛连续段最少行数
 * 实测: 真环岛 29~48 行, 误报 7 行 */
#define V2E_ISLE_MIN_RUN     (15)

/* 核心段内最大间距上限
 * 实测: 环岛 31~45, 十字/误报 59~79 */
#define V2E_ISLE_SEP_MAX_SEG (55)

'''.replace('\n', '\r\n')

    raw = raw.replace(anchor, anchor + b'\r\n' + add_text.encode('gbk'), 1)
    open(p, 'wb').write(raw)
    print('v2_elem.h 已加新宏')
