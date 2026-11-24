# -*- coding: utf-8 -*-
p = 'code/vision/vision_cfg.h'
raw = open(p, 'rb').read()

if b'CFG_CROSS_X_SEP_MIN' in raw:
    print('已存在，跳过')
else:
    anchor = b'#define CFG_TOP_CUT          (22)'
    if anchor not in raw:
        raise SystemExit('没找到 CFG_TOP_CUT')

    add = '''/* ---------- X 形十字（两条斜线交叉）----------
 * 与环岛正好相反: 间距【收敛】，连续段短。
 * 实测: X 形 5~14 行, 环岛 >= 23 行 */

/* 双线间距范围 */
#define CFG_CROSS_X_SEP_MIN  (3)
#define CFG_CROSS_X_SEP_MAX  (60)

/* 连续段上限。>= 这么长就不是 X 形，交给环岛判据 */
#define CFG_CROSS_X_MAX_RUN  (15)

'''.replace('\n', '\r\n')

    raw = raw.replace(anchor, anchor + b'\r\n\r\n' + add.encode('gbk'), 1)
    open(p, 'wb').write(raw)
    print('vision_cfg.h 已加 X 形十字参数')
