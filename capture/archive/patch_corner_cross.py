# -*- coding: utf-8 -*-
"""在 post_detect_corner 里加"两侧都有臂 -> 是十字，不是直角弯"的过滤"""

P = 'code/vision/vision_track.c'
raw = open(P, 'rb').read()

# 找到 dl/dr 计算之后的位置，插入两侧检查
old = b'''    dl = iabs(wide_l - ref);
    dr = iabs(wide_r - ref);
    far = (dl > dr) ? wide_l : wide_r;
    if (far == ref) { return; }
'''

new_text = '''    dl = iabs(wide_l - ref);
    dr = iabs(wide_r - ref);

    /* ---- 5) 区分十字和直角弯 ----
     *
     * 十字的横臂和直角弯的横臂在像素上一样宽，区别是参考线的位置:
     *   直角弯: 参考线贴在横臂【一端】，一侧臂长另一侧接近 0
     *   十字  : 参考线在横臂【中间】，两侧都有臂
     *
     * 实测（2026-11-19 赛道图集）:
     *   十字   seq045: 左臂=36 右臂=36
     *   十字   seq079: 左臂=20 右臂=21
     *   右直角 seq077: 左臂= 2 右臂=62
     *   左直角 seq006: 左臂=35 右臂=12
     * 18 是这两类之间最干净的分界。 */
    if ((dl >= CFG_CORNER_ARM_MIN) && (dr >= CFG_CORNER_ARM_MIN)) {
        return;         /* 两侧都有臂 -> 十字，不是直角弯 */
    }

    far = (dl > dr) ? wide_l : wide_r;
    if (far == ref) { return; }
'''

old_crlf = old.replace(b'\n', b'\r\n')
if old_crlf not in raw:
    raise SystemExit('锚点未找到')

raw = raw.replace(old_crlf, new_text.replace('\n', '\r\n').encode('gbk'), 1)
open(P, 'wb').write(raw)
print('post_detect_corner 已加十字过滤: %d 字节' % len(raw))
