# -*- coding: utf-8 -*-
"""给 v2e_detect_island 加 v8 判据：
   1. 连续段门槛 5 -> 15 行
   2. 最大间距上限 55
"""

p = 'code/vision/v2_elem.c'
raw = open(p, 'rb').read()

# ---- 1) 连续段门槛 ----
a = b'if (best_len < V2E_ISLE_MIN_ROWS) { return 0; }'
b = b'if (n < V2E_ISLE_MIN_ROWS) { return 0; }'
if a not in raw or b not in raw:
    raise SystemExit('门槛字符串未找到')
raw = raw.replace(a, b'if (best_len < V2E_ISLE_MIN_RUN) { return 0; }', 1)
raw = raw.replace(b, b'if (n < V2E_ISLE_MIN_RUN) { return 0; }', 1)

# ---- 2) 插入最大间距检查 ----
anchor = b'    k = n / 3;\r\n    if (k < 1) { k = 1; }'
if anchor not in raw:
    raise SystemExit('anchor 未找到')

note_text = '''    /* ---- 4.5) 最大间距上限 ----
     * 真正的环岛边界离赛道线很近（实测最大 33~45px）。
     * 十字的横臂长度能到 60~80px，会被误当成"岛边界"。
     * 实测标注:
     *   seq002(环岛) 33   seq003(环岛) 45   seq013(环岛) 31
     *   seq045(十字) 62   seq004(误报) 79   seq055(误报) 79
     */
    {
        int mx = 0;

        for (i = 0; i < n; i++) {
            int yy = best_lo + i;

            if ((yy < V_H - 4) && (row_sep[yy] > mx)) { mx = row_sep[yy]; }
        }
        if (mx > V2E_ISLE_SEP_MAX_SEG) { return 0; }
    }

'''.replace('\n', '\r\n')

note = note_text.encode('gbk')
raw = raw.replace(anchor, note + anchor, 1)

open(p, 'wb').write(raw)
print('v2_elem.c 更新完成: %d 字节' % len(raw))
