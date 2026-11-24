# -*- coding: utf-8 -*-
"""把 img_threshold() 改成两遍法（相对黑角裁切）

上一版把文件写成 0 字节的教训:
  先构造好 bytes，编码/校验通过后再 open('wb')
"""

OUT = 'code/vision/vision_imgproc.c'

NEW_FUNC = r'''int img_threshold(void)
{
    int x, y;
    int otsu;
    int thre;
    int total = 0;
    int lo = 255;
    int hi = 0;
    int floor_thre;
    int dark_cut;

    /* ======================================================================
     * 两遍法：第一遍只求全图最大亮度，用它按【相对比例】定出黑角阈值
     *
     * 为什么不能用固定的 CFG_THRE_DARK_SKIP = 12:
     *   那是绝对灰度值。室内地板 20~40，赛道地板可能是 60~100。
     *   固定 12 在暗环境跳掉 36% 的像素（鱼眼黑角），
     *   到了亮环境只跳掉 5% —— 地板主峰就混进直方图了，
     *   大津法被地板拽走，阈值反而往下掉。
     *
     *   实测：整体亮度 +20 时阈值从 65 掉到 58，
     *   地板被当成白线，直道全被判成左右弯。
     *
     * 改成相对比例后，无论整体多亮，跳掉的比例都差不多。
     * ====================================================================== */
    for (y = 0; y < V_H; y++) {
        for (x = 0; x < V_W; x++) {
            int v = g_frame[y][x];
            if (v > hi) { hi = v; }
        }
    }

    /* 黑角 / 地板 取最高亮度的 CFG_THRE_DARK_PCT%（十几） */
    dark_cut = (hi * CFG_THRE_DARK_PCT) / 100;

    /* ---- 第二遍：只统计亮于 dark_cut 的像素 ---- */
    memset(s_hist, 0, sizeof(s_hist));
    hi = 0;
    for (y = 0; y < V_H; y++) {
        for (x = 0; x < V_W; x++) {
            int v = g_frame[y][x];

            if (v <= dark_cut) {
                continue;
            }
            s_hist[v]++;
            total++;

            if (v < lo) { lo = v; }
            if (v > hi) { hi = v; }
        }
    }

    /* 有效像素太少: 镜头盖着 / 完全欠曝 / 全黑。宁可不出结果，
     * 上层按"丢线"处理，也不要判成一片白。 */
    if (total < 64) {
        g_thre = CFG_THRE_MIN;
        return g_thre;
    }

    otsu = otsu_threshold(s_hist, total);
    thre = otsu + CFG_THRE_MARGIN + s_thre_bias;

    /* 动态范围兜底。
     * 直方图偶尔会被某个方向的光照占了大头，把最大值拉跑。
     * 强制阈值落在动态范围的 20% 以上:
     *   lo=20(地板), hi=140(白线) -> floor = 20 + 24 = 44
     * 这样 20~40 还是黑，130 还是白。 */
    floor_thre = lo + (hi - lo) * 2 / 10;
    if (thre < floor_thre) {
        thre = floor_thre;
    }

    if (CFG_THRE_FIXED > 0) {
        thre = CFG_THRE_FIXED;
    }
    if (thre < CFG_THRE_MIN) {
        thre = CFG_THRE_MIN;
    }
    if (thre > CFG_THRE_MAX) {
        thre = CFG_THRE_MAX;
    }
    g_thre = thre;
    return thre;
}
'''

src = open(OUT, 'rb').read().decode('gbk')
lines = src.split('\r\n')

start = None
for i, l in enumerate(lines):
    if l.startswith('int img_threshold(void)'):
        start = i
        break
if start is None:
    raise SystemExit('没找到 img_threshold')

end = None
depth = 0
for i in range(start, len(lines)):
    depth += lines[i].count('{') - lines[i].count('}')
    if i > start and depth == 0 and lines[i] == '}':
        end = i
        break
if end is None:
    raise SystemExit('没找到函数结尾')

print('替换区间: %d ~ %d 行' % (start + 1, end + 1))

body = '\r\n'.join(lines[:start] + NEW_FUNC.split('\n') + lines[end + 1:])
try:
    blob = body.encode('gbk')
except UnicodeEncodeError as e:
    raise SystemExit('GBK 编码失败（未写入）: %s' % e)

if len(blob) < 5000:
    raise SystemExit('内容太小 (%d 字节)，放弃' % len(blob))

open(OUT, 'wb').write(blob)
print('写入成功: %d 字节' % len(blob))
