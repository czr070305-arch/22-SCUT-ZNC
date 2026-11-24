/* ==========================================================================
 *  vision_imgproc.c —— 二值化 + 3x3 去噪 + 逐行白线段（MRS 移植版）
 *  --------------------------------------------------------------------------
 *  对应 VS Code 工程里的 code/imgproc.c。
 *
 *  本赛道的引导线是【一条 20mm 白线】而不是两条边线，所以每一行里
 *  "白线"就是一段（或几段）白像素。把每行切成段，后面无论跟线还是判元素，
 *  都只需要在这几个段上做选择，逻辑最省也最稳。
 *
 *  【移植改动汇总（都是为了 63KB RAM）】
 *   1) g_frame 在 vision.h 里被宏映射到 mt9v03x_image，不再拷副本 → 省 22.5KB
 *   2) 不再保存整幅二值图 g_binary：二值化、去噪、提段在一趟行循环里做完，
 *      只留 188 字节的行缓冲 → 省 22.5KB。屏幕上显示时直接用
 *      "原图 + 同一个阈值" 交给 tft180_show_gray_image() 判黑白，
 *      肉眼看到的仍然是二值图像，但 RAM 不用再存一份。
 *   3) 3x3 去噪只保留"上一行 / 本行 / 下一行"原始判白结果（3 行滚动），
 *      与原版整幅 s_raw 的逐像素结果完全一致。
 * ========================================================================== */
#include "vision.h"
#include <string.h>

v_run_t  g_runs[V_H][V_MAX_RUN];
uint8    g_run_num[V_H];
int      g_thre = 0;

static int s_hist[256];

/* 运行期阈值偏移, 由 TRACK 页按键调节, 见 vision.h 的说明 */
static int s_thre_bias = 0;

/* 行缓冲：原始判白 3 行 + 去噪结果 1 行（共 4 * 188 = 752 B） */
static unsigned char s_prev[V_W];       /* y-1 行的原始判白 */
static unsigned char s_cur[V_W];        /* y   行的原始判白 */
static unsigned char s_next[V_W];       /* y+1 行的原始判白 */
static unsigned char s_out[V_W];        /* y   行去噪后的结果（0 / 255） */

/* --------------------------------------------------------------------------
 * 大津法（类间方差最大）
 *
 * 计算要点：只有"背景/前景"两类都有像素时方差才有意义，所以跳过
 * w_bg == 0 与 w_bg == total 的情况。方差用 long long：
 * 188*120 像素下 w*(N-w)*(m1-m2)^2 会到 1e12 级，int 会溢出。
 * -------------------------------------------------------------------------- */
static int otsu_threshold(const int *hist, int total)
{
    int i;
    int w_bg = 0;
    int best_t = 0;
    long long sum_all = 0;
    long long sum_bg = 0;
    long long best_var = -1;

    for (i = 0; i < 256; i++) {
        sum_all += (long long)i * hist[i];
    }

    for (i = 0; i < 256; i++) {
        int w_fg;
        long long m_bg;
        long long m_fg;
        long long diff;
        long long var;

        w_bg += hist[i];
        if (w_bg == 0) {
            continue;
        }
        w_fg = total - w_bg;
        if (w_fg == 0) {
            break;
        }
        sum_bg += (long long)i * hist[i];
        m_bg = sum_bg / w_bg;
        m_fg = (sum_all - sum_bg) / w_fg;
        diff = m_bg - m_fg;
        var = (long long)w_bg * (long long)w_fg * diff * diff;
        if (var > best_var) {
            best_var = var;
            best_t = i;
        }
    }
    return best_t;
}

int img_threshold(void)
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


void vision_set_thre_bias(int bias)
{
    if (bias < -64) { bias = -64; }
    if (bias >  64) { bias =  64; }
    s_thre_bias = bias;
}

int vision_get_thre_bias(void)
{
    return s_thre_bias;
}

/* --------------------------------------------------------------------------
 * 二值化 + 3x3 去噪 + 逐行提段（一趟做完，不存整幅图）
 *
 *  ★ 为什么必须去噪：
 *    蓝布有织纹、镜头有噪声、光照有渐变，单纯按阈值判白会在布面上撒下
 *    成千上万个单像素白点。这些白点会让"一行里出现了两段白"这个判据
 *    彻底失效（岔路/环岛全靠这个判据），所以必须先杀掉它们。
 *
 *    判据：上下都为白 或 左右都为白 才保留（3x3 开运算的变形）
 *       · 单像素噪声          → 上下左右都不同时为白 → 杀掉
 *       · 引导线（竖直走向）  → 上下连续 → 保留
 *       · 十字横臂/环岛横段   → 左右连续 → 保留（横臂不能被竖向腐蚀掉）
 * -------------------------------------------------------------------------- */
void img_binarize_runs(int thre)
{
    int x, y;

    /* y = 0 行的"上一行"按原版语义恒为 0（画面外当作黑） */
    memset(s_prev, 0, sizeof(s_prev));
    for (x = 0; x < V_W; x++) {
        s_cur[x]  = (g_frame[0][x] > (unsigned char)thre) ? 1 : 0;
        s_next[x] = (g_frame[1][x] > (unsigned char)thre) ? 1 : 0;
    }

    for (y = 0; y < V_H; y++) {
        /* ---- 第一步：本行去噪，结果写进 s_out ---- */
        for (x = 0; x < V_W; x++) {
            unsigned char keep = 0;

            if (!s_cur[x]) {
                s_out[x] = 0;
                continue;
            }
            {
                int up = s_prev[x];
                int dn = s_next[x];
                int lf = (x > 0) ? s_cur[x - 1] : 0;
                int rt = (x < V_W - 1) ? s_cur[x + 1] : 0;

                if ((up && dn) || (lf && rt)) {
                    keep = 1;
                }
            }
            s_out[x] = keep ? 255 : 0;
        }

        /* ---- 第二步：直接从 s_out 提段（原 img_runs 的逻辑） ----
         * 两处细节与原版一致：
         *  1) 断点合并：白线被一小块暗斑（褶皱/反光）切成两段时，
         *     间隔 <= 2px 的两段合并，避免一行被误读成"双线"。
         *  2) 段数封顶：V_MAX_RUN 满后不再记录，防越界。
         */
        {
            int n = 0;
            int st = -1;

            for (x = 0; x <= V_W; x++) {
                int white = (x < V_W) && (s_out[x] != 0);

                if (white) {
                    if (st < 0) {
                        st = x;
                    }
                } else if (st >= 0) {
                    int end = x - 1;
                    int w = end - st + 1;

                    if ((w >= CFG_RUN_W_MIN) && (n < V_MAX_RUN)) {
                        if ((n > 0) && (st - g_runs[y][n - 1].r <= 2)) {
                            g_runs[y][n - 1].r = (short)end;   /* 与前一段贴得很近：合并 */
                        } else {
                            g_runs[y][n].l = (short)st;
                            g_runs[y][n].r = (short)end;
                            n++;
                        }
                    }
                    st = -1;
                }
            }
            g_run_num[y] = (uint8)n;
        }

        /* ---- 第三步：三行缓冲滚动 ---- */
        memcpy(s_prev, s_cur, sizeof(s_prev));
        memcpy(s_cur, s_next, sizeof(s_cur));
        if (y + 2 < V_H) {
            int i;
            for (i = 0; i < V_W; i++) {
                s_next[i] = (g_frame[y + 2][i] > (unsigned char)thre) ? 1 : 0;
            }
        } else {
            memset(s_next, 0, sizeof(s_next));      /* 画面外按黑处理 */
        }
    }
}
