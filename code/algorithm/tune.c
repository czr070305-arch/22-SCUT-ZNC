#include "tune.h"
#include "hw_flash.h"

/*
 * tune.c - 转向环在线调参实现（精简版）
 *
 * 数据流:
 *   S1/S2 -> tune_step() -> g_tune[] 立即生效 + dirty=1
 *   RIGHT -> tune_do_save() -> flash_param_save() -> dirty=0
 *
 *   上电 -> flash_param_load()
 *             Flash 有效 -> 直接填 g_tune[]
 *             Flash 无效 -> tune_load_factory()
 */

/* ---------- 当前生效值 ---------- */
int32 g_tune[TUNE_COUNT];

/* ---------- 参数表 ----------
 *
 * min/max 是"存储值"的范围，不是显示值。
 * 例：KP_LN 存 5~60，显示 0.5~6.0。 */
typedef struct
{
    const char *name;
    int32       min;
    int32       max;
    int32       step;
    uint8       dec;        /* 显示小数位 */
    int32       def;        /* 出厂值（烧录后就是它） */
} tune_item_t;

static const tune_item_t g_tune_items[TUNE_COUNT] =
{
    /* name      min   max  step  dec   def */
    { "KP_LN",     5,      200,      5,   1,    110},  /* 直道P：大=回正快但画龙，小=稳但回正慢 */
    { "KP_CR",    20,     250,      5,   1,  250 },  /* 弯道P：大=转得急，小=转不过来 */
    { "LOOK",     10,   80,    2,   0, 40 },   /* 前瞻行数            */
    { "SLOPE",     0,   12,    1,   0, 2 },   /* 斜率权重            */
    { "TMAX",    200,   2000,    50,   0, 1800 },  /* 差速上限RPM：大=转弯半径更小 */
    { "SLOW",     50,  100,    5,   2, 85 },   /* 弯道系数 0.50~1.00 */
    { "Speed",   600,   3000,   100,   0, 1000 },  /* 巡航RPM：直道目标轮速 */
    { "THRE",    -64,   64,    4,   0, 4 },   /* 阈值 bias           */
    { "ERR_LP",    0,   95,    5,   0, 50 },   /* err 低通 0~95      */
};

/* 存储值 -> 实际值的缩放系数，和 dec 一一对应 */
static const float g_tune_scale[TUNE_COUNT] =
{
    0.1f, 0.1f, 1.0f, 1.0f, 1.0f, 0.01f, 1.0f, 1.0f, 0.01f
};

/* ---------- 屏幕交互状态 ---------- */
static int   s_sel   = 0;
static uint8 s_dirty = 0;

/* ---------- 内部小工具 ---------- */

static int32 tune_clamp(int32 v, int32 lo, int32 hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* 整数转字符串（含负号） */
static uint8 tune_put_i32(int32 v, char *buf)
{
    char   tmp[12];
    uint8  n = 0;
    uint8  i = 0;
    uint32 u;

    if (v < 0)
    {
        buf[i ++] = '-';
        u = (uint32)(-v);
    }
    else
    {
        u = (uint32)v;
    }

    if (0 == u)
    {
        tmp[n ++] = '0';
    }
    while (0 != u)
    {
        tmp[n ++] = (char)('0' + (u % 10));
        u /= 10;
    }
    while (0 != n)
    {
        buf[i ++] = tmp[-- n];
    }
    buf[i] = '\0';
    return i;
}

/* ---------- 读写 ---------- */

float tune_get(int idx)
{
    if ((idx < 0) || (idx >= TUNE_COUNT))
    {
        return 0.0f;
    }
    return (float)g_tune[idx] * g_tune_scale[idx];
}

void tune_load_factory(void)
{
    int i;

    for (i = 0; i < TUNE_COUNT; i ++)
    {
        g_tune[i] = g_tune_items[i].def;
    }
    s_dirty = 0;
}

void tune_init(void)
{
    /* 载入由 flash_param_load() 负责：
     *   Flash 有效 -> 填 g_tune[]
     *   Flash 无效 -> 调 tune_load_factory()
     * 这里只复位交互状态。 */
    s_sel   = 0;
    s_dirty = 0;
}

/* ---------- 屏幕交互 ---------- */

int tune_get_sel(void)
{
    return s_sel;
}

void tune_sel_next(void)
{
    s_sel ++;
    if (s_sel >= TUNE_COUNT)
    {
        s_sel = 0;
    }
}

void tune_sel_prev(void)
{
    s_sel --;
    if (s_sel < 0)
    {
        s_sel = TUNE_COUNT - 1;
    }
}

int tune_step(int dir)
{
    int32 v;
    int32 old;
    const tune_item_t *it = &g_tune_items[s_sel];

    old = g_tune[s_sel];
    v   = old + ((dir > 0) ? it->step : -it->step);
    v   = tune_clamp(v, it->min, it->max);

    if (v == old)
    {
        return 0;
    }

    g_tune[s_sel] = v;
    s_dirty = 1;
    return 1;
}

/* ---------- 保存 ---------- */

uint8 tune_do_save(void)
{
    uint8 err = flash_param_save();

    if (0 == err)
    {
        s_dirty = 0;
    }
    return err;
}

int tune_is_dirty(void)
{
    return (int)s_dirty;
}

void tune_clear_dirty(void)
{
    s_dirty = 0;
}

/* ---------- 屏幕显示 ---------- */

const char *tune_name(int idx)
{
    if ((idx < 0) || (idx >= TUNE_COUNT))
    {
        return "?";
    }
    return g_tune_items[idx].name;
}

void tune_value_str(int idx, char *buf)
{
    int32 v;
    int32 ipart;
    int32 frac;
    int32 lim;
    uint8 n = 0;
    uint8 d;

    if ((idx < 0) || (idx >= TUNE_COUNT) || (0 == buf))
    {
        return;
    }

    v = g_tune[idx];

    /* dec=0：直接整数输出 */
    if (0 == g_tune_items[idx].dec)
    {
        tune_put_i32(v, buf);
        return;
    }

    if (v < 0)
    {
        buf[n ++] = '-';
        v = -v;
    }

    /* 除以 10^dec 拆整数/小数，除数用循环算，避免引 powf */
    lim = 1;
    d   = 0;
    while (d < g_tune_items[idx].dec)
    {
        lim *= 10;
        d ++;
    }

    ipart = v / lim;
    frac  = v % lim;

    n += tune_put_i32(ipart, &buf[n]);
    buf[n ++] = '.';

    /* 小数补前导 0：frac=5 dec=2 要输出 "05" */
    lim /= 10;
    while (lim > 1)
    {
        if (frac < lim)
        {
            buf[n ++] = '0';
        }
        else
        {
            break;
        }
        lim /= 10;
    }
    tune_put_i32(frac, &buf[n]);
}
