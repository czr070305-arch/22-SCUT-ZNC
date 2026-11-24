#include "task_track.h"
#include "app_ui.h"
#include "app_menu.h"
#include "algorithm/chassis.h"
#include "algorithm/ctrl.h"
#include "algorithm/tune.h"
#include "task_vofa.h"

#include "vision/vision.h"

/* ==========================================================================
 * 本文件是 视觉 <-> 控制 的适配层
 * --------------------------------------------------------------------------
 * 它是全工程唯一同时知道"视觉"和"控制"两个模块的文件。
 *
 * 职责:
 *   track_vision_err()   把视觉的转向误差喂给控制层
 *   track_is_valid()     本帧视觉可不可信(丢线时控制层要降速/停车)
 *
 * 设计原则: ctrl.c 和 vision 模块互不 include。
 *   换视觉算法 -> 只改本文件的三个函数
 *   换控制算法 -> 只改 ctrl.c
 * ========================================================================== */

/* ---------- 视觉接口薄封装 (统一入口, 换算法只改这一段) ---------- */
#define VIS_OUT             vision_get_output()
#define VIS_CAM_OK()        vision_cam_ok()
#define VIS_SET_DISP(on)    vision_set_disp(on)
#define VIS_DRAW_FRAME()    disp_frame()
#define VIS_BIAS_GET()      vision_get_thre_bias()
#define VIS_BIAS_SET(v)     vision_set_thre_bias(v)

//页面自己的状态
static uint8  track_mid_last  = 0;                 //中键上一圈的电平, 做边沿检测
static uint8  track_running   = 0;                 //1=正在跑
/* 巡航速度统一走 tune 的 TUNE_CRUISE，不再维护第二份副本 */  //巡航速度, 按键可调
static uint32 track_last_tick = 0;

/* ---- TRACK 页在线调参状态 ----
 * TRACK_FLASH_SHOW_MS: 存盘结果在屏幕右上角显示多久 */
static uint32 track_flash_msg_ms  = 0;   /* 最近一次 Flash 操作时刻 */
static uint8  track_flash_msg_ok  = 1;   /* 1=成功 / 0=失败 */

#define TRACK_FLASH_SHOW_MS   (1200)                 //上次刷文字

/* --------------------------------------------------------------------------
 * 视觉数据源
 *
 * 输出结构里对控制有用的字段:
 *   err        转向误差, 单位"像素当量", 右为正。限幅 ±110。
 *   err_valid  1=本帧可信; 0=丢线/路径太短, 控制层按丢线保护处理。
 *
 * 上电后到第一帧视觉之间, err_valid 初值是 0, 所以控制层会走丢线保护
 * 而不是按 err=0 直接起步。
 * -------------------------------------------------------------------------- */
static float track_vision_err(void)
{
    return VIS_OUT->err;
}

static uint8 track_is_valid(void)
{
    return (uint8)VIS_OUT->err_valid;
}

/* 拼接缓冲区上限
 *
 * tft180_show_string 用 6x8 字体时第 j 个字符画在 x + j*6,
 * 屏宽 128 -> 最多 21 字符, 超过就撞 tft180_set_region 的 zf_assert 死机.
 * 这里取 20 (120px), 留 8px 余量. */
#define TRACK_BUF_LIMIT         (20)

/* --------------------------------------------------------------------------
 * 整数转字符串 (带长度保护)
 * -------------------------------------------------------------------------- */
static uint8 track_put_int(char *buf, uint8 n, int32 v)
{
    char   tmp[12];
    uint8  len = 0;
    uint32 u;

    if(v < 0) { u = (uint32)(-v); }
    else      { u = (uint32)v;  }

    if(0 == u)
    {
        tmp[len ++] = '0';
    }
    while(0 != u)
    {
        tmp[len ++] = (char)('0' + (u % 10));
        u /= 10;
    }

    if(v < 0)
    {
        if(n + 1 + len > TRACK_BUF_LIMIT) return TRACK_BUF_LIMIT;
        buf[n ++] = '-';
    }
    else
    {
        if(n + len > TRACK_BUF_LIMIT) return TRACK_BUF_LIMIT;
    }

    while(0 != len)
    {
        buf[n ++] = tmp[-- len];
    }
    return n;
}

/* 一位小数, 入参是放大 10 倍的整数. 例如 123 -> "12.3" */
static uint8 track_put_x10(char *buf, uint8 n, int32 vx10)
{
    char   tmp[12];
    uint8  len = 0;
    uint8  neg = 0;
    uint32 u;
    uint8  one;
    uint8  need;

    if(vx10 < 0)
    {
        neg = 1;
        /* -(vx10+1)+1 避免 INT32_MIN 直接取反溢出 */
        u = (uint32)(-(vx10 + 1)) + 1u;
    }
    else
    {
        u = (uint32)vx10;
    }

    /* 小数位必须在除 10 之前取, 否则拿到的是整数部分的最低位 */
    one = (uint8)(u % 10);
    u  /= 10;

    if(0 == u)
    {
        tmp[len ++] = '0';
    }
    while(0 != u)
    {
        tmp[len ++] = (char)('0' + (u % 10));
        u /= 10;
    }

    need = (uint8)(len + 2 + (neg ? 1 : 0));
    if(n + need > TRACK_BUF_LIMIT)
    {
        return TRACK_BUF_LIMIT;
    }

    if(neg) buf[n ++] = '-';
    while(0 != len)
    {
        buf[n ++] = tmp[-- len];
    }
    buf[n ++] = '.';
    buf[n ++] = (char)('0' + one);
    return n;
}

static uint8 track_put_str(char *buf, uint8 n, const char *s)
{
    while(*s && n < TRACK_BUF_LIMIT)
    {
        buf[n ++] = *s ++;
    }
    return n;
}

/* 补空格到固定长度
 *
 * 为什么必须补:
 *   tft180_show_string 只画字符串里有的字符, 不擦除右侧旧内容.
 *   上一帧如果写了 20 字符, 这一帧只写 16 字符, 剩下 4 个字符
 *   会留在屏幕上, 看起来就是 "E14.1 L0 R00 L0 R0" 这种鬼影. */
static void track_pad(char *buf, uint8 n)
{
    while(n < TRACK_BUF_LIMIT)
    {
        buf[n ++] = ' ';
    }
    buf[n] = '\0';
}

/* --------------------------------------------------------------------------
 * 绘制
 * -------------------------------------------------------------------------- */

//画摄像头画面: 由视觉模块负责(二值图 + 边沿/中线/前瞻点覆盖层)
static void track_draw_image(void)
{
#if (TRACK_DRAW_EVERY > 0)
    if(VIS_CAM_OK())
    {
        VIS_DRAW_FRAME();
    }
    else
    {
        tft180_set_font(TFT180_8X16_FONT);
        tft180_set_color(RGB565_RED, UI_BG);
        tft180_show_string(8, 24, "CAMERA");
        tft180_show_string(8, 44, "FAIL");
        tft180_set_font(TFT180_6X8_FONT);
        tft180_set_color(UI_PEN, UI_BG);
        tft180_show_string(0, 70, "check FPC/3V3");
    }
#endif
}

/* 页面是否在跑车(控制层真正在驱动电机)
 *
 * 用途: 跑车时屏幕刷新必须降频 —— 一次 tft180_show_string 在 SPI 上
 * 阻塞 0.5~1ms, 三行加起来 2~3ms, 会拖慢主循环轮询视觉的节奏
 * (学长建议: 发车之后让其他代码尽量不运行, 保证快速轮询)。
 *
 * 判定用 RUN/LOST 而不是"!= IDLE": STOPPED 是丢线超时后自己刹停,
 * 此时车已经不动了, 恢复高频刷新方便看调试数据。 */
static uint8 track_is_driving(void)
{
    ctrl_state_enum st = ctrl_get_state();
    return (uint8)((CTRL_STATE_RUN == st) || (CTRL_STATE_LOST == st));
}

//画三行状态文字
static void track_draw_info(void)
{
    char  buf[24];
    uint8 n;
    float l, r, t;
    ctrl_state_enum st;

    ctrl_get_output(&l, &r, &t);
    st = ctrl_get_state();

    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(UI_PEN, UI_BG);

    /* 三行都用 6x8 字体, 屏宽 128px / 6 = 21 字符是硬上限.
     * 超过就撞 tft180_set_region 里的 zf_assert -> while(1) 死机.
     * 所以格式刻意压紧, 且 track_put_* 全部按 TRACK_BUF_LIMIT=20 截断.
     *
     * 最坏情况 (err=-110.0, L/R=1000):
     *   "E-110.0 L-1000 R1000"  <- 20 字符, 120px, 安全
     */

    //---- 第 0 行: 视觉误差 / 左右目标 ----
    n = 0;
    n = track_put_str(buf, n, "E");
    n = track_put_x10(buf, n, (int32)(track_vision_err() * 10.0f));
    n = track_put_str(buf, n, " L");
    n = track_put_int(buf, n, (int32)l);
    n = track_put_str(buf, n, " R");
    n = track_put_int(buf, n, (int32)r);
    track_pad(buf, n);
    tft180_show_string(0, TRACK_ROW0, buf);

    //---- 第 1 行: 状态 / 巡航速度 / 转向量 / 视觉状态 ----
    n = 0;
    switch(st)
    {
        case CTRL_STATE_RUN:     n = track_put_str(buf, n, "RUN ");  break;
        case CTRL_STATE_LOST:    n = track_put_str(buf, n, "LOST");  break;
        case CTRL_STATE_STOPPED: n = track_put_str(buf, n, "STOP");  break;
        default:                 n = track_put_str(buf, n, "IDLE");  break;
    }
    n = track_put_str(buf, n, " V");
    n = track_put_int(buf, n, (int32)tune_get(TUNE_CRUISE));
    n = track_put_str(buf, n, " T");
    n = track_put_int(buf, n, (int32)t);

    /* 末尾两个字符: 视觉状态
     *   第 1 个: + = err_valid=1 正常跟线 / - = 丢线(E 是旧值)
     *   第 2 个: 元素/折角类型
     *     I 环岛 / F 岔路 / C 十字 / L 左折角 / R 右折角 / N 普通
     */
    {
        n = track_put_str(buf, n, (VIS_OUT->err_valid) ? " +" : " -");

        if (VIS_OUT->island_on)         { n = track_put_str(buf, n, "I"); }
        else if (VIS_OUT->elem == 1)    { n = track_put_str(buf, n, "C"); }
        else if (g_corner_dir > 0)      { n = track_put_str(buf, n, "R"); }
        else if (g_corner_dir < 0)      { n = track_put_str(buf, n, "L"); }
        else                            { n = track_put_str(buf, n, "N"); }
    }

    track_pad(buf, n);
    tft180_show_string(0, TRACK_ROW1, buf);

    //---- 第 2 行: 实际转速 / 视觉诊断 ----
    n = 0;
    n = track_put_str(buf, n, "A");
    n = track_put_int(buf, n, chassis_get_speed(ENC_LEFT));
    n = track_put_str(buf, n, " ");
    n = track_put_int(buf, n, chassis_get_speed(ENC_RIGHT));

    /* 显示二值化阈值和偏移 */
    n = track_put_str(buf, n, " T");
    n = track_put_int(buf, n, g_thre);
    {
        int b = VIS_BIAS_GET();
        if(b > 0)      { n = track_put_str(buf, n, "+"); n = track_put_int(buf, n, b); }
        else if(b < 0) { n = track_put_str(buf, n, "-"); n = track_put_int(buf, n, -b); }
    }

    track_pad(buf, n);
    tft180_show_string(0, TRACK_ROW2, buf);
}

/* --------------------------------------------------------------------------
 * TRACK 页底部：在线调参行
 *
 *   >KP_LN   1.8   *
 *
 *   选择 = 屏幕用 '>' 指示
 *   *    = 还没落盘（手指停下 300ms 会自动写 Flash）
 *   FLASH OK / FLASH ERR 提示显示在右上角，1.2 秒后消失
 * -------------------------------------------------------------------------- */
#define TRACK_TUNE_ROW      ((uint16)(UI_FOOTER_Y - 10))

static void track_draw_tune(void)
{
    char  buf[24];
    char  val[12];
    uint8 n;
    int   sel = tune_get_sel();

    /* 左半: ">KP_LN" */
    n = 0;
    buf[n ++] = (sel >= 0) ? '>' : ' ';
    {
        const char *nm = tune_name(sel);
        uint8 i = 0;
        while (nm[i] && n < 8)
        {
            buf[n ++] = nm[i ++];
        }
        while (n < 8)
        {
            buf[n ++] = ' ';
        }
    }

    /* 右半: 数值 + 未落盘标记 */
    tune_value_str(sel, val);
    {
        uint8 i = 0;
        while (val[i] && n < 16)
        {
            buf[n ++] = val[i ++];
        }
    }
    while (n < 16)
    {
        buf[n ++] = ' ';
    }
    buf[16] = '\0';

    /* 未落盘：在行尾打 '*'，用不同颜色强调 */
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(UI_PEN, UI_BG);
    tft180_show_string(0, TRACK_TUNE_ROW, buf);

    if (tune_is_dirty())
    {
        tft180_set_color(RGB565_RED, UI_BG);
        tft180_show_string(16 * 6, TRACK_TUNE_ROW, "*");
    }

    /* Flash 操作结果提示：右上角覆盖 (只显示很短时间，不占用常态界面) */
    if (track_flash_msg_ms != 0 &&
        (ui_tick_get() - track_flash_msg_ms) < TRACK_FLASH_SHOW_MS)
    {
        tft180_set_color(track_flash_msg_ok ? RGB565_GREEN : RGB565_RED, UI_BG);
        tft180_show_string(56, TRACK_TUNE_ROW,
                           track_flash_msg_ok ? "FLASH OK" : "FAIL");
    }

    tft180_set_color(UI_PEN, UI_BG);
}

static void track_draw_footer(void)
{
    ui_draw_footer_line(0, "U/D:Sel S1/S2:Adj");
    ui_draw_footer_line(1, "R:Save  M:Run");
}

//整页重画(进页面时调)
static void track_redraw_all(void)
{
    ui_clear();
    track_draw_footer();
    track_draw_image();
    track_draw_info();
    track_draw_tune();
}

/* --------------------------------------------------------------------------
 * 页面接口
 * -------------------------------------------------------------------------- */

void task_track_show(void)
{
    track_running   = 0;
    track_flash_msg_ms  = 0;
    track_last_tick = ui_tick_get();

    //进页面时底盘必须停, 而且要把控制层也复位
    //否则从电机页跳过来时, 控制层还留着上次的 RUN 状态
    ctrl_stop();
    ctrl_set_cruise(tune_get(TUNE_CRUISE));
    track_mid_last = 0;
    vofa_release();

    //只有本页允许视觉刷图; 其它页面视觉照常算, 但不往屏幕上画
    VIS_SET_DISP(1);

    track_redraw_all();
}

void task_track_key(key_id_enum id)
{
    /* UP/DOWN 双用途:
     *   运行时(RUN/LOST)  -> 视觉阈值调节 (看着画面实时调)
     *   未运行时(IDLE/STOP)-> 调巡航速度
     *
     * 两者在时间上天然不重叠: 调阈值必须看画面, 那时车在跑;
     * 调速度时车是停的。
     */


    /* 调参只在停车状态生效：跑动中 UP/DOWN/S1/S2/RIGHT 全部忽略，
     * 只保留 MID（启停）和 LEFT（返回菜单，由 app_menu 处理）。
     * 这样跑车时不会误触改掉参数。 */
    /* 跑动中禁止调参键：MID（启停）和 LEFT（返回菜单）除外。
     * 这样跑车时不会误触把参数改掉。 */
    uint8 running = (CTRL_STATE_RUN == ctrl_get_state())
                 || (CTRL_STATE_LOST == ctrl_get_state());

    if (running && (KEY_ID_UP == id || KEY_ID_DOWN == id ||
                    KEY_ID_S1 == id || KEY_ID_S2 == id ||
                    KEY_ID_RIGHT == id))
    {
        return;
    }

    switch(id)
    {
        case KEY_ID_UP:
        {
            /* 停车状态：UP = 选中上一个参数
             * （跑动中已被上面的守卫拦掉；阈值并入参数表 THRE，
             *   不再保留第二条调节路径，避免两处状态不同步） */
            tune_sel_prev();
            track_draw_tune();
            track_draw_info();
            break;
        }

        case KEY_ID_DOWN:
        {
            /* 停车状态：DOWN = 选中下一个参数 */
            tune_sel_next();
            track_draw_tune();
            track_draw_info();
            break;
        }

        case KEY_ID_S1:
        {
            /* 左肩键：当前选中参数减一档 */
            tune_step(-1);
            track_draw_tune();
            break;
        }

        case KEY_ID_S2:
        {
            /* 右肩键：当前选中参数加一档 */
            tune_step(+1);
            track_draw_tune();
            break;
        }

        case KEY_ID_RIGHT:
        {
            /* RIGHT = 保存到 Flash
             * 没改过也返回成功（内部直接返回，不擦 Flash） */
            uint32 now = ui_tick_get();
            uint8  err = tune_do_save();

            track_flash_msg_ms = now;
            track_flash_msg_ok = (0 == err) ? 1 : 0;
            track_draw_tune();
            break;
        }

        default: break;
    }
}

void task_track_run(void)
{
    uint32 now = ui_tick_get();
    uint8  id  = (uint8)hw_key_get();

    //---- 中键: 启动/停止 (松手瞬间触发, 和电机页保持一致) ----
    {
        uint8 mid_now = (KEY_ID_MID == (key_id_enum)id) ? 1 : 0;

        if(0 == mid_now && 1 == track_mid_last)
        {
            if(track_running)
            {
                track_running = 0;
                ctrl_stop();
                //停车: 恢复视觉刷图(车不动, SPI 占用无所谓, 方便看线)
                VIS_SET_DISP(1);
            }
            else
            {
                track_running = 1;
                ctrl_set_cruise(tune_get(TUNE_CRUISE));
                ctrl_start();
                //跑车: 关掉视觉刷图. 一次 disp_frame() 在 SPI 上阻塞
                //10~13ms, 会让主循环一圈超过 20ms, 直接拖慢视觉轮询
                //(学长建议: 发车之后不要让刷屏占用时间)
                VIS_SET_DISP(0);
            }
            track_draw_info();
        }
        track_mid_last = mid_now;
    }


    /* ---- 把参数表的 THRE 送到视觉阈值 ----
     *
     * 为什么不放 tune_step()：tune.c 只管参数，不该反过来依赖视觉模块。
     * task_track.c 本来就是视觉与控制之间唯一的桥，放这里最合适。
     *
     * 每圈同步一次，开销只有一条赋值；好处是不论参数从哪条路径被改
     * （按键 / 恢复默认 / 上电载入），视觉阈值都在下一圈跟上，
     * 不会再出现"屏幕显示 20、实际还是 4"的情况。 */
    VIS_BIAS_SET((int)tune_get(TUNE_THRE_BIAS));

    //---- 喂视觉误差 ----
    //每圈都喂, 即使停车也喂: 让视觉始终跑着, 启动时第一帧就有数据
    ctrl_set_error(track_vision_err(), track_is_valid());

    //---- 控制节拍 ----
    //ctrl_tick() 已移入 TIM7 中断, 和速度环严格同一拍(见 isr.c).
    //主循环不再需要轮询 chassis_get_tick() 触发控制, 只做状态同步:
    //控制层因为丢线超时自己刹了车 -> 页面同步回停车态
    if(track_running && CTRL_STATE_STOPPED == ctrl_get_state())
    {
        track_running = 0;
        //车已刹停, 恢复视觉刷图
        VIS_SET_DISP(1);
        track_draw_info();
    }

    //---- 文字定时刷新 ----
    //跑车时降到 200ms: 刷三行文字在 SPI 上约 2~3ms, 跑车时少占一点;
    //停车时保持 100ms 全量刷新, 调参时能看清数值变化
    {
        uint32 period = track_is_driving() ? TRACK_RUN_REFRESH_MS
                                           : TRACK_REFRESH_MS;

        if(now - track_last_tick >= period)
        {
            track_last_tick = now;
            track_draw_info();
            /* 调参行也要定时刷：* 号（未落盘）只有重画才会消失。
             * 以前只刷 track_draw_info()，停手后 * 一直挂着，
             * 让人以为没存上，其实 300ms 后早就写进 Flash 了。 */
            track_draw_tune();
        }
    }
}

void task_track_leave(void)
{
    track_running = 0;
    ctrl_stop();

    //离开本页: 视觉停止刷图, 把屏幕还给菜单
    VIS_SET_DISP(0);
}
