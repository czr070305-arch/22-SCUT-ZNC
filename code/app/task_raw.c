#include "task_raw.h"
#include "app_ui.h"
#include "algorithm/chassis.h"
#include "vision/vision.h"

/*
 * 摄像头原画面预览页
 *
 * 只把 mt9v03x_image 原样刷到屏上, 不做任何处理。
 * 用来调镜头调焦环、判断曝光是否合适、确认摄像头角度。
 *
 * ---------------------------------------------------------------
 * 关于镜头调焦环
 *
 *   M12 镜头前端有一圈可以拧的环 (有些镜头外面还有个锁紧环,
 *   要先松开锁紧环才能拧调焦环)。转动它改变镜头到传感器的距离,
 *   就是调焦:
 *     - 拧到某个位置画面最清晰 -> 这就是焦点
 *     - 顺时针/逆时针拧过头都会变模糊
 *   MT9V034 是全局快门, 但对焦和普通相机一样, 对着赛道
 *   (约 20~40cm 处的白线) 拧到最清楚即可。
 *
 *   注意: 调完要把锁紧环拧回去, 否则跑车时震动会让焦点漂。
 * ---------------------------------------------------------------
 *
 * 屏幕布局:
 *   y = 0  .. 80    原图 128x81
 *   y = 83 .. 103   三行状态
 *   y = 124..133    footer
 */

//1 = 画面冻结
static uint8  raw_frozen    = 0;
/* 当前曝光时间
 *
 * 初值取 vision_cfg.h 的 VISION_CAM_EXP_TIME, 不要自己写死数字。
 * 否则会出现: vision_init() 下发一个值, 本页显示另一个值,
 * 每次上电看到的都是这里写死的那个, 调的成果全丢。
 *
 * 摄像头本身不保存曝光(set_exposure_time 的数据不进 EEPROM),
 * 每次上电都必须由 MCU 重新下发, 所以 MCU 侧必须有一个确定的值。 */
static uint16 raw_exp_time  = VISION_CAM_EXP_TIME;
//帧计数
static uint32 raw_frame_cnt = 0;
//上次刷文字的时间
static uint32 raw_last_tick = 0;

//灰度统计 (每 8 帧扫一次)
static uint8  raw_gray_min = 0;
static uint8  raw_gray_max = 0;
static uint8  raw_gray_avg = 0;

/* --------------------------------------------------------------------------
 * 灰度统计
 *
 * 为什么要统计:
 *   调焦时肉眼只能看"清不清晰", 但过曝和欠曝同样会让白线看不清。
 *   MIN/MAX/AVG 能直接告诉你动态范围够不够:
 *     - MAX 长期 255 -> 过曝, 白线糊成一片
 *     - MAX - MIN < 60 -> 对比度太低, 阈值没法分
 *     - AVG 偏低但 MAX 正常 -> 画面整体偏暗, 加曝光
 * -------------------------------------------------------------------------- */
static void raw_scan_gray(void)
{
    uint32 sum = 0;
    uint8  lo  = 255;
    uint8  hi  = 0;
    uint8  v;
    uint16 x;
    uint16 y;

    for(y = 0; y < MT9V03X_H; y ++)
    {
        for(x = 0; x < MT9V03X_W; x ++)
        {
            v = mt9v03x_image[y][x];
            if(v < lo) { lo = v; }
            if(v > hi) { hi = v; }
            sum += v;
        }
    }

    raw_gray_min = lo;
    raw_gray_max = hi;
    raw_gray_avg = (uint8)(sum / MT9V03X_IMAGE_SIZE);
}

/* --------------------------------------------------------------------------
 * 字符串拼接 (6x8 字体, 128px / 6 = 21 字符是硬上限)
 * -------------------------------------------------------------------------- */
#define RAW_BUF_LIMIT       (20)

static uint8 raw_put_str(char *buf, uint8 n, const char *s)
{
    while(*s && n < RAW_BUF_LIMIT)
    {
        buf[n ++] = *s ++;
    }
    return n;
}

static uint8 raw_put_int(char *buf, uint8 n, int32 v)
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
        if(n + 1 + len > RAW_BUF_LIMIT) return RAW_BUF_LIMIT;
        buf[n ++] = '-';
    }
    else
    {
        if(n + len > RAW_BUF_LIMIT) return RAW_BUF_LIMIT;
    }

    while(0 != len)
    {
        buf[n ++] = tmp[-- len];
    }
    return n;
}

/* 补空格到固定长度: 否则上一帧更长的内容会残留在屏幕上 */
static void raw_pad(char *buf, uint8 n)
{
    while(n < RAW_BUF_LIMIT)
    {
        buf[n ++] = ' ';
    }
    buf[n] = '\0';
}

/* --------------------------------------------------------------------------
 * 绘制
 * -------------------------------------------------------------------------- */

//原图 (threshold=0 表示按灰度画, 不做二值化)
static void raw_draw_image(void)
{
    tft180_show_gray_image(RAW_IMG_X, RAW_IMG_Y,
                           &mt9v03x_image[0][0],
                           MT9V03X_W, MT9V03X_H,
                           RAW_IMG_W, RAW_IMG_H, 0);
}

//三行状态
static void raw_draw_info(void)
{
    char  buf[24];
    uint8 n;

    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(UI_PEN, UI_BG);

    //---- 第 0 行: 曝光 / 冻结状态 ----
    n = 0;
    n = raw_put_str(buf, n, "EXP ");
    n = raw_put_int(buf, n, (int32)raw_exp_time);
    if(raw_frozen)
    {
        n = raw_put_str(buf, n, " FRZ");
    }
    raw_pad(buf, n);
    tft180_show_string(0, RAW_ROW0, buf);

    //---- 第 1 行: 灰度分布 MIN/MAX/AVG ----
    n = 0;
    n = raw_put_str(buf, n, "G ");
    n = raw_put_int(buf, n, raw_gray_min);
    n = raw_put_str(buf, n, " ");
    n = raw_put_int(buf, n, raw_gray_max);
    n = raw_put_str(buf, n, " ");
    n = raw_put_int(buf, n, raw_gray_avg);
    raw_pad(buf, n);
    tft180_show_string(0, RAW_ROW1, buf);

    //---- 第 2 行: 帧计数 / 摄像头状态 ----
    n = 0;
    if(vision_cam_ok())
    {
        n = raw_put_str(buf, n, "FRM ");
        n = raw_put_int(buf, n, (int32)(raw_frame_cnt % 10000));
    }
    else
    {
        n = raw_put_str(buf, n, "CAM FAIL");
    }
    raw_pad(buf, n);
    tft180_show_string(0, RAW_ROW2, buf);
}

static void raw_draw_footer(void)
{
    ui_draw_footer_line(0, "M:Freeze U/D:Exp");
    ui_draw_footer_line(1, "L:Back");
}

/* --------------------------------------------------------------------------
 * 页面接口
 * -------------------------------------------------------------------------- */

void task_raw_show(void)
{
    raw_frozen    = 0;
    raw_frame_cnt = 0;
    raw_last_tick = ui_tick_get();

    /* 让视觉让出取帧权, 否则 vision_task() 会先抢走 finish_flag,
     * 本页画面永远不刷新 */
    vision_set_hold(1);

    //吃掉可能残留的完成标志, 避免进来先画一帧旧图
    mt9v03x_finish_flag = 0;

    ui_clear();
    raw_scan_gray();
    raw_draw_image();
    raw_draw_info();
    raw_draw_footer();
}

void task_raw_key(key_id_enum id)
{
    switch(id)
    {
        case KEY_ID_UP:
        {
            if(raw_exp_time + RAW_EXP_STEP > RAW_EXP_MAX)
            {
                raw_exp_time = RAW_EXP_MAX;
            }
            else
            {
                raw_exp_time = (uint16)(raw_exp_time + RAW_EXP_STEP);
            }
            mt9v03x_set_exposure_time(raw_exp_time);
            raw_draw_info();
            break;
        }

        case KEY_ID_DOWN:
        {
            if(raw_exp_time < RAW_EXP_MIN + RAW_EXP_STEP)
            {
                raw_exp_time = RAW_EXP_MIN;
            }
            else
            {
                raw_exp_time = (uint16)(raw_exp_time - RAW_EXP_STEP);
            }
            mt9v03x_set_exposure_time(raw_exp_time);
            raw_draw_info();
            break;
        }

        default: break;
    }
}

void task_raw_run(void)
{
    static uint8 raw_mid_last = 0;
    uint8  mid_now;
    uint32 now;

    //---- 中键: 冻结 / 解冻 ----
    mid_now = (KEY_ID_MID == hw_key_get()) ? 1 : 0;
    if(0 == mid_now && 1 == raw_mid_last)
    {
        raw_frozen = raw_frozen ? 0 : 1;
        raw_draw_info();
    }
    raw_mid_last = mid_now;

    //---- 新帧来了才刷图 ----
    if(0 == mt9v03x_finish_flag)
    {
        return;
    }
    mt9v03x_finish_flag = 0;

    raw_frame_cnt ++;

    if(raw_frozen)
    {
        return;
    }

    //每 8 帧统计一次灰度, 不要每帧扫 (扫一遍约 1~2ms)
    if(0 == (raw_frame_cnt & 0x07))
    {
        raw_scan_gray();
    }

    raw_draw_image();

    //文字每 200ms 刷一次, 不用跟着图像刷
    now = ui_tick_get();
    if(now - raw_last_tick >= RAW_REFRESH_MS)
    {
        raw_last_tick = now;
        raw_draw_info();
    }
}

void task_raw_leave(void)
{
    raw_frozen = 0;

    //把摄像头取帧权还给视觉模块
    vision_set_hold(0);
}
