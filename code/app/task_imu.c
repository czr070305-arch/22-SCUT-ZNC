#include "task_imu.h"
#include "app_ui.h"
#include "hardware/hw_imu.h"

/* ============================================================================
 * IMU 角度测试页
 *
 * 目的只有一个：肉眼确认 IMU 到底能不能用。
 *   1. 通信是否正常      -> 标题栏显示 ONLINE / OFFLINE
 *   2. 零偏是否校准过    -> S1 校准，进度条走完才算好
 *   3. 角度准不准        -> 转车，看 YAW 跟不跟手、静止时漂不漂
 * ==========================================================================*/

//==================== 静态状态 ====================
static uint32 imu_last_tick = 0;
static uint8  imu_calibrating = 0;

//==================== 小工具 ====================
/*
 * 有符号小数转字符串，保留两位：-12.34 / 0.00 / 179.99
 *
 * ui_fmt_x100 只接受 uint16，符号要自己拼。
 * value 传进来是放大 100 倍的整数。
 */
static void imu_fmt_f2(float value, char *buf)
{
    uint8  n = 0;
    uint32 v;

    if (value < 0.0f)
    {
        buf[n++] = '-';
        v = (uint32)(-(int32)(value * 100.0f));
    }
    else
    {
        v = (uint32)((int32)(value * 100.0f));
    }

    n += ui_fmt_x100((uint16)v, &buf[n]);
    buf[n] = '\0';
}

/* 三个 int16 拼成 "a/b/c"，给角速度和零偏用 */
static void imu_fmt_triple(const int16 v[3], char *buf)
{
    uint8 n = 0;
    uint8 i;

    /* ui_num_to_str 不带符号，负数自己插 '-' */
    for (i = 0; i < 3; i++)
    {
        int32 x = (int32)v[i];

        if (x < 0)
        {
            buf[n++] = '-';
            x = -x;
        }
        n += ui_num_to_str((uint32)x, &buf[n]);
        if (i < 2)
        {
            buf[n++] = '/';
        }
    }
    buf[n] = '\0';
}

//==================== 绘制 ====================
static void imu_draw_title(void)
{
    if (!hw_imu_is_online())
    {
        ui_draw_title("IMU  OFFLINE");
        return;
    }
    if (imu_calibrating)
    {
        ui_draw_title("IMU CALIB...");
        return;
    }
    ui_draw_title("IMU  yaw");
}

/* 大字 YAW：转车时全靠它判断跟不跟手 */
static void imu_draw_yaw(void)
{
    imu_t  ang;
    char   buf[12];

    if (hw_imu_get_angle(&ang) != 0)
    {
        ui_draw_big_string(0, IMU_ROW_YAW_Y, "  ---", IMU_COLOR_WARN, UI_BG);
        return;
    }

    imu_fmt_f2(ang.yaw, buf);
    ui_draw_big_string(0, IMU_ROW_YAW_Y, buf, UI_PEN, UI_BG);
}

/* PITCH / ROLL，普通字号 */
static void imu_draw_angles(void)
{
    imu_t ang;
    char  buf[12];

    if (hw_imu_get_angle(&ang) != 0)
    {
        ui_draw_row_text(IMU_ROW_ANGLE_Y0,
                         " PITCH", "---", 0);
        ui_draw_row_text(IMU_ROW_ANGLE_Y0 + IMU_ROW_STEP,
                         "  ROLL", "---", 0);
        return;
    }

    imu_fmt_f2(ang.pitch, buf);
    ui_draw_row_text(IMU_ROW_ANGLE_Y0, " PITCH", buf, 0);

    imu_fmt_f2(ang.roll, buf);
    ui_draw_row_text(IMU_ROW_ANGLE_Y0 + IMU_ROW_STEP, "  ROLL", buf, 0);
}

/* 温度 + 当前角速度 + 零偏，用来判断"读到的数是不是真的" */
static void imu_draw_raw(void)
{
    imu_raw_struct raw;
    char  buf[20];
    uint16 y;

    y = IMU_ROW_ANGLE_Y0 + IMU_ROW_STEP * 2;

    if (hw_imu_get_raw(&raw) != 0)
    {
        ui_draw_row_text(y,                    "  TEMP", "---", 0);
        ui_draw_row_text(y + IMU_ROW_STEP,     " G X/Y/Z", "---", 0);
        ui_draw_row_text(y + IMU_ROW_STEP * 2, "  BIAS", "---", 0);
        return;
    }

    /* 温度：raw / 132.48 + 25 */
    {
        int32 t100 = (int32)(raw.temp_c * 100.0f);
        uint8 n = 0;

        if (t100 < 0) { buf[n++] = '-'; t100 = -t100; }
        n += ui_fmt_x100((uint16)t100, &buf[n]);
        buf[n++] = ' ';
        buf[n++] = 'C';
        buf[n]   = '\0';
        ui_draw_row_text(y, "  TEMP", buf, 0);
    }

    imu_fmt_triple(raw.gyro, buf);
    ui_draw_row_text(y + IMU_ROW_STEP, " G X/Y/Z", buf, 0);

    {
        int16 b[3];
        b[0] = hw_imu_calib_bias(0);
        b[1] = hw_imu_calib_bias(1);
        b[2] = hw_imu_calib_bias(2);
        imu_fmt_triple(b, buf);
        ui_draw_row_text(y + IMU_ROW_STEP * 2, "  BIAS", buf, 0);
    }
}

/* 校准进度条，只在校准期间显示 */
static void imu_draw_calib_bar(void)
{
    uint8  pct = hw_imu_calib_percent();
    char   buf[10];
    uint8  n = 0;

    n += ui_num_to_str(pct, &buf[n]);
    buf[n++] = '%';
    buf[n]   = '\0';

    ui_draw_row_center(IMU_ROW_ANGLE_Y0 + IMU_ROW_STEP * 3, buf, 0);
    ui_draw_rect(4, IMU_ROW_ANGLE_Y0 + IMU_ROW_STEP * 3 + 12,
                 (uint16)(120 * pct / 100), 4, IMU_COLOR_OK);
}

//==================== 页面接口 ====================

void task_imu_show(void)
{
    imu_last_tick  = ui_tick_get();
    imu_calibrating = 0;

    ui_clear();
    imu_draw_title();
    imu_draw_yaw();
    imu_draw_angles();
    imu_draw_raw();

    if (!hw_imu_is_online())
    {
        ui_draw_footer_line(0, "CHECK WIRE / CS");
        ui_draw_footer_line(1, "L:Back");
        return;
    }

    ui_draw_footer_line(0, "S1:Calib S2:Yaw0");
    ui_draw_footer_line(1, "L:Back");
}

void task_imu_key(key_id_enum id)
{
    switch (id)
    {
        case KEY_ID_S1:
        {
            if (!hw_imu_is_online())
            {
                break;
            }
            /* 车必须静止，否则把真实转动算进零偏 */
            imu_calibrating = 1;
            hw_imu_calib_start();
            imu_draw_title();
            ui_draw_footer_line(0, "HOLD STILL...");
            break;
        }

        case KEY_ID_S2:
        {
            if (!hw_imu_is_online())
            {
                break;
            }
            mahony_reset_yaw();
            imu_draw_yaw();
            break;
        }

        default:
            break;
    }
}

void task_imu_run(void)
{
    /* 校准期间不受刷新间隔限制，进度条要跟得上 */
    if (imu_calibrating)
    {
        if (hw_imu_calib_poll())
        {
            imu_calibrating = 0;
            imu_draw_title();
            ui_draw_footer_line(0, "S1:Calib S2:Yaw0");
        }
        imu_draw_calib_bar();
        return;
    }

    if (ui_tick_get() - imu_last_tick < IMU_REFRESH_MS)
    {
        return;
    }
    imu_last_tick = ui_tick_get();

    imu_draw_yaw();
    imu_draw_angles();
    imu_draw_raw();
}

void task_imu_leave(void)
{
    imu_calibrating = 0;
}
