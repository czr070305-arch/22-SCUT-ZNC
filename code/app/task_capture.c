#include "task_capture.h"
#include "app_ui.h"
#include "algorithm/chassis.h"
#include "vision/vision.h"

/*
 * 图像采集页
 *
 * 把 mt9v03x_image 通过 UART3 逐字节传给电脑, 用于离线训练。
 *
 * ---------------------------------------------------------------------------
 * 核心问题: 发送期间 DVP 会覆盖图像
 *
 *   uart_write_buffer() 是阻塞的, 22560 字节 @115200 要接近 2 秒。
 *   这 2 秒里 DVP 的 DMA 一直在往 mt9v03x_image 写新帧,
 *   发出去的就是"前一半新的 + 后一半旧的"混合图。
 *
 *   解决: 发送前停 DMA + 停 DVP, 发送完恢复。
 *      DVP->CR1 &= ~RB_DVP_DMA_EN
 *      DVP->CR0 &= ~RB_DVP_ENABLE
 * ---------------------------------------------------------------------------
 */

/* 帧头长度 */
#define CAP_HDR_LEN         (9)

/* 连拍张数 */
#define CAP_BURST_10        (10)
#define CAP_BURST_50        (50)

/* 界面刷新周期 (ms) */
#define CAP_REFRESH_MS      (100)

static uint8  cap_seq      = 0;         /* 帧序号, 溢出回绕 */
static uint8  cap_burst    = 0;         /* 剩余连拍张数 */
static uint32 cap_sent     = 0;         /* 本次已发送张数 */
static uint32 cap_last_tick = 0;

/* ==========================================================================
 * DVP 采集的暂停 / 恢复
 * ========================================================================== */

/* 采集页"等一帧再停"标志。
 * 由 DVP 帧完成中断消费：中断触发的那一刻 DMA 刚写完最后一行，
 * 图像完整，当场关 DMA 就不会撕裂。
 * 定义成 volatile，因为中断里会改。 */
volatile uint8 cap_wait_frame = 0;

static void cap_dvp_pause(void)
{
    /* 清掉可能残留的完成标志，重新等一个新帧 */
    mt9v03x_finish_flag = 0;

    /* 请求中断在下一个帧完成点关 DMA。
     * 50fps 下最多等 20ms，超出说明摄像头没在出帧，
     * 降级为直接关，避免死锁。 */
    cap_wait_frame = 1;
    {
        uint32 t0 = 0;
        while (cap_wait_frame) {
            system_delay_us(200);
            t0 += 200;
            if (t0 > 40000u) {          /* 40ms 超时 */
                cap_wait_frame = 0;
                DVP->CR1 &= ~RB_DVP_DMA_EN;
                DVP->CR0 &= ~RB_DVP_ENABLE;
                break;
            }
        }
    }
    mt9v03x_finish_flag = 0;
}

static void cap_dvp_resume(void)
{
    DVP->CR1 |= RB_DVP_DMA_EN;
    DVP->CR0 |= RB_DVP_ENABLE;
    mt9v03x_finish_flag = 0;
}

/* ==========================================================================
 * 发送一帧
 *
 * 阻塞约 2 秒。期间屏幕和按键都不响应, 这是串口带宽决定的, 无法避免。
 * ========================================================================== */
static void cap_send_frame(void)
{
    uint8  hdr[CAP_HDR_LEN];
    uint8  chk = 0;
    uint8  i;
    uint16 y;

    /* ---- 1) 冻结图像 ---- */
    cap_dvp_pause();

    /* ---- 2) 组帧头 ---- */
    hdr[0] = 0xA5;
    hdr[1] = 0x5A;
    hdr[2] = 0x01;                      /* 版本 */
    hdr[3] = (uint8)MT9V03X_W;          /* 188 */
    hdr[4] = (uint8)MT9V03X_H;          /* 120 */
    hdr[5] = cap_seq;
    hdr[6] = (uint8)((g_thre > 0 && g_thre < 256) ? g_thre : 0);
    hdr[7] = 0x00;

    chk = 0;
    for(i = 0; i < 8; i ++)
    {
        chk ^= hdr[i];
    }
    hdr[8] = chk;

    /* ---- 3) 发帧头 ---- */
    uart_write_buffer(DEBUG_UART_INDEX, hdr, CAP_HDR_LEN);

    /* ---- 4) 逐行发像素 ----
     *
     * 逐行发而不是整块发, 是为了让看门狗/中断有机会插进来。
     * 如果以后加 WWDT, 在这里喂狗即可。
     */
    for(y = 0; y < MT9V03X_H; y ++)
    {
        uart_write_buffer(DEBUG_UART_INDEX,
                          (const uint8 *)&mt9v03x_image[y][0],
                          MT9V03X_W);
    }

    cap_seq ++;
    cap_sent ++;

    /* ---- 5) 恢复采集 ---- */
    cap_dvp_resume();
}

/* ==========================================================================
 * 绘制
 * ========================================================================== */

static void cap_draw_info(void)
{
    char  buf[24];
    uint8 n;

    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(UI_PEN, UI_BG);

    /* 第 0 行: 已发送张数 */
    n = 0;
    n += ui_num_to_str(cap_sent, &buf[n]);
    buf[n++] = ' ';
    buf[n++] = 'F';
    buf[n++] = 'R';
    buf[n++] = 'M';
    buf[n++] = 'S';
    while(n < 20) { buf[n ++] = ' '; }
    buf[n] = '\0';
    tft180_show_string(0, CAP_ROW0, buf);

    /* 第 1 行: 连拍剩余 */
    n = 0;
    if(cap_burst > 0)
    {
        n += ui_num_to_str(cap_burst, &buf[n]);
        buf[n++] = ' ';
        buf[n++] = 'L';
        buf[n++] = 'E';
        buf[n++] = 'F';
        buf[n++] = 'T';
    }
    else
    {
        n += ui_num_to_str(cap_sent, &buf[n]);
        buf[n++] = ' ';
        buf[n++] = 'S';
        buf[n++] = 'E';
        buf[n++] = 'N';
        buf[n++] = 'T';
    }
    while(n < 20) { buf[n ++] = ' '; }
    buf[n] = '\0';
    tft180_show_string(0, CAP_ROW1, buf);

    /* 第 2 行: 阈值 */
    n = 0;
    n += ui_num_to_str((uint32)((g_thre > 0) ? g_thre : 0), &buf[n]);
    buf[n++] = ' ';
    buf[n++] = 'T';
    buf[n++] = 'H';
    buf[n++] = 'R';
    while(n < 20) { buf[n ++] = ' '; }
    buf[n] = '\0';
    tft180_show_string(0, CAP_ROW2, buf);
}

static void cap_draw_footer(void)
{
    ui_draw_footer_line(0, "M:1  U:10  D:50");
    ui_draw_footer_line(1, "L:Back");
}

static void cap_redraw_all(void)
{
    ui_clear();
    ui_draw_title("CAPTURE");
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(UI_PEN, UI_BG);
    tft180_show_string(0, 24, "Send to PC via");
    tft180_show_string(0, 36, "UART3 B10/B11");
    tft180_show_string(0, 48, "115200 8N1");
    cap_draw_info();
    cap_draw_footer();
}

/* ==========================================================================
 * 页面接口
 * ========================================================================== */

void task_capture_show(void)
{
    cap_burst = 0;
    cap_sent  = 0;
    cap_last_tick = ui_tick_get();

    /* 让视觉让出摄像头, 否则 vision_task() 会抢走 finish_flag */
    vision_set_hold(1);
    vision_set_disp(0);

    /* 确保 DVP 处于运行状态 (上一次异常退出可能留在暂停态) */
    cap_dvp_resume();

    cap_redraw_all();
}

void task_capture_key(key_id_enum id)
{
    switch(id)
    {
        case KEY_ID_MID:   cap_burst = 1;               break;
        case KEY_ID_UP:    cap_burst = CAP_BURST_10;    break;
        case KEY_ID_DOWN:  cap_burst = CAP_BURST_50;    break;
        default: break;
    }
    cap_draw_info();
}

void task_capture_run(void)
{
    uint32 now;

    /* 有待发送的帧: 一次只发一张, 发完立刻刷新屏幕,
     * 这样连拍时屏幕上能看到进度在动 */
    if(cap_burst > 0)
    {
        cap_burst --;
        cap_send_frame();
        cap_draw_info();
    }

    /* 定时刷新, 让屏幕不至于长时间不更新 */
    now = ui_tick_get();
    if(now - cap_last_tick >= CAP_REFRESH_MS)
    {
        cap_last_tick = now;
        cap_draw_info();
    }
}

void task_capture_leave(void)
{
    cap_burst = 0;

    /* 恢复采集, 把摄像头还给视觉 */
    cap_dvp_resume();
    vision_set_hold(0);
}
