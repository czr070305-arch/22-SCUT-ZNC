#ifndef _TASK_CAPTURE_H_
#define _TASK_CAPTURE_H_

#include "zf_common_headfile.h"
#include "hw_key.h"

/*
 * 图像采集页 (CAPTURE)
 *
 * 干什么:
 *   把摄像头原图通过 UART3 (调试串口, B10/B11) 传到电脑,
 *   电脑端用 Python 脚本接收并存成 PNG, 用于离线训练。
 *
 * ---------------------------------------------------------------------------
 * 为什么需要这一页
 *
 *   调试串口只有 115200 bps, 约 11.5 KB/s。
 *   一帧 188x120 = 22560 字节, 传一帧要接近 2 秒。
 *
 *   如果边采集边发送, DVP 的 DMA 会在这 2 秒里持续覆盖 mt9v03x_image,
 *   发出去的数据是"上半帧新的 + 下半帧旧的"混合图, 没法用。
 *
 *   所以发帧前必须先把采集停掉:
 *     DVP->CR1 &= ~RB_DVP_DMA_EN;   停 DMA
 *     DVP->CR0 &= ~RB_DVP_ENABLE;   停 DVP
 *   发完再恢复。
 *
 * ---------------------------------------------------------------------------
 * 协议 (二进制, 不走 VOFA)
 *
 *   PC -> MCU: 单字节 'C' (0x43)  请求一帧
 *   MCU -> PC: 9 字节帧头 + 22560 字节像素
 *
 *   帧头:
 *     [0] 0xA5        同步字节 1
 *     [1] 0x5A        同步字节 2
 *     [2] 0x01        格式版本
 *     [3] width       固定 188
 *     [4] height      固定 120
 *     [5] seq         帧序号(自增, 溢出回绕)
 *     [6] thr         当前二值化阈值(调试用, 0=未算)
 *     [7] flags       预留, 当前 0
 *     [8] checksum    前 8 字节异或
 *
 *   像素: 逐行从上到下, 每行从左到右, 1 字节一个灰度。
 *
 * ---------------------------------------------------------------------------
 * 按键
 *   M      发送当前帧
 *   UP     连拍 10 帧
 *   DOWN   连拍 50 帧
 *   LEFT   返回
 *
 * ---------------------------------------------------------------------------
 * 使用步骤
 *   1. 板子连 USB 转串口到电脑, 打开对应 COM 口
 *   2. 打开 capture/recv_images.py, 填 COM 口和波特率
 *   3. 运行脚本, 脚本会周期性发 'C'
 *   4. 板子进 CAPTURE 页, 按 M / UP / DOWN
 *
 *   注意: 采集期间预览页和 TRACK 页的摄像头访问都要让出,
 *         task_capture_show() 里已经调用 vision_set_hold(1)。
 */

/* 文字行位置 */
#define CAP_ROW0            (24)
#define CAP_ROW1            (36)
#define CAP_ROW2            (48)

/* 采集状态 */
typedef enum
{
    CAPTURE_IDLE = 0,       /* 空闲, 等按键 */
    CAPTURE_SENDING,        /* 正在发送 (阻塞, 每帧约 2 秒) */
} capture_state_enum;

void task_capture_show(void);
void task_capture_key(key_id_enum id);
void task_capture_run(void);
void task_capture_leave(void);

#endif
