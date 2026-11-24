#ifndef _TASK_RAW_H_
#define _TASK_RAW_H_

#include "zf_common_headfile.h"
#include "hw_key.h"

/*
 * 摄像头原画面预览页 (RAW)
 *
 * 干什么用的:
 *   只把 mt9v03x_image 原样刷到屏上, 不做任何处理。
 *   用来:
 *     1. 调镜头调焦环 (拧到画面最清晰)
 *     2. 调光圈 (看画面过曝/欠曝)
 *     3. 确认摄像头角度、视野范围
 *     4. 判断是不是摄像头本身有问题
 *
 * 和 TRACK 页的区别:
 *   TRACK: 二值图 + 中线/前瞻点覆盖层, 用于跑车调试
 *   RAW  : 纯灰度原图, 用于硬件调试
 *
 * 布局 (竖屏 128x160):
 *   y = 0  .. 80    原图 128x81 (188x120 等比缩放)
 *   y = 83 .. 103   三行状态 (曝光 / 灰度分布 / 帧率)
 *   y = 124          footer
 *
 * 按键:
 *   UP / DOWN   曝光时间 +- 64
 *   M           冻结 / 解冻画面
 *   LEFT        返回
 */

/* 图像显示尺寸: 188x120 等比缩放到 128 宽 -> 81 高 */
#define RAW_IMG_W           (128)
#define RAW_IMG_H           (81)
#define RAW_IMG_X           (0)
#define RAW_IMG_Y           (0)

/* 文字区起始 y */
#define RAW_INFO_Y0         ((uint16)(RAW_IMG_Y + RAW_IMG_H + 2))

/* 三行文字 */
#define RAW_ROW0            ((uint16)(RAW_INFO_Y0))
#define RAW_ROW1            ((uint16)(RAW_INFO_Y0 + 10))
#define RAW_ROW2            ((uint16)(RAW_INFO_Y0 + 20))

/* 曝光调节
 *
 * 下限从 64 降到 16: 你这颗镜头在 64 时 G avg 只有 27, 已经把
 * "判白更严格" 的余量用满了. 再往下留一档, 万一场地特别亮还有空间。
 * 步长 32 而不是 64: 64 太粗, 一按就跨过最佳点。 */
#define RAW_EXP_STEP        (32)
#define RAW_EXP_MIN         (16)
#define RAW_EXP_MAX         (2000)

/* 文字刷新周期(ms) */
#define RAW_REFRESH_MS      (200)

void task_raw_show(void);
void task_raw_key(key_id_enum id);
void task_raw_run(void);
void task_raw_leave(void);

#endif
