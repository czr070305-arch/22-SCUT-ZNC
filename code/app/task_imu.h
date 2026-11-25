#ifndef _TASK_IMU_H_
#define _TASK_IMU_H_

#include "zf_common_headfile.h"
#include "hw_key.h"

/*
 * IMU 角度测试页
 *
 * 用途：验证 ICM-42688-P 能不能正常通信、Mahony 解算出来的角度对不对。
 *
 * 屏幕布局（128x160）：
 *
 *   IMU  yaw                    标题栏
 *   --------------------------
 *     YAW      12.34           大字，最醒目，转车看它跟不跟手
 *    PITCH     -1.05
 *     ROLL      0.42
 *    TEMP      28.5 C
 *    GX/GY/GZ  2/1/0           当前角速度 LSB，静态时应接近 0
 *    BIAS      -3/5/2          校准出来的零偏
 *
 * 操作：
 *   S1            开始零偏校准（此时车必须静止）
 *   S2            把 yaw 归零，方便观察相对转角
 *   返回          由菜单统一处理
 *
 * 为什么要单独一个页面：
 *   陀螺零偏每颗片子都不一样，还会随温度漂。上电不校准直接跑，
 *   静止时 yaw 会缓慢往一个方向爬，看起来像"算法有问题"。
 *   校准后静止 10 秒 yaw 漂移应该小于 1 度。
 */

/* 屏幕刷新的最小间隔(ms)。
 * 这个页面只是给人看的，20Hz 已经够流畅；刷太快会挤占主循环里的 I2C 采样。 */
#define IMU_REFRESH_MS          (50)

/* 行位置：标题栏占 0~16，数据从 24 开始，每行 16 像素 */
#define IMU_ROW_YAW_Y           (24)
#define IMU_ROW_ANGLE_Y0        (60)    /* PITCH / ROLL 起始 */
#define IMU_ROW_STEP            (16)

#define IMU_COLOR_OK            (RGB565_GREEN)
#define IMU_COLOR_WARN          (RGB565_YELLOW)
#define IMU_COLOR_ERR           (RGB565_RED)

void task_imu_show(void);
void task_imu_key(key_id_enum id);
void task_imu_run(void);
void task_imu_leave(void);

#endif /* _TASK_IMU_H_ */
