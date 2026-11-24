#ifndef _TASK_MOTOR_H_
#define _TASK_MOTOR_H_

#include "zf_common_headfile.h"
#include "hw_key.h"

/*
 * 电机 + 编码器测试页
 *
 * 两种模式, 长按中键切换:
 *
 *   闭环模式(默认) 标题 MOTOR TEST / MOTOR RUN
 *     >L TGT     50 rpm    目标转速, 当前选中那一路前面带 >
 *      L ACT     48 rpm    实际转速
 *     >R TGT     50 rpm
 *      R ACT     48 rpm
 *      L CNT   1024
 *      R CNT   1024
 *
 *   开环模式 标题 MOTOR OPEN / OPEN  RUN
 *     >L DUTY  3000        直接给占空比(0~10000), 绕过 PID
 *      L ACT   1234 rpm    实际转速仍然显示, 用来验证接线和编码器
 *     >R DUTY  3000
 *      R ACT   1234 rpm
 *      L CNT   1234
 *      R CNT   1234
 *      L mm      0
 *
 * 开环模式的用途:
 *   PID 参数还是 0 时闭环不出力, 开环可以直接给占空比验证:
 *     1. 轮子转不转        -> 查供电、驱动板 nSLEEP
 *     2. 转向对不对        -> 改 MOTOR_x_INVERT
 *     3. 编码器有没有计数  -> 查接线、编码器板 R-ABZ 有没有焊
 *   这三项确认完, 再回闭环调 PID
 *
 * 按键:
 *   S1            选中左轮
 *   S2            选中右轮
 *   上/下         选中那一路 +/- (闭环调 RPM, 开环调占空比)
 *   中键 短按     运行 / 停止
 *   中键 长按     切换闭环 / 开环模式
 *   左键          返回上级(菜单框架统一处理)
 *
 * 为什么左右键不参与调值:
 *   菜单框架把左键固定用作"返回", 任务页再占用会一边改参数一边退出页面
 */

/* 目标转速步长(RPM), 闭环模式用
 *
 * 25 而不是 100: 这个页面还要用来测"内轮死区"(CTRL_WHEEL_MIN_RPM),
 * 死区在 400~600 之间, 步长 100 只能测到 ±50 的精度, 不够。
 *
 * 调车嫌慢可以连按, 或者长按(主循环 100ms 一次, 连按 4 下 = 100 RPM)。 */
#define MOTOR_TARGET_STEP       (100)

//占空比步长, 开环模式用
//满量程 10000, 一次调 500 就是 5%
#define MOTOR_DUTY_STEP         (200)

//中键按住多久算长按(切模式)
#define MOTOR_HOLD_MODE_MS      (800)

//屏幕刷新间隔(ms), 太快会把主循环拖慢
#define MOTOR_REFRESH_MS        (100)

//数值行的 y 坐标, 6 行 x 16 像素, 占 24~119
//底部提示从 UI_FOOTER_Y(124) 开始, 和最后一行间隔 5 像素, 不重叠
//屏幕只有 128x160: 7 行数据(到 135)会压住提示行, 再多放一行提示(y=154~161)直接超出底边
#define MOTOR_ROW_COUNT         (6)
#define MOTOR_ROW_Y0            (24)
#define MOTOR_ROW_STEP          (16)

void task_motor_show(void);
void task_motor_key(key_id_enum id);
void task_motor_run(void);
void task_motor_leave(void);

#endif
