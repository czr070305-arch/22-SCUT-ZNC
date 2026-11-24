#ifndef _TASK_TRACK_H_
#define _TASK_TRACK_H_

#include "zf_common_headfile.h"
#include "hw_key.h"

/*
 * 循迹调试页 (TRACK) —— 上车跑的主页面
 *
 * 页面布局 (竖屏 TFT180 128x160):
 *
 *   +--------------------------------+  y=0
 *   |                                |
 *   |   摄像头二值图 128 x 81         |
 *   |   + 红色中线点                  |
 *   |   + 绿色前瞻十字                |
 *   |   + 黄色折角竖线                |
 *   |                                |
 *   +--------------------------------+  y=81
 *   |  E-25.2 L520 R520              |  误差 / 左右目标转速
 *   |  RUN V800 T-105                |  状态 / 巡航速度 / 转向量
 *   |  ACT 518 522                   |  实际转速
 *   +--------------------------------+  y=103
 *   |  M:Start/Stop                  |  footer
 *   |  U/D:Speed L:Back              |
 *   +--------------------------------+  y=160
 *
 * ---------------------------------------------------------------
 * 画面由谁画
 *
 *   图像和覆盖层全部交给 vision_display.c 的 disp_frame():
 *     原图 + 本帧阈值直接判黑白 -> 屏幕上就是当前阈值下的二值图
 *     再叠加中线/前瞻点/折角, 一眼能看出爬线跟得对不对
 *   本文件只负责 image 区下方的三行文字和 footer。
 *
 *   刷图节奏由 vision_cfg.h 的 CFG_DISP_EVERY_N 控制(默认每 5 帧),
 *   TRACK_DRAW_EVERY 只是本页的开关。
 *
 * ---------------------------------------------------------------
 * 三行文字含义
 *
 *   E  = 视觉转向误差(像素当量), 正=线在右需右转, 限幅 ±110
 *   L/R= 左右轮目标转速(RPM), 由 base±turn 合成
 *   RUN/LOST/STOP/IDLE = 控制状态
 *   V  = 巡航速度设定(RPM), UP/DOWN 可调 600~1200
 *   T  = 本拍转向分量(RPM), 与 E 反号
 *   ACT= 左右轮实测转速(RPM), 用来判断速度环跟不跟得上
 *
 * ---------------------------------------------------------------
 * 按键
 *
 *   M 键      启动 / 停止 (松手瞬间触发)
 *   UP / DOWN 巡航速度 +- 100 RPM (立即生效, 掉电不保存)
 *   LEFT      返回任务列表
 *
 * ---------------------------------------------------------------
 * 关于丢线
 *
 *   视觉找不到线时 err_valid=0, 控制层按三阶段处理:
 *     0~100ms    保持上一拍转向, 速度 x0.75
 *     100~400ms  转向回正, 速度 x0.5
 *     400ms+     刹停, 状态变 STOP, 需要重按 M 启动
 *   所以宿舍里对着地板按 M, 车会转几下就停 —— 这是保护在起作用。
 */

/* 刷图由 vision_display.c 的 disp_frame() 负责, 节奏由 vision_cfg.h
 * 的 CFG_DISP_EVERY_N 控制(默认每 5 帧)。这里只作为开关:
 *   1 = 进本页时允许视觉刷图 (当前)
 *   0 = 本页只显示文字, 不刷图 */
#define TRACK_DRAW_EVERY        (1)

/* 巡航速度调整步长 (RPM) */
#define TRACK_SPEED_STEP        (100)

/* 巡航速度调节范围 (RPM)
 * 下限 600: 低于它差速空间只剩 80 RPM, 弯道必过不去
 * 上限 1200: 370 电机在这个转速附近已接近极限 */
#define TRACK_SPEED_MIN         (600)
#define TRACK_SPEED_MAX         (1200)

/* 页面刷新周期(ms)
 * 停车时: 100ms 全量刷新, 方便调参看数值
 * 跑车时: 200ms, 尽量减少 SPI 刷字对主循环视觉轮询的占用 */
#define TRACK_REFRESH_MS        (100)
#define TRACK_RUN_REFRESH_MS    (200)

/* 图像显示尺寸: 188x120 等比缩放到 128 宽 -> 81 高 */
#define TRACK_IMG_W             (128)
#define TRACK_IMG_H             (81)
#define TRACK_IMG_X             (0)
#define TRACK_IMG_Y             (0)

/* 文字区起始 y */
#define TRACK_INFO_Y0           ((uint16)(TRACK_IMG_Y + TRACK_IMG_H + 2))

/* 三行文字 */
#define TRACK_ROW0              ((uint16)(TRACK_INFO_Y0))
#define TRACK_ROW1              ((uint16)(TRACK_INFO_Y0 + 10))
#define TRACK_ROW2              ((uint16)(TRACK_INFO_Y0 + 20))

void task_track_show(void);
void task_track_key(key_id_enum id);
void task_track_run(void);
void task_track_leave(void);

#endif
