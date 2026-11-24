#ifndef _V2_ELEM_H_
#define _V2_ELEM_H_

#include "vision.h"

/*
 * v2_elem.h —— 学长的元素识别算法 (独立模块)
 *
 * 只做元素识别, 不碰二值化和爬线。
 * 输入是第一版 vision_track.c 的爬线结果 + 摄像头原始灰度。
 *
 * 提供两个函数:
 *   v2e_detect_corner()   直角弯 -> 写 g_corner_dir
 *   v2e_detect_island()   环岛   -> 返回 -1 左 / +1 右 / 0 无
 */

/* ---------- 边沿类型 ----------
 * 命名沿用学长的枚举, 含义是"往哪个方向由暗变亮"
 */
#define V2_NONE     (0)
#define V2_LEFT     (1)     /* 水平: 暗 -> 亮 (白线左边沿) */
#define V2_RIGHT    (2)     /* 水平: 亮 -> 暗 (白线右边沿) */
#define V2_TOP      (3)     /* 垂直: 暗 -> 亮 (白线上边沿) */
#define V2_BOTTOM   (4)     /* 垂直: 亮 -> 暗 (白线下边沿) */
#define V2_BORDER   (5)     /* 越界 */
#define V2_SEARCHED (6)     /* 已判定为非边沿 */

/* ---------- 参数 ----------
 *
 * 归一化梯度阈值 (学长原值 150)
 *   gx = (右 - 左) * 1024 / (右 + 左)
 *   这是比值, 与曝光无关。调小更敏感, 调大只留强边沿。
 */
#define V2E_RATIO_THRES         (150)

/* 黑区判断: 左右像素灰度之和低于此值, 认为整片是黑的, 不算边沿。
 * 用来屏蔽鱼眼黑角。曝光 64 时蓝布灰度 20~40, 和 40~80,
 * 取 60 能屏蔽纯黑(0~10)同时保留暗蓝布的边沿。 */
#define V2E_BLACK_SUM           (60)

/* 一条白线的像素宽度范围 (学长原值 3 / 25)
 * 我们 20mm 胶带: 近处 10~16px, 远处 4~6px。 */
#define V2E_LANE_MIN_W          (3)
#define V2E_LANE_MAX_W          (30)

/* 横向扫描步长 (列) */
#define V2E_HORI_DX             (4)

/* 搜索半宽 (以预估位置为中心, 向两侧各搜这么多像素) */
#define V2E_HALF_SEARCH_W       (15)

/* 环岛: 每侧射线数 / 第一条离中心的距离 / 射线间距 */
#define V2E_CIRCLE_RAYS         (5)
#define V2E_CIRCLE_DX_START     (10)
#define V2E_CIRCLE_DX_STEP      (5)

/* 环岛: 相邻射线 y 值差的最小斜率 (太小不算趋势) */
#define V2E_CIRCLE_SLOPE_MIN    (2)

/* ---------- 接口 ---------- */
void v2e_detect_corner(void);       /* 直角弯: 写 g_corner_dir */
int  v2e_detect_island(void);       /* 环岛: -1 左 / +1 右 / 0 无 */

#endif
