/* ==========================================================================
 *  vision_element.h --- 元素识别模块接口
 *
 *  文件分成两块（2026-11-25 重构）：
 *
 *    vision_feature.c   逐行特征提取
 *                       probe_rows()  -> s_sep[] / s_br[] 供上层的元素判据读取
 *                       path_x_at()   -> 某个 y 的参考 x
 *
 *    vision_element.c   元素状态机
 *                       find_fork / find_island / detect_cross / find_exit_x
 *                       elem_update_core  入岛 / 出岛 / 十字 / 环岛接管
 *
 *  对外只暴露 elem_reset() 与 elem_update()（声明在 vision.h）。
 *  本文件存在的意义是把两个编译单元之间的内部接口集中到一处，
 *  免得跨文件的隐式声明各改各的。
 * ========================================================================== */
#ifndef _VISION_ELEMENT_H_
#define _VISION_ELEMENT_H_

#include "vision_cfg.h"

/* --------------------------------------------------------------------------
 * 逐行特征（vision_feature.c 产出，vision_element.c 消费）
 *
 *   s_sep[y] = 同一条路上两段白之间夹着的黑缝宽度（只有一段白时为 0）
 *   s_br[y]  = 支线相对参考线的偏移（无支线时为 0）
 *
 *  两个数组都按行索引、每帧整帧重建。
 * -------------------------------------------------------------------------- */
extern int s_sep[V_H];
extern int s_br[V_H];

/* 逐行扫描：重建 s_sep[] / s_br[]。依赖 g_path_top / g_runs / g_run_num。 */
void feat_probe_rows(void);

/* 某个 y 的参考 x：有跟踪点就用跟踪点，没有就用下方最近的有效点线性外推。
 * 元素判据里所有"这段白离路中心有多远"都靠它当基准。 */
int feat_path_x_at(int y);

/* 整数小工具（vision_feature.c 提供，vision_element.c 也用） */
int v_abs(int v);
int v_sign(int v);

#endif /* _VISION_ELEMENT_H_ */
