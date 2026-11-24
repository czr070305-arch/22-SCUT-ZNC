#include "v2.h"

/* 运行时阈值偏移, 由 TRACK 页 UP/DOWN 调节 (定义在 v2_task.c) */
extern int s_thre_bias;


/*
 * v2_edge.c —— 归一化梯度边缘检测 (学长的核心思想)
 *
 * 为什么不用大津法:
 *
 *   大津法找一个全局灰度阈值, 然后"大于阈值的是白"。
 *   问题: 曝光变化、蓝布反光、鱼眼黑角都会让这个阈值飘,
 *        同一个场地调好的阈值, 换个位置就失效。
 *
 * 归一化梯度用的是"相邻像素的比值", 而不是绝对灰度:
 *
 *   gx = (right - left) * 1024 / (right + left)
 *
 *   举例: 蓝布 30, 白线 150
 *     跨过左边沿:  left=30,  right=150
 *                 gx = (150-30)*1024/(180) = +683   -> 强正(暗->亮)
 *     跨过右边沿:  left=150, right=30
 *                 gx = (30-150)*1024/(180) = -683   -> 强负(亮->暗)
 *     白线内部:    left=150, right=150
 *                 gx = 0
 *
 *   如果整体曝光减半: 蓝布 15, 白线 75
 *     左边沿: gx = (75-15)*1024/(90) = +683   <- 完全一样!
 *
 * 所以这个算法对光照不敏感, 换场地基本不用重新调阈值。
 *
 * 代价: 每一行/列都要现场算, 但只算需要的像素, 不建全图缓存。
 */

/* ==========================================================================
 * 边缘判定
 * ========================================================================== */

/*
 * 水平方向的边沿类型 (在一行里, 沿 x 方向判断)
 *
 * 返回:
 *   V2_LEFT   暗 -> 亮  (白线的左边沿)
 *   V2_RIGHT  亮 -> 暗  (白线的右边沿)
 *   V2_SEARCHED  既不是左也不是右(平地/噪声)
 *   V2_BORDER    越界
 */
uint8_t v2_edge_x(int x, int y)
{
    int left_px, right_px;
    int numerator, denom, gx;
    int thres;

    /* 越界: 留 2 像素余量, 因为要读 x±2 */
    if ((y < 2) || (y > V2_H - 4) || (x < 2) || (x > V2_W - 3)) {
        return (uint8_t)V2_BORDER;
    }

    left_px  = mt9v03x_image[y][x - 2];
    right_px = mt9v03x_image[y][x + 2];

    /* 两边都很黑: 鱼眼黑角或纯黑背景, 不算边沿。
     * 这一步替代了原 vision 里的 CFG_THRE_DARK_SKIP。 */
    if ((left_px + right_px) < V2_BLACK_SUM) {
        return (uint8_t)V2_SEARCHED;
    }

    /* 归一化梯度, 放大 1024 倍保留精度 */
    denom = left_px + right_px;
    numerator = (right_px - left_px) << 10;
    gx = numerator / denom;

    /* 运行时偏移: TRACK 页 UP/DOWN 调节, 正=更敏感(阈值更低) */
    thres = V2_RATIO_THRES - s_thre_bias;

    if (gx >  thres) { return (uint8_t)V2_LEFT;  }   /* 右边比左边亮 */
    if (gx < -thres) { return (uint8_t)V2_RIGHT; }   /* 左边比右边亮 */
    return (uint8_t)V2_SEARCHED;
}

/*
 * 垂直方向的边沿类型 (在一列里, 沿 y 方向判断)
 *
 * 返回:
 *   V2_TOP     暗 -> 亮 (从上往下, 刚开始变白 = 白线上边沿)
 *   V2_BOTTOM  亮 -> 暗 (白线下边沿)
 *
 * 注意 y 轴向下增大, 所以"上边沿"对应 image[y-2] 暗、image[y+2] 亮。
 */
uint8_t v2_edge_y(int x, int y)
{
    int up_px, down_px;
    int numerator, denom, gy;
    int thres;

    if ((y < 2) || (y > V2_H - 4) || (x < 2) || (x > V2_W - 3)) {
        return (uint8_t)V2_BORDER;
    }

    up_px   = mt9v03x_image[y - 2][x];
    down_px = mt9v03x_image[y + 2][x];

    if ((up_px + down_px) < V2_BLACK_SUM) {
        return (uint8_t)V2_SEARCHED;
    }

    denom = up_px + down_px;
    numerator = (down_px - up_px) << 10;     /* 下减上: 正值 = 往下变亮 */
    gy = numerator / denom;

    thres = V2_RATIO_THRES - s_thre_bias;

    if (gy >  thres) { return (uint8_t)V2_TOP;    }  /* 往下变亮 = 白线上边沿 */
    if (gy < -thres) { return (uint8_t)V2_BOTTOM; }  /* 往下变暗 = 白线下边沿 */
    return (uint8_t)V2_SEARCHED;
}
