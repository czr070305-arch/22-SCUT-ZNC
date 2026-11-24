#ifndef _V2_H_
#define _V2_H_

#include "zf_common_headfile.h"
#include "v2_cfg.h"

/*
 * vision2 —— 基于学长(20届)算法的视觉模块
 *
 * 与 vision/ 并行存在, 用 task_track.c 里的编译开关切换。
 * 两者接口一致(vision_out_t / vision_get_output 等), 上层不用改。
 *
 * 数据流:
 *
 *   mt9v03x_image
 *        |
 *        v  ratio_x_judge / ratio_y_judge   (归一化梯度, 现算不缓存)
 *   边沿类型: 暗->亮(LEFT/TOP) / 亮->暗(RIGHT/BOTTOM)
 *        |
 *        v  get_start_point   (底部找白线的左右边沿)
 *        |  get_center_line   (纵向逐行爬, 每行找左右边沿配对)
 *        v
 *   l_edge_x[] / r_edge_x[] / center_x[]
 *        |
 *        +--> get_left/right_turn_line  (横向逐列扫, 直角弯)
 *        |
 *        +--> circle_process            (5 条射线投票, 环岛)
 *        |
 *        v  get_full_line     (取两个前瞻点)
 *        |
 *        v  err = 加权横偏
 */

/* ---------- 边沿类型 ----------
 * 命名沿用学长的枚举, 含义是"往哪个方向由暗变亮"
 */
typedef enum {
    V2_NONE   = 0,      /* 无 */
    V2_LEFT   = 1,      /* 水平方向: 暗 -> 亮 (白线的左边沿) */
    V2_RIGHT  = 2,      /* 水平方向: 亮 -> 暗 (白线的右边沿) */
    V2_TOP    = 3,      /* 垂直方向: 暗 -> 亮 (白线的上边沿) */
    V2_BOTTOM = 4,      /* 垂直方向: 亮 -> 暗 (白线的下边沿) */
    V2_BORDER = 5,      /* 越界 */
    V2_SEARCHED = 6     /* 已判定为非边沿 */
} v2_edge_enum;

/* ---------- 输出结构 (与原 vision_out_t 对齐) ---------- */
typedef struct {
    float err;              /* 转向误差, 右为正, 单位像素 */
    int   err_valid;        /* 1=本帧可信 */
    int   elem;             /* 0 无 / 1 十字 / 2 岔路 / 3 环岛 */
    int   elem_side;        /* -1 左 / +1 右 */
    int   island_on;        /* 是否在环岛中 */
    int   lap;              /* 圈数 1/2 */
    int   fork_pass;
    int   look_x;           /* 前瞻点在图像里的 x (调试用) */
    int   look_y;
    int   path_rows;        /* 本帧爬了多少行 */
    unsigned int seq;
    unsigned int lost_cnt;
} v2_out_t;

/* ---------- 中间结果 (调试用, 可视化时读) ---------- */
extern int v2_l_edge_x[V2_H];       /* 每行白线左边沿, -1=无效 */
extern int v2_r_edge_x[V2_H];       /* 每行白线右边沿 */
extern int v2_center_x[V2_H];       /* 每行中线 */
extern int v2_center_end_row;       /* 爬到的最顶行 */
extern int v2_center_cnt;           /* 有效行数 */
extern int v2_corner_dir;           /* 折角: -1 左 / +1 右 / 0 无 */
extern int v2_island_side;          /* 环岛: -1 左 / +1 右 / 0 无 */

/* ---------- 生命周期 ---------- */
void v2_init(void);                 /* 摄像头 + 算法状态初始化 */
void v2_task(void);                 /* 主循环调用: 取帧 -> 处理 -> 显示 */
int  v2_process(void);              /* 处理当前帧, 返回 1=有新结果 */
const v2_out_t *v2_get_output(void);

/* ---------- 控制层接口 (与原 vision 一致) ---------- */
int  v2_cam_ok(void);
void v2_set_disp(int on);
void v2_set_hold(int on);

/* ---------- 运行时阈值调节 (TRACK 页 UP/DOWN 用) ---------- */
void v2_set_thre_bias(int bias);    /* -64 ~ +64, 正=更敏感 */
int  v2_get_thre_bias(void);

/* ---------- 边缘检测 (v2_edge.c) ---------- */
uint8_t v2_edge_x(int x, int y);   /* 水平方向边沿类型 */
uint8_t v2_edge_y(int x, int y);   /* 垂直方向边沿类型 */

/* ---------- 爬线 (v2_track.c) ---------- */
int  v2_find_start(int *out_y);                  /* 底部找起点, 返回 1=成功 */
int  v2_climb(int start_y);                      /* 纵向逐行爬, 返回最顶行 */
void v2_scan_row(int y, int *lx, int *rx,
                 int prev_l, int prev_r,
                 int last_center, int half_w);   /* 单行找左右边沿 */

/* ---------- 工具 ---------- */
int v2_iabs(int v);
int v2_imax(int a, int b);
int v2_imin(int a, int b);

/* ---------- 元素识别 (v2_elem.c) ---------- */
void v2_detect_corner(void);        /* 直角弯: 横向扫描 */
int  v2_detect_island(void);        /* 环岛: 射线投票 */

/* ---------- 显示 ---------- */
void v2_disp_frame(void);           /* 画二值图 + 边沿点 + 中线 */

/* ---------- 与 vision/ 同名别名 ----------
 *
 * 让 app_menu.c / main.c 不用改成套 #if。
 * 它们只要 include 本文件, 调 vision_init()/vision_task() 就自动落到 v2。
 *
 * 注意: 只有确定要切到 v2 时才 include 本文件, 否则会和 vision/ 冲突。
 *       切换点统一在 task_track.c 的 TRACK_USE_V2。
 */
#define vision_init         v2_init
#define vision_task         v2_task

#endif
