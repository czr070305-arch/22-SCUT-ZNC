/* ==========================================================================
 *  vision.h —— 视觉模块共享类型、全局量与接口（MRS / CH32V307 移植版）
 *  --------------------------------------------------------------------------
 *  模块划分（和 VS Code 工程一一对应，方便你对照看）：
 *      vision_imgproc.c   取图 / 大津阈值 / 二值化 / 逐行白线段
 *      vision_track.c     爬线（逐行跟线）→ 中线 / 边线 / 前瞻误差
 *      vision_element.c   十字 / 岔路 / 环岛识别 + 两圈岔路记忆
 *      vision_display.c   屏幕显示（TFT180 或 OLED）
 *      vision_task.c      环境入口（init / process / task）+ 对外输出结构
 *      vision_control_link.c  视觉→控制 的联动层（差速/PID 示例）
 *
 *  ★ 坐标系：x 向右增大，y 向下增大（第 0 行在最上面）。
 *    "往上走一行" = y 减 1，所以前瞻是 y 变小。
 * ========================================================================== */
#ifndef VISION_H
#define VISION_H

#include "zf_common_headfile.h"
#include "vision_cfg.h"

#define V_W         (MT9V03X_W)         /* 图像宽 188 */
#define V_H         (MT9V03X_H)         /* 图像高 120 */
#define V_MAX_RUN   (8)                 /* 每行最多记录几段白 */

/* --------------------------------------------------------------------------
 * 【移植关键点 1】不再另外开辟 22.5KB 的 g_frame 副本
 * --------------------------------------------------------------------------
 *  PC 仿真里 img_capture() 会把环境给的图像拷进 g_frame，避免"处理到一半
 *  图像被下一帧覆盖"。但在 CH32V307 上：
 *      摄像头图像 mt9v03x_image 已经占 22.5KB，
 * 所以这里用宏把 g_frame 直接映射到摄像头缓冲区：
 *      代码里所有 g_frame[y][x] 都等价于 mt9v03x_image[y][x]，一行代码不用改。
 *  代价：DVP 是单缓冲 DMA，处理期间新帧可能已经开始写入，画面最下面
 *  几行偶尔有撕裂——对"爬线 + 前瞻"影响很小（前瞻用的是上半部分）。
 *
 *  【移植关键点 3】不保存整幅二值图
 *  二值化 / 去噪 / 提段在一趟行循环里完成，只留 4 行缓冲（752B），
 *  屏幕显示时把"原图 + g_thre"交给屏驱动自己判黑白，
 *  所以 RAM 里只有一份 188x120 的图像。
 * -------------------------------------------------------------------------- */
#define g_frame     mt9v03x_image

/* 一行里的一段白（两端都含）
 * 用 short 而不是 int：188 的坐标 short 足够，120*8*4B=3.8KB → 1.9KB */
typedef struct {
    short l;                            /* 左端列 */
    short r;                            /* 右端列 */
} v_run_t;

/* --------------------------------------------------------------------------
 * 图像缓冲（算法自己的，不碰环境输出图）
 * -------------------------------------------------------------------------- */
extern v_run_t       g_runs[V_H][V_MAX_RUN];        /* 每行的白线段 */
extern uint8         g_run_num[V_H];                /* 每行有效段数 */
extern int           g_thre;                        /* 本帧二值化阈值 */

/* --------------------------------------------------------------------------
 * 爬线结果（vision_track.c）
 * -------------------------------------------------------------------------- */
extern int   g_mid_line[V_H];                       /* 路径中线，-1=该行无效 */
extern int   g_edge_l[V_H];                         /* 该段白线左端 */
extern int   g_edge_r[V_H];                         /* 该段白线右端 */
extern int   g_path_top;                            /* 路径最高有效行 */
extern int   g_path_top_x;                          /* 最高有效行的 x */
extern float g_path_slope;                          /* 顶部斜率（x 每上一行变化量） */
extern int   g_path_rows;                           /* 路径有效行数 */
extern int   g_bottom_x;                            /* 底部跟踪点 x */
extern int   g_corner_dir;                          /* 折角方向：-1 左 / +1 右 / 0 无 */
extern int   g_corner_far_x;                        /* 折角远端 x（前瞻就指它） */
extern int   g_look_x;                              /* 本帧前瞻点 */
extern int   g_look_y;
extern float g_steer_err;                           /* 转向误差（像素当量） */
extern int   g_lost;                                /* 丢线标志 */
extern int   g_lost_cnt;                            /* 连续丢线帧数 */

/* --------------------------------------------------------------------------
 * 元素识别（vision_element.c）
 * -------------------------------------------------------------------------- */
extern int g_elem;              /* 0 无 / 1 十字 / 3 环岛 */
extern int g_elem_side;         /* 分支在哪侧：-1 左 / +1 右 / 0 无 */
extern int g_prefer_side;       /* 本帧爬线的偏好侧（0=中立） */
extern int g_island_on;         /* 是否处于环岛过程中 */
extern int g_island_side;       /* 环岛走的是哪一侧 */
extern int g_fork_rows;         /* 本帧"主线继续+分支"的行数（调试用） */
extern int g_isle_rows;         /* 本帧"两侧夹击"的行数（调试用） */
extern int g_cross_rows;        /* 本帧横向长段行数（调试用） */

/* --------------------------------------------------------------------------
 * ★ 2026-11-21：元素状态机对外量（控制层可直接用，本工程当前只接 speed_pct）
 * --------------------------------------------------------------------------
 *  g_state        0 普通 / 1 直角 / 2 十字 / 3 岔路 / 4 环岛
 *  g_state_frames 进入本状态后的帧数（超时由 CFG_STATE_MAX_* 强制回 0）
 *  g_speed_pct    建议速度百分比（100 = 全速）
 * -------------------------------------------------------------------------- */
extern int   g_state;
extern int   g_state_frames;
extern int   g_speed_pct;

/* --------------------------------------------------------------------------
 * 【移植关键点 2】对外的"一帧结果"打包成结构体
 * --------------------------------------------------------------------------
 *  控制层只读这个结构体，不去碰任何图像/全局量。这样将来换算法，
 *  控制代码完全不用改。字段含义：
 *      err        转向误差（像素当量，右为正）。控制层乘 K 就是差速/舵机量
 *      err_valid  1=本帧误差可信；0=丢线/太短，控制层应沿用上一帧或丢线策略
 *      elem       本帧元素类型 0/1/2/3（十字/岔路/环岛）
 *      elem_side  分支侧 -1 左 / +1 右
 *      island_on  是否正在环岛（控制层可据此降速）
 *      lap        当前圈数 1/2
 *      fork_pass  已通过岔路次数（两圈策略用）
 *      look_x/y   本帧前瞻点（屏幕上那个绿十字）
 *      path_rows  本帧路径有效行数（越大说明看得越远）
 *      seq        帧序号，控制层可用它判断视觉是否还在跑（溢出回绕）
 * -------------------------------------------------------------------------- */
typedef struct {
    float err;
    int   err_valid;
    int   elem;
    int   elem_side;
    int   island_on;
    int   lap;
    int   fork_pass;
    int   look_x;
    int   look_y;
    int   path_rows;
    int   state;        /* ★ 0 普通 / 1 直角 / 2 十字 / 3 岔路 / 4 环岛 */
    int   speed_pct;    /* ★ 建议速度 0~100（100 = 全速） */
    unsigned int seq;
    unsigned int lost_cnt;
} vision_out_t;

/* --------------------------------------------------------------------------
 * 畸变模型（vision_task.c 里实现，参数来自你给的标定结果）
 *  理想像素 (u,v) --正投影--> 畸变像素 (xd,yd)      vision_distort_pixel()
 *  畸变像素 (xd,yd) --迭代反解--> 理想像素 (u,v)    vision_undistort_pixel()
 * -------------------------------------------------------------------------- */
extern const float VISION_CAM_FX;       /* 148.412 */
extern const float VISION_CAM_FY;       /* 151.871 */
extern const float VISION_CAM_CX;       /* 96.731  */
extern const float VISION_CAM_CY;       /* 50.592  */
extern const float VISION_CAM_MOVE_X;   /* 0       */
extern const float VISION_CAM_MOVE_Y;   /* -9      */

void  vision_distort_pixel   (float u, float v, float *xd, float *yd);
int   vision_undistort_pixel (float xd, float yd, float *u, float *v);
float vision_undistort_x     (float xd, float yd);   /* 只反解 x（转向补偿用） */

/* --------------------------------------------------------------------------
 * 模块接口
 * -------------------------------------------------------------------------- */
/* vision_imgproc.c */
void img_capture(void);                         /* 等待新帧（不再拷贝，见 vision.h 顶部说明） */
int  img_threshold(void);                       /* 大津法（含偏移与限幅） */

/* 【移植改动】运行期阈值偏移
 *
 * 为什么需要:
 *   大津法按全图直方图算阈值. 白线只占画面 1~2%, 蓝底占 98%,
 *   光照不均时(蓝布上出现亮斑)大津算出的阈值会落在蓝底灰度区间里,
 *   于是整片蓝底被判成白 —— 屏幕上就是一个大圆斑.
 *
 *   CFG_THRE_MARGIN 是编译期常量, 每次调都要重烧录, 效率太低.
 *   这里给一个运行期偏移, TRACK 页用按键实时加减, 看着二值图调.
 *
 *   实际阈值 = clamp(大津值 + CFG_THRE_MARGIN + 运行期偏移, MIN, MAX)
 *   偏移范围 ±64, 0 = 只用编译期常量.
 */
void vision_set_thre_bias(int bias);   /* -64 ~ +64, 正数=判白更严格 */
int  vision_get_thre_bias(void);
void img_binarize_runs(int thre);               /* 二值化 + 3x3 去噪 + 逐行提段（一趟做完） */

/* vision_track.c */
int  track_build(int prefer_side);              /* 爬线，返回是否跟到线 */
void track_steer(void);                         /* 算前瞻点与转向误差 */
int  track_ref_x(int y);                        /* 某行的"参考 x"（无效行用外推） */

/* vision_element.c */
void elem_reset(void);                          /* 状态清零（初始化 / 切图时用） */
void elem_update(void);                         /* 每帧元素判定 + 状态机 */

/* vision_display.c */
void disp_init(void);                           /* 屏幕初始化（TFT180 或 OLED） */
void disp_frame(void);                          /* 刷一帧：二值图 + 中线 + 前瞻点 + 状态 */
void disp_status(const char *s1, const char *s2);/* 显示两行文字（无图时用） */
int  disp_key_pressed(void);                    /* 控制层有没有在按键（决定暂停刷图） */

/* vision_task.c */
void vision_init(void);                         /* 摄像头 + 屏幕 + 算法状态初始化 */
int  vision_process(void);                      /* 处理当前帧一次，返回 1=有新结果 */
void vision_task(void);                         /* 主循环里调用：取帧→处理→显示→输出 */
const vision_out_t *vision_get_output(void);    /* 取最新结果（只读） */

/* --------------------------------------------------------------------------
 * 控制层扩展接口
 * --------------------------------------------------------------------------
 *  vision_cam_ok()   摄像头是否初始化成功(1=成功). 控制层用它决定显示什么.
 *  vision_set_disp() 开关屏幕刷图. 默认关:
 *                    只有进 TRACK 页时才打开, 否则视觉会在菜单页、
 *                    参数页上乱画, 把控制层的 UI 覆盖掉.
 *                    关的时候 vision_task() 照常处理图像(控制不受影响),
 *                    只是不往屏幕写.
 * -------------------------------------------------------------------------- */
int  vision_cam_ok(void);

void vision_set_disp(int on);

/* --------------------------------------------------------------------------
 * 【移植改动】取帧权让出
 * --------------------------------------------------------------------------
 *  RAW 原画面预览页也要读 mt9v03x_finish_flag / mt9v03x_image。
 *  谁先清标志谁拿到这一帧, 所以必须显式让出, 否则 RAW 页画面不刷新
 *  (或者在两处交替刷新, 看起来像卡顿)。
 *
 *  vision_set_hold(1): vision_task() 立刻返回, 不取帧、不处理。
 *                      摄像头和 DMA 照常工作, 标志位留给别处消费。
 *  vision_set_hold(0): 恢复正常处理。
 *
 *  task_raw_show() 打开, task_raw_leave() 关闭。
 * -------------------------------------------------------------------------- */
void vision_set_hold(int on);


#endif /* VISION_H */
