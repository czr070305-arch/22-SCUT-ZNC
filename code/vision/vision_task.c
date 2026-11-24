/* ==========================================================================
 *  vision_task.c —— 视觉模块的"环境入口"：摄像头、取帧节拍、结果打包
 *  --------------------------------------------------------------------------
 *  对应 VS Code 工程里的 code/camera.c（init / process / draw / report），
 *  另外把"畸变模型"也放在这里，因为它只服务视觉输出。
 *
 *  【一帧的完整流水线】
 *
 *      mt9v03x_finish_flag == 1        （DVP 硬件采集完一帧，中断里置位）
 *              │
 *              ▼
 *      img_threshold()   大津法求阈值（含偏移 + 限幅）
 *              │
 *              ▼
 *      img_binarize_runs()  按阈值二值化 + 3x3 去噪 + 逐行切成"白线段"
 *              │              → g_runs / g_run_num（不存整幅二值图，省 22.5KB）
 *              ▼
 *      track_build()     从底部往上爬线            → g_mid_line / g_path_*
 *              │
 *              ▼
 *      track_steer()     前瞻点 + 转向误差         → g_look_x/y / g_steer_err
 *              │
 *              ▼
 *      elem_update()     十字 / 环岛             → g_elem / g_prefer_side
 *              │
 *              ▼
 *      vision_out_t      打包给控制层（vision_get_output()）
 *
 *  【时间预算】120MHz、188x120、不开优化也会在 6~9ms 内算完，
 *  50 帧 = 20ms 一帧，所以视觉约占 1/3 CPU，剩下留给控制环。
 * ========================================================================== */
#include "vision.h"

/* 你给的标定结果（188x120 图像，单位像素）
 *   fx/fy : 焦距（像素单位）
 *   cx/cy : 主点。注意 cx=96.7 而图像中心是 94，说明光轴比画面中心偏右 2.7px，
 *           这是标定出来的真实结果，别改成 94。
 *   k1/k2/k3 : 径向畸变系数（k1 为负 = 桶形畸变）
 *   p1/p2    : 切向畸变系数
 *   move_xy  : 理想图像相对畸变图像的平移，y 方向 -9 表示实际画面比理想网格
 *              低 9 行（镜头/安装导致）
 */
const float VISION_CAM_FX     = 148.404692f;
const float VISION_CAM_FY     = 151.874202f;
const float VISION_CAM_CX     = 96.738556f;
const float VISION_CAM_CY     = 50.592531f;
const float VISION_CAM_MOVE_X = 0.0f;
const float VISION_CAM_MOVE_Y = -9.0f;

static const float VISION_K1 = -1.101773f;
static const float VISION_K2 =  1.298787f;
static const float VISION_K3 = -0.684578f;
static const float VISION_P1 = -0.006304f;
static const float VISION_P2 =  0.004262f;

static vision_out_t s_out;
static uint8  s_cam_ok = 0;
static uint32 s_frame_cnt = 0;
static uint8  s_inited = 0;
static uint8  s_disp_on = 0;
static uint8  s_hold    = 0;      /* 1=让出取帧权(RAW 页在用摄像头) */

/* --------------------------------------------------------------------------
 * 畸变模型（正投影）：理想像素 (u,v) → 畸变像素 (xd,yd)
 *
 *  和 PC 工程里的 find_xy 完全同一套公式：
 *      x  = (u - move_x - cx) / fx
 *      y  = (v - move_y - cy) / fy
 *      r2 = x^2 + y^2
 *      k  = 1 + k1*r2 + k2*r2^2 + k3*r2^3
 *      xd = cx + fx * (x*k + 2*p1*x*y + p2*(r2 + 2*x*x))
 *      yd = cy + fy * (y*k + 2*p2*x*y + p1*(r2 + 2*y*y))
 *
 *  ★ 这个方向（理想→畸变）是标定时定的方向，反解要迭代，见下一个函数。
 * -------------------------------------------------------------------------- */
void vision_distort_pixel(float u, float v, float *xd, float *yd)
{
    float x = (u - VISION_CAM_MOVE_X - VISION_CAM_CX) / VISION_CAM_FX;
    float y = (v - VISION_CAM_MOVE_Y - VISION_CAM_CY) / VISION_CAM_FY;
    float r2 = x * x + y * y;
    float k  = 1.0f + VISION_K1 * r2 + VISION_K2 * r2 * r2 + VISION_K3 * r2 * r2 * r2;
    float dx = x * k + 2.0f * VISION_P1 * x * y + VISION_P2 * (r2 + 2.0f * x * x);
    float dy = y * k + 2.0f * VISION_P2 * x * y + VISION_P1 * (r2 + 2.0f * y * y);

    *xd = VISION_CAM_CX + VISION_CAM_FX * dx;
    *yd = VISION_CAM_CY + VISION_CAM_FY * dy;
}

/* --------------------------------------------------------------------------
 * 反解：畸变像素 (xd,yd) → 理想像素 (u,v)
 *
 *  用不动点迭代：把当前估计正投影出去，看差多少像素，直接补回去。
 *  桶形畸变下 6~8 次就收敛到 0.05px 以内；每帧只调 2 次，开销可忽略。
 *  返回 1=收敛，0=达到迭代上限（结果仍可用）。
 * -------------------------------------------------------------------------- */
int vision_undistort_pixel(float xd, float yd, float *u, float *v)
{
    float uu = xd;
    float vv = yd;
    int i;

    /* 迭代次数上限 16。
     *
     * 实测（全图逐像素 vs 200 次迭代真解）:
     *    8 次  -> 58 个点结果偏差 >3px（0.1%）
     *   16 次  -> 1 个点偏差 >3px
     *   20 次  -> 0 个点
     * 这些坏点集中在画面四角（超出赛道区域），前瞻点很少落到那里；
     * 但路径极短时 look_y 会退化成 y0（画面底部），所以留 16 次。
     * 每帧只调 2 次，多 8 次迭代的开销可忽略。 */
    for (i = 0; i < 16; i++) {
        float px, py;
        float du, dv;

        vision_distort_pixel(uu, vv, &px, &py);
        du = xd - px;
        dv = yd - py;
        uu += du;
        vv += dv;

        /* 发散保护: 鱼眼边缘取值会让迭代跑飞, 产生 nan/inf.
         * 一旦流进 g_steer_err 就会把 PID 打爆.
         * 超出范围直接放弃, 返回原值. */
        if ((uu > 1.0e4f) || (uu < -1.0e4f) ||
            (vv > 1.0e4f) || (vv < -1.0e4f) ||
            (!(uu == uu)) || (!(vv == vv))) {
            *u = xd;
            *v = yd;
            return 0;
        }

        if ((du < 0.02f) && (du > -0.02f) && (dv < 0.02f) && (dv > -0.02f)) {
            *u = uu;
            *v = vv;
            return 1;
        }
    }
    if ((uu > 1.0e4f) || (uu < -1.0e4f) || (!(uu == uu))) {
        uu = xd;
    }
    if ((vv > 1.0e4f) || (vv < -1.0e4f) || (!(vv == vv))) {
        vv = yd;
    }
    *u = uu;
    *v = vv;
    return 0;
}

/* 只反解 x，行坐标固定：转向误差补偿专用（见 vision_track.c 的说明） */
float vision_undistort_x(float xd, float yd)
{
    float u, v;

    vision_undistort_pixel(xd, yd, &u, &v);
    return u;
}

/* --------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------- */
void vision_init(void)
{
    int retry;

    /* ---- 1) 摄像头：逐飞 MT9V03X 模块（MT9V034 + 片内固件） ---- */
    for (retry = 0; retry < 5; retry++) {
        if (mt9v03x_init() == 0) {          /* 逐飞约定：0 = 成功 */
            s_cam_ok = 1;
            break;
        }
        system_delay_ms(200);
    }

    /* ---- 2) 屏幕 ---- */
    /* TFT 后端(VISION_DISP_SEL=0)这里什么也不做: 竖屏由 app_ui.c 的
     * ui_hardware_init() 初始化, 视觉只负责往上画。
     * OLED 后端才会在这里真正 init。 */
    disp_init();

    if (!s_cam_ok) {
#if (CFG_INIT_SHOW_STATUS)
        disp_status("CAMERA FAIL", "check FPC/3V3");
#endif
        /* 不 return：屏幕能亮就先把错误显示出来，方便排查接线 */
    } else {
        /* ---- 3) 曝光：把参数表里的手动曝光值下发给摄像头 ---- */
        /* 说明：分辨率/帧率/增益在 zf_device_mt9v03x_dvp.h 的宏里，
         *      这里只调最常动的曝光；曝光越大画面越亮。 */
        mt9v03x_set_exposure_time(VISION_CAM_EXP_TIME);
#if (CFG_INIT_SHOW_STATUS)
        disp_status("VISION READY", "MT9V03X 188x120");
        system_delay_ms(300);
#endif
    }

    /* ---- 4) 算法状态清零 ---- */
    elem_reset();
    g_thre = 0;
    g_steer_err = 0.0f;
    g_lost = 0;
    g_lost_cnt = 0;
    g_look_x = -1;
    g_look_y = -1;

    s_out.err = 0.0f;
    s_out.err_valid = 0;
    s_out.elem = 0;
    s_out.elem_side = 0;
    s_out.island_on = 0;
    s_out.lap = 1;
    s_out.fork_pass = 0;
    s_out.look_x = -1;
    s_out.look_y = -1;
    s_out.path_rows = 0;
    s_out.seq = 0;
    s_out.lost_cnt = 0;
    s_out.state = 0;
    s_out.speed_pct = 100;

    s_frame_cnt = 0;
    s_inited = 1;
}

/* --------------------------------------------------------------------------
 * 处理一帧（不含取帧等待，调用前应保证 finish_flag 已被清掉）
 * -------------------------------------------------------------------------- */
int vision_process(void)
{
    int thre;
    int tracked;

    if (!s_cam_ok) {
        return 0;
    }

    /* 1) 阈值 → 2) 二值化+去噪+逐行提段（一趟完成，不存整幅二值图） */
    thre = img_threshold();
    img_binarize_runs(thre);

    /* 4) 爬线（用上一帧元素判定给出的偏好侧） → 5) 前瞻误差 */
    tracked = track_build(g_prefer_side);
    if (tracked) {
        track_steer();
        g_lost = 0;
        if (g_lost_cnt > 0) {
            g_lost_cnt = 0;
        }
    } else {
        g_lost = 1;
        if (g_lost_cnt < 100000) {
            g_lost_cnt++;
        }
        /* 丢线时不覆盖 g_steer_err：让控制层按"沿用上一帧"处理 */
    }

    /* 6) 元素判定（会更新 g_prefer_side，供下一帧爬线使用） */
    elem_update();

    /* 7) 打包输出 */
    s_frame_cnt++;
    s_out.err        = g_steer_err;
    /* ★ 2026-11-21：判出折角时，就算路径只剩几行，误差也是有效的
     *   （折角本身就是"往哪边走"的结论）。否则控制层会把直角弯当成丢线，
     *   进入 HOLD 阶段，车直冲出去。 */
    s_out.err_valid  = ((tracked && (g_path_rows >= CFG_MIN_ROWS_STEER))
                        || (g_corner_dir != 0)) ? 1 : 0;
    s_out.elem       = g_elem;
    s_out.elem_side  = g_elem_side;
    s_out.island_on  = g_island_on;
    /* 岔路已删除（见 vision_cfg.h 关于 CFG_NO_FORK 的说明），
     * 这两个字段保留在结构体里只为兼容旧调用方，恒为初值。 */
    s_out.lap        = 1;
    s_out.fork_pass  = 0;
    s_out.look_x     = g_look_x;
    s_out.look_y     = g_look_y;
    s_out.path_rows  = g_path_rows;
    s_out.seq        = s_frame_cnt;
    s_out.lost_cnt   = (unsigned int)g_lost_cnt;
    s_out.state      = g_state;
    s_out.speed_pct  = g_speed_pct;

    return 1;
}

/* --------------------------------------------------------------------------
 * 主循环任务：取帧 → 处理 → 刷屏 → 联动
 *
 *  调用位置：main.c 的 while(1) 里，和 assessment_app_loop() 并排。
 *  它自己判断有没有新帧，没帧时立刻返回，不会阻塞控制代码。
 * -------------------------------------------------------------------------- */
void vision_task(void)
{
    if (!s_inited) {
        return;
    }

    /* 别处在用摄像头(RAW 原画面预览页): 让出取帧权。
     * 不处理图像, 也就不会清 mt9v03x_finish_flag, 那一位留给调用方。 */
    if (s_hold) {
        return;
    }

    /* ---- 摄像头没起来：每约 0.5s 重试一次 ---- */
    if (!s_cam_ok) {
        static uint32 s_retry_tick = 0;

        if (++s_retry_tick > 25) {
            s_retry_tick = 0;
            if (mt9v03x_init() == 0) {
                s_cam_ok = 1;
                mt9v03x_set_exposure_time(VISION_CAM_EXP_TIME);
            }
        }
        return;
    }

    /* ---- 有新帧才处理；CFG_PROC_EVERY_N 控制"隔几帧算一次" ---- */
    if (mt9v03x_finish_flag) {
        mt9v03x_finish_flag = 0;

        if ((s_frame_cnt % CFG_PROC_EVERY_N) == 0) {
            vision_process();

            /* ---- 屏幕：只有控制层明确打开时才刷 ----
             * 菜单页/参数页也在用这块屏, 视觉无条件刷图会把它们盖掉。
             * task_track_show() 打开, task_track_leave() 关闭。
             * 关的时候图像照常处理, 控制逻辑完全不受影响。 */
            if (s_disp_on && ((s_frame_cnt % CFG_DISP_EVERY_N) == 0)) {
                disp_frame();
            }

            /* ---- 视觉→控制 联动（差速/舵机目标在这里算出来） ---- */
        }
    }
}

const vision_out_t *vision_get_output(void)
{
    return &s_out;
}

/* 摄像头是否初始化成功 (1=成功) */
int vision_cam_ok(void)
{
    return (int)s_cam_ok;
}

/* 开关屏幕刷图: 0=不刷(默认), 1=刷。算法不受影响 */
void vision_set_disp(int on)
{
    s_disp_on = (on) ? 1 : 0;
}

void vision_set_hold(int on)
{
    s_hold = (on) ? 1 : 0;
}
