#include "v2.h"

/*
 * v2_task.c —— 主流程、摄像头、显示
 *
 * 一帧的处理顺序:
 *
 *   1. 找起点      v2_find_start()    底部找一条完整白线的左右边沿
 *   2. 纵向爬线    v2_climb()         逐行往上, 每行边沿配对
 *   3. 横向扫描    v2_detect_corner() 判断直角弯方向
 *   4. 环岛投票    v2_detect_island() 左右射线判断环岛
 *   5. 前瞻取点    取两个远度的点算 err
 *   6. 打包输出
 */

/* ---------- 内部状态 ---------- */
static v2_out_t s_out;
static uint8  s_cam_ok   = 0;
static uint8  s_inited   = 0;
static uint8  s_disp_on  = 0;
static uint8  s_hold     = 0;
static uint32 s_frame_cnt = 0;

/* 阈值偏移, v2_edge.c 里 extern 使用 */
int s_thre_bias = 0;

/* 环岛状态机 */
static int s_island_frames = 0;      /* 已连续多少帧检测到环岛特征 */
static int s_island_side   = 0;      /* 已确认的方向 */
static int s_island_on     = 0;      /* 是否在环岛中 */
static int s_island_cooldown = 0;    /* 冷却帧数 */
static int s_lap = 1;
static int s_fork_pass = 0;

/* 丢线计数 */
static unsigned int s_lost_cnt = 0;

/* ==========================================================================
 * 前瞻取点与误差计算
 *
 * 学长方案: 取两个远度的点做加权
 *   FAR_1 = 55 行 (近)
 *   FAR_2 = 70 行 (远)
 *   err = err1 * (1-ratio) + err2 * ratio
 *
 * 为什么用两个点:
 *   只看近点 -> 弯道反应慢, 等看到弯就晚了
 *   只看远点 -> 直道抖动大, 远处的像素误差被放大
 *   两点加权兼顾
 * ========================================================================== */
static void v2_calc_err(int start_y)
{
    int y1 = start_y - V2_TRACK_FAR_1;
    int y2 = start_y - V2_TRACK_FAR_2;
    int x1 = -1, x2 = -1;
    int y;
    float err1 = 0.0f, err2 = 0.0f;
    float err;
    int half = V2_W / 2;

    /* y 必须落在有效范围内, 并在 center_x 里找到最近的有效行 */
    if (y1 < 2) { y1 = 2; }
    if (y2 < 2) { y2 = 2; }

    for (y = y1; y <= start_y; y++) {
        if (v2_center_x[y] >= 0) { x1 = v2_center_x[y]; y1 = y; break; }
    }
    for (y = y2; y <= start_y; y++) {
        if (v2_center_x[y] >= 0) { x2 = v2_center_x[y]; y2 = y; break; }
    }

    /* 一个都没有: 丢线 */
    if ((x1 < 0) && (x2 < 0)) {
        s_out.err_valid = 0;
        s_out.look_x = -1;
        s_out.look_y = -1;
        if (s_lost_cnt < 100000) { s_lost_cnt++; }
        return;
    }

    if (x1 >= 0) { err1 = (float)(x1 - half); }
    if (x2 >= 0) { err2 = (float)(x2 - half); }

    if ((x1 >= 0) && (x2 >= 0)) {
        err = err1 * (1.0f - V2_TRACK_RATIO) + err2 * V2_TRACK_RATIO;
        s_out.look_x = x2;
        s_out.look_y = y2;
    } else if (x1 >= 0) {
        err = err1;
        s_out.look_x = x1;
        s_out.look_y = y1;
    } else {
        err = err2;
        s_out.look_x = x2;
        s_out.look_y = y2;
    }

    /* 折角时额外往弯里推 */
    if (v2_corner_dir != 0) {
        err += (float)v2_corner_dir * V2_CORNER_BIAS;
    }

    /* NaN 兜底: NaN 与任何数比较都是 false, 会绕过下面的限幅 */
    if (!(err == err)) {
        s_out.err_valid = 0;
        if (s_lost_cnt < 100000) { s_lost_cnt++; }
        return;
    }

    if (err >  110.0f) { err =  110.0f; }
    if (err < -110.0f) { err = -110.0f; }

    s_out.err = err;
    s_out.err_valid = 1;
    s_lost_cnt = 0;
}

/* ==========================================================================
 * 元素状态机
 * ========================================================================== */
static void v2_update_element(void)
{
    int isle;

    s_out.elem = 0;
    s_out.elem_side = 0;
    s_out.island_on = 0;

    if (s_island_cooldown > 0) { s_island_cooldown--; }

    /* ---- 已在环岛中 ---- */
    if (s_island_on) {
        s_island_frames++;
        s_out.island_on = 1;
        s_out.elem = 3;
        s_out.elem_side = s_island_side;
        v2_island_side = s_island_side;

        /* 退出: 连续几十帧都没再看到环岛特征 */
        if (s_island_frames > V2_CIRCLE_COOLDOWN * 3) {
            s_island_on = 0;
            s_island_frames = 0;
            s_island_side = 0;
            v2_island_side = 0;
            s_island_cooldown = V2_CIRCLE_COOLDOWN;
            s_fork_pass++;
            s_lap = (s_lap == 1) ? 2 : 1;
        }
        return;
    }

    /* ---- 环岛入口判定 ---- */
    if (s_island_cooldown > 0) { return; }

    isle = v2_detect_island();

    if (isle != 0) {
        /* 方向连续一致才确认, 防止单帧误判 */
        if (s_island_side == isle) {
            s_island_frames++;
        } else {
            s_island_side = isle;
            s_island_frames = 1;
        }

        if (s_island_frames >= V2_CIRCLE_CONFIRM_CNT) {
            s_island_on = 1;
            s_island_frames = 0;
            v2_island_side = s_island_side;
            s_out.island_on = 1;
            s_out.elem = 3;
            s_out.elem_side = s_island_side;
        }
    } else {
        /* 特征消失, 计数衰减而不是直接清零,
         * 避免环岛入口的弧线被车身的遮挡打断时前功尽弃 */
        if (s_island_frames > 0) { s_island_frames--; }
    }
}

/* ==========================================================================
 * 处理一帧
 * ========================================================================== */
int v2_process(void)
{
    int start_y = -1;

    if (!s_cam_ok) { return 0; }

    /* 清空中间结果 */
    {
        int i;
        for (i = 0; i < V2_H; i++) {
            v2_l_edge_x[i] = -1;
            v2_r_edge_x[i] = -1;
            v2_center_x[i] = -1;
        }
        v2_center_end_row = V2_H - 1;
        v2_center_cnt = 0;
        v2_corner_dir = 0;
    }

    /* 1. 找起点 */
    if (!v2_find_start(&start_y)) {
        s_out.err_valid = 0;
        s_out.path_rows = 0;
        if (s_lost_cnt < 100000) { s_lost_cnt++; }
        s_out.lost_cnt = s_lost_cnt;
        s_out.seq = ++s_frame_cnt;
        return 1;
    }

    /* 2. 纵向爬线 */
    v2_climb(start_y);
    s_out.path_rows = v2_center_cnt;

    if (v2_center_cnt < 5) {
        s_out.err_valid = 0;
        if (s_lost_cnt < 100000) { s_lost_cnt++; }
        s_out.lost_cnt = s_lost_cnt;
        s_out.seq = ++s_frame_cnt;
        return 1;
    }

    /* 3. 直角弯 */
    v2_detect_corner();

    /* 4. 元素 */
    v2_update_element();

    /* 5. 误差 */
    v2_calc_err(start_y);

    /* 6. 打包 */
    s_out.lap = s_lap;
    s_out.fork_pass = s_fork_pass;
    s_out.lost_cnt = s_lost_cnt;
    s_out.seq = ++s_frame_cnt;

    return 1;
}

/* ==========================================================================
 * 显示
 * ========================================================================== */
void v2_disp_frame(void)
{
    int y;

    /* 二值图: 用固定的显示阈值画, 不是算法用的阈值 —— 只是为了看清 */
    tft180_show_gray_image(0, 0,
                           (const uint8 *)&mt9v03x_image[0][0],
                           V2_W, V2_H,
                           128, 81, 110);

#if (V2_DRAW_EDGES)
    /* 左边沿: 红 */
    for (y = 2; y < V2_H; y += 2) {
        int x = v2_l_edge_x[y];
        if (x < 0) { continue; }
        {
            uint16 px = (uint16)(x * 128 / V2_W);
            uint16 py = (uint16)(y * 81 / V2_H);
            if ((px < 128) && (py < 81)) {
                tft180_draw_point(px, py, RGB565_RED);
            }
        }
    }

    /* 右边沿: 绿 */
    for (y = 2; y < V2_H; y += 2) {
        int x = v2_r_edge_x[y];
        if (x < 0) { continue; }
        {
            uint16 px = (uint16)(x * 128 / V2_W);
            uint16 py = (uint16)(y * 81 / V2_H);
            if ((px < 128) && (py < 81)) {
                tft180_draw_point(px, py, RGB565_GREEN);
            }
        }
    }

    /* 中线: 黄 */
    for (y = 2; y < V2_H; y += 3) {
        int x = v2_center_x[y];
        if (x < 0) { continue; }
        {
            uint16 px = (uint16)(x * 128 / V2_W);
            uint16 py = (uint16)(y * 81 / V2_H);
            if ((px < 128) && (py < 81)) {
                tft180_draw_point(px, py, RGB565_YELLOW);
            }
        }
    }

    /* 前瞻点: 青色十字 */
    if (s_out.look_x >= 0) {
        int lx = s_out.look_x * 128 / V2_W;
        int ly = s_out.look_y * 81 / V2_H;
        int k;
        for (k = -2; k <= 2; k++) {
            if ((lx + k >= 0) && (lx + k < 128) && (ly >= 0) && (ly < 81)) {
                tft180_draw_point((uint16)(lx + k), (uint16)ly, RGB565_CYAN);
            }
            if ((lx >= 0) && (lx < 128) && (ly + k >= 0) && (ly + k < 81)) {
                tft180_draw_point((uint16)lx, (uint16)(ly + k), RGB565_CYAN);
            }
        }
    }
#endif
}

/* ==========================================================================
 * 初始化 / 任务 / 接口
 * ========================================================================== */
void v2_init(void)
{
    int retry;

    for (retry = 0; retry < 5; retry++) {
        if (mt9v03x_init() == 0) {
            s_cam_ok = 1;
            break;
        }
        system_delay_ms(200);
    }

    if (s_cam_ok) {
        mt9v03x_set_exposure_time(V2_CAM_EXP_TIME);
    }

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

    s_island_frames = 0;
    s_island_side = 0;
    s_island_on = 0;
    s_island_cooldown = 0;
    s_lap = 1;
    s_fork_pass = 0;
    s_lost_cnt = 0;
    s_thre_bias = 0;
    v2_island_side = 0;
    s_frame_cnt = 0;
    s_inited = 1;
}

void v2_task(void)
{
    if (!s_inited) { return; }
    if (s_hold) { return; }

    if (!s_cam_ok) {
        static uint32 retry = 0;
        if (++retry > 25) {
            retry = 0;
            if (mt9v03x_init() == 0) {
                s_cam_ok = 1;
                mt9v03x_set_exposure_time(V2_CAM_EXP_TIME);
            }
        }
        return;
    }

    if (mt9v03x_finish_flag) {
        mt9v03x_finish_flag = 0;

        v2_process();

        if (s_disp_on && ((s_frame_cnt % V2_DISP_EVERY_N) == 0)) {
            v2_disp_frame();
        }
    }
}

const v2_out_t *v2_get_output(void)
{
    return &s_out;
}

int v2_cam_ok(void)
{
    return (int)s_cam_ok;
}

void v2_set_disp(int on)
{
    s_disp_on = (on) ? 1 : 0;
}

void v2_set_hold(int on)
{
    s_hold = (on) ? 1 : 0;
}

void v2_set_thre_bias(int bias)
{
    if (bias < -64) { bias = -64; }
    if (bias >  64) { bias =  64; }
    s_thre_bias = bias;
}

int v2_get_thre_bias(void)
{
    return s_thre_bias;
}
