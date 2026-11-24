/* ==========================================================================
 *  vision_display.c —— 把视觉图像实时刷到屏幕上（TFT180 / OLED 双后端）
 *  --------------------------------------------------------------------------
 *  【为什么需要这个文件】
 *  PC 仿真里图像是显示在电脑窗口上的，车上必须自己画到屏幕上。
 *  逐飞库已经提供了 tft180_show_gray_image() / oled_show_gray_image()，
 *  它们的最后一个参数 threshold 不为 0 时会"按阈值画成黑白"，
 *  所以把"原始灰度图 + 本帧阈值 g_thre"直接传进去就是二值显示，
 *  不需要额外存一份二值图，也不需要缩放缓存。
 *
 *  【屏幕选择】
 *  - VISION_DISP_SEL = 0：板载 1.91 寸 ST7735 TFT（160x128 横屏），
 *    控制层 assessment_app.c 已经初始化过，这里只负责往上画。
 *  - VISION_DISP_SEL = 1：0.96 寸 SSD1306 OLED（128x64）。
 *  两者引脚完全相同（SPI2 的 B13/B15 + B7/D7/D4），同一时刻只能用其一。
 *
 *  【刷新节奏】
 *  刷一屏 160x112 需要约 10~13ms（SPI 24MHz，逐点写），如果每帧都刷，
 *  50 帧的图像处理会被拖慢一半。所以每 CFG_DISP_EVERY_N 帧刷一次，
 *  默认 5 → 10Hz，肉眼看起来已经是"实时"。
 *
 *  【和控制菜单共屏】
 *  控制层按键时会重画菜单，视觉图会把它盖掉。这里用"按键活动检测"：
 *  检测到有人按键就暂停刷图 CFG_DISP_PAUSE_FRAMES 帧，让菜单能看清；
 *  不按键（跑车时）就一直显示视觉图。这样不需要改控制代码一行。
 * ========================================================================== */
#include "vision.h"

/* --------------------------------------------------------------------------
 * 屏幕布局常量
 * -------------------------------------------------------------------------- */
#if (VISION_DISP_SEL == VISION_DISP_OLED)
    #define SCR_W           (128)
    #define SCR_H           (64)
    #define IMG_W           (128)           /* 图像显示宽 */
    #define IMG_H           (56)            /* 图像显示高（下面留 8px 状态行） */
    #define STATUS_Y        (56)
    #define STATUS_LINES    (1)
    #define INK_ON          (1)
    #define INK_OFF         (0)
#else
    /* 本工程屏幕: 逐飞 TFT180 竖屏 128x160 (app_ui.c 里 tft180_set_dir 设的)
     *
     * 为什么是竖屏而不是横屏:
     *   控制层的菜单/参数页全部按竖屏排版 (UI_SCREEN_W=128, UI_SCREEN_H=160),
     *   视觉如果按横屏 160x128 画, tft180_show_gray_image 内部的
     *   zf_assert(x2 < tft180_x_max) 会失败 -> while(1) 死机。
     *
     * 布局:
     *   y = 0  .. 80    二值图 128x81 (188x120 等比缩放)
     *   y = 83 .. 103   三行技术信息 (task_track.c 画)
     *   y = 124..133    footer (app_ui.c 画)
     */
    #define SCR_W           (128)
    #define SCR_H           (160)
    #define IMG_W           (128)
    #define IMG_H           (81)
    #define STATUS_LINES    (0)             /* 0=本文件不画状态行, 交给 task_track.c */
    #define INK_ON          (RGB565_WHITE)
    #define INK_OFF         (RGB565_BLACK)
#endif

/* 图像坐标 → 屏幕坐标（整数缩放，避免浮点） */
#define MAP_X(x)        ((uint16)((x) * IMG_W / V_W))
#define MAP_Y(y)        ((uint16)((y) * IMG_H / V_H))

/* --------------------------------------------------------------------------
 * 统一的"画点 / 画字"小接口，让下面的覆盖层代码不用写两遍
 * -------------------------------------------------------------------------- */
static void scr_point(uint16 x, uint16 y, uint16 color)
{
    /* 越界钳位: 覆盖层会画 3x3 十字(lx+-1)和折角竖线(path_top+-3),
     * 边缘处会越界。逐飞 tft180_draw_point 内有 zf_assert,
     * 越界直接 while(1) 死机, 所以必须在这里挡住。 */
    if (x >= SCR_W) { return; }
    if (y >= SCR_H) { return; }
#if (VISION_DISP_SEL == VISION_DISP_OLED)
    oled_draw_point(x, y, (uint8)color);
#else
    tft180_draw_point(x, y, color);
#endif
}

static void scr_string(uint16 x, uint16 y, const char *s)
{
#if (VISION_DISP_SEL == VISION_DISP_OLED)
    oled_show_string(x, y, s);
#else
    tft180_show_string(x, y, s);
#endif
}

/* --------------------------------------------------------------------------
 * 屏幕初始化
 * -------------------------------------------------------------------------- */
void disp_init(void)
{
#if (VISION_DISP_SEL == VISION_DISP_OLED)
    /* OLED：128x64 横屏、6x8 字体 */
    oled_set_dir(OLED_CROSSWISE);
    oled_set_font(OLED_6X8_FONT);
    oled_init();
    oled_clear();
#else
    /* TFT180：控制层已经初始化（见 assessment_app.c 的 tft180_init）。
     * 若单独跑视觉，把 VISION_DISP_TFT_SELF_INIT 改成 1。 */
    #if (VISION_DISP_TFT_SELF_INIT)
        tft180_set_dir(TFT180_CROSSWISE);
        tft180_set_font(TFT180_6X8_FONT);
        tft180_init();
        tft180_clear();
    #endif
#endif
}

/* --------------------------------------------------------------------------
 * 按键活动检测（只读引脚，不影响控制层的按键扫描）
 *
 *  板上三个按键（原理图 H1）：KEY1 = D8，KEY2 = B12，KEY3 = B14，按下为低。
 *  这里不做消抖，只判断"电平有没有变化"，用来决定要不要让出屏幕。
 * -------------------------------------------------------------------------- */
/* 板载五向按键是三线编码（KEY_ENCODER_A0/A1/A2），见 assessment_app.c：
 *      A0 = D8，A1 = B12，A2 = B14（上拉输入，按下变低）
 * 这里只"旁听"这三根线的电平变化，不做消抖、不消费任何按键事件，
 * 所以不会影响控制层自己的按键扫码逻辑。 */
#define VKEY_A0     (D8)
#define VKEY_A1     (B12)
#define VKEY_A2     (B14)

int disp_key_pressed(void)
{
    static uint8 s_last = 0xFF;
    uint8 now;

    now = (uint8)((gpio_get_level(VKEY_A0) ? 1 : 0) |
                  ((gpio_get_level(VKEY_A1) ? 1 : 0) << 1) |
                  ((gpio_get_level(VKEY_A2) ? 1 : 0) << 2));

    if (now != s_last) {
        s_last = now;
        return 1;
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * 画覆盖层：中线 / 前瞻点 / 折角点
 *
 *  为什么只画这几个：
 *   - 中线（红）：一眼能看出爬线有没有跟错段；
 *   - 前瞻点（绿 3x3 十字）：转向误差就是拿它的 x 减画面中心算的，
 *     屏幕上看到它偏左/偏右，就等于看到控制量；
 *   - 折角远端（黄竖线）：折角时前瞻会被强制指到这里。
 *  元素类型/圈数写在状态行，不画在图上，避免挡住赛道。
 * -------------------------------------------------------------------------- */
static void draw_overlay(void)
{
    int y;

    /* 中线：每 3 行画一个点（整行都画会太慢、也太粗） */
    for (y = 0; y < V_H; y += 3) {
        if (g_mid_line[y] >= 0) {
#if (VISION_DISP_SEL == VISION_DISP_OLED)
            scr_point(MAP_X(g_mid_line[y]), MAP_Y(y), 1);
#else
            scr_point(MAP_X(g_mid_line[y]), MAP_Y(y), RGB565_RED);
#endif
        }
    }

    /* 折角远端：画一小段竖线，提示"线往这边拐了" */
    if ((g_corner_dir != 0) && (g_path_top >= 0)) {
        int k;
        uint16 cx = MAP_X(g_corner_far_x);

        for (k = -3; k <= 3; k++) {
            int yy = g_path_top + k;
            if ((yy >= 0) && (yy < V_H)) {
#if (VISION_DISP_SEL == VISION_DISP_OLED)
                scr_point(cx, MAP_Y(yy), 1);
#else
                scr_point(cx, MAP_Y(yy), RGB565_YELLOW);
#endif
            }
        }
    }

    /* 前瞻点：3x3 十字 */
    if (g_look_x >= 0) {
        uint16 lx = MAP_X(g_look_x);
        uint16 ly = MAP_Y(g_look_y);
        int k;

        for (k = -1; k <= 1; k++) {
#if (VISION_DISP_SEL == VISION_DISP_OLED)
            scr_point((uint16)(lx + k), ly, 1);
            scr_point(lx, (uint16)(ly + k), 1);
#else
            scr_point((uint16)(lx + k), ly, RGB565_GREEN);
            scr_point(lx, (uint16)(ly + k), RGB565_GREEN);
#endif
        }
    }
}

/* --------------------------------------------------------------------------
 * 刷一帧
 * -------------------------------------------------------------------------- */
void disp_frame(void)
{
    /* 屏幕上的"二值图"是怎么来的：
     * 不单独存一份二值图（那样要多 22.5KB RAM），而是把原始灰度图 + 本帧阈值
     * 一起交给屏驱动，由驱动逐像素判断"大于阈值画白、否则画黑"。
     * 所以你看到的仍然是货真价实的二值图像，而且和算法用的是同一个阈值，
     * 阈值调好没好，一眼就能看出来。 */
#if (VISION_DISP_SEL == VISION_DISP_OLED)
    oled_show_gray_image(0, 0, (const uint8 *)&mt9v03x_image[0][0], V_W, V_H,
                         IMG_W, IMG_H, (uint8)((g_thre > 0) ? g_thre : 1));
#else
    tft180_show_gray_image(0, 0, (const uint8 *)&mt9v03x_image[0][0], V_W, V_H,
                           IMG_W, IMG_H, (uint8)((g_thre > 0) ? g_thre : 1));
#endif
    draw_overlay();

    /* 状态行：阈值 / 误差 / 元素 / 圈数 / 丢线计数 */
#if (STATUS_LINES > 0)
    {
        char line[24];
        int i, v;
        const char *elem_name = "NORM";

        if (g_elem == 1) { elem_name = "CROS"; }
        else if (g_elem == 2) { elem_name = "FORK"; }
        else if (g_elem == 3) { elem_name = "ISLE"; }

        /* 手写格式化，避免 sprintf 在嵌入式里吃栈和时间 */
        i = 0;
        line[i++] = 'T';
        v = g_thre;
        line[i++] = (char)('0' + (v / 100) % 10);
        line[i++] = (char)('0' + (v / 10) % 10);
        line[i++] = (char)('0' + v % 10);
        line[i++] = ' ';
        line[i++] = 'e';
        v = (int)(g_steer_err);
        if (v < 0) { line[i++] = '-'; v = -v; }
        else { line[i++] = '+'; }
        if (v > 99) { v = 99; }
        line[i++] = (char)('0' + (v / 10) % 10);
        line[i++] = (char)('0' + v % 10);
        line[i++] = ' ';
        line[i++] = elem_name[0];
        line[i++] = elem_name[1];
        line[i++] = ' ';
        line[i++] = 'L';
        line[i++] = (char)('0' + (g_lap % 10));
        line[i++] = ' ';
        line[i++] = 'F';
        line[i++] = (char)('0' + (g_fork_pass % 10));
        line[i] = '\0';

        /* 清掉状态行背景，再写字（否则上一帧的字符会重叠） */
        {
            int xx, yy;
            for (yy = STATUS_Y; yy < SCR_H; yy++) {
                for (xx = 0; xx < SCR_W; xx++) {
                    scr_point((uint16)xx, (uint16)yy, INK_OFF);
                }
            }
        }
        scr_string(0, STATUS_Y, line);

#if (STATUS_LINES >= 2)
        /* 第二行：路径行数 / 前瞻点 / 丢线计数，调试时很有用 */
        i = 0;
        line[i++] = 'R';
        v = g_path_rows;
        line[i++] = (char)('0' + (v / 100) % 10);
        line[i++] = (char)('0' + (v / 10) % 10);
        line[i++] = (char)('0' + v % 10);
        line[i++] = ' ';
        line[i++] = 'X';
        v = g_look_x;
        if (v < 0) { line[i++] = '-'; line[i++] = '-'; line[i++] = '-'; }
        else {
            line[i++] = (char)('0' + (v / 100) % 10);
            line[i++] = (char)('0' + (v / 10) % 10);
            line[i++] = (char)('0' + v % 10);
        }
        line[i++] = ' ';
        line[i++] = 'Y';
        v = g_look_y;
        if (v < 0) { line[i++] = '-'; line[i++] = '-'; line[i++] = '-'; }
        else {
            line[i++] = (char)('0' + (v / 100) % 10);
            line[i++] = (char)('0' + (v / 10) % 10);
            line[i++] = (char)('0' + v % 10);
        }
        line[i++] = ' ';
        line[i++] = 'o';
        v = g_lost_cnt;
        if (v > 99) { v = 99; }
        line[i++] = (char)('0' + (v / 10) % 10);
        line[i++] = (char)('0' + v % 10);
        line[i] = '\0';

        scr_string(0, (uint16)(STATUS_Y + 8), line);
#endif
    }
#endif  /* STATUS_LINES > 0 */
}

/* --------------------------------------------------------------------------
 * 只显示两行文字（摄像头没初始化成功时用来报错）
 * -------------------------------------------------------------------------- */
void disp_status(const char *s1, const char *s2)
{
    int xx, yy;

#if (VISION_DISP_SEL == VISION_DISP_OLED)
    oled_clear();
#else
    tft180_clear();
#endif
    for (yy = 0; yy < SCR_H; yy++) {
        for (xx = 0; xx < SCR_W; xx++) {
            scr_point((uint16)xx, (uint16)yy, INK_OFF);
        }
    }
    scr_string(0, 0, s1);
    if (s2 != 0) {
        scr_string(0, 10, s2);
    }
}
