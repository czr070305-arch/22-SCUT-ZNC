#include "task_flash.h"
#include "app_ui.h"
#include "app_menu.h"
#include "hw_flash.h"
#include "algorithm/chassis.h"

/*
 * PID 参数查看页 (只读)
 *
 * 为什么不做编辑:
 *   编辑要处理按键、光标、步长、未保存状态, 一大套状态机,
 *   而 VOFA 那边已经有滑块和 SAVE 了, 重复做一套反而多一处出错的地方。
 *   这里只负责"看": 参数现在是多少、跟 Flash 里存的一致不一致。
 *
 * 页面内容:
 *   PID / FLASH
 *   Speed Loop PID
 *   ----------------
 *    Kp      1.500
 *    Ki      0.050
 *    Kd      0.000
 *    Flash   SAVED | UNSAVED | EMPTY
 *   ----------------
 *   Hold M:Back
 */

//============ 布局 =============
/* 布局: 标题(0~16) + 表头(18) + 6 行参数 + 1 行状态
 * 7 行 × 12px, 从 32 起 = 32+84 = 116 < footer(124), 放得下。
 * 用 6x8 小字体: 8x16 会顶到 footer。 */
#define FLASH_HEADER_Y      (18)
#define FLASH_ROW_Y0        (32)
#define FLASH_ROW_STEP      (12)

#define FLASH_REFRESH_MS    (200)               //自动刷新周期

//============ 状态 =============

//上一轮画到屏幕上的值, 用来判断要不要重画
//只有值真的变了才重画, 否则每 200ms 刷一次屏会闪
static float  flash_shown_pid[6] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
static uint8  flash_shown_state = 0xFF;

static uint32 flash_last_ms = 0;

//============ 小工具 =============

//浮点转字符串, 固定 3 位小数
//不用 sprintf("%.3f"): 没有硬件浮点, 那套格式化代码又大又慢
static void flash_fmt_float(float v, char *buf)
{
    int32  n1000;
    int32  ipart;
    int32  frac;
    uint8  i = 0;
    uint8  d = 0;
    char   tmp[8];

    if(v < 0.0f)
    {
        buf[i ++] = '-';
        v = -v;
    }

    n1000 = (int32)(v * 1000.0f + 0.5f);        //四舍五入到 0.001
    if(n1000 > 9999000)
    {
        n1000 = 9999000;                        //防止超长
    }

    ipart = n1000 / 1000;
    frac  = n1000 % 1000;

    if(0 == ipart)
    {
        buf[i ++] = '0';
    }
    else
    {
        d = 0;
        while((ipart > 0) && (d < 4))
        {
            tmp[d ++] = (char)('0' + (ipart % 10));
            ipart /= 10;
        }
        while(0 != d)
        {
            buf[i ++] = tmp[-- d];
        }
    }

    buf[i ++] = '.';
    buf[i ++] = (char)('0' + (frac / 100) % 10);
    buf[i ++] = (char)('0' + (frac / 10) % 10);
    buf[i ++] = (char)('0' + frac % 10);
    buf[i] = '\0';
}

//读一个参数(下标 0/1/2), 从底盘拿当前真正在用的值
/* idx: 0=左Kp 1=左Ki 2=左Kd 3=右Kp 4=右Ki 5=右Kd */
static float flash_get(uint8 idx)
{
    float kp;
    float ki;
    float kd;

    chassis_get_param((idx < 3) ? ENC_LEFT : ENC_RIGHT, &kp, &ki, &kd);

    switch(idx % 3)
    {
        case 0:  return kp;
        case 1:  return ki;
        default: return kd;
    }
}

//比较两组参数是否相同(按 0.001 的粒度)
//直接比 float 相等不可靠, 只比显示出来的三位小数就够了
static uint8 flash_same(float a, float b)
{
    int32 x = (int32)(a * 1000.0f + 0.5f);
    int32 y = (int32)(b * 1000.0f + 0.5f);

    return (uint8)(x == y);
}

//当前状态: 0=和Flash一致 1=有改动没保存 2=Flash里没有有效数据
static uint8 flash_state(void)
{
    uint8 i;

    //先看内存里在用的值和 Flash 里的快照(g_param)一不一致
    //不一致说明上位机改过但还没 SAVE
    for(i = 0; i < 6; i ++)
    {
        float saved;

        switch(i)
        {
            case 0:  saved = g_param.l_kp; break;
            case 1:  saved = g_param.l_ki; break;
            case 2:  saved = g_param.l_kd; break;
            case 3:  saved = g_param.r_kp; break;
            case 4:  saved = g_param.r_ki; break;
            default: saved = g_param.r_kd; break;
        }

        if(!flash_same(flash_get(i), saved))
        {
            return 1;                           //UNSAVED
        }
    }

    //值一致, 还要看 Flash 里到底有没有数据
    //光靠"全是 0"判断不出来: 0 也是用户可能存进去的合法值
    if(!g_param_valid)
    {
        return 2;                               //EMPTY
    }

    return 0;                                   //SAVED
}

//============ 绘制 =============

//画一行 "名字 + 三位小数"
static void flash_draw_param_row(uint8 idx, const char *name)
{
    char buf[12];

    flash_fmt_float(flash_get(idx), buf);
    tft180_set_font(TFT180_6X8_FONT);
    ui_draw_row_text(FLASH_ROW_Y0 + idx * FLASH_ROW_STEP, name, buf, 0);
}

//画状态行
static void flash_draw_state_row(void)
{
    static const char * const state_text[3] =
    {
        "SAVED",        //和 Flash 一致
        "UNSAVED",      //VOFA 改了但没 SAVE
        "EMPTY",        //还没调过参数
    };

    uint8 st = flash_state();

    //ui_draw_row_text 内部会把整行补满到 UI_ROW_CHARS, 直接画状态文字即可
    //（原来先垫一层空格的写法会写到 buf[16], 越界一个字节）

    ui_draw_row_text(FLASH_ROW_Y0 + 6 * FLASH_ROW_STEP, "Flash", state_text[st], 0);
}

//画整页
static void flash_draw_all(void)
{
    static const char * const names[6] =
    {
        "L Kp", "L Ki", "L Kd", "R Kp", "R Ki", "R Kd"
    };
    uint8 i;

    ui_clear();
    ui_draw_title("PID / FLASH");

    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(UI_PEN, UI_BG);
    tft180_show_string(0, FLASH_HEADER_Y, "Speed Loop PID L/R");

    tft180_draw_line(0, FLASH_HEADER_Y + 9, UI_SCREEN_W - 1, FLASH_HEADER_Y + 9, UI_PEN);

    for(i = 0; i < 6; i ++)
    {
        flash_draw_param_row(i, names[i]);
        flash_shown_pid[i] = flash_get(i);
    }

    flash_draw_state_row();
    flash_shown_state = flash_state();

    ui_draw_footer_line(0, "Read Only");
    ui_draw_footer_line(1, "Hold M: Back");
}

//============ 对外接口 =============

void task_flash_show(void)
{
    flash_last_ms   = ui_tick_get();
    flash_shown_pid[0] = -1.0f;                 //强制刷新一次
    flash_shown_pid[1] = -1.0f;
    flash_shown_pid[2] = -1.0f;
    flash_shown_pid[3] = -1.0f;
    flash_shown_pid[4] = -1.0f;
    flash_shown_pid[5] = -1.0f;
    flash_shown_state  = 0xFF;

    flash_draw_all();
}

//只读页面, 按键不做事
void task_flash_key(key_id_enum id)
{
    (void)id;
}

//主循环: 参数变了才重画
void task_flash_run(void)
{
    uint8 i;
    uint8 changed = 0;

    if(ui_tick_get() - flash_last_ms < FLASH_REFRESH_MS)
    {
        return;
    }
    flash_last_ms = ui_tick_get();

    for(i = 0; i < 6; i ++)
    {
        static const char * const names[6] =
        {
            "L Kp", "L Ki", "L Kd", "R Kp", "R Ki", "R Kd"
        };

        if(!flash_same(flash_shown_pid[i], flash_get(i)))
        {
            flash_draw_param_row(i, names[i]);
            flash_shown_pid[i] = flash_get(i);
            changed = 1;
        }
    }

    if(flash_shown_state != flash_state())
    {
        flash_draw_state_row();
        flash_shown_state = flash_state();
    }

    (void)changed;
}

void task_flash_leave(void)
{
}
