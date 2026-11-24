#include "app_ui.h"

//软毫秒计数, 由ui_tick_step推进
static volatile uint32 ui_tick_ms = 0;

//上电时调用一次: 初始化屏幕, 设置竖屏和8x16字体, 清屏
void ui_hardware_init(void)
{
    tft180_init();                              //屏幕初始化(内部有约250ms延时, 只能调用一次)
    tft180_set_dir(TFT180_PORTAIT);             //竖屏 128x160
    tft180_set_font(TFT180_8X16_FONT);          //8x16字体(库只支持6x8和8x16两种)
    ui_clear();
}

//用基准配色清屏
void ui_clear(void)
{
    tft180_set_color(UI_PEN, UI_BG);
    tft180_clear();
}

//用基准配色在指定位置画一段普通文字
void ui_draw_text(uint16 x, uint16 y, const char *text)
{
    tft180_set_color(UI_PEN, UI_BG);
    tft180_show_string(x, y, text);
}

//画顶部标题栏, 文字自动居中, 下面画一条分隔线
void ui_draw_title(const char *title)
{
    uint8 len = 0;
    uint16 x;

    while('\0' != title[len])                   //先量出字符串长度
    {
        len ++;
    }

    //8x16字体每个字符宽8像素, 算出让整串文字居中的起始x
    //如果字符串比屏幕还长, 就贴左显示, 不让它跑到屏幕外
    if(len * 8 >= UI_SCREEN_W)
    {
        x = 0;
    }
    else
    {
        x = (UI_SCREEN_W - len * 8) / 2;
    }

    ui_draw_text(x, UI_TITLE_Y, title);
    tft180_draw_line(0, UI_LINE_Y, UI_SCREEN_W - 1, UI_LINE_Y, UI_PEN);
}

//画底部提示行, line从0开始, 用6x8小字
void ui_draw_footer_line(uint8 line, const char *text)
{
    //行号算大了会画到屏幕外, tft180 内部的 assert 会直接 while(1) 卡死
    //这里提前拦掉: 越界就不画, 免得因为一行提示的位置没算对把整个程序挂住
    //6x8 字体高 8 像素, y + 8 超过屏幕高度就说明画不全了
    if((UI_FOOTER_Y + (uint16)line * UI_FOOTER_STEP + 8) > UI_SCREEN_H)
    {
        return;
    }

    tft180_set_font(TFT180_6X8_FONT);           //底部位置窄, 只有6x8塞得下
    tft180_set_color(UI_PEN, UI_BG);
    tft180_show_string(0, UI_FOOTER_Y + line * UI_FOOTER_STEP, text);
    tft180_set_font(TFT180_8X16_FONT);          //立刻换回来, 不影响后面的绘制
}

//画一行列表条目
//text长度不足UI_ROW_CHARS时会自动补空格铺满整行
void ui_draw_row(uint16 y, const char *text, uint8 selected)
{
    char buf[UI_ROW_CHARS + 1];
    uint8 i = 0;

    while('\0' != text[i] && i < UI_ROW_CHARS)  //先拷贝文字
    {
        buf[i] = text[i];
        i ++;
    }
    while(i < UI_ROW_CHARS)                     //再用空格补满整行
    {
        buf[i] = ' ';
        i ++;
    }
    buf[i] = '\0';

    if(selected)
    {
        tft180_set_color(UI_PEN_SEL, UI_BG_SEL);
    }
    else
    {
        tft180_set_color(UI_PEN, UI_BG);
    }
    tft180_show_string(0, y, buf);
}

//把"序号. 名字"拼成一段字符串
//序号支持1~99两位, 超过99只显示后两位(菜单不会有那么多条)
void ui_format_item(uint8 number, const char *name, char *buf)
{
    uint8 i = 0;
    uint8 j = 0;

    if(number >= 10)                            //十位以上才写高位
    {
        buf[i ++] = (char)('0' + (number / 10) % 10);
    }
    buf[i ++] = (char)('0' + (number % 10));
    buf[i ++] = '.';
    buf[i ++] = ' ';

    while('\0' != name[j])
    {
        buf[i ++] = name[j];
        j ++;
    }
    buf[i] = '\0';
}

//数字转十进制字符串, 返回长度
uint8 ui_num_to_str(uint32 num, char *buf)
{
    char  tmp[12];
    uint8 n = 0;
    uint8 i = 0;

    if(0 == num)                                    //0单独处理, 否则下面循环一次都不进
    {
        buf[0] = '0';
        buf[1] = '\0';
        return 1;
    }

    while(0 != num && n < 10)                       //先倒着取每一位
    {
        tmp[n] = (char)('0' + (num % 10));
        num /= 10;
        n ++;
    }
    while(0 != n)                                   //再正过来
    {
        n --;
        buf[i] = tmp[n];
        i ++;
    }
    buf[i] = '\0';

    return i;
}

//画一行"名称 + 字符串", 字符串靠右
//名称+字符串超过UI_ROW_CHARS时截断名称, 字符串始终完整显示
void ui_draw_row_text(uint16 y, const char *name, const char *value, uint8 selected)
{
    char  buf[UI_ROW_CHARS + 1];
    uint8 i = 0;
    uint8 j = 0;
    uint8 len = 0;

    while('\0' != value[len] && len < UI_ROW_CHARS)  //先量出字符串长度(超长截断)
    {
        len ++;
    }

    while('\0' != name[j] && i < UI_ROW_CHARS)       //拷贝名称
    {
        buf[i ++] = name[j];
        j ++;
    }

    if(i + len > UI_ROW_CHARS)                      //名称太长会挤掉字符串, 截断名称
    {
        i = (uint8)(UI_ROW_CHARS - len);
    }

    while(i < UI_ROW_CHARS - len)                   //中间补空格, 让字符串靠右
    {
        buf[i ++] = ' ';
    }

    j = 0;
    while(j < len)                                  //写字符串
    {
        buf[i ++] = value[j];
        j ++;
    }
    buf[i] = '\0';

    ui_draw_row(y, buf, selected);
}

//画一行"名称 + 数值", 数值靠右
void ui_draw_row_value(uint16 y, const char *name, uint32 value, uint8 selected)
{
    char num[12];

    ui_num_to_str(value, num);
    ui_draw_row_text(y, name, num, selected);
}

//把放大100倍的定点数转成"12.34"形式, 返回长度
uint8 ui_fmt_x100(uint16 value, char *buf)
{
    char   tmp[6];
    uint8  n = 0;
    uint8  i = 0;
    uint16 ipart = value / 100;                     //整数部分
    uint8  dpart = (uint8)(value % 100);            //小数部分

    if(0 == ipart)
    {
        buf[i ++] = '0';
    }
    else
    {
        while(0 != ipart && n < 5)                  //先倒着取整数各位
        {
            tmp[n ++] = (char)('0' + ipart % 10);
            ipart /= 10;
        }
        while(0 != n)                               //再正过来
        {
            n --;
            buf[i ++] = tmp[n];
        }
    }

    buf[i ++] = '.';
    buf[i ++] = (char)('0' + dpart / 10);
    buf[i ++] = (char)('0' + dpart % 10);
    buf[i] = '\0';

    return i;
}

//画一行"名称 + 定点数", 数值按x100带两位小数
void ui_draw_row_x100(uint16 y, const char *name, uint16 value, uint8 selected)
{
    char val[8];

    ui_fmt_x100(value, val);
    ui_draw_row_text(y, name, val, selected);
}

//画一行居中的文字
void ui_draw_row_center(uint16 y, const char *text, uint8 selected)
{
    char  buf[UI_ROW_CHARS + 1];
    uint8 len = 0;
    uint8 head;
    uint8 i = 0;
    uint8 j = 0;

    while('\0' != text[len] && len < UI_ROW_CHARS)   //先量长度(超长截断)
    {
        len ++;
    }

    head = (uint8)((UI_ROW_CHARS - len) / 2);       //左边留多少空格
    while(i < head)
    {
        buf[i ++] = ' ';
    }

    while(j < len)
    {
        buf[i ++] = text[j];
        j ++;
    }

    while(i < UI_ROW_CHARS)                         //补满整行, 盖掉旧内容
    {
        buf[i ++] = ' ';
    }
    buf[i] = '\0';

    ui_draw_row(y, buf, selected);
}

//画矩形边框(空心)
void ui_draw_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color)
{
    tft180_draw_line(x, y, x + w - 1, y, color);                    //上边
    tft180_draw_line(x, y + h - 1, x + w - 1, y + h - 1, color);     //下边
    tft180_draw_line(x, y, x, y + h - 1, color);                     //左边
    tft180_draw_line(x + w - 1, y, x + w - 1, y + h - 1, color);     //右边
}

//用放大2倍的字模画字符串, 每个字符占16x32像素
//字模是按列存的: ascii_font_8x16[ch-32][col] 是第col列的上半8行,
//[col+8]是同一列的下半8行, 每字节的bit0是这一列最上面的点
//放大2倍就是把每个点画成2x2的小方块
void ui_draw_big_string(uint16 x, uint16 y, const char *text, uint16 pen, uint16 bg)
{
    uint8  ch;
    uint8  col;
    uint8  row;
    uint8  src_row;
    uint8  lit;
    uint8  in_run;
    uint16 run_x;

    while('\0' != *text)
    {
        ch = (uint8)(*text);

        if(ch >= 32 && ch <= 127)
        {
            for(row = 0; row < 32; row ++)              //每个字符32行(16行放大2倍)
            {
                src_row = row / 2;                      //对应字模的第几行

                tft180_draw_line(x, y + row, x + 15, y + row, bg);   //先铺这一行的背景

                //再把这一行里连续亮的点拼成横线画出来
                //比逐点画快很多: 一个字符最多32次 draw_line, 逐点要512次
                in_run = 0;
                for(col = 0; col < 8; col ++)
                {
                    if(src_row < 8)
                    {
                        lit = (ascii_font_8x16[ch - 32][col] >> src_row) & 0x01;
                    }
                    else
                    {
                        lit = (ascii_font_8x16[ch - 32][col + 8] >> (src_row - 8)) & 0x01;
                    }

                    if(lit)
                    {
                        if(0 == in_run)
                        {
                            run_x  = x + col * 2;
                            in_run = 1;
                        }
                    }
                    else if(in_run)
                    {
                        tft180_draw_line(run_x, y + row, x + col * 2 - 1, y + row, pen);
                        in_run = 0;
                    }
                }

                if(in_run)                              //这一行结尾还有一段亮着
                {
                    tft180_draw_line(run_x, y + row, x + 15, y + row, pen);
                }
            }
        }

        x += 16;                                        //下一个字符
        text ++;
    }
}

//软毫秒计数, 主循环每圈调一次
void ui_tick_step(void)
{
    ui_tick_ms ++;
}

//取当前软毫秒计数
uint32 ui_tick_get(void)
{
    return ui_tick_ms;
}
