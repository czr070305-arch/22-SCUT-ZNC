#ifndef _APP_UI_H_
#define _APP_UI_H_

#include "zf_common_headfile.h"

//屏幕基准尺寸(竖屏 128x160)
#define UI_SCREEN_W         (128)
#define UI_SCREEN_H         (160)

//8x16字体一行刚好铺满屏幕的字符数(128 / 8 = 16)
//画整行时必须凑够这个长度, 才能把上一次的内容完全盖住
#define UI_ROW_CHARS        (16)

//统一配色
//普通项 = 蓝底白字, 选中项 = 青底黑字(整行色块高亮)
#define UI_PEN              (RGB565_WHITE)
#define UI_BG               (RGB565_BLUE)
#define UI_PEN_SEL          (RGB565_BLACK)
#define UI_BG_SEL           (RGB565_CYAN)

//标题栏和分隔线
#define UI_TITLE_Y          (0)
#define UI_LINE_Y           (16)

//底部提示区(用6x8小字, 每行间隔10像素)
#define UI_FOOTER_Y         (124)
#define UI_FOOTER_STEP      (10)

//上电时调用一次: 初始化屏幕, 设置竖屏和8x16字体, 清屏
//tft180_init内部有约250ms延时, 只能调用一次
void ui_hardware_init(void);

//用基准配色清屏(切换页面时调用)
void ui_clear(void);

//画顶部标题栏, 文字自动居中, 下面画一条分隔线
void ui_draw_title(const char *title);

//画底部提示行, line从0开始, 用6x8小字
//内部画完会把字体换回8x16, 不影响别的绘制
void ui_draw_footer_line(uint8 line, const char *text);

//画一行列表条目
//text长度不足UI_ROW_CHARS时会自动补空格铺满整行
//selected=1用青底黑字, 否则蓝底白字
void ui_draw_row(uint16 y, const char *text, uint8 selected);

//把"序号. 名字"拼成一段字符串(不补空格, 交给ui_draw_row处理)
void ui_format_item(uint8 number, const char *name, char *buf);

//数字转成十进制字符串, 返回长度(不含结束符)
//自己实现不用zf_sprintf: 它返回前不写结束符, 直接拿来用会读到乱码
//buf 至少 11 字节(uint32 最大 10 位数字 + 结束符)
uint8 ui_num_to_str(uint32 num, char *buf);

//画一行"左侧名称 + 右侧字符串", 字符串靠右对齐
//名称+字符串超过UI_ROW_CHARS时自动截断名称, 字符串始终完整显示
//selected=1用青底黑字, 否则蓝底白字
void ui_draw_row_text(uint16 y, const char *name, const char *value, uint8 selected);

//画一行"左侧名称 + 右侧数值", 数值靠右对齐
//名称+数值超过UI_ROW_CHARS时自动截断名称, 数值始终完整显示
//selected=1用青底黑字, 否则蓝底白字
void ui_draw_row_value(uint16 y, const char *name, uint32 value, uint8 selected);

//把放大100倍的定点数转成"12.34"形式, 返回长度(不含结束符)
//例: 100 -> "1.00", 1234 -> "12.34"
//buf 至少 8 字节
uint8 ui_fmt_x100(uint16 value, char *buf);

//画一行"左侧名称 + 右侧定点数", 数值按x100带两位小数
void ui_draw_row_x100(uint16 y, const char *name, uint16 value, uint8 selected);

//画一行居中的文字(用于SAVE这类没有"名称+数值"结构的行)
void ui_draw_row_center(uint16 y, const char *text, uint8 selected);

//用基准配色在指定位置画一段普通文字
void ui_draw_text(uint16 x, uint16 y, const char *text);

//画矩形边框(空心), w/h 是宽高
void ui_draw_rect(uint16 x, uint16 y, uint16 w, uint16 h, uint16 color);

//用放大2倍的字模画字符串, 每个字符占16x32像素
//自带背景填充, 会把上一次的旧内容盖掉
//用于参数编辑页的大号数字显示
void ui_draw_big_string(uint16 x, uint16 y, const char *text, uint16 pen, uint16 bg);

//软毫秒计数
//主循环每圈固定延时1ms来推进, 精度约正负2%, 做流水灯这类动画足够
//需要精确计时的话等第6项用定时器中断实现
uint32  ui_tick_get(void);
void    ui_tick_step(void);

#endif
