#include "app_menu.h"
#include "app_ui.h"
#include "hw_led.h"
#include "hw_key.h"
#include "hw_flash.h"
#include "task_flash.h"
#include "task_motor.h"
#include "vision/vision.h"
#include "task_raw.h"
#include "task_capture.h"
#include "task_track.h"
#include "task_vofa.h"
#include "task_imu.h"
#include "algorithm/chassis.h"
#include "algorithm/ctrl.h"

//列表布局
//标题栏占 y=0~16, 条目从 y=24 开始, 每行16像素, 最多放6行到 y=120
#define MENU_ITEM_Y0        (24)
#define MENU_ITEM_STEP      (16)
#define MENU_ITEM_VISIBLE   (6)

//==================================================
// 页面状态(对应培训资料里的 page 和 page_temp)
//==================================================

//当前页面
static page_enum menu_page = PAGE_MAIN;

//光标位置, 主菜单和任务列表共用, 切换页面时清零
static uint8 menu_cursor = 0;

//列表滚动偏移(条目多于可见行时才会用到)
static uint8 menu_scroll = 0;

//上一次的按键值, 用来做边沿检测
static uint8 menu_last_id = 0xFF;

//刚进任务页时置1, 等按键松开再让任务响应(否则进入任务的那次按键会被任务自己再处理一遍)
static uint8 menu_wait_release = 0;

//任务页请求返回上级菜单的标志(比如按键页长按中键)
//任务页自己不直接切页, 只把这个标志置1, 交给主循环统一处理
//这样切页逻辑始终只在一处, 任务页不用关心菜单怎么工作
static uint8 menu_back_request = 0;

//请求返回时要回到哪一页
//进页面时自动记下上级, 收到返回请求时回到这里
//初值给PAGE_MAIN, 保证上电还没进过任何页面时也有个安全的落点
static page_enum menu_back_page = PAGE_MAIN;

//==================================================
// 页面内容表
//==================================================

//列表条目: 显示文字和对应页面绑在一起
typedef struct
{
    const char *name;                           //列表上显示的文字
    page_enum   page;                           //选中后进入的页面
}menu_item_struct;

//主菜单目录
static const menu_item_struct menu_main_items[] =
{
    {"TASK TEST",   PAGE_TASK},
    {"PID / FLASH", PAGE_PARAM},
};
#define MENU_MAIN_COUNT     (sizeof(menu_main_items) / sizeof(menu_main_items[0]))

//任务列表目录
static const menu_item_struct menu_task_items[] =
{
    {"TRACK",       PAGE_TASK_TRACK},
    {"IMU TEST",    PAGE_TASK_IMU},
    {"MOTOR TEST",  PAGE_TASK_MOTOR},
    {"RAW CAM",     PAGE_TASK_RAW},
    {"CAPTURE",     PAGE_TASK_CAPTURE},
};
#define MENU_TASK_COUNT     (sizeof(menu_task_items) / sizeof(menu_task_items[0]))

//==================================================
// 任务页操作表
// 新增一个测试页只需在这里加一行
//==================================================
typedef struct
{
    page_enum page;                             //页面编号
    void (*show)(void);                         //进页面时画界面
    void (*key)(key_id_enum id);                //本页的按键处理
    void (*run)(void);                          //主循环每圈调一次
    void (*leave)(void);                        //离开本页时收尾
    uint8 left_back;                            //1=左键返回上级, 0=左键交给本页自己处理
    page_enum back_page;                        //左键返回时回到哪一页
}page_ops_struct;

static const page_ops_struct task_page_ops[] =
{
    {PAGE_TASK_TRACK,   task_track_show,   task_track_key,   task_track_run,   task_track_leave,   1, PAGE_TASK},
    {PAGE_TASK_MOTOR,   task_motor_show,   task_motor_key,   task_motor_run,   task_motor_leave,   1, PAGE_TASK},
    {PAGE_TASK_RAW,     task_raw_show,     task_raw_key,     task_raw_run,     task_raw_leave,     1, PAGE_TASK},
    {PAGE_TASK_CAPTURE, task_capture_show, task_capture_key, task_capture_run, task_capture_leave, 1, PAGE_TASK},
    {PAGE_TASK_IMU,     task_imu_show,     task_imu_key,     task_imu_run,     task_imu_leave,     1, PAGE_TASK},
    {PAGE_PARAM,        task_flash_show,   task_flash_key,   task_flash_run,   task_flash_leave,   1, PAGE_MAIN},
};
#define TASK_PAGE_COUNT     (sizeof(task_page_ops) / sizeof(task_page_ops[0]))

/**
 * 按页面编号查任务页操作表
 *
 * @param page  要查的页面编号
 * @return      找到: 那一行表项的地址, 用 -> 就能取出里面的函数来调
 *              没找到(主菜单/关于页): 0
 */
static const page_ops_struct * menu_find_ops(page_enum page)
{
    uint8 i;

    for(i = 0; i < TASK_PAGE_COUNT; i ++)
    {
        if(task_page_ops[i].page == page)
        {
            return &task_page_ops[i];
        }
    }
    return 0;
}

//==================================================
// 绘制
//==================================================

//按当前页面取条目表, 供绘制和光标处理共用
static const menu_item_struct * menu_get_items(uint8 *count)
{
    if(PAGE_MAIN == menu_page)
    {
        *count = MENU_MAIN_COUNT;
        return menu_main_items;
    }

    *count = MENU_TASK_COUNT;
    return menu_task_items;
}

//画列表的全部条目
static void menu_draw_rows(void)
{
    const menu_item_struct *items = 0;
    uint8 count = 0;
    char  buf[24];
    uint8 i;

    items = menu_get_items(&count);

    for(i = 0; i < MENU_ITEM_VISIBLE; i ++)
    {
        uint16 y   = MENU_ITEM_Y0 + i * MENU_ITEM_STEP;
        uint8  idx = menu_scroll + i;

        if(idx < count)
        {
            ui_format_item(idx + 1, items[idx].name, buf);
            ui_draw_row(y, buf, (idx == menu_cursor));
        }
        else
        {
            ui_draw_row(y, "", 0);              //空行也要铺底色, 盖掉上一页残留
        }
    }
}

//移动光标并只重画条目区(不整屏清, 避免闪烁)
static void menu_cursor_move(int8 step)
{
    uint8 count = 0;
    int16 next;

    menu_get_items(&count);
    if(0 == count)
    {
        return;
    }

    next = (int16)menu_cursor + step;
    if(next < 0)
    {
        next = (int16)count - 1;                //越界就绕回去
    }
    else if(next >= (int16)count)
    {
        next = 0;
    }
    menu_cursor = (uint8)next;

    //保证光标始终在可见范围内
    if(menu_cursor < menu_scroll)
    {
        menu_scroll = menu_cursor;
    }
    else if(menu_cursor >= menu_scroll + MENU_ITEM_VISIBLE)
    {
        menu_scroll = menu_cursor - MENU_ITEM_VISIBLE + 1;
    }

    menu_draw_rows();
}

//==================================================
// LCD_show: 按 page 画对应界面
//对应培训资料里的 LCD_show() 函数
//==================================================
static void menu_show(void)
{
    switch(menu_page)
    {
        case PAGE_MAIN:
        {
            ui_clear();
            ui_draw_title("MAIN MENU");
            menu_draw_rows();
            ui_draw_footer_line(0, "U/D:Move  M/R:Enter");
            break;
        }

        case PAGE_TASK:
        {
            ui_clear();
            ui_draw_title("TASK TEST");
            menu_draw_rows();
            ui_draw_footer_line(0, "U/D:Move  M/R:Enter");
            ui_draw_footer_line(1, "L:Back");
            break;
        }

        case PAGE_ABOUT:
        {
            ui_clear();
            ui_draw_title("ABOUT");
            ui_draw_text(0, 28, "BOARD CH32V307");
            ui_draw_text(0, 46, "LCD   TFT180 ");
            ui_draw_text(0, 64, "KEY   5WAY+2 ");
            ui_draw_text(0, 82, "CAM   MT9V034");
            ui_draw_footer_line(0, "L:Back");
            break;
        }

        //任务页交给各自的模块画, 不在表里的就是菜单自己的页面
        default:
        {
            //用当前页面编号去查任务表,将查到的表项的地址存进ops
            const page_ops_struct *ops = menu_find_ops(menu_page);
            if(ops)
            {
                ops->show();
            }
            break;
        }
    }
}

static void menu_goto(page_enum page)
{
    //将离开页的表项地址存进old_ops
    //将进入页的表项地址存进new_ops
    const page_ops_struct *old_ops = menu_find_ops(menu_page);
    const page_ops_struct *new_ops = menu_find_ops(page);

    //离开任务页时让本页收尾(比如停车、清状态)
    if(old_ops)
    {
        old_ops->leave();
    }

    menu_back_page = menu_page;             //记下"从哪来", 返回时回到这里
    menu_page   = page;
    menu_cursor = 0;
    menu_scroll = 0;

    //进任务页时等按键松开再开始响应, 避免进入的那次按键被重复处理
    if(new_ops)
    {
        menu_wait_release = 1;
    }

    menu_show();
}

//==================================================
// UI_keycut: 按 page 处理按键
//对应培训资料里的 UI_keycut() 函数
//==================================================
static void menu_keycut(key_id_enum id)
{
    switch(menu_page)
    {
        //主菜单: 上下移动光标, 中/右进入
        case PAGE_MAIN:
        {
            if(KEY_ID_UP == id)
            {
                menu_cursor_move(-1);
            }
            else if(KEY_ID_DOWN == id)
            {
                menu_cursor_move(1);
            }
            else if(KEY_ID_MID == id || KEY_ID_RIGHT == id)
            {
                menu_goto(menu_main_items[menu_cursor].page);   //查表进入
            }
            break;
        }

        //任务列表: 上下移动光标, 中/右进入任务, 左返回
        case PAGE_TASK:
        {
            if(KEY_ID_UP == id)
            {
                menu_cursor_move(-1);
            }
            else if(KEY_ID_DOWN == id)
            {
                menu_cursor_move(1);
            }
            else if(KEY_ID_MID == id || KEY_ID_RIGHT == id)
            {
                menu_goto(menu_task_items[menu_cursor].page);   //查表进入
            }
            else if(KEY_ID_LEFT == id)
            {
                menu_goto(PAGE_MAIN);
            }
            break;
        }

        //关于页: 左返回
        case PAGE_ABOUT:
        {
            if(KEY_ID_LEFT == id)
            {
                menu_goto(PAGE_MAIN);
            }
            break;
        }

        //任务页: 左键退出, 其余交给任务自己的处理函数
        case PAGE_TASK_MOTOR:
        case PAGE_TASK_CAPTURE:
        case PAGE_TASK_RAW:
        case PAGE_TASK_TRACK:
        case PAGE_TASK_IMU:
        case PAGE_PARAM:
        {
            //查当前页面编号去查任务表,将查到的表项的地址存进ops
            const page_ops_struct *ops = menu_find_ops(menu_page);

            if(0 == ops)
            {
                break;
            }

            //左键返回上级, 但表里标了 left_back=0 的页面要自己处理左键
            //按键测试页就需要这样: 左键要留着给它自己测, 不能拿去返回
            if(KEY_ID_LEFT == id && ops->left_back)
            {
                menu_goto(ops->back_page);      //返回到本页登记的上级
                break;
            }

            ops->key(id);
            break;
        }

        default: break;
    }
}

//==================================================
// 对外接口
//==================================================

//初始化全工程外设并画出主菜单 (顺序要求见 app_menu.h 的说明)
void app_menu_init(void)
{
    led_init();
    hw_key_init();

    /* flash_param_load() 已挪到 main.c，在 hw_imu_init() 之前:
     *   IMU 初始化时要用 Flash 里的零偏，比这里早。
     * 这里不能再调一次 —— load 里会 param_set_default() 覆盖 g_param,
     * 把 hw_imu_init 刚读进 s_bias 的零偏跟 g_param 对不上。 */

    chassis_init();                             //初始化底盘: 电机 + 编码器 + 启动速度环节拍
    ctrl_init();                                //循迹控制层: 转向环 + 巡航速度
    ui_hardware_init();                         //屏幕初始化(内部有长延时, 只能调用一次)
    vision_init();                              //摄像头 + 视觉算法, 由 vision 统一负责
    vofa_init();                                //清掉串口残留, VOFA 状态归零

    menu_page   = PAGE_MAIN;
    menu_cursor = 0;
    menu_scroll = 0;
    menu_show();

    menu_last_id = (uint8)hw_key_get();         //记下当前按键, 避免开机就触发一次
}

//菜单主循环: 刷新当前页
void app_menu_run(void)
{
    uint8 id;

    system_delay_ms(1);                 //固定1ms节拍
    ui_tick_step();                     //软毫秒+1
    vofa_run();                         //VOFA+ 上位机服务, 不占页面

    id = (uint8)hw_key_get();           //读按键

    //边沿检测
    if(id != menu_last_id)
    {
        menu_last_id = id;

        if(KEY_ID_NONE == id)           //松手, 解除等待
        {
            menu_wait_release = 0;
        }
        else if(!menu_wait_release)     //按下, 且已过等松手阶段
        {
            menu_keycut((key_id_enum)id);
        }
    }

    //等松手期间不刷新页面
    if(0 == menu_wait_release)
    {
        //先处理返回请求
        if(menu_back_request)
        {
            menu_back_request = 0;
            menu_goto(menu_back_page);
        }
        //跑当前页
        else                           
        {
            //按当前页号查表, 主菜单和列表页查不到, 它们不需要刷新
            const page_ops_struct *ops = menu_find_ops(menu_page);

            if(ops)
            {
                ops->run();
            }
        }
    }
}


//任务页请求返回上级菜单
//任务页只登记请求, 真正切页由 app_menu_run 的主循环去做
//这样避免了切页过程里还在用旧页面变量这类问题
void menu_back_to_task(void)
{
    menu_back_request = 1;
}

//登记当前页的上级页面
//子页面在 show() 里调用, 告诉菜单"我从哪来的"
//因为同一个页面可能从不同入口进入, 上级页面不固定
void menu_set_back_page(page_enum page)
{
    menu_back_page = page;
}
