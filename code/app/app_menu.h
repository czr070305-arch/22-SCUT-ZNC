#ifndef _APP_MENU_H_
#define _APP_MENU_H_

#include "zf_common_headfile.h"

//页面编号
//和培训资料里的 page 变量是一回事, 每个值代表一块界面
typedef enum
{
    PAGE_MAIN = 0,          //主菜单
    PAGE_TASK,              //任务列表
    PAGE_TASK_MOTOR,        //电机+编码器测试
    PAGE_TASK_IMU,          //IMU角度测试
    PAGE_TASK_RAW,          //摄像头原画面(调镜头/曝光)
    PAGE_TASK_CAPTURE,      //图像采集(传PC做训练集)
    PAGE_TASK_TRACK,        //循迹调试(上车跑, 含视觉画面)
    PAGE_PARAM,             //PID参数(Flash保存)
    PAGE_ABOUT,             //关于
    PAGE_MAX
}page_enum;

/*
 * 初始化全工程外设, 并画出主菜单
 *
 * 顺序有讲究, 不要调换:
 *   1. flash_param_load  先从 Flash 读 PID 参数
 *   2. chassis_init      底盘用它初始化速度环
 *   3. ctrl_init         控制层
 *   4. ui_hardware_init  屏幕(内部有 250ms 延时, 全工程只能调一次)
 *   5. vision_init       摄像头 + 视觉算法
 *                        ★ 必须在 ui_hardware_init 之后:
 *                          视觉只往屏幕上画, 不自己初始化屏
 *                        ★ 不要在这里再调 mt9v03x_init():
 *                          视觉内部已经做了, 重复握手会失败
 *   6. vofa_init         清串口残留
 *   7. menu_show         最后画菜单, 避免被前面的初始化覆盖
 */
void app_menu_init(void);

//菜单主循环(在主循环里反复调用)
//内部会推进软件毫秒计数, 任务页做动画时用得上
void app_menu_run(void);

//任务页请求返回上级菜单(比如按键测试页长按中键)
//任务页自己不能切页, 只能登记请求, 由主循环统一处理
//返回到哪一页由进页面时自动记录的上级页面决定
void menu_back_to_task(void);

//登记当前页的上级页面, 由子页面自己的 show() 里调用
//因为同一个页面可能从不同入口进入, 上级页面不固定
void menu_set_back_page(page_enum page);

#endif
