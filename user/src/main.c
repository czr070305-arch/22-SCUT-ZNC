/*********************************************************************************************************************
* CH32V307VCT6 Opensourec Library 即（CH32V307VCT6 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是CH32V307VCT6 开源库的一部分
*
* CH32V307VCT6 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MounRiver Studio V1.8.1
* 适用平台          CH32V307VCT6
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期                                      作者                             备注
* 2022-09-15        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "hw_led.h"
#include "app_menu.h"
#include "vision/vision.h"
#include "hardware/hw_imu.h"
#include "hardware/hw_flash.h"


int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);      // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                       // 初始化调试串口
                 
    /* flash_param_load 必须先跑: 它从 Flash 读出 PID 参数和 IMU 零偏。
     * hw_imu_init 要用 g_param_imu_valid 判断"有没有存过零偏"，
     * 放反了就会读到全 0 的 g_param，白白多校一次。
     * 原来这一步在 app_menu_init 里，提前到这里。 */
    flash_param_load();

    /* IMU 必须在 app_menu_init 之前: 菜单里各页面都可能要读角度。
     * 内部会禁 JTAG、建软 I2C、启 TIM6 1ms 时基。
     * 零偏优先用 Flash 里的，没有才现场校准。
     * 失败也不停机 —— 进 IMU 页会显示 OFFLINE, 方便查接线。 */
    hw_imu_init();

    app_menu_init();                    // 初始化外设，并画出主菜单

    while(1)
    {
        app_menu_run();             // 菜单主循环(内部会自动刷新当前页面)

        /* 视觉任务: 内部判断有没有新帧, 没帧立刻返回, 不会拖慢控制。
         * 放在 app_menu_run() 之后: 菜单负责按键/UI, 视觉只补图像处理。 */
        vision_task();

        /* IMU 采样 + Mahony 解算。内部按 tick 差值判断到点没有,
         * 没到 2ms 就立刻返回。不加这一行永远拿不到角度。 */
        hw_imu_update();
    }
}
