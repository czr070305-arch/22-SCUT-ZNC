#ifndef _HW_FLASH_H_
#define _HW_FLASH_H_

#include "zf_common_headfile.h"

//Flash存储位置
#define FLASH_PARAM_SECTOR      (63)
#define FLASH_PARAM_PAGE        (3)

//数据有效性标志
#define FLASH_PARAM_MAGIC       (0x54554E48)    //ASCII "TUNG", 第五版: 同步 tune.c 默认值(KP_CR=250/LOOK=40/Speed=1000)
// 2026-11-24: 换无畸变镜头，旧 Flash 里的转向参数(KP_LN=110/KP_CR=200)
// 是按鱼眼镜头调的，不再适用。改 MAGIC 让旧数据自动失效，
// 上电后走 param_set_default() -> tune_load_factory()，用 tune.c 的默认值。
//MAGIC 每改一次存储布局就换一个值, 让老片子里的旧数据自动失效

//参数结构体
//直接存 float 而不是放大整数: Flash 一个槽位是 32bit, 正好放一个 float,
//精度不受限, 以后想在界面上提高显示小数位只改界面就行, 存储格式不用动
/* 左右轮各自的 PID 参数
 *
 * 为什么要分开: 两个电机的换向器磨损、编码器安装精度、磁铁间隙都不完全
 * 一样, 同一组参数在一侧平滑、在另一侧毛刺明显。2026-11-16 实测:
 *   左轮 kp=18.2 ki=1.0 kd=8.6   曲线毛刺大, 需要更大的 D 抑制
 *   右轮 kp=15.0 ki=1.0 kd=15.0  同一组参数下本来就平顺
 * 所以分开存。
 *
 * 注意: 这是补偿硬件差异的权宜手段, 不是消除差异。
 * 以后换了电机/编码器板, 两组要重新调; 硬件修好后尽量合并回一组。
 */
typedef struct{
    /* ---- 速度环 PID（8 项，VOFA 在线调 / 屏上只读） ---- */
    float l_kp;
    float l_ki;
    float l_kd;
    float r_kp;
    float r_ki;
    float r_kd;

    /* ---- 转向环在线调参 ----
     *
     * 实际存储不靠这个结构体：zf 的 Flash 驱动是按
     * flash_union_buffer[槽号] 逐槽读写，和结构体布局无关。
     * 转向参数存在 SLOT_TUNE_CUR，由 hw_flash.c 直接读写
     * 全局 g_tune[]（见 tune.h）。
     *
     * 为什么和速度环 PID 同页同一次写：
     *   flash_write_page_from_buffer 内部会先擦整个 4KB 扇区再写。
     *   转向参数如果单独占一页，存一次就会把速度环 PID 一起擦掉。
     *   所以一次写全。
     *
     * 为什么用 int32 定点而不是 float：
     *   显示格式化省代码，Flash 读回垃圾位概率也低。
     */
}param_struct;

extern param_struct g_param;

//Flash 里有没有有效参数
//1 = 读到过有效的 MAGIC, 0 = 空片子或 MAGIC 对不上
//光看 g_param 全是 0 分不清"没存过"和"存的就是 0", 得靠这个标志
extern uint8 g_param_valid;

//注意: flash_param_load 必须在 chassis_init 之前调用
//底盘初始化时要拿这里的参数去填 PID

void  flash_param_load(void);
uint8 flash_param_save(void);
void  flash_param_reset(void);

#endif
