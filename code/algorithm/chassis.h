#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "zf_common_headfile.h"
#include "hw_motor.h"
#include "hw_encoder.h"

/*
 * 底盘速度环闭环
 *
 * 闭环链路:
 *   编码器测实际转速 -> 目标减实际得到误差 -> PID 算占空比 -> 驱动电机
 *   转起来后编码器重新测, 误差变小, 占空比自动收敛
 *
 * 为什么需要它:
 *   占空比不等于速度。电池电压从 8.4V 掉到 7.4V, 同样 50% 占空比车速会变慢;
 *   车重、地面摩擦、转弯阻力也会改变同一占空比对应的实际速度。
 *   速度环靠编码器反馈自动补偿这些变化, 上层只需要说"我要跑 60 RPM"。
 *
 * 节拍要求:
 *   PID 的积分项是"每个周期累加多少", 采样周期一抖, 等效 Ki 就跟着变。
 *   所以必须跑在固定周期上, 由 TIM7 中断驱动, 不受主循环影响。
 *
 * 速度单位统一用 RPM, 目标值和实际值同量纲, 屏幕上可以直接对比。
 */

//==================== 节拍 ====================
//闭环定时器
#define CHASSIS_PIT_TIM         (TIM7_PIT)

//采样周期(ms), 必须和 ENC_SAMPLE_MS 完全一致
#define CHASSIS_PERIOD_MS       (ENC_SAMPLE_MS)

//==================== 限幅 ====================
/*
 * 目标速度上限(RPM)
 *
 * 编码器装在电机轴(减速前), 所以这里是电机转速。
 * 本车用 370 有刷电机, 铭牌 12V 8000RPM 是空载值,
 * 电池满电约 12.6V, 带载会掉到 5000~6000。
 * 上限取 9000: 比空载略高一点, 保证滑块拉满能覆盖真实能力,
 * 又不至于大得离谱让积分在够不着的目标上一直累加。
 *
 * 注意: 这个值只钳位目标值, 不限制输出。
 *      实际占空比另有 MOTOR_DUTY_MAX(0~10000) 兜底。
 */
/* 闭环目标转速上限(RPM)
 *
 * 9000 是 370 电机的空载极限, 作为默认值太宽松:
 * 视觉给错或 PID 调错时, 速度环会一路加速到 9000 才停。
 *
 * 1000 是调车初期的安全值, 落地大约对应 0.5m/s。
 * 赛道熟了再往上加, 但没必要超过 2000。 */
#define CHASSIS_TARGET_MAX      (5000)

//==================== PID 参数 ====================
/*
 * 增量式速度环
 *
 *   du(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
 *
 * 误差 e 的单位是 RPM, 输出 u 的单位是占空比(0~10000)
 * 所以 Ki 的含义是"每差 1 RPM, 每个 20ms 周期加多少占空比"
 *
 * ---------------------------------------------------------------------------
 * *** PID 参数不在这里定义 ***
 * ---------------------------------------------------------------------------
 *
 * 参数的唯一来源是 code/hardware/hw_flash.c 的 param_set_default()
 *
 *   上电流程:
 *     flash_param_load()             <- app_menu_init 里第一个调用
 *       +-- Flash 有有效参数  -> 用 Flash 里的
 *       +-- Flash 空/校验失败 -> 用 param_set_default() 的默认值
 *     chassis_init()                 <- 之后才跑, 拿 g_param 去填 PID
 *
 *   运行期改参数:
 *     VOFA 发 KPL=/KPR= 等  ->  chassis_set_param()  ->  同步到 g_param
 *     再发 SAVE             ->  flash_param_save()   ->  写进 Flash, 掉电不丢
 *
 *   想改"出厂保底值"(Flash 空时用哪组): 改 param_set_default()
 *
 * ---------------------------------------------------------------------------
 * 为什么原来这里的三个宏被删掉了
 * ---------------------------------------------------------------------------
 * 原来这里写着 CHASSIS_KP/KI/KD, 但全工程没有任何一处引用它们,
 * 改了完全没作用; 而且值和真正生效的默认值对不上, 极易误导。
 * 保留一个会骗人的"参数表"比没有更糟, 所以删掉, 只留这条说明。
 * ---------------------------------------------------------------------------
 */


//PID 输出限幅, 就是占空比上限
#define CHASSIS_OUT_MAX         ((float)MOTOR_DUTY_MAX)

//死区(RPM): 误差绝对值小于这个值时算 0, 防止静止时抖动
#define CHASSIS_DEAD_ZONE       (2.0f)

//==================== 反馈滤波 ====================
/*
 * 一阶低通, 滤波后的速度 = 旧值 x (1-a) + 新值 x a
 *
 * 为什么要滤:
 *   370 电机没有减速箱, 换向器纹波很明显。实测 1500RPM 时
 *   10ms 采样窗口内的读数在 1266~1676 之间跳, 波动 ±14%。
 *   Kp 会把这个噪声原样放大成占空比抖动, 电机就一阵一阵地抽。
 *
 * 代价:
 *   测速滞后。系数 3/10 时, 等效时间常数约 3 个采样周期(30ms),
 *   速度环本身就是慢环, 这点延迟完全可以接受。
 *
 * 怎么调:
 *   CHASSIS_FILTER_SHIFT 是右移位数, 越大越平滑但延迟越大
 *     0 = 关闭滤波(直接用原始值)
 *     1 = 系数 1/2, 时间常数约 10ms
 *     2 = 系数 1/4, 时间常数约 20ms
 *     3 = 系数 1/8, 时间常数约 40ms
 *     4 = 系数 1/16, 时间常数约 80ms   ← 默认
 *
 *   默认给的 3 能吃掉大部分纹波, 同时保住响应速度。
 *   如果调参时发现转速反应迟钝, 改小一档试试。
 */
#define CHASSIS_FILTER_SHIFT    (4)

//==================== 参数结构体 ====================
//把可调参数集中在一起, 方便以后接无线调参或存 Flash
/* 左右轮各自的 PID 参数
 *
 * 两个电机的换向器磨损 / 编码器安装精度 / 磁铁间隙不完全一样,
 * 同一组参数在一侧平滑、另一侧毛刺明显。2026-11-16 实测确认后分开存。
 * 这是补偿硬件差异的权宜手段, 硬件修好后应尽量合并回一组。 */
typedef struct
{
    float l_kp;
    float l_ki;
    float l_kd;
    float r_kp;
    float r_ki;
    float r_kd;
    float out_max;
    float dead_zone;
}t_chassis_param;

//运行状态
typedef enum
{
    CHASSIS_STATE_STOP = 0,     //停车刹车
    CHASSIS_STATE_RUN,          //闭环运行
    CHASSIS_STATE_OPEN,         //开环调试, 直接给占空比
}chassis_state_enum;

//==================== 接口 ====================

//初始化: 电机 + 编码器 + PID + 启动 5ms 节拍中断
void chassis_init(void);

//设定左右目标转速(RPM), 正=前进
void chassis_set_target(int32 left_rpm, int32 right_rpm);

//读目标转速
int32 chassis_get_target(uint8 side);

//读实际转速(RPM)
int32 chassis_get_speed(uint8 side);

//开环模式: 直接给占空比, 用来验证接线
//left/right 取 -10000 ~ +10000
void chassis_open_loop(int16 left_duty, int16 right_duty);

//停车: 清目标 + 清 PID + 刹车
void chassis_stop(void);

//当前状态
chassis_state_enum chassis_get_state(void);

//PID 参数在线调整(接无线调参时用)
void chassis_set_param(uint8 side, float kp, float ki, float kd);

//读回当前 PID 参数
//VOFA 只改其中一个时, 另外两个要保持原值, 所以得先读出来
void chassis_get_param(uint8 side, float *kp, float *ki, float *kd);


//取当前 PID 输出(占空比), 调试用
int16 chassis_get_output(uint8 side);

//控制周期计数, TIM7 每进一次中断加 1 (1 计数 = CHASSIS_PERIOD_MS)
//上层需要"和速度环对齐"的定时都用它, 别用主循环软件计时
uint32 chassis_get_tick(void);

//节拍处理, 由 TIM7 中断调用
void chassis_tick(void);

#endif
