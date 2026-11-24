#include "hw_encoder.h"

//上一次的原始计数, 用来做差分
static int16 enc_last[2] = {0, 0};

//最近一个周期的转速, 单位 0.01 RPM
static int32 enc_rpm_x100[2] = {0, 0};

//累计计数, 用于里程
static int32 enc_total[2] = {0, 0};

//最近一个周期的计数增量, 用于算线速度
//不存这里的话, 每次算线速度都要从 RPM 反推, 又绕又容易错
static int16 enc_delta[2] = {0, 0};

/*
 * RPM 换算
 *
 *   圈数      = 计数差值 / 每圈计数
 *   秒数      = 采样周期(ms) / 1000
 *   RPM       = 圈数 / 秒数 x 60
 *             = 计数差值 x 60000 / (每圈计数 x 采样ms)
 *
 * 要返回 0.01 RPM 的整数, 再乘 100:
 *   0.01RPM   = 计数差值 x 6000000 / (每圈计数 x 采样ms)
 *
 * 10ms 采样、1024 线时: 系数 = 6000000 / 10240 = 585.94
 * 即 1 个计数约 5.86 RPM, 这也是当前方案的分辨率极限
 */
#define ENC_RPM_X100_NUM    (6000000L)
#define ENC_RPM_X100_DEN    ((int32)ENC_COUNT_PER_REV * (int32)ENC_SAMPLE_MS)

//初始化两路编码器
void hw_encoder_init(void)
{
    encoder_quad_init(ENC_L_TIM, ENC_L_CH1, ENC_L_CH2);
    encoder_quad_init(ENC_R_TIM, ENC_R_CH1, ENC_R_CH2);

    hw_encoder_reset();
}

//周期采样
//调用周期必须等于 ENC_SAMPLE_MS, 否则 RPM 不准
void hw_encoder_update(void)
{
    uint8 i;
    int16 raw;
    int16 delta;

    for(i = 0; i < 2; i ++)
    {
        raw = encoder_get_count((i == ENC_LEFT) ? ENC_L_TIM : ENC_R_TIM);

        //16位计数器会从 65535 回绕到 0, 先转 uint16 再转 int16
        //这样回绕时差值依然正确, 不会跳出一个 65535
        delta = (int16)((uint16)((uint16)raw - (uint16)enc_last[i]));

        //方向标定: 前进时应该是正数
        if((i == ENC_LEFT) ? ENC_L_INVERT : ENC_R_INVERT)
        {
            delta = -delta;
        }

        //必须用 int64 做中间量:
        //delta 最大能到 2000 多(电机高速时), 乘 6000000 会超过 int32 上限
        //实测 3000 RPM 时 delta 约 512, 512 x 6000000 = 30.7 亿, 已经溢出
        enc_rpm_x100[i]  = (int32)(((int64)delta * ENC_RPM_X100_NUM) / ENC_RPM_X100_DEN);
        enc_delta[i]     = delta;
        enc_total[i]    += delta;
        enc_last[i]      = raw;
    }
}

//取转速, 单位 0.01 RPM
int32 hw_encoder_get_rpm_x100(uint8 side)
{
    return enc_rpm_x100[side & 0x01];
}

//取累计计数
int32 hw_encoder_get_total(uint8 side)
{
    return enc_total[side & 0x01];
}

//累计里程(mm)
//标定系数为 0 表示还没推车标定, 直接返回 0, 避免显示无意义的数
int32 hw_encoder_get_distance_mm(uint8 side)
{
    if(0 == ENC_MM_X1000_PER_CNT)
    {
        return 0;
    }

    //enc_total 是计数, 乘系数得 0.001mm, 再除 1000 得 mm
    //用 int64 防止长时间跑动后计数很大时溢出
    return (int32)(((int64)enc_total[side & 0x01] * ENC_MM_X1000_PER_CNT) / 1000);
}

//当前线速度(mm/s)
//
// enc_delta 是"每 ENC_SAMPLE_MS 的计数增量"
// 每秒计数 = delta x 1000 / ENC_SAMPLE_MS
// 每秒毫米 = 每秒计数 x ENC_MM_X1000_PER_CNT / 1000
//          = delta x ENC_MM_X1000_PER_CNT / ENC_SAMPLE_MS
int32 hw_encoder_get_speed_mm_s(uint8 side)
{
    if(0 == ENC_MM_X1000_PER_CNT)
    {
        return 0;
    }

    return (int32)(((int64)enc_delta[side & 0x01] * ENC_MM_X1000_PER_CNT) / ENC_SAMPLE_MS);
}

//清计数
void hw_encoder_reset(void)
{
    uint8 i;

    encoder_clear_count(ENC_L_TIM);
    encoder_clear_count(ENC_R_TIM);

    for(i = 0; i < 2; i ++)
    {
        enc_last[i]     = 0;
        enc_rpm_x100[i] = 0;
        enc_delta[i]    = 0;
        enc_total[i]    = 0;
    }
}
