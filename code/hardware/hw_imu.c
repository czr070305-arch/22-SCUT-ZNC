#include "hw_imu.h"

/* ============================================================================
 * hw_imu.c --- ICM-42688-P 驱动实现（软件 I2C）
 *
 * 全部寄存器地址和配置值来源：TDK DS-000347 Rev 1.8（本地手册）
 *   p.65  DEVICE_CONFIG  (0x11)  bit0 SOFT_RESET_CONFIG
 *   p.66  TEMP_DATA1     (0x1D)  连续 14 字节的起点
 *   p.77  PWR_MGMT0      (0x4E)  bit3:2 GYRO_MODE / bit1:0 ACCEL_MODE
 *   p.78  GYRO_CONFIG0   (0x4F)  bit7:5 FS_SEL / bit3:0 ODR
 *   p.79  ACCEL_CONFIG0  (0x50)  bit7:5 FS_SEL / bit3:0 ODR
 *   p.89  WHO_AM_I       (0x75)  固定 0x47
 *   p.53  §9.6 CS 下降沿会把芯片永久切到 SPI 模式
 * ==========================================================================*/

//==================== 寄存器地址 ====================
#define IMU_REG_DEVICE_CONFIG       (0x11)
#define IMU_REG_TEMP_DATA1          (0x1D)
#define IMU_REG_PWR_MGMT0           (0x4E)
#define IMU_REG_GYRO_CONFIG0        (0x4F)
#define IMU_REG_ACCEL_CONFIG0       (0x50)
#define IMU_REG_WHO_AM_I            (0x75)

//==================== 配置值 ====================
/* GYRO_CONFIG0：FS_SEL=000(±2000dps) | ODR=0110(1kHz) = 0x00 | 0x06 */
#define IMU_CFG_GYRO                (0x06)
/* ACCEL_CONFIG0：FS_SEL=010(±4g) | ODR=0110(1kHz) = 0x40 | 0x06 */
#define IMU_CFG_ACCEL               (0x46)
/* PWR_MGMT0：GYRO_MODE=11(LN) | ACCEL_MODE=11(LN) = 0x0C | 0x03 */
#define IMU_CFG_PWR_LN              (0x0F)

#define IMU_WHO_AM_I_VAL            (0x47)
#define IMU_BURST_LEN               (14)    /* 温度2 + 加速度6 + 角速度6 */

/* 温度换算：DS-000347 Table 4，灵敏度 132.48 LSB/℃，25℃ 时零点 0 */
#define IMU_TEMP_LSB_PER_C          (132.48f)

/* 每次 poll 连续采多少个样。
 * 500 / 10 = 50 次调用，主循环 20ms 一圈约 1 秒完成；
 * 单次阻塞约 3.5ms，不会卡住 UI。 */
#define IMU_CALIB_BATCH             (10)

//==================== 内部状态 ====================
static soft_iic_info_struct s_iic;
static uint8  s_online          = 0;                    /* 初始化成功 */
static int16  s_bias[3]         = {0, 0, 0};            /* 陀螺三轴零偏 LSB */
static uint16 s_calib_cnt       = 0;                    /* 已采样次数 */
static int32  s_calib_sum[3]    = {0, 0, 0};            /* 三轴累加 */

/* --- 采样调度状态 --- */
static volatile uint32 s_tick_ms   = 0;                 /* 1ms 中断自增，只增不清 */
static uint32          s_last_tick = 0;                 /* 上次解算时的 tick 快照 */
static imu_t      s_mahony;                           /* 最近一次解算结果 */
static uint8         s_mahony_valid = 0;                 /* 0 = 还没算出过数据 */
static imu_raw_struct s_last_raw;                        /* 最近一次读到的原始值 */

//==================== 底层寄存器读写 ====================

/* 写一个字节：Start -> ADDR+W -> REG -> DATA -> Stop */
static void imu_write_reg(uint8 reg, uint8 val)
{
    soft_iic_write_8bit_register(&s_iic, reg, val);
}

/* 读单个寄存器 */
static uint8 imu_read_reg(uint8 reg)
{
    return soft_iic_read_8bit_register(&s_iic, reg);
}

/* 突发读：Start -> ADDR+W -> REG -> RepeatStart -> ADDR+R -> N 字节 -> Stop
 * 芯片内部地址自增，对应手册第 51 页 Figure 14 的 Burst Read Sequence */
static void imu_read_regs(uint8 reg, uint8 *buf, uint32 len)
{
    soft_iic_read_8bit_registers(&s_iic, reg, buf, len);
}

//==================== 初始化 ====================

/* 探芯片：依次用 0x69 / 0x68 读 WHO_AM_I，读对 0x47 就锁定该地址 */
static uint8 imu_probe(void)
{
    static const uint8 addr_try[2] = { IMU_IIC_ADDR, (IMU_IIC_ADDR == 0x69) ? 0x68 : 0x69 };
    uint8 i, who;

    for (i = 0; i < 2; i++)
    {
        s_iic.addr = addr_try[i];
        who = imu_read_reg(IMU_REG_WHO_AM_I);
        if (who == IMU_WHO_AM_I_VAL)
        {
            return 1;
        }
    }
    return 0;
}

uint8 hw_imu_init(void)
{
    uint8 who;

    s_online = 0;

    /* 1. 禁用 JTAG，把 PB3(JTDO)/PB4(NJTRST) 释放给普通 GPIO 用。
     *    只关 JTAG 保留 SWD，WCH-Link 照常烧录；
     *    绝不能用 GPIO_Remap_SWJ_Disable，那个连 SWD 一起关掉。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 2. CS 先拉高再建 I2C。
     *    芯片上电默认就是 I2C 模式，只要 CS 不出现下降沿就不会切到 SPI。
     *    必须先于任何 I2C 动作，否则 I2C 从第一拍就没人应答。 */
#if IMU_CS_SOFT_DRIVE
    gpio_init(IMU_CS_PIN, GPO, 1, GPO_PUSH_PULL);
#endif

    /* 3. 建立软件 I2C（SCL 推挽 / SDA 开漏由库内部配置） */
    soft_iic_init(&s_iic, IMU_IIC_ADDR, IMU_IIC_DELAY, IMU_SCL_PIN, IMU_SDA_PIN);

    /* 4. 等芯片上电稳定，再探地址 */
    system_delay_ms(10);
    if (!imu_probe())
    {
        return 1;
    }

    /* 5. 软复位（DS-000347 p.65，写完等 1ms 才能再访问寄存器） */
    imu_write_reg(IMU_REG_DEVICE_CONFIG, 0x01);
    system_delay_ms(1);

    /* 6. 复位后所有配置回到默认值，必须重配 */
    imu_write_reg(IMU_REG_GYRO_CONFIG0,  IMU_CFG_GYRO);
    imu_write_reg(IMU_REG_ACCEL_CONFIG0, IMU_CFG_ACCEL);

    /* 7. 最后再上电。手册要求从 OFF 切到 LN 后等 45ms 输出才稳定，
     *    写 PWR_MGMT0 之前不能有寄存器写，否则 200us 内会被丢弃 */
    imu_write_reg(IMU_REG_PWR_MGMT0, IMU_CFG_PWR_LN);
    system_delay_ms(50);

    /* 8. 回读 WHO_AM_I 确认配置过程中没有掉线 */
    who = imu_read_reg(IMU_REG_WHO_AM_I);
    if (who != IMU_WHO_AM_I_VAL)
    {
        return 1;
    }

    /* 9. 复位解算器。必须调 —— mstate 是 static 变量，默认初值是全 0，
     *    四元数 [0,0,0,0] 归一化会算出 1/sqrt(0)，结果变成 NaN，
     *    之后所有角度都是 NaN，屏幕上显示 0.00 但永远不动。 */
    mahony_init();

    /* 10. 自动零偏校准。上电这一刻车必然是静止的（还没发车），
     *     直接采 500 个样把零偏扣掉，省掉手动按 S1。
     *     这 300 多毫秒阻塞换来的是 yaw 静止不漂，值得。
     *     S1 保留，用于运行中温度变化后重新校准。 */
    hw_imu_calib_start();
    while (!hw_imu_calib_poll())
    {
    }

    /* 11. 启动 1ms 时基。放在最后：前面任何一步失败就直接返回，
     *    不会留下一个空转的定时器中断 */
    pit_ms_init(TIM6_PIT, 1);

    s_online = 1;
    return 0;
}

uint8 hw_imu_is_online(void)
{
    return s_online;
}

//==================== 数据读取 ====================

uint8 hw_imu_read_raw(imu_raw_struct *out)
{
    uint8 buf[IMU_BURST_LEN];
    uint8 i;

    if (out == 0)
    {
        return 1;
    }

    imu_read_regs(IMU_REG_TEMP_DATA1, buf, IMU_BURST_LEN);

    /* 全 0 / 全 0xFF 说明 SDA 没有应答（芯片没供电、CS 被拉低切成 SPI、
     * 或者 SDA 断了）。真实数据三轴同时为 0 或同时 -1 几乎不可能出现，
     * 静止时至少有一轴接近 1g。 */
    if (((buf[2] == 0x00) && (buf[3] == 0x00) && (buf[4] == 0x00) &&
         (buf[5] == 0x00) && (buf[6] == 0x00) && (buf[7] == 0x00)) ||
        ((buf[2] == 0xFF) && (buf[3] == 0xFF) && (buf[4] == 0xFF) &&
         (buf[5] == 0xFF) && (buf[6] == 0xFF) && (buf[7] == 0xFF)))
    {
        return 1;
    }

    out->temp_raw = (int16)(((uint16)buf[0] << 8) | buf[1]);
    out->temp_c   = ((float)out->temp_raw / IMU_TEMP_LSB_PER_C) + 25.0f;

    /* 大端序：高字节在前 */
    for (i = 0; i < 3; i++)
    {
        out->acc[i]  = (int16)(((uint16)buf[2 + i * 2] << 8) | buf[2 + i * 2 + 1]);
        out->gyro[i] = (int16)(((uint16)buf[8 + i * 2] << 8) | buf[8 + i * 2 + 1]);
    }

    /* 陀螺减零偏，加速度不需要（加速度靠重力做绝对参考，没有累积漂移） */
    for (i = 0; i < 3; i++)
    {
        out->gyro[i] -= s_bias[i];
    }

    return 0;
}

//==================== 零偏校准 ====================
/*
 * 车体静止时角速度理论上是 0，读到的非零值就是零偏。
 * 取 500 次平均，随机噪声降到 1/sqrt(500) ≈ 4.5%，对 Mahony 完全够用。
 *
 * 不做成一次阻塞到底：那样 UI 会卡 175ms。改成每次 poll 采 10 个，
 * 分 50 次调用完成，中间照样能刷进度条。
 */

void hw_imu_calib_start(void)
{
    s_calib_sum[0] = 0;
    s_calib_sum[1] = 0;
    s_calib_sum[2] = 0;
    s_calib_cnt    = 0;
}

uint8 hw_imu_calib_poll(void)
{
    uint8 buf[6];
    uint8 i;
    uint8 a;
    int16 raw;

    if (s_calib_cnt >= IMU_CALIB_SAMPLES)
    {
        return 1;                       /* 已经采够，直接返回完成 */
    }

    /* 只读陀螺 6 字节，跳过温度 2 字节和加速度 6 字节 */
    for (i = 0; i < IMU_CALIB_BATCH; i++)
    {
        imu_read_regs(IMU_REG_TEMP_DATA1 + 8, buf, 6);

        for (a = 0; a < 3; a++)
        {
            raw = (int16)(((uint16)buf[a * 2] << 8) | buf[a * 2 + 1]);
            s_calib_sum[a] += (int32)raw;
        }

        s_calib_cnt++;
        if (s_calib_cnt >= IMU_CALIB_SAMPLES)
        {
            break;
        }
    }

    if (s_calib_cnt < IMU_CALIB_SAMPLES)
    {
        return 0;
    }

    /* 采够了：算平均，正负对称做四舍五入 */
    for (i = 0; i < 3; i++)
    {
        if (s_calib_sum[i] >= 0)
        {
            s_bias[i] = (int16)((s_calib_sum[i] + IMU_CALIB_SAMPLES / 2) / IMU_CALIB_SAMPLES);
        }
        else
        {
            s_bias[i] = (int16)((s_calib_sum[i] - IMU_CALIB_SAMPLES / 2) / IMU_CALIB_SAMPLES);
        }
    }

    return 1;
}

uint8 hw_imu_calib_percent(void)
{
    if (s_calib_cnt >= IMU_CALIB_SAMPLES)
    {
        return 100;
    }
    return (uint8)((uint32)s_calib_cnt * 100 / IMU_CALIB_SAMPLES);
}

int16 hw_imu_calib_bias(uint8 axis)
{
    if (axis >= 3)
    {
        return 0;
    }
    return s_bias[axis];
}

//==================== 安装方向补偿 ====================

/*
 * 把芯片坐标系变换到车体坐标系。
 *
 * 芯片翻面装（Z 轴朝下）时，加速度计测到的重力在 -Z 方向，
 * Mahony 会认为车是倒着的，把 roll 解成 180°，yaw 也跟着乱转。
 * 所以在送进解算器之前先把坐标系掰正。
 *
 * 变换表：每个值是乘在该轴上的符号/交换关系。
 * 这里只实现符号翻转（最常见的翻面 / 平转装法）。
 */
static void imu_apply_mount(int16 acc[3], int16 gyro[3])
{
#if   (IMU_MOUNT_FLIP == 0)     /* 正装 */
    (void)acc; (void)gyro;

#elif (IMU_MOUNT_FLIP == 1)     /* 绕 X 翻转 180°: Y,Z 取反 */
    acc[1]  = (int16)(-acc[1]);   acc[2]  = (int16)(-acc[2]);
    gyro[1] = (int16)(-gyro[1]);  gyro[2] = (int16)(-gyro[2]);

#elif (IMU_MOUNT_FLIP == 2)     /* 绕 Y 翻转 180°: X,Z 取反 */
    acc[0]  = (int16)(-acc[0]);   acc[2]  = (int16)(-acc[2]);
    gyro[0] = (int16)(-gyro[0]);  gyro[2] = (int16)(-gyro[2]);

#elif (IMU_MOUNT_FLIP == 3)     /* 绕 Z 翻转 180°: X,Y 取反 */
    acc[0]  = (int16)(-acc[0]);   acc[1]  = (int16)(-acc[1]);
    gyro[0] = (int16)(-gyro[0]);  gyro[1] = (int16)(-gyro[1]);

#elif (IMU_MOUNT_FLIP == 4)     /* 绕 X 转 +90°: 芯片 Y<->车体 Z, Z<->-Y */
    {
        int16 t;
        t = acc[1];  acc[1] = (int16)(-acc[2]);  acc[2] = t;
        t = gyro[1]; gyro[1] = (int16)(-gyro[2]); gyro[2] = t;
    }
#elif (IMU_MOUNT_FLIP == 5)     /* 绕 X 转 -90°: 芯片 Y<->-车体 Z, Z<->车体 Y */
    {
        int16 t;
        t = acc[1];  acc[1] = acc[2];  acc[2] = (int16)(-t);
        t = gyro[1]; gyro[1] = gyro[2]; gyro[2] = (int16)(-t);
    }
#endif
}

//==================== 采样 + 解算调度 ====================
/*
 * 时间基准来自 TIM6 的 1ms 中断，这里只存一个自增计数，不需要清零。
 *
 * 为什么不用「标志位」而是计数：
 *   主循环一圈 15~30ms，中间中断跑了十几次。标志位只能记住至少发生过一次，
 *   dt 会永远按 1ms 算，角度就只有真实值的十几分之一。
 *   计数则能算出这段时间到底过了多少毫秒。
 *
 * dt 上限保护：
 *   上电初期 s_last_tick 还是 0，第一次 update 会算出几百毫秒的 dt；
 *   调度页切走时也可能停很久。超过 100ms 的间隔直接按 100ms 截断，
 *   否则四元数一步积分跨太大，数值会崩。
 */

void imu_tick_inc(void)
{
    s_tick_ms++;
}

void hw_imu_update(void)
{
    imu_raw_struct raw;
    float dt;
    uint32 elapsed;

    elapsed = s_tick_ms - s_last_tick;
    if (elapsed < IMU_PERIOD_MS)
    {
        return;                         /* 还没到点，直接返回，不做任何 I2C */
    }

    s_last_tick = s_tick_ms;

    if (hw_imu_read_raw(&raw) != 0)
    {
        return;                         /* 通信异常，保留上一次的角度 */
    }

    dt = (float)elapsed * 0.001f;
    if (dt > 0.1f)
    {
        dt = 0.1f;
    }

    imu_apply_mount(raw.acc, raw.gyro);
    mahony_update(raw.acc, raw.gyro, dt, &s_mahony);

    s_last_raw     = raw;
    s_mahony_valid = 1;
}

uint8 hw_imu_get_raw(imu_raw_struct *out)
{
    if ((out == 0) || (s_mahony_valid == 0))
    {
        return 1;
    }
    *out = s_last_raw;
    return 0;
}

uint8 hw_imu_get_angle(imu_t *out)
{
    if ((out == 0) || (s_mahony_valid == 0))
    {
        return 1;
    }
    *out = s_mahony;
    return 0;
}
