#ifndef _HW_IMU_H_
#define _HW_IMU_H_

#include "zf_common_headfile.h"
/* zf_common_headfile.h 里这一行被注释掉了，软件 I2C 要自己加 */
#include "zf_driver_soft_iic.h"
/* imu_t 定义在这里；mahony.h 不依赖 hw_imu.h，不会循环包含 */
#include "algorithm/mahony.h"

/* ============================================================================
 * hw_imu.h --- ICM-42688-P 六轴 IMU 驱动（软件 I2C）
 *
 * 硬件连接：
 *   SCL -> PB3 (PINCM7)   原 JTDO，初始化时必须先禁 JTAG
 *   SDA -> PB5 (PINCM9)
 *   CS  -> PB0 (PINCM4)   推挽输出高
 *
 * 为什么用软件 I2C 而不是 SPI：
 *   主板陀螺仪接口只引出 PB3/PB5 两根线，没有 MISO。
 *   ICM-42688-P 的三线 SPI 要在 bank1 的 INTF_CONFIG4 里切模式，
 *   切错了读回来是 0xFF，无法区分是模式没切对还是线没接好，调试成本高。
 *   官方手册 DS-000347 Table 10 明确：I2C 模式下 CS 接 VDDIO 即可。
 *
 * CS 的作用（手册第 53 页 §9.6）：
 *   芯片上电默认 I2C 模式，CS 一旦出现下降沿就永久切到 SPI 模式，
 *   期间 I2C 完全失效，只有重新上电才能恢复。
 *   所以这根线只能保持高电平，绝不能拉低。
 * ==========================================================================*/

//==================== 引脚 ====================
#define IMU_SCL_PIN             (B3)
#define IMU_SDA_PIN             (B5)
#define IMU_CS_PIN              (B0)

/* CS 是否由软件驱动
 *   1 = CS 接在 PB0，初始化配成推挽输出高（推荐，抗走线干扰）
 *   0 = CS 不接，靠模块自带的 4.7k 上拉到 3V3 */
#define IMU_CS_SOFT_DRIVE       (1)

//==================== I2C 时序 ====================
/* AD0 接 3V3 时 7 位地址是 0x69；AD0 悬空/接地的批次是 0x68。
 * 初始化会自动两个都试，这个宏只是优先尝试的顺序。 */
#define IMU_IIC_ADDR            (0x69)

/* 软 I2C 半周期空循环次数。120MHz、-O0 下每次约 0.8us，
 * 一个 SCL 周期约 2~4 次 -> 200~400kHz。走线长的话调大。 */
#define IMU_IIC_DELAY           (40)

//==================== 量程 ====================
//加速度 ±4g（8192 LSB/g），陀螺仪 ±2000dps（16.4 LSB/(°/s)），ODR 1kHz
//数据寄存器为大端序，组合方式 (高字节 << 8) | 低字节
#define IMU_ACC_SENS_G      8192.0f     // ±4g:      32768 / 4    = 8192 LSB/g
#define IMU_GYRO_SENS_DPS   16.4f       // ±2000dps: 32768 / 2000 = 16.4 LSB/(°/s)

/* 直接喂给 mahony_update() 的灵敏度（单位与 mahony.h 约定一致） */
#define IMU_ACC_SENS        (1.0f / IMU_ACC_SENS_G)                     /* g/LSB     */
#define IMU_GYRO_SENS       (0.0174532925f / IMU_GYRO_SENS_DPS)         /* rad/s/LSB */

//==================== 安装方向补偿 ====================
/*
 * 把芯片坐标系变换到车体坐标系。芯片怎么装在车上，这里就要怎么补。
 *
 *   0 = 正装，Z 轴朝上（芯片丝印朝上，元件面朝向车顶）
 *   1 = 绕 X 翻转 180°（翻面贴装，X 保持原方向）      <- 当前
 *   2 = 绕 Y 翻转 180°（翻面贴装，Y 保持原方向）
 *   3 = 绕 Z 翻转 180°（平放但转了半圈）
 *   4 = 绕 X 转 +90°（侧立）
 *   5 = 绕 X 转 -90°（侧立，另一面）
 *
 * 判断方法：车平放静止，进 IMU TEST 页看 roll / pitch。
 *   正装      roll ≈ 0    pitch ≈ 0
 *   绕X翻转   roll ≈ ±180 pitch ≈ 0
 *   绕Y翻转   roll ≈ 0    pitch ≈ ±180
 *   侧立      roll ≈ ±90
 * 平放静止时看遥测的 az：+8000 左右是对的，-8000 就是 Z 朝下。
 */
#define IMU_MOUNT_FLIP          (1)

//==================== 零偏校准 ====================
/* 500 次平均，噪声降到 1/sqrt(500) ≈ 4.5%。
 * 校准期间车必须完全静止，否则会把真实转动算进零偏。 */
#define IMU_CALIB_SAMPLES       (500)

//==================== 数据结构 ====================
typedef struct
{
    int16 acc[3];       /* 加速度 X/Y/Z，单位 LSB，未减零偏（加速度不需要） */
    int16 gyro[3];      /* 角速度 X/Y/Z，单位 LSB，已减零偏 */
    int16 temp_raw;     /* 温度原始值 */
    float temp_c;       /* 摄氏度 */
} imu_raw_struct;

//==================== 接口 ====================

/* 初始化：禁 JTAG -> 配 CS -> 建软 I2C -> 探地址 -> 软复位 -> 配量程
 * 返回 0 成功，1 读不到 WHO_AM_I（接线错、CS 被拉低、芯片没供电） */
uint8 hw_imu_init(void);

/* 一次突发读 14 字节（温度 2 + 加速度 6 + 角速度 6），陀螺已减零偏
 * 返回 0 成功，1 通信异常（读回全 0 或全 -1，说明 I2C 没应答） */
uint8 hw_imu_read_raw(imu_raw_struct *out);

/* 1 = 初始化成功 */
uint8 hw_imu_is_online(void);

/* 零偏校准：调用 start 后在车静止时反复 poll，每次 poll 采一个样
 * poll 返回 1 表示校准完成 */
void    hw_imu_calib_start(void);
uint8   hw_imu_calib_poll(void);

/* 校准进度 0~100，供测试页画进度条 */
uint8   hw_imu_calib_percent(void);

/* 读某个轴的零偏（LSB），调试用。axis = 0/1/2 */
int16   hw_imu_calib_bias(uint8 axis);

//==================== 采样 + 解算调度 ====================
/* 解算节拍(ms)。不放在 1ms 中断里跑：软件 I2C 读 14 字节约 315us，
 * 加上 Mahony 的 atan2f/asinf 共约 500us，1ms 中断里会挤占 DVP 和速度环。
 * 2ms 对 500dps 的车最多丢 1° 转角，远小于环岛判据的 25° 余量。 */
#define IMU_PERIOD_MS           (2)

/* 仅由 TIM6(1ms) 中断调用，内部只有一句自增 */
void  imu_tick_inc(void);

/* 主循环每圈调用：攒够 IMU_PERIOD_MS 才真正读 I2C + 跑 Mahony。
 * dt 用 tick 差值算，主循环抖动不影响精度，忙的时候少算几拍而已。 */
void  hw_imu_update(void);

/* 取最近一次解算结果。返回 0 表示已有有效数据，1 表示还没有 */
uint8 hw_imu_get_angle(imu_t *out);

/* 取最近一次原始数据（温度/加速度/已减零偏的角速度）。
 * 不额外读 I2C，直接用 hw_imu_update() 刚读到的那份。 */
uint8 hw_imu_get_raw(imu_raw_struct *out);

#endif /* _HW_IMU_H_ */
