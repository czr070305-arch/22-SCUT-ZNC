#include "hw_flash.h"
#include "algorithm/tune.h"

//参数表
param_struct g_param;

//Flash 里有没有有效参数, 见 hw_flash.h 的说明
uint8 g_param_valid = 0;

//上次成功写进 Flash 的值, 用来判断这次要不要真的写
static float saved_l_kp = 0.0f;
static float saved_l_ki = 0.0f;
static float saved_l_kd = 0.0f;
static float saved_r_kp = 0.0f;
static float saved_r_ki = 0.0f;
static float saved_r_kd = 0.0f;

/* 上次成功写盘的转向参数，用来判断"这次要不要真的写 Flash" */
static int32 saved_tune[TUNE_COUNT] = {0};

//槽位对照表
/* 槽位表（1 槽 = 4 字节，一页 1024B = 256 槽，够用）
 *
 * 转向参数从槽 8 开始，TUNE_COUNT 个 int32。
 * MAGIC 已改为 "TUNE"：结构体新增了 t_cur/t_def，旧 Flash 内容
 * 在那几个槽是随机的，首次上电会判无效并回默认，这是故意的。 */
#define SLOT_MAGIC      (0)     /* uint32  有效性标志 */
#define SLOT_L_KP       (1)     /* float   左轮 Kp */
#define SLOT_L_KI       (2)     /* float   左轮 Ki */
#define SLOT_L_KD       (3)     /* float   左轮 Kd */
#define SLOT_R_KP       (4)     /* float   右轮 Kp */
#define SLOT_R_KI       (5)     /* float   右轮 Ki */
#define SLOT_R_KD       (6)     /* float   右轮 Kd */
#define SLOT_TUNE_CUR   (8)     /* int32[8] 转向当前值 */

//把参数表设置成默认值
static void param_set_default(void)
{
    /* 2026-11-16 在 VOFA 上实测的最优值, 左右轮分开存。
     *
     * 左轮: kp=18.2 ki=1.0 kd=8.6
     *   这组是在左编码器噪声较大的条件下调出来的, 曲线毛刺最小。
     *   左轮毛刺偏大的根因是编码器硬件(磁铁间距/排线接触), 待排查。
     * 右轮: kp=15.0 ki=1.0 kd=15.0
     *   右轮换向器状态比左侧好, 同样的 D 能压得更平。
     *
     * 写在这里作为保底: Flash 没存上(空片/写入失败/MAGIC 校验不过)时用它,
     * 上电就能直接跑, 不会退回"电机不动"或过时的旧值。
     *
     * 改这里才有效: 底盘初始化读的是 g_param, 而 g_param 的来源就是这里。
     * chassis.h 里的宏不参与任何计算, 改那里不会有任何作用。
     *
     * 注意: 左右分开是补偿硬件差异的权宜手段, 不是消除差异。
     * 换了电机/编码器板之后两组都要重调; 硬件修好后尽量合并回一组。 */
    g_param.l_kp = 18.2f;
    g_param.l_ki = 1.0f;
    g_param.l_kd = 8.6f;

    g_param.r_kp = 15.0f;
    g_param.r_ki = 1.0f;
    g_param.r_kd = 15.0f;

    /* 转向参数出厂默认由 tune 模块维护（和屏上参数表同源），
     * 这里调一次，把 g_tune[] 都填成出厂值。 */
    tune_load_factory();
}

//从Flash读参数(上电调一次)
void flash_param_load(void)
{
    uint32 magic;

    flash_read_page_to_buffer(FLASH_PARAM_SECTOR, FLASH_PARAM_PAGE);
    magic = flash_union_buffer[SLOT_MAGIC].uint32_type;

    //先验MAGIC再搬数据
    //不验的话, 空片读出来是 0xE339E339, 会被当成参数用
    if(FLASH_PARAM_MAGIC != magic)
    {
        g_param_valid = 0;
        param_set_default();
        saved_l_kp = g_param.l_kp;
        saved_l_ki = g_param.l_ki;
        saved_l_kd = g_param.l_kd;
        saved_r_kp = g_param.r_kp;
        saved_r_ki = g_param.r_ki;
        saved_r_kd = g_param.r_kd;

        {
            uint8 i;
            for (i = 0; i < TUNE_COUNT; i ++)
            {
                saved_tune[i] = g_tune[i];
            }
        }

        return;
    }

    g_param_valid = 1;

    //读的类型必须和写的时候一致, 写uint16就读uint16
    g_param.l_kp = flash_union_buffer[SLOT_L_KP].float_type;
    g_param.l_ki = flash_union_buffer[SLOT_L_KI].float_type;
    g_param.l_kd = flash_union_buffer[SLOT_L_KD].float_type;
    g_param.r_kp = flash_union_buffer[SLOT_R_KP].float_type;
    g_param.r_ki = flash_union_buffer[SLOT_R_KI].float_type;
    g_param.r_kd = flash_union_buffer[SLOT_R_KD].float_type;

    /* 转向参数：直接落到全局 g_tune[]，
     * 不经过 g_param.t_cur，避免两处副本不同步。 */
    {
        uint8 i;
        for (i = 0; i < TUNE_COUNT; i ++)
        {
            g_tune[i] = flash_union_buffer[SLOT_TUNE_CUR + i].int32_type;
        }
    }

    saved_l_kp = g_param.l_kp;
    saved_l_ki = g_param.l_ki;
    saved_l_kd = g_param.l_kd;
    saved_r_kp = g_param.r_kp;
    saved_r_ki = g_param.r_ki;
    saved_r_kd = g_param.r_kd;

    {
        uint8 i;
        for (i = 0; i < TUNE_COUNT; i ++)
        {
            saved_tune[i] = g_tune[i];
        }
    }
}

//将修改后的参数表写进Flash(成功返回0)
uint8 flash_param_save(void)
{
    //和上次保存的值完全一样就直接返回, 不重复写
    //两个原因:
    //   1. Flash 写入寿命有限(约10万次), 上位机按钮一按可能连发十几条
    //   2. 每次写要先擦一整个扇区再逐字编程, 期间关中断, 整个系统停顿几百毫秒
    //这种情况下重复写既没意义又有害
    /* 转向参数也参与"是否需要重写"的判断：
     * 只比速度环 PID 的话，改完转向参数调 tune_save 会直接返回，
     * 新参数永远落不了盘。 */
    {
        uint8 diff = 0;
        uint8 i;

        for (i = 0; i < TUNE_COUNT; i ++)
        {
            if (saved_tune[i] != g_tune[i])
            {
                diff = 1;
                break;
            }
        }

        if ((0 == diff) && g_param_valid &&
            (saved_l_kp == g_param.l_kp) &&
            (saved_l_ki == g_param.l_ki) &&
            (saved_l_kd == g_param.l_kd) &&
            (saved_r_kp == g_param.r_kp) &&
            (saved_r_ki == g_param.r_ki) &&
            (saved_r_kd == g_param.r_kd))
        {
            return 0;
        }
    }

    flash_buffer_clear();

    //MAGIC也要一起写, 不然下次上电load会当成无效数据
    flash_union_buffer[SLOT_MAGIC].uint32_type = FLASH_PARAM_MAGIC;
    flash_union_buffer[SLOT_L_KP].float_type   = g_param.l_kp;
    flash_union_buffer[SLOT_L_KI].float_type   = g_param.l_ki;
    flash_union_buffer[SLOT_L_KD].float_type   = g_param.l_kd;
    flash_union_buffer[SLOT_R_KP].float_type   = g_param.r_kp;
    flash_union_buffer[SLOT_R_KI].float_type   = g_param.r_ki;
    flash_union_buffer[SLOT_R_KD].float_type   = g_param.r_kd;

    /* 转向参数和速度环 PID 同页写出：
     * flash_write_page_from_buffer 会先擦整个扇区，
     * 所以必须一次写全，不能分两次调用。 */
    {
        uint8 i;
        for (i = 0; i < TUNE_COUNT; i ++)
        {
            flash_union_buffer[SLOT_TUNE_CUR + i].int32_type = g_tune[i];
        }
    }

    //不用自己调 flash_erase_sector, 写函数的内部会先检查并擦除
    {
        uint8 err = flash_write_page_from_buffer(FLASH_PARAM_SECTOR, FLASH_PARAM_PAGE);

        if(0 == err)
        {
            g_param_valid = 1;
            saved_l_kp = g_param.l_kp;
            saved_l_ki = g_param.l_ki;
            saved_l_kd = g_param.l_kd;

            {
                uint8 i;
                for (i = 0; i < TUNE_COUNT; i ++)
                {
                    saved_tune[i] = g_tune[i];
                }
            }
        }

        return err;
    }
}

//恢复默认值
void flash_param_reset(void)
{
    param_set_default();

    //恢复到默认值, 此时 RAM 和 Flash 已经不一致, 标志要清掉
    g_param_valid = 0;
    saved_l_kp = g_param.l_kp;
    saved_l_ki = g_param.l_ki;
    saved_l_kd = g_param.l_kd;
    saved_r_kp = g_param.r_kp;
    saved_r_ki = g_param.r_ki;
    saved_r_kd = g_param.r_kd;

    {
        uint8 i;
        for (i = 0; i < TUNE_COUNT; i ++)
        {
            saved_tune[i] = g_tune[i];
        }
    }
}
