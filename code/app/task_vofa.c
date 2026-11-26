#include "task_vofa.h"
#include "algorithm/island.h"
#include "algorithm/chassis.h"
#include "algorithm/ctrl.h"
#include "hw_flash.h"
#include "vision/vision.h"
#include "hardware/hw_imu.h"

#if (VOFA_ENABLE != 0)

//==================== 内部状态 ====================

//1 = 底盘控制权在 VOFA 手上
//只管"TGT/STEP 这些改目标的命令要不要执行", 不管波形:
//波形是常开的, 上电就开始发, 停车之后也继续发, 方便回看曲线
static uint8  vofa_online = 0;

//命令行缓冲, 逐字节攒, 遇到结束符再解析
static char   vofa_line[VOFA_CMD_MAX_LEN];
static uint8  vofa_line_len   = 0;
static uint8  vofa_line_bad   = 0;              //1=这条命令超长, 丢弃到下一个结束符

//上一次收到字节是在哪个控制周期
//用来判断命令帧有没有发完, 兼容上位机不带结束符的情况
//用控制周期而不是主循环计数: 主循环会被 TFT 刷新拖慢, 计数会漂
static uint32 vofa_last_rx_tick = 0;

//上一次发波形的周期号
static uint32 vofa_last_send = 0;

//自动阶跃
static uint8  vofa_step_on   = 0;
static int32  vofa_step_hi   = 0;               //高电平目标(RPM)
static int32  vofa_step_per  = 5000;            //半周期(ms)
static uint32 vofa_step_tick = 0;               //上次切换的周期号
static uint8  vofa_step_ph   = 0;               //0=低 1=高

//当前目标(RPM), 仅接管时写进 chassis
static int32  vofa_tgt_l     = 0;
static int32  vofa_tgt_r     = 0;

//发送缓冲
//6 个 int32 各最长 6 字符 + 5 个逗号 + 换行 + 结束符 = 44 字节, 留 8 字节余量
static char   vofa_tx[512];               /* 37 col x 12 + 37 = 481, 512 safe */

//==================== 字符串小工具 ====================

//把十进制整数写成字符串, 返回总长度
//自己写而不调 sprintf: 省掉 newlib 里那一大坨格式化代码
static uint8 vofa_i32_to_str(int32 v, char *buf)
{
    char   tmp[12];
    uint8  n = 0;
    uint8  i = 0;
    uint32 u;

    if(v < 0)
    {
        buf[i ++] = '-';
        u = (uint32)(-v);
    }
    else
    {
        u = (uint32)v;
    }

    if(0 == u)
    {
        buf[i ++] = '0';
        return i;
    }

    while((0 != u) && (n < 10))
    {
        tmp[n ++] = (char)('0' + (u % 10));
        u /= 10;
    }
    while(0 != n)
    {
        buf[i ++] = tmp[-- n];
    }

    return i;
}

//拼一帧 FireWater 数据: v0,v1,...,v5\n
static uint16 vofa_build_frame(void)
{
    int32  v[37];
    uint16 n = 0;
    uint8  i;

    v[0]  = chassis_get_target(ENC_LEFT);
    v[1]  = chassis_get_target(ENC_RIGHT);
    v[2]  = chassis_get_speed(ENC_LEFT);
    v[3]  = chassis_get_speed(ENC_RIGHT);
    v[4]  = (int32)chassis_get_output(ENC_LEFT);
    v[5]  = (int32)chassis_get_output(ENC_RIGHT);
    v[6]  = (int32)(ctrl_get_error() * 10.0f);   /* 转向误差 x10 */
    v[7]  = (int32)ctrl_get_turn();              /* 本拍转向分量 RPM */

    /* 2026-11-21 加：排查"提速后抽搐"
     * 屏幕 5Hz 太慢，抓不到抽搐瞬间；串口 50Hz 才能看到 */
    v[8]  = g_speed_pct;                 /* 元素建议速度 0~100（被砍了没） */
    v[9]  = g_state;                     /* 0普通/1直角/2十字/4环岛 */
    v[10] = g_corner_dir;                /* 直角弯判据 -1/+1/0（误报看这个） */
    v[11] = (int32)ctrl_get_cruise();    /* 巡航设定，配合 v[8] 看 base */
    /* 2026-11-24 加：环岛入口调参用
     *   v[12] g_isle_rows  入口1的判据（两窄段/梭形连续行数）
     *   v[13] g_fork_rows  入口2的判据（主线+支线连续行数）
     * 跑一圈就能看到这两个值在不同距离下的实际大小，
     * 再决定 CFG_ISLE_MIN_ROWS 和 CFG_ISLE_ENTRY_ROWS 该设多少。 */
    v[12] = g_isle_rows;
    v[13] = g_fork_rows;

    v[14] = g_exit_active;
    v[15] = g_exit_x;
    v[16] = g_exit_hits;
    v[17] = g_bottom_x;

    /* 2026-11-25: exit handoff debug columns */
    v[18] = g_prefer_side;               /* 0 = bite released */
    v[19] = g_island_side;               /* island side -1/+1 */

    /* 2026-11-26 加: IMU 姿态与原始值, 供 imu_monitor.py 分析漂移
     *   角度 x100 保留两位小数; 陀螺用 LSB 原值, 便于判断零偏量级 */
    {
        imu_t          ang;
        imu_raw_struct raw;
        uint8          ok  = hw_imu_get_angle(&ang);
        uint8          okr = hw_imu_get_raw(&raw);

        if (ok)  { ang.yaw = 0.0f; ang.pitch = 0.0f; ang.roll = 0.0f; }
        if (okr) { raw.temp_c = 0.0f; raw.gyro[0] = 0; raw.gyro[1] = 0; raw.gyro[2] = 0; }

        v[20] = (int32)(ang.yaw   * 100.0f);
        v[21] = (int32)(ang.pitch * 100.0f);
        v[22] = (int32)(ang.roll  * 100.0f);
        v[23] = (int32)raw.gyro[0];
        v[24] = (int32)raw.gyro[1];
        v[25] = (int32)raw.gyro[2];
        v[26] = (int32)hw_imu_calib_bias(0);
        v[27] = (int32)hw_imu_calib_bias(1);
        v[28] = (int32)hw_imu_calib_bias(2);
        v[29] = (int32)(raw.temp_c * 100.0f);
        v[30] = (int32)raw.acc[0];
        v[31] = (int32)raw.acc[1];
        v[32] = (int32)raw.acc[2];
    }

    /* 2026-11-26 加: 环岛状态机调试列
     *
     *   v[33] 状态        0=IDLE 1=RUNNING 2=AFTER(yaw锁朝向)
     *   v[34] Δyaw x100   相对入环起点的已转角度(度 x100)，-18000~+18000
     *   v[35] armed       1 = 已转过 60 度，允许判出岛了
     *   v[36] island_on   视觉这一帧报的进环标志
     *
     * 调参时怎么看:
     *   进环瞬间   v[33] 0->1，v[34] 从 0 开始，速度降到 75%
     *   转过 60°   v[35] 0->1
     *   转到 180°  v[34] 接近 ±18000
     *   回到起点   |v[34]| < 2500 且 v[35]=1 -> 出岛，v[33] 变 2
     *   AFTER 阶段  v[34] 应该收敛到 0 附近（yaw 锁朝向生效）
     *               250ms 后 v[33] 回 0，交回视觉
     */
    v[33] = (int32)island_get_state();
    v[34] = (int32)(island_get_delta_yaw() * 100.0f);
    v[35] = (int32)island_is_exit_armed();
    v[36] = (int32)g_island_on;

    for(i = 0; i < 37; i ++)
    {
        n += vofa_i32_to_str(v[i], &vofa_tx[n]);
        vofa_tx[n ++] = (i < 36) ? ',' : '\n';
    }

    return n;
}

//==================== 数字解析 ====================

//严格解析一个 int32, 返回 1 表示成功
//不用 atoi: atoi 解析失败静默返回 0, KP=abc 会被当成 KP=0 执行,
//速度环 Kp=0 等于失控, 这个失败的代价太大
static uint8 vofa_parse_i32(const char *s, int32 *out)
{
    int32 v      = 0;
    uint8 neg    = 0;
    uint8 digits = 0;

    if(0 == s)
    {
        return 0;
    }

    while((' ' == *s) || ('\t' == *s))
    {
        s ++;
    }

    if('-' == *s)
    {
        neg = 1;
        s ++;
    }
    else if('+' == *s)
    {
        s ++;
    }

    while((*s >= '0') && (*s <= '9'))
    {
        if(digits >= 6)                     //超过 6 位直接拒绝, 防止溢出
        {
            return 0;
        }
        v = v * 10 + (int32)(*s - '0');
        digits ++;
        s ++;
    }

    if(0 == digits)
    {
        return 0;
    }

    while((' ' == *s) || ('\t' == *s))
    {
        s ++;
    }

    if('\0' != *s)
    {
        return 0;                           //还有残留, 例如 "12abc"
    }

    *out = neg ? -v : v;
    return 1;
}

//解析 "500,300" 这种两段式
//第一段存 first; 有逗号才解析第二段, 否则 second 保持原值
static uint8 vofa_parse_pair(const char *s, int32 *first, int32 *second)
{
    char  buf[12];
    uint8 n = 0;

    while(('\0' != *s) && (',' != *s))
    {
        if(n >= (uint8)(sizeof(buf) - 1))
        {
            return 0;
        }
        buf[n ++] = *s;
        s ++;
    }
    buf[n] = '\0';

    if(!vofa_parse_i32(buf, first))
    {
        return 0;
    }

    if(',' == *s)
    {
        s ++;
        if(!vofa_parse_i32(s, second))
        {
            return 0;
        }
    }

    return 1;
}

//解析定点小数, 结果放大 1000 倍返回
//"1.5" -> 1500,  "0.05" -> 50,  "5" -> 5000,  "-0.2" -> -200
//
//为什么不直接 atof/strtod:
//   CH32V307 是 RV32IMAC, 没有硬件浮点, strtod 要走 double 软浮点,
//   又慢又占 flash。这里全用整数算, 只在最后换成 float, 快得多。
//为什么不沿用"放大100倍下发"的约定:
//   那样滑块显示 5 实际是 0.05, 两边对不上, 极易误会。
//   现在滑块显示什么就是什么, 精度到 0.001, 速度环足够。
static uint8 vofa_parse_fixed3(const char *s, int32 *out)
{
    int32 ipart  = 0;
    int32 frac   = 0;
    uint8 neg    = 0;
    uint8 digits = 0;
    uint8 fdig   = 0;

    if(0 == s)
    {
        return 0;
    }

    while((' ' == *s) || ('\t' == *s))
    {
        s ++;
    }

    if('-' == *s)
    {
        neg = 1;
        s ++;
    }
    else if('+' == *s)
    {
        s ++;
    }

    while((*s >= '0') && (*s <= '9'))
    {
        if(digits >= 4)                     //整数部分超过 4 位直接拒绝
        {
            return 0;
        }
        ipart = ipart * 10 + (int32)(*s - '0');
        digits ++;
        s ++;
    }

    if('.' == *s)
    {
        s ++;
        while((*s >= '0') && (*s <= '9'))
        {
            if(fdig < 3)                    //只要前 3 位小数, 多的丢掉
            {
                frac = frac * 10 + (int32)(*s - '0');
                fdig ++;
            }
            s ++;
        }
        while(fdig < 3)                     //不足 3 位补齐
        {
            frac *= 10;
            fdig ++;
        }
    }

    while((' ' == *s) || ('\t' == *s))
    {
        s ++;
    }

    if('\0' != *s)
    {
        return 0;                           //还有残留, 例如 "1.5abc"
    }

    if((0 == digits) && (0 == fdig))
    {
        return 0;
    }

    *out = ipart * 1000 + frac;
    if(neg)
    {
        *out = -*out;
    }
    return 1;
}

/*
 * 大小写不敏感的前缀比较
 *
 * 为什么需要: VOFA+ 面板上的按钮名字是用户自己填的, 常见写法是小写的
 * "start/stop/save/load". 而这里原来用 strncmp 跟大写常量比, 大小写
 * 敏感 -> 点按钮完全没反应, 而且未知命令静默丢弃, 现象是"点了没反应",
 * 极难排查. 统一走这个函数, 大小写怎么写都认.
 */
static uint8 vofa_cmd_is(const char *cmd, const char *key, uint8 len)
{
    uint8 i;

    for(i = 0; i < len; i ++)
    {
        char a = cmd[i];
        char b = key[i];

        if((a >= 'a') && (a <= 'z')) a = (char)(a - 'a' + 'A');
        if((b >= 'a') && (b <= 'z')) b = (char)(b - 'a' + 'A');

        if(a != b)
        {
            return 0;
        }
    }
    return 1;
}

//判断 cmd 是不是 "key=" 开头
static uint8 vofa_key_is(const char *cmd, const char *key)
{
    while('\0' != *key)
    {
        char a = *cmd;
        char b = *key;

        if((a >= 'a') && (a <= 'z')) a = (char)(a - 'a' + 'A');
        if((b >= 'a') && (b <= 'z')) b = (char)(b - 'a' + 'A');

        if(a != b)
        {
            return 0;
        }
        cmd ++;
        key ++;
    }
    return (uint8)('=' == *cmd);
}

//==================== 应答 ====================

//给上位机回一行文本, 方便确认命令收到了
//波形一直在发, 所以这行文本会插在数据帧中间。
//FireWater 按行分类: 数字行进波形, 非数字行归到"其他数据", 不会把图搞崩,
//只会在字节区多出一行。命令本来就不会连续发, 这个频率可以忽略
static void vofa_ack(const char *msg)
{
    uint16 n = 0;

    while(('\0' != msg[n]) && (n < (uint16)(sizeof(vofa_tx) - 2)))
    {
        vofa_tx[n] = msg[n];
        n ++;
    }
    vofa_tx[n ++] = '\n';

    uart_write_buffer(DEBUG_UART_INDEX, (const uint8 *)vofa_tx, n);
}

//==================== 接管 / 交还 ====================

//进入接管: 停车 + 目标归零
//顺序不能反, 先停车再开工, 否则会带着上一次的目标突然启动
//注意这里不控制波形: 波形是常开的, 上电就在发, 和接管状态无关
static void vofa_enter(void)
{
    chassis_stop();

    vofa_tgt_l   = 0;
    vofa_tgt_r   = 0;
    vofa_step_on = 0;
    vofa_online  = 1;
}

//退出接管: 停车
//不停车就交还的话, chassis 里还留着上一个目标, 下次接管车会立刻窜出去
//退出后波形照发, 方便停车后回看刚才的曲线
static void vofa_exit(void)
{
    vofa_tgt_l   = 0;
    vofa_tgt_r   = 0;
    vofa_step_on = 0;

    chassis_stop();

    vofa_online = 0;
}

//==================== 命令处理 ====================

//处理一条完整命令 (已去掉结束符, 以 \0 结尾)
static void vofa_handle_cmd(const char *cmd)
{
    int32 a;
    int32 b;
    float kp;
    float ki;
    float kd;

    while((' ' == *cmd) || ('\t' == *cmd))
    {
        cmd ++;
    }

    if('\0' == *cmd)
    {
        return;
    }

    //START / STOP 用前缀比较而不是整串比较:
    //VOFA+ 的按钮控件在 Str 模式下会给命令追加参数(按下发 1, 松开发 0),
    //命令内容里只要带了占位符, 实际发出去的就是 "STOP1" 这种。
    //整串比较会让这类命令静默失效, 前缀比较能兼容
    //
    if(vofa_cmd_is(cmd, "START", 5))
    {
        vofa_ack("CTRL OK START");
        vofa_enter();
        return;
    }

    if(vofa_cmd_is(cmd, "STOP", 4))
    {
        vofa_exit();
        vofa_ack("CTRL OK STOP");
        return;
    }

    //SAVE: 把当前 PID 写进 Flash, 掉电不丢
    //chassis_set_param 已经把参数同步到 g_param 了, 这里只管落盘
    //注意 Flash 有写入寿命, 别在循环里连续发这条命令
    if(vofa_cmd_is(cmd, "SAVE", 4))
    {
        uint8 err = flash_param_save();

        /* 回显实际写进 Flash 的值。
         * 只回 "OK" 的话, 用户没法确认存的到底是哪三个数,
         * 上电后看到不一样也不知道是没存上还是存了别的值。 */
        if(err)
        {
            vofa_ack("CTRL ERR SAVE");
        }
        else
        {
            char b[80];
            uint16 n = 0;
            const char *tag = "CTRL OK SAVE L ";
            float v[6];
            uint8 k;

            v[0] = g_param.l_kp; v[1] = g_param.l_ki; v[2] = g_param.l_kd;
            v[3] = g_param.r_kp; v[4] = g_param.r_ki; v[5] = g_param.r_kd;

            while('\0' != tag[n]) { b[n] = tag[n]; n ++; }
            for(k = 0; k < 6; k ++)
            {
                if(3 == k)
                {
                    b[n++] = ' '; b[n++] = 'R'; b[n++] = ' ';
                }
                n += vofa_i32_to_str((int32)(v[k] * 1000.0f), &b[n]);
                if(5 != k) b[n++] = ' ';
            }
            b[n] = '\0';
            vofa_ack(b);
        }
        return;
    }

    //LOAD: 从 Flash 重新读参数并作用到速度环
    //调乱了想退回上次保存的值, 发这条
    if(vofa_cmd_is(cmd, "LOAD", 4))
    {
        flash_param_load();
        chassis_set_param(ENC_LEFT,  g_param.l_kp, g_param.l_ki, g_param.l_kd);
        chassis_set_param(ENC_RIGHT, g_param.r_kp, g_param.r_ki, g_param.r_kd);
        vofa_ack("CTRL OK LOAD");
        return;
    }

    //TGT=500 或 TGT=500,300
    //必须在接管状态, 否则正常跑任务时会被一条命令改掉目标
    if(vofa_key_is(cmd, "TGT"))
    {
        if(!vofa_online)
        {
            return;
        }

        a = vofa_tgt_l;
        b = vofa_tgt_r;                     //只发一个值时, 右轮保持原值

        if(!vofa_parse_pair(cmd + 4, &a, &b))
        {
            return;
        }

        vofa_tgt_l   = a;
        vofa_tgt_r   = b;
        vofa_step_on = 0;                   //手动给目标就停自动阶跃
        chassis_set_target(vofa_tgt_l, vofa_tgt_r);
        return;
    }

    //TGL=500 / TGR=500  单边单独设定
    //左右各用一个滑块时用这两条。和 TGT 一样必须已接管
    if(vofa_key_is(cmd, "TGL") || vofa_key_is(cmd, "TGR"))
    {
        if(!vofa_online)
        {
            return;
        }

        if(!vofa_parse_i32(cmd + 4, &a))
        {
            return;
        }

        if('L' == cmd[2])
        {
            vofa_tgt_l = a;
        }
        else
        {
            vofa_tgt_r = a;
        }

        vofa_step_on = 0;
        chassis_set_target(vofa_tgt_l, vofa_tgt_r);
        return;
    }

    //STEP=500,5  自动阶跃, 第二段是半周期秒数(可省, 默认 5 秒)
    //STEP=0      停止阶跃
    if(vofa_key_is(cmd, "STEP"))
    {
        if(!vofa_online)
        {
            return;
        }

        a = 0;
        b = 5;

        if(!vofa_parse_pair(cmd + 5, &a, &b))
        {
            return;
        }

        if(0 == a)
        {
            vofa_step_on = 0;
            return;
        }

        if(b < 1)
        {
            b = 1;                          //下限 1 秒, 再短 PID 来不及收敛
        }

        vofa_step_hi   = a;
        vofa_step_per  = b * 1000;
        vofa_step_ph   = 0;
        vofa_step_on   = 1;
        vofa_step_tick = chassis_get_tick();

        vofa_tgt_l = 0;
        vofa_tgt_r = 0;
        chassis_set_target(0, 0);
        return;
    }

    //KP= / KI= / KD=
    //滑块值直接就是参数值, 支持小数: KP=1.5 / KI=0.05 / KD=0
    //解析出来是放大 1000 倍的整数, 除以 1000 得到真实值
    /*
     * PID 参数命令
     *
     *   KP= / KI= / KD=     左右轮同时改 (沿用老写法, 保持兼容)
     *   KPL= / KIL= / KDL=  只改左轮
     *   KPR= / KIR= / KDR=  只改右轮
     *
     * 为什么分左右: 两个电机的换向器磨损 / 编码器精度 / 磁铁间隙不同,
     * 同一组参数在一侧平滑、另一侧毛刺明显。2026-11-16 实测后分开调。
     */
    {
        uint8  is_pid = 0;
        uint8  sides  = 0;      /* bit0 = 左, bit1 = 右 */
        uint8  which  = 0;      /* 0=Kp, 1=Ki, 2=Kd */
        uint8  klen   = 2;      /* 命令名前缀长度, 默认 "KP" */

        if(vofa_cmd_is(cmd, "KPL", 3))      { is_pid = 1; sides = 0x1; which = 0; klen = 3; }
        else if(vofa_cmd_is(cmd, "KIL", 3)) { is_pid = 1; sides = 0x1; which = 1; klen = 3; }
        else if(vofa_cmd_is(cmd, "KDL", 3)) { is_pid = 1; sides = 0x1; which = 2; klen = 3; }
        else if(vofa_cmd_is(cmd, "KPR", 3)) { is_pid = 1; sides = 0x2; which = 0; klen = 3; }
        else if(vofa_cmd_is(cmd, "KIR", 3)) { is_pid = 1; sides = 0x2; which = 1; klen = 3; }
        else if(vofa_cmd_is(cmd, "KDR", 3)) { is_pid = 1; sides = 0x2; which = 2; klen = 3; }
        else if(vofa_cmd_is(cmd, "KP", 2))  { is_pid = 1; sides = 0x3; which = 0; klen = 2; }
        else if(vofa_cmd_is(cmd, "KI", 2))  { is_pid = 1; sides = 0x3; which = 1; klen = 2; }
        else if(vofa_cmd_is(cmd, "KD", 2))  { is_pid = 1; sides = 0x3; which = 2; klen = 2; }

        if(is_pid)
        {
            if(!vofa_parse_fixed3(cmd + klen + 1, &a))
            {
                return;
            }

            if(sides & 0x1)
            {
                chassis_get_param(ENC_LEFT, &kp, &ki, &kd);
                if(0 == which) kp = (float)a / 1000.0f;
                if(1 == which) ki = (float)a / 1000.0f;
                if(2 == which) kd = (float)a / 1000.0f;
                chassis_set_param(ENC_LEFT, kp, ki, kd);
            }
            if(sides & 0x2)
            {
                chassis_get_param(ENC_RIGHT, &kp, &ki, &kd);
                if(0 == which) kp = (float)a / 1000.0f;
                if(1 == which) ki = (float)a / 1000.0f;
                if(2 == which) kd = (float)a / 1000.0f;
                chassis_set_param(ENC_RIGHT, kp, ki, kd);
            }
            return;
        }
    }

    /* 未知命令回显。
     *
     * 为什么改: 原来完全静默, 用户点了按钮没反应时无从判断是
     * "命令没收到" 还是 "收到但不认识"。实测踩过这个坑:
     * 面板按钮名是小写 save, 代码比较大写 "SAVE", 点了完全没反应,
     * 却没有任何提示, 白白浪费很久。
     *
     * 只对短命令回显: 滑块拖动时会连发 KP=... 这类长命令,
     * 每条都回文本会把波形区刷爆。 */
    if(strlen(cmd) <= 12)
    {
        vofa_ack("CTRL UNKNOWN");
    }
}

//从串口捞字节, 攒成命令行
static void vofa_rx_service(void)
{
    uint8  buf[VOFA_RX_MAX_PER_CYCLE];
    uint32 n;
    uint32 i;
    uint8  ch;

    n = debug_read_ring_buffer(buf, VOFA_RX_MAX_PER_CYCLE);
    if(0 == n)
    {
        return;
    }

    vofa_last_rx_tick = chassis_get_tick(); //有新字节, 帧还没结束

    for(i = 0; i < n; i ++)
    {
        ch = buf[i];

        //\n \r ! 都算结束符, VOFA 命令框里写不写都能用
        if(('\n' == ch) || ('\r' == ch) || ('!' == ch))
        {
            if((vofa_line_len > 0) && (0 == vofa_line_bad))
            {
                vofa_line[vofa_line_len] = '\0';
                vofa_handle_cmd(vofa_line);
            }
            vofa_line_len = 0;
            vofa_line_bad = 0;
            continue;
        }

        if(vofa_line_len < (uint8)(VOFA_CMD_MAX_LEN - 1))
        {
            vofa_line[vofa_line_len ++] = (char)ch;
        }
        else
        {
            vofa_line_bad = 1;              //超长整条丢弃
        }
    }
}

//超时兜底: 收到字节但等不到结束符, 也要把命令执行掉
//对应 VOFA 命令框/按钮里不写 \n 的情况
static void vofa_frame_timeout(void)
{
    if(0 == vofa_line_len)
    {
        return;
    }

    if((chassis_get_tick() - vofa_last_rx_tick) < (uint32)VOFA_FRAME_TIMEOUT_TICK)
    {
        return;
    }

    if(0 == vofa_line_bad)
    {
        vofa_line[vofa_line_len] = '\0';
        vofa_handle_cmd(vofa_line);
    }
    vofa_line_len = 0;
    vofa_line_bad = 0;
}

//推进自动阶跃
static void vofa_step_service(void)
{
    uint32 now;

    if((0 == vofa_step_on) || (0 == vofa_online))
    {
        return;
    }

    now = chassis_get_tick();

    if((now - vofa_step_tick) < (uint32)(vofa_step_per / CHASSIS_PERIOD_MS))
    {
        return;
    }

    vofa_step_tick = now;

    if(0 == vofa_step_ph)
    {
        vofa_step_ph = 1;
        vofa_tgt_l   = vofa_step_hi;
        vofa_tgt_r   = vofa_step_hi;
    }
    else
    {
        vofa_step_ph = 0;
        vofa_tgt_l   = 0;
        vofa_tgt_r   = 0;
    }

    chassis_set_target(vofa_tgt_l, vofa_tgt_r);
}

//每个控制周期发一帧波形
static void vofa_tx_service(void)
{
    uint32 now;
    uint16 len;

    now = chassis_get_tick();
    if(now == vofa_last_send)
    {
        return;                             //这个周期已经发过
    }
    vofa_last_send = now;

    len = vofa_build_frame();
    uart_write_buffer(DEBUG_UART_INDEX, (const uint8 *)vofa_tx, len);
}

//==================== 对外接口 ====================

void vofa_init(void)
{
    uint8  junk[VOFA_RX_MAX_PER_CYCLE];

    //刚上电串口里可能还挂着上位机的残包, 先清干净
    //不清的话第一圈就会被当成命令解析
    while(debug_read_ring_buffer(junk, VOFA_RX_MAX_PER_CYCLE) > 0)
    {
    }

    vofa_online       = 0;
    vofa_line_len     = 0;
    vofa_line_bad     = 0;
    vofa_last_rx_tick = 0;
    vofa_last_send    = 0;
    vofa_step_on      = 0;
    vofa_step_hi      = 0;
    vofa_step_per     = 5000;
    vofa_step_tick    = 0;
    vofa_step_ph      = 0;
    vofa_tgt_l        = 0;
    vofa_tgt_r        = 0;

    /*
     * 上电第一件事: 把速度环参数和来源打到串口。
     *
     * 为什么必须加: Flash 没存上时会退回 param_set_default() 的默认值,
     * 但现象和"存上了旧值"完全一样, 肉眼分不出来。曾经为了排查这个
     * 花掉大量时间 —— 在 VOFA 里调了半天, 重启后发现是 2.0/0.08/0,
     * 不知道是没存上还是存了别的。
     *
     * 现在上电连串口第一行就能看到:
     *   PARAM SRC=FLASH  kp=10.000 ki=1.000 kd=15.000    <- 存上了
     *   PARAM SRC=DEFAULT kp=2.000 ki=0.080 kd=0.000     <- 没存上!
     */
    {
        /* 上电打印参数和来源, 连 VOFA 第一行就能看到:
         *   PARAM SRC=FLASH   L 18200 1000 8600 R 15000 1000 15000
         *   PARAM SRC=DEFAULT L 18200 1000 8600 R 15000 1000 15000
         * SRC=DEFAULT 说明 Flash 里没有有效参数(空片/MAGIC 不符),
         * 用的是 param_set_default() 的保底值。
         * 数值都是 x1000 的定点, 18200 = 18.2 */
        char   b[96];
        uint16 n = 0;
        const char *tag = g_param_valid ? "PARAM SRC=FLASH L "
                                        : "PARAM SRC=DEFAULT L ";
        float v[6];
        uint8 k;

        v[0] = g_param.l_kp; v[1] = g_param.l_ki; v[2] = g_param.l_kd;
        v[3] = g_param.r_kp; v[4] = g_param.r_ki; v[5] = g_param.r_kd;

        while('\0' != tag[n]) { b[n] = tag[n]; n ++; }
        for(k = 0; k < 6; k ++)
        {
            if(3 == k) { b[n++] = ' '; b[n++] = 'R'; b[n++] = ' '; }
            n += vofa_i32_to_str((int32)(v[k] * 1000.0f), &b[n]);
            if(5 != k) b[n++] = ' ';
        }
        b[n] = '\0';
        vofa_ack(b);
    }
}

void vofa_run(void)
{
    vofa_rx_service();
    vofa_frame_timeout();
    vofa_step_service();
    vofa_tx_service();
}

void vofa_release(void)
{
    if(vofa_online)
    {
        vofa_exit();
    }
}

#else   /* VOFA_ENABLE == 0 */

//比赛固件: 全部空实现
//保留符号而不是在调用处加 #if, 这样切回比赛固件只翻一个宏, 调用点一行都不改

void vofa_init(void)
{
}

void vofa_run(void)
{
}

void vofa_release(void)
{
}

#endif  /* VOFA_ENABLE */
