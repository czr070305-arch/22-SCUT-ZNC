#ifndef _TUNE_H_
#define _TUNE_H_

#include "zf_common_headfile.h"

/*
 * tune.h - 转向环在线调参（精简版）
 *
 * ---------------------------------------------------------------
 * 操作（只在 TRACK 页停车时生效）
 *
 *   UP / DOWN      选择参数
 *   S1 / S2        当前参数 -1 / +1 档
 *   RIGHT          保存到 Flash（屏幕显示 SAVED）
 *   MID            启动 / 停止（随时可用）
 *   LEFT           返回菜单（随时可用）
 *
 * ---------------------------------------------------------------
 * 数据流
 *
 *   改值 -> g_tune[] 立即变化 -> 控制层下一帧就用新值（约 20ms）
 *   按 RIGHT -> flash_param_save() -> 掉电不丢
 *
 *   没有自动存盘，没有"存为默认"，没有第二层默认值。
 *   只有一条路：改 -> 存。
 *
 * ---------------------------------------------------------------
 * 为什么只有一层默认值
 *
 *   之前有"出厂默认 def"和"用户默认 g_tune_def"两层，
 *   短按/长按 RIGHT 各自触发不同动作，容易误触也容易搞混。
 *
 *   现在 Flash 只存当前值，代码里只存出厂值：
 *     上电：Flash 有效 -> 用 Flash 的值
 *            Flash 无效 -> 用代码里的 def
 *     烧录：MRS 的烧录配置是 erase=true，整片擦除，
 *           所以烧录后一定回到代码里的 def。
 *
 *   想让某一版成为"烧录后就是它"，把数值改进 tune.c 的
 *   g_tune_items[] 的 def 列，重新编译。
 *
 * ---------------------------------------------------------------
 * 存储格式
 *
 *   全部 int32 定点，dec 记录显示的小数位：
 *     dec=0  值 44   显示 "44"
 *     dec=1  值 18   显示 "1.8"
 *     dec=2  值 85   显示 "0.85"
 *
 *   定点而不是 float：显示格式化省代码，Flash 读回垃圾位概率也低。
 */

/* 参数个数。加参数：改这里 + tune.c 的 g_tune_items[] + 下面编号宏 */
#define TUNE_COUNT              (9)

/* ---------- 参数编号（即 tune_get 的下标） ---------- */
#define TUNE_KP_LO              (0)     /* 直线 P */
#define TUNE_KP_HI              (1)     /* 弯道 P */
#define TUNE_LOOKAHEAD          (2)     /* 前瞻行数 */
#define TUNE_SLOPE_W            (3)     /* 斜率权重 */
#define TUNE_TURN_MAX           (4)     /* 转向限幅 RPM */
#define TUNE_CURVE_SLOW         (5)     /* 弯道速度系数 */
#define TUNE_CRUISE             (6)     /* 巡航 RPM */
#define TUNE_THRE_BIAS          (7)     /* 二值化阈值偏置 */
#define TUNE_ERR_LP             (8)     /* err 低通强度 0~95，0=关闭 */

/* ---------- 当前生效值（控制层直接读） ---------- */
extern int32 g_tune[TUNE_COUNT];

/* ---------- 读值（已按 dec 换算成实际值） ----------
 *
 * tune_get(TUNE_KP_LO) -> 1.0f
 * tune_get(TUNE_CURVE_SLOW) -> 0.85f
 *
 * 注意 TIM7 中断里也会调用（ctrl_tick），只做一次数组索引 + 除法，安全。 */
float tune_get(int idx);

/* ---------- 载入出厂值（Flash 无效时兜底） ---------- */
void tune_load_factory(void);

/* ---------- 初始化（在 flash_param_load 之后调用） ---------- */
void tune_init(void);

/* ---------- 屏幕交互 ---------- */

int  tune_get_sel(void);        /* 当前选中第几项 */
void tune_sel_next(void);       /* 下一项（环形） */
void tune_sel_prev(void);       /* 上一项（环形） */

/* 改当前选中项。dir=+1 加一档 / -1 减一档
 * 到边界就停住，返回 1 表示值真的变了 */
int  tune_step(int dir);

/* 保存到 Flash。返回 0 = 成功
 * 只有值变过才真的写，没改过直接返回 0（不擦 Flash） */
uint8 tune_do_save(void);

/* 是否有未保存的改动（屏幕显示 UNSAVED） */
int  tune_is_dirty(void);

/* 清 dirty 标记（保存成功后由内部调用） */
void tune_clear_dirty(void);

/* ---------- 屏幕显示 ---------- */

const char *tune_name(int idx);      /* 参数名 */
void        tune_value_str(int idx, char *buf);   /* 当前值字符串 */

#endif /* _TUNE_H_ */
