#ifndef _TASK_FLASH_H_
#define _TASK_FLASH_H_

#include "zf_common_headfile.h"
#include "hw_key.h"

//PID参数页(L2) + 参数编辑页(L3)
//左右键要用来改参数, 所以本页左键不返回, 只能长按中键退出
//   L2: 上/下选行, 左/右微调, 中键进编辑, SAVE行按中键写Flash
//   L3: 左/右加减, 上/下切步长(x1/x10/x100), 中键回L2
//   长按中键1秒: 回上级菜单
void task_flash_show(void);
void task_flash_key(key_id_enum id);
void task_flash_run(void);
void task_flash_leave(void);

#endif
