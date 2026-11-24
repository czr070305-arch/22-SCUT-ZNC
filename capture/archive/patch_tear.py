# -*- coding: utf-8 -*-
"""修复 task_capture.c 的采集撕裂

根因分析（基于逐飞库源码确认）:

  1. mt9v03x_init() 调用
         dvp_camera_init(&mt9v03x_image[0], &mt9v03x_image[0], ...)
     两个 buffer 地址都是同一块内存，DMA 是纯循环覆盖模式，
     没有双缓冲，任何时刻图像都在被改写。

  2. 帧完成中断 camera_dvp_handler() -> mt9v03x_dvp_handler()
     只做了一件事： mt9v03x_finish_flag = 1;
     它【不会】停止 DMA。

  3. 所以原来的 cap_dvp_pause() 是在任意时刻硬关 DMA，
     此时 DMA 可能刚写到第 37 行。上半部分是新的，
     下半部分还是上一帧 —— 这就是撕裂。

  修复方向:
     必须"等一个帧完成中断，然后立刻关 DMA"。
     中断发生的那一瞬间，DMA 正好写完最后一行，图像是完整的。
     在这之后关掉 DMA，图像就定格在一个完整帧上。

     为了把这中间的空窗压到最短，直接在 DVP 中断处理函数里
     检查采集页是否在等待，是就当场关 DMA。

实现:
    isr.c   : 中断里加 3 行，检测 cap_wait_frame 标志
    task_capture.c : pause 时置标志并阻塞等待；超时降级为直接关
"""

# ---------------------------------------------------------------- isr.c
ISR_PATH = 'user/src/isr.c'

ISR_OLD_FUNC = '''void DVP_IRQHandler(void)
{
    if (DVP->IFR & RB_DVP_IF_FRM_DONE)
    {
        camera_dvp_handler();
        DVP->IFR &= ~RB_DVP_IF_FRM_DONE;
    }
}'''

ISR_NEW_FUNC = '''void DVP_IRQHandler(void)
{
    if (DVP->IFR & RB_DVP_IF_FRM_DONE)
    {
        camera_dvp_handler();
        DVP->IFR &= ~RB_DVP_IF_FRM_DONE;

        /* 图像采集页(CAPTURE)的定帧请求。
         *
         * 为什么必须在这里做:
         *   每个帧完成中断发生时，DMA 刚刚写完最后一行，
         *   mt9v03x_image 是完整的一帧。晚哪怕几微秒，
         *   DMA 就开始覆盖下一帧的第 0 行，图像就撕裂了。
         *
         *   主循环检测 finish_flag 再关 DMA 的延迟是几十微秒
         *   到几毫秒（取决于当前在跑什么），来不及。
         *
         * 代价: 中断里多一次位操作 + 一次分支判断。 */
        if (cap_wait_frame) {
            cap_wait_frame = 0;
            DVP->CR1 &= ~RB_DVP_DMA_EN;
            DVP->CR0 &= ~RB_DVP_ENABLE;
        }
    }
}'''


def patch_isr():
    raw = open(ISR_PATH, 'rb').read().decode('gbk')
    if 'cap_wait_frame' in raw:
        print('isr.c: 已经改过，跳过')
        return

    if ISR_OLD_FUNC not in raw:
        raise SystemExit('isr.c: 没找到 DVP_IRQHandler 原文')

    raw = raw.replace(ISR_OLD_FUNC, ISR_NEW_FUNC, 1)

    # 声明外部标志。isr.c 已经 include zf_common_headfile.h，
    # 但 cap_wait_frame 定义在 task_capture.c 里，需要 extern 声明。
    anchor = 'void DVP_IRQHandler (void) __attribute__((interrupt()));'
    if anchor not in raw:
        raise SystemExit('isr.c: 没找到前置声明锚点')
    decl = ('/* 图像采集页的"等一帧再停"标志，定义在 task_capture.c */\r\n'
            'extern volatile uint8 cap_wait_frame;\r\n'
            '\r\n'
            + anchor)
    raw = raw.replace(anchor, decl, 1)

    blob = raw.encode('gbk')
    assert len(blob) > 5000
    open(ISR_PATH, 'wb').write(blob)
    print('isr.c: 修改完成 (%d 字节)' % len(blob))


# ------------------------------------------------------- task_capture.c
CAP_PATH = 'code/app/task_capture.c'

CAP_OLD = '''static void cap_dvp_pause(void)
{
    DVP->CR1 &= ~RB_DVP_DMA_EN;         /* \xd4\xdd\xcd\xa3 DMA, \xb7\xc0\xd6\xb9\xb0\xeb\xb0\xb5 */
    DVP->CR0 &= ~RB_DVP_ENABLE;         /* \xd4\xdd\xcd\xa3 DVP */
    mt9v03x_finish_flag = 0;
}'''


def patch_capture():
    raw = open(CAP_PATH, 'rb').read().decode('gbk')
    if 'cap_wait_frame' in raw:
        print('task_capture.c: 已经改过，跳过')
        return

    # 用文本定位而不是字节字面量（更稳）
    start = raw.find('static void cap_dvp_pause(void)')
    if start < 0:
        raise SystemExit('task_capture.c: 没找到 cap_dvp_pause')
    end = raw.find('\n}', start)
    if end < 0:
        raise SystemExit('task_capture.c: 没找到函数结尾')
    end += 2

    new_func = '''/* 采集页"等一帧再停"标志。
 * 由 DVP 帧完成中断消费：中断触发的那一刻 DMA 刚写完最后一行，
 * 图像完整，当场关 DMA 就不会撕裂。
 * 定义成 volatile，因为中断里会改。 */
volatile uint8 cap_wait_frame = 0;

static void cap_dvp_pause(void)
{
    /* 清掉可能残留的完成标志，重新等一个新帧 */
    mt9v03x_finish_flag = 0;

    /* 请求中断在下一个帧完成点关 DMA。
     * 50fps 下最多等 20ms，超出说明摄像头没在出帧，
     * 降级为直接关，避免死锁。 */
    cap_wait_frame = 1;
    {
        uint32 t0 = 0;
        while (cap_wait_frame) {
            system_delay_us(200);
            t0 += 200;
            if (t0 > 40000u) {          /* 40ms 超时 */
                cap_wait_frame = 0;
                DVP->CR1 &= ~RB_DVP_DMA_EN;
                DVP->CR0 &= ~RB_DVP_ENABLE;
                break;
            }
        }
    }
    mt9v03x_finish_flag = 0;
}'''

    raw = raw[:start] + new_func + raw[end:]
    blob = raw.encode('gbk')
    assert len(blob) > 5000
    open(CAP_PATH, 'wb').write(blob)
    print('task_capture.c: 修改完成 (%d 字节)' % len(blob))


if __name__ == '__main__':
    patch_isr()
    patch_capture()
