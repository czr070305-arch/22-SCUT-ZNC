# -*- coding: utf-8 -*-
"""串口诊断：确认 COM11 上到底有没有数据

排查顺序：
  1. 串口能不能打开（有没有被别的程序占用）
  2. 10 秒内能不能收到任何字节
  3. 收到的字节里能不能找到 0xA5 0x5A 帧头

用法:
    py -3 capture/diag_serial.py -p COM11
"""
import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print('需要 pyserial:  py -3 -m pip install pyserial')
    sys.exit(1)


def list_all():
    print('=== 系统现有串口 ===')
    ports = list(list_ports.comports())
    if not ports:
        print('  （一个都没有 —— USB 转串口没插好或驱动没装）')
    for p in ports:
        print('  %-8s %s' % (p.device, p.description))
    print()


def probe(port, baud, seconds):
    print('=== 打开 %s @ %d ===' % (port, baud))
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print('  [失败] %s' % e)
        print()
        print('  常见原因:')
        print('    - 另一个程序占着这个口（VOFA+ / MRS 串口终端 / 之前的脚本没关）')
        print('    - COM 号写错')
        return

    print('  已打开')
    ser.reset_input_buffer()

    total = 0
    found_sync = False
    first_bytes = b''
    t0 = time.time()

    print('  监听 %.0f 秒，请现在按板子上的 M 键……' % seconds)
    while time.time() - t0 < seconds:
        chunk = ser.read(4096)
        if not chunk:
            continue
        total += len(chunk)
        if len(first_bytes) < 32:
            first_bytes += chunk[:32]
        if (b'\xa5\x5a' in chunk) or (len(first_bytes) >= 2 and first_bytes[:2] == b'\xa5\x5a'):
            found_sync = True

    ser.close()

    print()
    print('=== 结果 ===')
    print('  收到字节: %d' % total)
    print('  A5 5A 帧头: %s' % ('找到' if found_sync else '没找到'))

    if total == 0:
        print()
        print('  [问题] 一个字节都没收到。检查:')
        print('    1. 板子 CAPTURE 页的 SENT 数在涨吗？（涨 = 板子在发）')
        print('    2. USB 转串口的 TX 接板子的 PB10(UART3_TX) 了吗？')
        print('    3. 地线接了吗？')
        print('    4. 波特率 115200 对吗？')
        print('    5. COM11 是那个 USB 转串口吗？（看上面的串口列表）')
    elif found_sync:
        print()
        print('  [正常] 板子数据能收到。可以运行采集脚本。')
        print('         如果采集脚本还是存不下图，问题在脚本的帧解析。')
    else:
        print()
        print('  [异常] 收到数据但不是图像帧。看前 32 字节:')
        print('    ' + ' '.join('%02X' % b for b in first_bytes))
        print('  可能是 VOFA+ 的文本帧（正常情况下 MOTOR TEST 页会发）。')
        print('  进 CAPTURE 页应该停发 VOFA，如果还在发说明页面没切过去。')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', default='COM11')
    ap.add_argument('-b', '--baud', type=int, default=115200)
    ap.add_argument('-t', '--seconds', type=float, default=10.0)
    args = ap.parse_args()

    list_all()
    probe(args.port, args.baud, args.seconds)


if __name__ == '__main__':
    main()
