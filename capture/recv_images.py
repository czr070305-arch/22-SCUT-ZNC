#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
recv_images.py —— 从 CH32V307 接收摄像头图像并保存为 PNG

用法
----
    pip install pyserial pillow
    python recv_images.py                 # 用下面的默认配置
    python recv_images.py -p COM5         # 指定串口
    python recv_images.py -p COM5 -o data # 指定输出目录

工作方式
--------
    1. 打开串口 (默认 115200, 8N1)
    2. 每 2.5 秒向板子发一个 'C' 字节, 请求一帧
    3. 板子回: 9 字节帧头 + 188*120 字节像素
    4. 脚本校验帧头、尺寸、异或, 通过就存成 PNG
    5. Ctrl+C 退出

为什么发 'C' 而不是让板子主动推
------------------------------
    板子发送一帧要阻塞约 2 秒, 期间不响应按键。
    由 PC 主动请求, 板子只在收到请求后才发, 用户可以在板子上
    按 M/UP/DOWN 控制节奏, PC 这边只是"随时准备好接收"。

    如果板子端的自动应答没启用, 脚本发的 'C' 会被忽略,
    板子仍然按自己的节奏发 —— 脚本一样能收到。

帧格式
------
    [0] 0xA5      同步 1
    [1] 0x5A      同步 2
    [2] 版本      固定 0x01
    [3] width     188
    [4] height    120
    [5] seq       帧序号
    [6] thre      板子当前二值化阈值 (仅调试信息)
    [7] flags     预留
    [8] checksum  前 8 字节异或
    之后是 width*height 字节的灰度数据, 逐行从上到下。
"""

import argparse
import os
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial:  pip install pyserial")
    sys.exit(1)

try:
    from PIL import Image
except ImportError:
    print("需要 pillow:  pip install pillow")
    sys.exit(1)


# ---------- 协议常量 ----------
SYNC0 = 0xA5
SYNC1 = 0x5A
HDR_LEN = 9

# ---------- 默认配置 ----------
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 115200
DEFAULT_OUT = "dataset"
DEFAULT_INTERVAL = 2.5      # 请求间隔(秒), 必须大于板子发一帧的时间


def checksum(buf: bytes) -> int:
    c = 0
    for b in buf:
        c ^= b
    return c


def read_exact(ser: serial.Serial, n: int, timeout: float) -> bytes:
    """读满 n 字节, 超时返回已收到的部分。

    为什么要这个:
      22560 字节 @115200 需要 1.96 秒。pyserial 的 read(n) 只用
      ser.timeout 那点时间等, 到点就返回现有数据。
      timeout=1 时只能收到约 11520 字节(正好 1 秒的量),
      这就是之前"像素不全 (11819/22560)"的原因。
    """
    buf = bytearray()
    deadline = time.time() + timeout

    while len(buf) < n:
        remain = n - len(buf)
        chunk = ser.read(min(remain, 4096))

        if chunk:
            buf.extend(chunk)
        elif time.time() > deadline:
            break

    return bytes(buf)


def find_sync(ser: serial.Serial, timeout: float = 30.0):
    """在串口流里找 0xA5 0x5A。返回 True 表示找到了。"""
    deadline = time.time() + timeout
    state = 0

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        v = b[0]

        if state == 0:
            if v == SYNC0:
                state = 1
        else:
            if v == SYNC1:
                return True
            # 不是 0x5A: 回到初始态, 但当前字节可能就是新的 0xA5
            state = 1 if v == SYNC0 else 0

    return False


def recv_one(ser: serial.Serial):
    """接收一帧。成功返回 (w, h, seq, thre, pixels), 失败返回 None。"""
    if not find_sync(ser):
        return None

    # 同步字节已经吃掉, 读剩下的 7 字节
    rest = read_exact(ser, HDR_LEN - 2, 2.0)
    if len(rest) != HDR_LEN - 2:
        return None

    hdr = bytes([SYNC0, SYNC1]) + rest
    ver, w, h, seq, thre, flags, chk = rest

    if checksum(hdr[:8]) != chk:
        print(f"  [跳过] 校验失败")
        return None

    if ver != 0x01:
        print(f"  [跳过] 版本 {ver} 不认识")
        return None

    n_pix = w * h
    if n_pix == 0 or n_pix > 200 * 200:
        print(f"  [跳过] 尺寸异常 {w}x{h}")
        return None

    # 按帧时长估超时: 每字节 10 bit, 再留一倍余量
    need_sec = n_pix * 10.0 / ser.baudrate
    pixels = read_exact(ser, n_pix, need_sec * 2.0 + 1.0)

    if len(pixels) != n_pix:
        print(f"  [跳过] 像素不全 ({len(pixels)}/{n_pix})")
        return None

    return w, h, seq, thre, pixels


def main():
    ap = argparse.ArgumentParser(description="从 CH32V307 接收图像")
    ap.add_argument("-p", "--port", default=DEFAULT_PORT, help="串口名, 如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help="波特率")
    ap.add_argument("-o", "--out", default=DEFAULT_OUT, help="输出目录")
    ap.add_argument("-i", "--interval", type=float, default=DEFAULT_INTERVAL,
                    help="请求间隔(秒), 必须大于板子发一帧的时间")
    ap.add_argument("--no-request", action="store_true",
                    help="不主动发 'C', 纯被动接收(板子自己按按键发)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    print(f"打开串口 {args.port} @ {args.baud}")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"打不开串口: {e}")
        print("Windows 上先确认 COM 号(设备管理器 -> 端口), 并且没有别的程序占用。")
        sys.exit(1)

    ser.reset_input_buffer()
    print(f"输出目录: {os.path.abspath(args.out)}")
    print("在板子上进 TASK TEST -> CAPTURE, 按 M/UP/DOWN 发图")
    print("Ctrl+C 退出\n")

    saved = 0
    last_req = 0.0

    try:
        while True:
            # 周期性请求一帧
            if not args.no_request:
                now = time.time()
                if now - last_req >= args.interval:
                    ser.write(b"C")
                    ser.flush()
                    last_req = now

            res = recv_one(ser)
            if res is None:
                continue

            w, h, seq, thre, pixels = res

            ts = time.strftime("%Y%m%d_%H%M%S")
            name = f"{ts}_seq{seq:03d}.png"
            path = os.path.join(args.out, name)

            img = Image.frombytes("L", (w, h), pixels)
            img.save(path)

            saved += 1
            print(f"[{saved:4d}] {name}  {w}x{h}  thre={thre}")

    except KeyboardInterrupt:
        print(f"\n\n共保存 {saved} 张到 {os.path.abspath(args.out)}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
