# -*- coding: utf-8 -*-
"""
实时抓取 TRACK 页遥测，找抽搐的瞬间

50Hz 接收（屏幕只有 5Hz，抓不到抽搐瞬间）
抽搐时看哪一列在跳，就知道是什么引起的：

  列名      含义                              抽搐时如果跳 -> 原因
  ------    --------------------------------   ----------------------
  tgtL/tgtR 左右轮目标 RPM                     转向环在输出，正常
  actL/actR 左右轮实际 RPM                     速度环跟不上
  outL/outR 占空比                             速度环输出撞限幅
  err      视觉误差 x10                       视觉在抖
  turn     转向分量 RPM                        转向量在跳
  spd%     元素建议速度 0~100  ★               元素误判砍速度
  state    状态机 0普通/1直角/2十字/4环岛 ★     元素误判
  cdir     直角弯判据 -1/+1/0  ★★              ★ 这列跳 = 误报直角弯
  cruise   巡航设定                            配合 spd% 看 base

用法:
    py -3 capture/telemetry.py -p COM11
    py -3 capture/telemetry.py -p COM11 --watch spd,cdir,state
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print('需要 pyserial:  py -3 -m pip install pyserial')
    sys.exit(1)

COLS = ['tgtL', 'tgtR', 'actL', 'actR', 'outL', 'outR',
        'err', 'turn', 'spd', 'state', 'cdir', 'cruise',
        'isle', 'fork', 'exit', 'exit_x', 'exit_hits', 'track_x',
        'prefer', 'side',
        # 2026-11-26 加: IMU 姿态与原始值
        'yaw', 'pitch', 'roll', 'gx', 'gy', 'gz',
        'bias_x', 'bias_y', 'bias_z', 'temp']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', required=True)
    ap.add_argument('-b', '--baud', type=int, default=115200)
    ap.add_argument('--watch', default='',
                    help='只在这些列变化时打印，逗号分隔，如 spd,cdir,state')
    ap.add_argument('--seconds', type=float, default=0,
                    help='抓多久后退出，0=一直抓')
    ap.add_argument('--raw', action='store_true', help='打印原始行')
    ap.add_argument('--log', default='',
                    help='把每一帧写进这个 CSV 文件（一帧不丢）')
    ap.add_argument('--island-events', action='store_true',
                    help='Report transitions into and out of island state')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print('打不开串口: %s' % e)
        return 1

    print('已连接 %s @ %d' % (args.port, args.baud))
    print('%-8s %s' % ('时间', ' '.join('%7s' % c for c in COLS)))
    print('-' * 88)

    watch_idx = []
    if args.watch:
        names = [w.strip() for w in args.watch.split(',') if w.strip()]
        for w in names:
            if w in COLS:
                watch_idx.append(COLS.index(w))
            else:
                print('警告: 未知列 %s' % w)
        print('只监视: %s（这些列变化时才打印）' % ', '.join(names))

    logf = None
    if args.log:
        logf = open(args.log, 'w', encoding='utf-8', buffering=1)
        logf.write('t,' + ','.join(COLS) + '\n')
        print('日志写入: %s' % args.log)

    buf = b''
    last = None
    last_watch = None
    t0 = time.monotonic()
    nframes = 0
    nprint = 0
    island_start = None
    state_idx = COLS.index('state')

    try:
        while True:
            if args.seconds > 0 and (time.monotonic() - t0) > args.seconds:
                break
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip()
                if not line:
                    continue
                txt = line.decode('ascii', errors='replace')

                if args.raw:
                    print(txt)
                    continue

                parts = txt.split(',')
                if len(parts) not in (14, 18, 20, len(COLS)):
                    continue
                try:
                    vals = [int(p) for p in parts]
                    if len(vals) == 14:
                        vals.extend([-1] * (len(COLS) - len(vals)))
                except ValueError:
                    continue

                nframes += 1
                ts_now = time.monotonic() - t0

                if logf is not None:
                    logf.write('%.3f,%s\n' % (ts_now,
                                               ','.join(str(v) for v in vals)))

                if nframes == 1:
                    print('First valid telemetry frame received.', flush=True)
                if args.island_events:
                    if vals[state_idx] == 4 and island_start is None:
                        island_start = ts_now
                        label = 'island active at capture start' if nframes == 1 else 'island enter'
                        print(f'{ts_now:.3f}s {label}', flush=True)
                    elif vals[state_idx] != 4 and island_start is not None:
                        print(f'{ts_now:.3f}s island exit '
                              f'(observed {ts_now - island_start:.3f}s, next state={vals[state_idx]})',
                              flush=True)
                        island_start = None

                if watch_idx:
                    cur = tuple(vals[i] for i in watch_idx)
                    if cur == last_watch:
                        continue
                    last_watch = cur

                if vals == last:
                    continue
                last = vals

                ts = time.monotonic() - t0
                print('%7.2fs %s' % (ts, ' '.join('%7d' % v for v in vals)))
                nprint += 1

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if logf is not None:
            logf.close()
        print('-' * 88)
        print('收到 %d 帧，打印 %d 行' % (nframes, nprint))
        if nframes == 0:
            print()
            print('一帧都没收到？检查：')
            print('  1. 串口号对不对（设备管理器）')
            print('  2. 固件里 VOFA_ENABLE 是不是 1')
            print('  3. 是不是被别的程序（VOFA+ 上位机）占了串口')
    return 0


if __name__ == '__main__':
    sys.exit(main())
