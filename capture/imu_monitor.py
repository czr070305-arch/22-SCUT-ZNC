# -*- coding: utf-8 -*-
"""
IMU 实时监测 + 漂移分析

固件从 DEBUG_UART(UART3, PB10) 每 10ms 发一帧 CSV，共 37 列。
本脚本抓最近 20 列里的 IMU 部分，实时显示，并在结束时给出漂移统计。

列定义（0 起）：
    20 yaw_x100      航向角 x100（度）
    21 pitch_x100    俯仰角 x100
    22 roll_x100     横滚角 x100
    23 gx            陀螺 X 原始值 LSB（已减零偏）
    24 gy
    25 gz
    26 bias_x        校准得到的零偏 LSB
    27 bias_y
    28 bias_z
    29 temp_x100     温度 x100（摄氏度）
    30 ax            加速度 X 原始值 LSB
    31 ay
    32 az
    33 ist           环岛状态 0=IDLE 1=RUNNING 2=DONE
    34 dyaw_x100     相对入环起点的已转角度 x100（度）
    35 arm           1 = 已转过 60 度，允许判出岛
    36 ion           视觉 island_on

用法:
    py -3 capture/imu_monitor.py -p COM11
    py -3 capture/imu_monitor.py -p COM11 --watch        # 只打印变化
    py -3 capture/imu_monitor.py -p COM11 --log imu.csv  # 存 CSV 便于后分析
    py -3 capture/imu_monitor.py -p COM11 --seconds 30   # 30 秒后自动出报告
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print('需要 pyserial:  py -3 -m pip install pyserial')
    sys.exit(1)


TOTAL_COLS = 37

# 索引 -> (名字, 显示宽度, 缩放)
IMU_COLS = [
    (20, 'yaw',   8, 0.01),
    (21, 'pitch', 8, 0.01),
    (22, 'roll',  8, 0.01),
    (23, 'gx',    6, 1.0),
    (24, 'gy',    6, 1.0),
    (25, 'gz',    6, 1.0),
    (26, 'bx',    6, 1.0),
    (27, 'by',    6, 1.0),
    (28, 'bz',    6, 1.0),
    (29, 'temp',  7, 0.01),
    (30, 'ax',    7, 1.0),
    (31, 'ay',    7, 1.0),
    (32, 'az',    7, 1.0),
    # 2026-11-26 环岛状态机调试列
    (33, 'ist',   4, 1.0),   # 0=IDLE 1=RUNNING 2=AFTER(yaw锁朝向)
    (34, 'dyaw',  8, 0.01),  # 相对入环起点的已转角度（度）
    (35, 'arm',   4, 1.0),   # 1=已过 60 度门限
    (36, 'ion',   4, 1.0),   # 视觉 island_on
]

LIVE  = {'yaw', 'pitch', 'roll', 'gz', 'temp', 'ist', 'dyaw'}


def judge_mount(ax, ay, az):
    """根据平放时的重力方向判断芯片安装朝向"""
    m = max(abs(ax), abs(ay), abs(az))
    if m < 4000:
        return '重力不明显（车在动或加速度计异常）'
    if az > 4000 and abs(ax) < 3000 and abs(ay) < 3000:
        return 'Z 轴朝上，正常平放'
    if az < -4000 and abs(ax) < 3000 and abs(ay) < 3000:
        return '*** Z 轴朝下（芯片装反了）—— roll 会卡在 180 度 ***'
    if ax > 4000:
        return 'X 轴朝上（芯片侧立，绕 X 转了 90 度）'
    if ax < -4000:
        return 'X 轴朝下（芯片侧立，绕 X 转了 -90 度）'
    if ay > 4000:
        return 'Y 轴朝上（芯片侧立，绕 Y 转了 90 度）'
    if ay < -4000:
        return 'Y 轴朝下（芯片侧立，绕 Y 转了 -90 度）'
    return '姿态倾斜，不明确'


def fmt(name, raw, width, scale):
    if scale != 1.0:
        return '%*.2f' % (width, raw * scale)
    return '%*d' % (width, raw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', required=True)
    ap.add_argument('-b', '--baud', type=int, default=115200)
    ap.add_argument('--watch', action='store_true',
                    help='只在角度变化时打印，便于看漂移')
    ap.add_argument('--log', default='', help='把每帧写进 CSV')
    ap.add_argument('--seconds', type=float, default=0,
                    help='抓多久后自动出统计报告，0=一直抓')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print('打不开串口: %s' % e)
        return 1

    print('已连接 %s @ %d' % (args.port, args.baud))
    print('-' * 92)
    print('%-8s %s' % ('时间', ' '.join('%*s' % (w, n) for _, n, w, _ in IMU_COLS)))
    print('-' * 92)

    logf = None
    if args.log:
        logf = open(args.log, 'w', encoding='utf-8', buffering=1)
        logf.write('t,' + ','.join(n for _, n, _, _ in IMU_COLS) + '\n')
        print('日志写入: %s' % args.log)

    # 统计用：只在最后一段静止数据上算漂移
    yaw_samples = []      # (t, yaw_deg)
    gyro_samples = []     # (gx, gy, gz)
    bias_last = None
    temp_last = None
    acc_samples = []

    buf = b''
    last_show = None
    t0 = time.monotonic()
    nframes = 0

    def report():
        if not yaw_samples:
            print('\n没收到足够数据，无法统计')
            return
        dur = yaw_samples[-1][0] - yaw_samples[0][0]
        if dur <= 0.2:
            print('\n采到的时长太短, 至少跑 5 秒')
            return

        ys = [y for _, y in yaw_samples]
        drift = ys[-1] - ys[0]
        rate = drift / dur
        print('\n' + '=' * 60)
        print('  采    样: %.1f 秒, %d 帧' % (dur, len(yaw_samples)))
        print('  YAW 起止: %.2f  ->  %.2f' % (ys[0], ys[-1]))
        print('  总漂移  : %+.2f 度' % drift)
        print('  漂移速率: %+.3f 度/秒' % rate)
        if abs(rate) < 0.2:
            print('  结论    : 很稳（<0.2 度/秒）, 不用担心')
        elif abs(rate) < 1.0:
            print('  结论    : 可接受（0.2~1 度/秒）, 出岛判据够用')
        else:
            print('  结论    : 偏大（>1 度/秒）, 建议按 S1 重新校准')

        if gyro_samples:
            ng = len(gyro_samples)
            gx = sum(g[0] for g in gyro_samples) / ng
            gy = sum(g[1] for g in gyro_samples) / ng
            gz = sum(g[2] for g in gyro_samples) / ng
            print('  平均 G  : x=%+.1f  y=%+.1f  z=%+.1f  LSB' % (gx, gy, gz))
            print('          (对应 %.3f / %.3f / %.3f 度/秒)'
                  % (gx / 16.4, gy / 16.4, gz / 16.4))

        if acc_samples:
            na = len(acc_samples)
            ax = sum(a[0] for a in acc_samples) / na
            ay = sum(a[1] for a in acc_samples) / na
            az = sum(a[2] for a in acc_samples) / na
            g  = (ax * ax + ay * ay + az * az) ** 0.5 / 8192.0
            print('  平均 A  : x=%+.1f  y=%+.1f  z=%+.1f  LSB  (合模 %.2f g)'
                  % (ax, ay, az, g))
            print('  安装判定: %s' % judge_mount(ax, ay, az))

        if bias_last:
            print('  当前零偏: x=%d  y=%d  z=%d  LSB' % bias_last)
            if bias_last == (0, 0, 0):
                print('            ^^^ 全 0 说明校准没生效！')
        if temp_last is not None:
            print('  温度    : %.1f C' % temp_last)
        print('=' * 60)

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
                parts = line.decode('ascii', errors='replace').split(',')
                if len(parts) != TOTAL_COLS:
                    continue
                try:
                    v = [int(p) for p in parts]
                except ValueError:
                    continue

                nframes += 1
                ts = time.monotonic() - t0
                vals = {name: v[idx] for idx, name, _, _ in IMU_COLS}

                yaw_deg = vals['yaw'] * 0.01
                yaw_samples.append((ts, yaw_deg))
                gyro_samples.append((vals['gx'], vals['gy'], vals['gz']))
                bias_last = (vals['bx'], vals['by'], vals['bz'])
                acc_samples.append((vals['ax'], vals['ay'], vals['az']))
                temp_last = vals['temp'] * 0.01

                if logf:
                    logf.write('%.3f,' % ts +
                               ','.join(str(vals[n]) for _, n, _, _ in IMU_COLS) + '\n')

                # 打印
                if args.watch:
                    cur = (vals['yaw'], vals['pitch'], vals['roll'])
                    if cur == last_show:
                        continue
                    last_show = cur

                row = ' '.join(fmt(n, vals[n], w, s) for _, n, w, s in IMU_COLS)
                print('%7.2fs %s' % (ts, row))

                # 控制台不要太刷屏
                if len(yaw_samples) > 200000:
                    yaw_samples = yaw_samples[-100000:]
                    gyro_samples = gyro_samples[-100000:]
                    acc_samples = acc_samples[-100000:]

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if logf:
            logf.close()
        print('-' * 92)
        print('收到 %d 帧' % nframes)
        if nframes == 0:
            print()
            print('一帧都没收到？检查：')
            print('  1. 串口号对不对（设备管理器）')
            print('  2. 是不是被 VOFA+ 占了串口')
            print('  3. USB-TTL 的 RX 是不是接到 PC10（板子 TX）')
        else:
            report()

    return 0


if __name__ == '__main__':
    sys.exit(main())
