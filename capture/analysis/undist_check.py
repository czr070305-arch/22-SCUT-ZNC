# -*- coding: utf-8 -*-
"""验证学长畸变模型在 188x120 上的数值行为"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import vision_sim as vs

print('=== 扫描 undistort_x 是否发散 ===')
print('%4s %4s %10s %10s' % ('x', 'y', 'undist_x', '状态'))
bad = 0
total = 0
for y in range(0, vs.V_H, 10):
    for x in range(0, vs.V_W, 10):
        u = vs.undistort_x(float(x), float(y))
        total += 1
        if not (-500 < u < 500):
            bad += 1
            if bad <= 20:
                print('%4d %4d %10.2f %10s' % (x, y, u, '发散'))
print('发散点 %d / %d' % (bad, total))

print()
print('=== 畸变函数在图像边缘的雅可比（近似）===')
print('%4s %4s %10s %10s %10s' % ('u', 'v', 'xd', 'yd', 'dx/du'))
for (u, v) in [(0, 0), (0, 60), (0, 119), (94, 0), (94, 119), (187, 0), (187, 60), (187, 119)]:
    xd, yd = vs.distort_pixel(float(u), float(v))
    xd2, yd2 = vs.distort_pixel(float(u) + 1.0, float(v))
    print('%4d %4d %10.1f %10.1f %10.3f' % (u, v, xd, yd, xd2 - xd))

print()
print('=== 中心行 y=60 的畸变映射 ===')
for x in range(0, 188, 20):
    xd, yd = vs.distort_pixel(float(x), 60.0)
    print('u=%3d -> xd=%7.1f yd=%7.1f' % (x, xd, yd))
