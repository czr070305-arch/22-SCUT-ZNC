# -*- coding: utf-8 -*-
"""删除三个未使用的 static 函数"""
import re


def remove_func(path, sig_prefix, encoding):
    raw = open(path, 'rb').read()
    src = raw.decode(encoding)
    nl = '\r\n' if '\r\n' in src else '\n'
    lines = src.split(nl)

    # 找函数定义行
    start = None
    for i, l in enumerate(lines):
        if l.startswith(sig_prefix):
            start = i
            break
    if start is None:
        print('%s: 没找到 %s' % (path, sig_prefix))
        return

    # 往前吃掉紧邻的块注释
    cstart = start
    while cstart > 0:
        prev = lines[cstart - 1].strip()
        if prev.startswith('*') or prev.startswith('/*'):
            cstart -= 1
        else:
            break

    # 找函数结尾的 '}'（第一列）
    end = None
    for i in range(start, len(lines)):
        if lines[i] == '}':
            end = i
            break
    if end is None:
        print('%s: 没找到函数结尾' % path)
        return

    # 顺带把结尾后面的一个空行也删掉
    if end + 1 < len(lines) and lines[end + 1].strip() == '':
        end += 1

    del lines[cstart:end + 1]
    blob = nl.join(lines).encode(encoding)
    assert len(blob) > 1000
    open(path, 'wb').write(blob)
    print('%s: 删除 %s  (%d ~ %d 行)' % (path, sig_prefix.split('(')[0].split()[-1],
                                        cstart + 1, end + 1))


remove_func('code/vision/v2_elem.c', 'static int v2e_ray_y(', 'gbk')
remove_func('code/vision/v2_elem.c', 'static int v2e_check_trend(', 'gbk')
remove_func('code/vision/vision_element.c', 'static int v_abs(', 'gbk')
