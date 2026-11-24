# -*- coding: utf-8 -*-
"""检查 C 源文件: 编码 / NUL / 括号平衡 / 换行风格"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'core'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import re
import sys


def strip_code(s):
    """去掉注释和字符串字面量，避免误判括号"""
    out = []
    i = 0
    n = len(s)
    while i < n:
        if s.startswith('/*', i):
            j = s.find('*/', i + 2)
            i = n if j < 0 else j + 2
            out.append(' ')
            continue
        if s.startswith('//', i):
            j = s.find('\n', i)
            i = n if j < 0 else j
            continue
        ch = s[i]
        if ch == '"':
            i += 1
            while i < n and s[i] != '"':
                i += 2 if s[i] == '\\' else 1
            i += 1
            out.append('""')
            continue
        if ch == "'":
            i += 1
            while i < n and s[i] != "'":
                i += 2 if s[i] == '\\' else 1
            i += 1
            out.append("''")
            continue
        out.append(ch)
        i += 1
    return ''.join(out)


def check(p):
    raw = open(p, 'rb').read()
    try:
        s = raw.decode('gbk')
        enc = 'GBK-OK'
    except Exception as e:
        s = raw.decode('gbk', errors='replace')
        enc = 'GBK-BAD(%s)' % str(e)[:20]

    nul = raw.count(b'\x00')
    crlf = raw.count(b'\r\n')
    lf = raw.count(b'\n')
    bare_cr = raw.count(b'\r') - crlf

    t = strip_code(s)
    brace = t.count('{') - t.count('}')
    paren = t.count('(') - t.count(')')
    brack = t.count('[') - t.count(']')

    ok = (nul == 0 and brace == 0 and paren == 0 and brack == 0
          and bare_cr == 0 and crlf == lf and 'BAD' not in enc)
    print('%-32s %-12s NUL=%d {}%+d ()%+d []%+d  CRLF=%d LF=%d bareCR=%d  %s' % (
        p, enc, nul, brace, paren, brack, crlf, lf, bare_cr, 'OK' if ok else '!!!'))
    return ok


if __name__ == '__main__':
    files = sys.argv[1:]
    allok = all(check(f) for f in files)
    print()
    print('全部通过' if allok else '存在问题')
