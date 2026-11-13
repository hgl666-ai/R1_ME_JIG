# -*- coding: utf-8 -*-
"""解码 0x0023 波形抓取应答，画成 ASCII 图并统计上升沿。

用法（三种都行）：
    python decode_wave.py "A5 5B 00 23 ..."          # 直接贴整帧
    python decode_wave.py "[09:20:07.950]收←◆A5 5B ..."  # 贴整行日志
    python decode_wave.py 抓波.txt                    # 文件，一行一帧

注意：贴的必须是**接收到的应答**（252 字节），不是发送的请求（11 字节）。
"""
import re
import sys

HEXPAIR = re.compile(r'[0-9A-Fa-f]{2}')


def crc16(x):
    c = 0xFFFF
    for v in x:
        c ^= v
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def to_bytes(s):
    """从任意文本里挑出十六进制字节。
    先定位帧头 A5 5B, 再从那里往后取, 避免把日志时间戳(如 09:20:07)当成数据。"""
    m = re.search(r'A5\s*5B', s, re.IGNORECASE)
    if not m:
        return b''
    toks = HEXPAIR.findall(s[m.start():])
    return bytes(int(t, 16) for t in toks)


def parse(s):
    b = to_bytes(s)

    if len(b) < 10:
        raise SystemExit('  ✗ 只解析到 %d 个字节，太短。是不是贴错了？' % len(b))
    if b[0] != 0xA5 or b[1] != 0x5B:
        raise SystemExit('  ✗ 帧头不是 A5 5B（解析到 %d 字节，开头是 %02X %02X）'
                         % (len(b), b[0], b[1]))

    cmd = (b[2] << 8) | b[3]
    ln = (b[6] << 8) | b[7]

    if cmd != 0x0023:
        raise SystemExit('  ✗ 命令码是 0x%04X，不是 0x0023（波形抓取）' % cmd)

    if ln <= 1:
        raise SystemExit(
            '\n  ✗ 这是一帧【请求】（LEN=%d，共 %d 字节）。\n'
            '    脚本要的是【应答】：先把请求发出去，等约 2.2 秒，\n'
            '    再把收到的 252 字节应答贴进来。\n'
            '    发送帧: A5 5B 00 23 00 01 00 01 01 D7 4F' % (ln, len(b)))

    if len(b) != 10 + ln:
        if ln == 1:
            raise SystemExit(
                '\n  ✗ 这是一帧【请求】（LEN=1，共 11 字节）。\n'
                '    脚本要的是【应答】：先把请求发出去，等约 2.2 秒，\n'
                '    再把收到的 252 字节应答贴进来。')
        raise SystemExit('  ✗ 长度不符：LEN 字段=%d，解析到 %d 字节（应为 %d）'
                         % (ln, len(b), 10 + ln))

    got = (b[-2] << 8) | b[-1]
    calc = crc16(b[:-2])
    if calc != got:
        raise SystemExit('  ✗ CRC 不符：帧里 %04X，计算 %04X' % (got, calc))

    d = b[8:]
    status = d[0]
    cnt = d[1]
    if len(d) < 2 + 2 * cnt:
        raise SystemExit('  ✗ 点数声明 %d，但数据不够' % cnt)
    vals = [(d[2 + 2 * i] << 8) | d[3 + 2 * i] for i in range(cnt)]
    return status, vals


def plot(vals, width=76, height=18):
    lo, hi = min(vals), max(vals)
    if hi == lo:
        hi = lo + 1
    print('  raw %d ~ %d   (峰峰 %d LSB = %.0f mV)'
          % (lo, hi, hi - lo, (hi - lo) * 3300 / 4095))
    print()
    for row in range(height, -1, -1):
        level = lo + (hi - lo) * row / height
        line = []
        for c in range(width):
            i0 = c * len(vals) // width
            i1 = max(i0 + 1, (c + 1) * len(vals) // width)
            seg = vals[i0:i1]
            line.append('#' if (row == 0 or any(v >= level for v in seg)) else ' ')
        print('  %5d |%s' % (int(level), ''.join(line)))
    print('        +' + '-' * width)


def count_edges(vals, high, low):
    n = 0
    armed = False
    for v in vals:
        if armed:
            if v >= high:
                n += 1
                armed = False
        elif v <= low:
            armed = True
    return n


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    arg = ' '.join(sys.argv[1:])
    if arg.endswith('.txt'):
        lines = [l for l in open(arg, encoding='utf-8')
                 if l.strip() and not l.strip().startswith('#')]
    else:
        lines = [arg]

    for k, h in enumerate(lines, 1):
        print('=== 帧 %d ===' % k)
        status, vals = parse(h)
        print('  状态=%02X  点数=%d' % (status, len(vals)))
        if not vals:
            continue
        plot(vals)
        print()
        for hi_th, lo_th in ((1400, 1000), (1000, 500), (800, 400), (600, 300)):
            print('  阈值 %4d/%4d -> 上升沿 %d 个'
                  % (hi_th, lo_th, count_edges(vals, hi_th, lo_th)))
        print('  前 20 点: %s' % vals[:20])
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
