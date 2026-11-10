# -*- coding: utf-8 -*-
"""把串口助手的收发日志解成可读的字段。

用法:
    python decode_log.py <日志文件>
    (日志里每行含 "发" 或 "收" 即可, 十六进制字节会被自动挑出来)

支持把 发/收 配对, 并解出状态码、flags、MRE 快照、电机状态。
"""
import re
import sys

STATUS = {
    0x00: 'OK', 0x01: 'BUSY', 0x02: 'ERR_SIZE', 0x03: 'ERR_FLASH',
    0x04: 'ERR_PROG', 0x05: 'ERR_TEST', 0x06: 'ERR_NO_FW',
    0x07: 'ERR_INCOMPLETE', 0x08: 'ERR_CRC', 0x09: 'ERR_SEQ',
    0x0A: 'ERR_COUNT', 0x0B: 'ERR_PARAM', 0x0C: 'ERR_TIMEOUT',
    0x0D: 'ERR_OVERFLOW', 0xFF: 'NIY',
}

CMD = {
    0x0001: 'PING', 0x0002: 'GET_VERSION', 0x0010: 'MOTOR_MOVE',
    0x0011: 'MOTOR_STOP', 0x0012: 'MOTOR_STATUS', 0x0020: 'MRE_QUERY',
}


def crc16(b):
    crc = 0xFFFF
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def u16(b, i):
    return (b[i] << 8) | b[i + 1]


def u32(b, i):
    return (b[i] << 24) | (b[i + 1] << 16) | (b[i + 2] << 8) | b[i + 3]


def decode(buf):
    """返回 (cmd, 描述行列表)；不是合法帧时返回 (None, ...)"""
    if len(buf) < 10:
        return None, ['  帧太短 (%d 字节)' % len(buf)]
    if buf[0] != 0xA5 or buf[1] != 0x5B:
        return None, ['  帧头不是 A5 5B: %02X %02X' % (buf[0], buf[1])]

    cmd = u16(buf, 2)
    seq = u16(buf, 4)
    ln = u16(buf, 6)
    total = 10 + ln

    lines = ['  cmd=0x%04X(%s) seq=0x%04X len=%d 实到=%d'
             % (cmd, CMD.get(cmd, '未知'), seq, ln, len(buf))]

    if len(buf) < total:
        lines.append('  帧不完整, 缺 %d 字节' % (total - len(buf)))
        return cmd, lines

    d = buf[8:8 + ln]
    calc = crc16(buf[:8 + ln])
    recv = u16(buf, 8 + ln)
    if calc != recv:
        lines.append('  CRC 错: 算得 %04X, 帧里是 %04X' % (calc, recv))
        return cmd, lines
    lines.append('  CRC 正确 (%04X)' % recv)

    if ln == 0:
        return cmd, lines

    st = d[0]
    lines.append('  状态码 0x%02X (%s)' % (st, STATUS.get(st, '?')))

    if cmd == 0x0002 and ln >= 4:
        lines.append('  版本 %d.%d.%d' % (d[1], d[2], d[3]))

    elif cmd == 0x0012 and ln >= 10:
        lines.append('  运行中=%d  已走=%d 步  目标=%d 步'
                     % (d[1], u32(d, 2), u32(d, 6)))

    elif cmd == 0x0020 and ln >= 22:
        flags = d[1]
        mv_last = u16(d, 10)
        vref = u16(d, 12)
        lines.append('  flags=0x%02X  ADC在跑=%d  TMC就绪=%d'
                     % (flags, flags & 1, (flags >> 1) & 1))
        lines.append('  MRE  last=%5d  min=%5d  max=%5d  mean=%5d  峰峰=%d  (%d mV)'
                     % (u16(d, 2), u16(d, 4), u16(d, 6), u16(d, 8),
                        u16(d, 6) - u16(d, 4), mv_last))
        lines.append('  3V3  raw=%5d  (%d mV, 理论 1650)   窗口=%d 点'
                     % (vref, u16(d, 14), u16(d, 16)))
        lines.append('  累计采样 = %d' % u32(d, 18))

    return cmd, lines


HEX = re.compile(r'\b((?:[0-9A-Fa-f]{2}[\s]+){4,}[0-9A-Fa-f]{2})\b')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    text = open(sys.argv[1], encoding='utf-8', errors='replace').read()
    prev_samples = None

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        m = HEX.search(line)
        if not m:
            continue
        data = bytes(int(x, 16) for x in m.group(1).split())
        direction = '发' if '发' in line else ('收' if '收' in line else '?')
        ts = re.match(r'\[([^\]]+)\]', line)
        ts = ts.group(1) if ts else ''

        if direction == '发':
            print('%s  发 %s' % (ts, m.group(1).strip()))
            c, ls = decode(data)
        else:
            print('%s  收 %s' % (ts, m.group(1).strip()))
            c, ls = decode(data)
            for l in ls:
                print(l)
            # 复位检测: MRE_QUERY 的 total_samples 倒退
            if c == 0x0020 and len(data) >= 32:
                s = u32(data, 26)
                if prev_samples is not None and s < prev_samples:
                    print('  !!! 累计采样倒退 (%d -> %d) => 板子在两次查询之间复位过'
                          % (prev_samples, s))
                prev_samples = s
        print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
