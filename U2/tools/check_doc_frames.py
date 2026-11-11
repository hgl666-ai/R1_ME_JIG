# -*- coding: utf-8 -*-
"""校验 INFO/命令.md 里出现的每一个样例帧：长度字段、CRC、命令码与 LEN 语义。

用法: python check_doc_frames.py <markdown 文件>
退出码: 0 全部通过 / 1 有失败 / 2 用法错误

设计要点:
  - 扫描所有含 "A5 5B" 的行，不依赖行首标记，所以表格里的帧也会被检查。
  - 允许帧被 "|" 拆开书写（如 ACK 分段举例），提取时把分隔符过滤掉。
  - 不足 10 字节的片段（多行示例的续行）跳过并计数，不判失败。
  - 判断请求/应答优先看行首的 "PC →" / "U2 →"；表格行没有标记，就按
    该命令码的 请求LEN / 应答LEN 任一匹配即通过，并打印实际匹配到的那个。
"""
import re
import sys

# 期望: 命令码 -> (请求LEN, 应答LEN)
EXPECT = {
    0x0001: (0, 1),
    0x0002: (0, 4),
    0x0010: (7, 1),
    0x0011: (0, 1),
    0x0012: (0, 10),
    0x0020: (0, 22),
    0x0021: (0, 44),
    0x0022: (1, 9),
    0x0101: (0, 8),
    0x0102: (0, 22),
}

CMD_NAME = {
    0x0001: 'PING', 0x0002: 'GET_VERSION', 0x0010: 'MOTOR_MOVE',
    0x0011: 'MOTOR_STOP', 0x0012: 'MOTOR_STATUS', 0x0020: 'MRE_QUERY',
    0x0021: 'CROSS_CHECK', 0x0022: 'MRE_PEAK', 0x0101: 'LINK_PING', 0x0102: 'LINK_GET_SNAP',
}

# 一段连续十六进制字节，允许被空格 / 制表符 / '|' 分隔
HEXRUN = re.compile(r'A5[ \t|]+5B(?:[ \t|]+[0-9A-F]{2})+')


def crc16(b):
    crc = 0xFFFF
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def u16(b, i):
    return (b[i] << 8) | b[i + 1]


def check(buf, hint):
    """返回 (ok, 描述)"""
    if len(buf) < 10:
        return None, 'fragment (%d bytes)' % len(buf)
    if len(buf) != 10 + u16(buf, 6):
        return False, 'LEN 字段=%d，实际 %d 字节' % (u16(buf, 6), len(buf))
    if crc16(buf[:-2]) != u16(buf, len(buf) - 2):
        return False, 'CRC 不匹配（算得 %04X）' % crc16(buf[:-2])

    cmd = u16(buf, 2)
    ln = u16(buf, 6)
    if cmd not in EXPECT:
        return False, '未定义命令码 0x%04X' % cmd

    req, ack = EXPECT[cmd]
    if hint == 'ack':
        if ln != ack:
            return False, '%s 应答 LEN 应为 %d，实为 %d' % (CMD_NAME[cmd], ack, ln)
        kind = 'ACK'
    elif hint == 'req':
        if ln != req:
            return False, '%s 请求 LEN 应为 %d，实为 %d' % (CMD_NAME[cmd], req, ln)
        kind = 'REQ'
    else:
        if ln == ack:
            kind = 'ACK'
        elif ln == req:
            kind = 'REQ'
        else:
            return False, '%s LEN=%d，既非请求(%d)也非应答(%d)' % (
                CMD_NAME[cmd], ln, req, ack)

    return True, '%-12s %s LEN=%2d CRC ok' % (CMD_NAME[cmd], kind, ln)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    lines = open(sys.argv[1], encoding='utf-8').read().splitlines()

    ok_cnt = bad_cnt = frag_cnt = 0

    for i, line in enumerate(lines, 1):
        if 'A5' not in line or '5B' not in line:
            continue
        if not HEXRUN.search(line):
            continue

        for m in HEXRUN.finditer(line):
            buf = bytes(int(x, 16) for x in re.findall(r'[0-9A-F]{2}', m.group(0)))

            # 方向约定按命令码段区分：
            #   0x00xx = PC<->U2 ： "PC →" 是请求, "U2 →" 是应答
            #   0x01xx = U2<->U3 ： "U2 →" 是请求, "U3 →" 是应答
            hint = None
            if len(buf) >= 4:
                cmdv = (buf[2] << 8) | buf[3]
                if cmdv < 0x0100:
                    if 'PC →' in line:
                        hint = 'req'
                    elif 'U2 →' in line:
                        hint = 'ack'
                else:
                    if 'U2 →' in line:
                        hint = 'req'
                    elif 'U3 →' in line:
                        hint = 'ack'

            ok, msg = check(buf, hint)
            if ok is None:
                frag_cnt += 1
                continue
            if ok:
                ok_cnt += 1
                print('  [OK]   line %-4d %s' % (i, msg))
            else:
                bad_cnt += 1
                print('  [FAIL] line %-4d %s   <%s>' % (i, msg, m.group(0).strip()))

    print()
    print('通过 %d 帧，失败 %d 帧，跳过 %d 个不完整片段'
          % (ok_cnt, bad_cnt, frag_cnt))
    return 0 if bad_cnt == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
