# -*- coding: utf-8 -*-
"""生成 R1 治具 U2 的串口助手测试帧（发送帧 + 期望应答帧）。"""


def crc16(b):
    crc = 0xFFFF
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def hx(b):
    return ' '.join('%02X' % x for x in b)


def frame(cmd, seq, data=b''):
    f = bytes([0xA5, 0x5B,
               cmd >> 8, cmd & 0xFF,
               seq >> 8, seq & 0xFF,
               len(data) >> 8, len(data) & 0xFF]) + data
    c = crc16(f)
    return f + bytes([c >> 8, c & 0xFF])


SEQ = 0x0001

#  ---------- 发送帧 ----------
tx = {}
tx['PING'] = frame(0x0001, SEQ, b'')
tx['GET_VERSION'] = frame(0x0002, SEQ, b'')
tx['MOTOR_STATUS'] = frame(0x0012, SEQ, b'')
tx['MRE_QUERY'] = frame(0x0020, SEQ, b'')

# MOTOR_MOVE: dir(1) + steps(4, BE) + freq_hz(2, BE)
tx['MOVE 200步/1000Hz/正转'] = frame(0x0010, SEQ,
                                bytes([0x00, 0x00, 0x00, 0x00, 0xC8, 0x03, 0xE8]))
tx['MOVE 3200步/4000Hz/正转'] = frame(0x0010, SEQ,
                                 bytes([0x00, 0x00, 0x00, 0x0C, 0x80, 0x0F, 0xA0]))
tx['MOVE 3200步/4000Hz/反转'] = frame(0x0010, SEQ,
                                 bytes([0x01, 0x00, 0x00, 0x0C, 0x80, 0x0F, 0xA0]))
tx['MOTOR_STOP'] = frame(0x0011, SEQ, b'')

# 慢速长转: 用来「边转边查 MRE」。3200 步 @500Hz ≈ 6.4 秒一圈
tx['MOVE 3200步/500Hz/正转(6.4s)'] = frame(0x0010, SEQ,
                                    bytes([0x00, 0x00, 0x00, 0x0C, 0x80, 0x01, 0xF4]))
tx['MOVE 3200步/500Hz/反转(6.4s)'] = frame(0x0010, SEQ,
                                    bytes([0x01, 0x00, 0x00, 0x0C, 0x80, 0x01, 0xF4]))
tx['MOVE 6400步/800Hz/正转(8s)'] = frame(0x0010, SEQ,
                                   bytes([0x00, 0x00, 0x00, 0x19, 0x00, 0x03, 0x20]))

print('================ 发送帧 (串口助手 HEX 模式) ================')
for k, v in tx.items():
    print('%-26s %s' % (k, hx(v)))

#  ---------- 期望应答 ----------
print()
print('================ 期望应答 ================')

ack = {}
ack['PING'] = frame(0x0001, SEQ, bytes([0x00]))
ack['GET_VERSION'] = frame(0x0002, SEQ, bytes([0x00, 0x01, 0x00, 0x01]))
# 刚上电, 没发过 MOVE: running=0, done=0, target=0
ack['MOTOR_STATUS (刚上电)'] = frame(0x0012, SEQ,
                                  bytes([0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00]))
# 发过 3200 步之后, 运动结束: running=0, done=0x00000C80, target=0x00000C80
ack['MOTOR_STATUS (3200步走完)'] = frame(0x0012, SEQ,
                                     bytes([0x00, 0x00,
                                            0x00, 0x00, 0x0C, 0x80,
                                            0x00, 0x00, 0x0C, 0x80]))
ack['MOTOR_MOVE 成功'] = frame(0x0010, SEQ, bytes([0x00]))
ack['MOTOR_MOVE 忙(连发)'] = frame(0x0010, SEQ, bytes([0x01]))
ack['MOTOR_STOP'] = frame(0x0011, SEQ, bytes([0x00]))

for k, v in ack.items():
    print('%-26s %s' % (k, hx(v)))

# MRE_QUERY 应答: 前 2 字节固定, 中间是变量, 给个"典型值"骨架
t0 = 23800                       # 采样 1 秒后的 total_samples 典型值
payload = bytes([0x00, 0x03,          # 状态码, flags(bit0=ADC在跑,bit1=TMC正常)
                 0x08, 0x00,          # mre_last
                 0x00, 0x00,          # mre_min
                 0xFF, 0x0F,          # mre_max
                 0x80, 0x07,          # mre_mean
                 0x06, 0x06,          # mre_mv  (0x0800*3300/4095 ≈ 1650mV)
                 0x07, 0xFF,          # vref_last (≈2047)
                 0x06, 0x5F,          # vref_mv  (≈1631mV)
                 0x01, 0x00,          # window_size = 256
                 (t0 >> 24) & 0xFF, (t0 >> 16) & 0xFF, (t0 >> 8) & 0xFF, t0 & 0xFF])
print('%-26s %s' % ('MRE_QUERY 应答骨架', hx(frame(0x0020, SEQ, payload))))
print('   ^ 只有前 2 字节(status/flags)和 window_size=01 00 是固定的, 其余随采样变')

print()
print('================ MRE_QUERY 应答字段偏移 ================')
fields = [('0', 1, '状态码'), ('1', 1, 'flags'), ('2', 2, 'mre_last'),
          ('4', 2, 'mre_min'), ('6', 2, 'mre_max'), ('8', 2, 'mre_mean'),
          ('10', 2, 'mre_mv'), ('12', 2, 'vref_last'), ('14', 2, 'vref_mv'),
          ('16', 2, 'window_size'), ('18', 4, 'total_samples')]
print('  (偏移相对"数据 D"的第 0 字节, 即整帧第 8 字节)')
for off, ln, name in fields:
    print('  D[%-2s..%-2s] %-14s %d 字节' % (off, int(off) + ln - 1, name, ln))
