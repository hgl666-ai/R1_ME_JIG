/**
  ******************************************************************************
  * @file    protocol_frame.c
  * @brief   帧的 CRC / 组帧 / 解帧。算法与参考项目 (diameter_JIG/U7) 一致,
  *          现场已验证可与 PC 端互通。
  ******************************************************************************
  */

#include "protocol_frame.h"
#include <string.h>

uint16_t Frame_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8u; j++) {
            if (crc & 0x0001u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     const uint8_t *data, uint16_t len)
{
    uint16_t total;
    uint16_t crc;

    buf[0] = FRAME_HDR0;
    buf[1] = FRAME_HDR1;
    buf[2] = (uint8_t)(cmd >> 8);
    buf[3] = (uint8_t)(cmd);
    buf[4] = (uint8_t)(seq >> 8);
    buf[5] = (uint8_t)(seq);
    buf[6] = (uint8_t)(len >> 8);
    buf[7] = (uint8_t)(len);
    if ((len > 0u) && (data != NULL)) {
        memcpy(&buf[8], data, len);
    }

    total = (uint16_t)(8u + len);
    crc = Frame_CRC16(buf, total);
    buf[total]      = (uint8_t)(crc >> 8);
    buf[total + 1u] = (uint8_t)(crc);

    return (uint16_t)(total + 2u);
}

uint8_t Frame_Parse(const uint8_t *raw, uint16_t raw_len,
                    uint16_t *cmd, uint16_t *seq,
                    uint8_t *data, uint16_t *len)
{
    uint16_t dlen;
    uint16_t calc;
    uint16_t recv;

    if (raw_len < FRAME_OVERHEAD) {
        return 0u;
    }
    if ((raw[0] != FRAME_HDR0) || (raw[1] != FRAME_HDR1)) {
        return 0u;
    }

    dlen = (uint16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    if (dlen > FRAME_MAX_DATA) {
        return 0u;
    }
    if (raw_len != (uint16_t)(8u + dlen + 2u)) {
        return 0u;
    }

    calc = Frame_CRC16(raw, (uint16_t)(8u + dlen));
    recv = (uint16_t)(((uint16_t)raw[8u + dlen] << 8) | raw[9u + dlen]);
    if (calc != recv) {
        return 0u;
    }

    if (cmd != NULL) {
        *cmd = (uint16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    }
    if (seq != NULL) {
        *seq = (uint16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    }
    if ((data != NULL) && (dlen > 0u)) {
        memcpy(data, &raw[8], dlen);
    }
    if (len != NULL) {
        *len = dlen;
    }
    return 1u;
}
