#ifndef __PROTOCOL_FRAME_H
#define __PROTOCOL_FRAME_H

#include "main.h"
#include <stdint.h>

/*==============================================================================
 * 数据帧格式 (与 INFO/M1 测试治具PC端通讯协议.xlsx 一致)
 *
 *   +--------+---------+---------+---------+--------+----------+
 *   | 帧头 F | 命令 C  | 帧序号N | 长度 L  | 数据 D | CRC16    |
 *   | 2 字节 | 2 字节  | 2 字节  | 2 字节  | L 字节 | 2 字节   |
 *   | A5 5B  | uint16  | uint16  | uint16  |  *     | uint16   |
 *   +--------+---------+---------+---------+--------+----------+
 *
 * - 所有多字节字段一律大端 (高字节在前)
 * - CRC16 为 MODBUS 算法, 计算范围从帧头 F 到数据 D
 * - 帧序号: PC 发送从 0x01 递增, 到 0xFFFF 回到 1; 治具回 ACK 时原样回填;
 *           治具主动上报的帧序号固定为 0
 * - 单帧最短 10 字节 (L=0)
 *============================================================================*/

#define FRAME_HDR0        0xA5u
#define FRAME_HDR1        0x5Bu
#define FRAME_HDR_LEN     2u
#define FRAME_CMD_LEN     2u
#define FRAME_SEQ_LEN     2u
#define FRAME_LEN_LEN     2u
#define FRAME_CRC_LEN     2u
#define FRAME_OVERHEAD    (FRAME_HDR_LEN + FRAME_CMD_LEN + FRAME_SEQ_LEN + FRAME_LEN_LEN + FRAME_CRC_LEN) /* 10 */

#define FRAME_MAX_DATA    256u
#define FRAME_BUF_SIZE    (FRAME_OVERHEAD + FRAME_MAX_DATA)

/** @brief MODBUS CRC16 (多项式 0xA001 反射, 初值 0xFFFF) */
uint16_t Frame_CRC16(const uint8_t *data, uint16_t len);

/**
  * @brief 组帧
  * @param buf  输出缓冲, 至少需要 (FRAME_OVERHEAD + len) 字节
  * @return 整帧字节数
  */
uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     const uint8_t *data, uint16_t len);

/**
  * @brief 解析一帧 (要求 raw_len 恰好等于整帧长度)
  * @return 1=帧头/长度/CRC 全部正确, 0=失败
  */
uint8_t Frame_Parse(const uint8_t *raw, uint16_t raw_len,
                    uint16_t *cmd, uint16_t *seq,
                    uint8_t *data, uint16_t *len);

#endif /* __PROTOCOL_FRAME_H */
