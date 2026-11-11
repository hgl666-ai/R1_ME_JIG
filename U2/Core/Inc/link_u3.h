#ifndef __LINK_U3_H
#define __LINK_U3_H

#include "main.h"
#include <stdint.h>
#include "link_proto.h"

/*==============================================================================
 * U2 -> U3 内部链路：主机侧 API
 *
 * 线上契约（命令码 / 状态码 / 快照布局）在 link_proto.h，两个工程各一份副本，
 * 必须完全一致。本文件只放主机侧的调用接口。
 *============================================================================*/

/*===== LINK_Command 返回码 =====*/
#define LINK_OK             0u   /* 收到合法应答 */
#define LINK_ERR_TIMEOUT    1u   /* 无应答 / 应答不完整 */
#define LINK_ERR_FRAME      2u   /* 帧头、长度或 CRC 错 */
#define LINK_ERR_SEQ        3u   /* 序号不匹配（陈旧帧） */
#define LINK_ERR_SIZE       4u   /* 应答数据超出调用方缓冲 */

/*===== 超时与重试 =====*/
#define LINK_RX_TIMEOUT_MS  15u
#define LINK_TX_TIMEOUT_MS  20u
#define LINK_TRIES          2u

/** @brief 复位序号并清空接收。上电调用一次即可。 */
void    LINK_Init(void);

/**
  * @brief 发起一次请求/应答事务（阻塞，含超时与重试）
  * @param cmd     命令码
  * @param req     请求数据，可为 NULL
  * @param reqlen  请求数据长度
  * @param ack     应答数据输出缓冲
  * @param ackcap  ack 缓冲容量
  * @param acklen  实际应答数据长度输出，可为 NULL
  * @retval LINK_OK / LINK_ERR_*
  */
uint8_t LINK_Command(uint16_t cmd, const uint8_t *req, uint16_t reqlen,
                     uint8_t *ack, uint16_t ackcap, uint16_t *acklen);

#endif /* __LINK_U3_H */
