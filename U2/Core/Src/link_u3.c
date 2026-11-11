/**
  ******************************************************************************
  * @file    link_u3.c
  * @brief   U2 -> U3 内部链路主机侧 (USART1)
  *
  * 一次事务: 建帧 -> 清 RX -> 发送 -> 收 8 字节头 -> 收 数据+CRC -> 整帧校验
  *           校验项: 帧头 / 长度 / CRC16 / 命令回显 / 序号回显
  * 失败按 LINK_TRIES 重试; 每次重试序号都递增, 陈旧帧会被序号比对挡掉。
  ******************************************************************************
  */

#include "link_u3.h"
#include "protocol_frame.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

static uint16_t s_seq = 0;

/*==============================================================================
 * 工具
 *============================================================================*/
static void link_flush_rx(void)
{
    volatile uint32_t dump;

    while (USART1->SR & USART_SR_RXNE) {
        dump = USART1->DR;
        (void)dump;
    }
    if (USART1->SR & USART_SR_ORE) {
        dump = USART1->SR;
        dump = USART1->DR;
        (void)dump;
    }
}

void LINK_Init(void)
{
    s_seq = 0;
    link_flush_rx();
}

/*==============================================================================
 * 一次请求/应答
 *============================================================================*/
uint8_t LINK_Command(uint16_t cmd, const uint8_t *req, uint16_t reqlen,
                     uint8_t *ack, uint16_t ackcap, uint16_t *acklen)
{
    uint8_t  tx[FRAME_BUF_SIZE];
    uint8_t  rx[FRAME_BUF_SIZE];
    uint8_t  hdr[8];
    uint16_t flen;
    uint16_t dlen;
    uint16_t rcmd;
    uint16_t rseq;
    uint16_t rlen;
    uint8_t  err = LINK_ERR_TIMEOUT;
    uint8_t  attempt;

    if (acklen != NULL) {
        *acklen = 0;
    }

    for (attempt = 0; attempt < LINK_TRIES; attempt++) {

        s_seq++;
        if (s_seq == 0u) {
            s_seq = 1u;
        }

        flen = Frame_Build(tx, cmd, s_seq, req, reqlen);

        link_flush_rx();
        if (HAL_UART_Transmit(&huart1, tx, flen, LINK_TX_TIMEOUT_MS) != HAL_OK) {
            err = LINK_ERR_TIMEOUT;
            continue;
        }

        /* 8 字节头 */
        if (HAL_UART_Receive(&huart1, hdr, 8u, LINK_RX_TIMEOUT_MS) != HAL_OK) {
            err = LINK_ERR_TIMEOUT;
            continue;
        }
        if ((hdr[0] != FRAME_HDR0) || (hdr[1] != FRAME_HDR1)) {
            err = LINK_ERR_FRAME;
            continue;
        }

        dlen = (uint16_t)(((uint16_t)hdr[6] << 8) | hdr[7]);
        if (dlen > FRAME_MAX_DATA) {
            err = LINK_ERR_FRAME;
            continue;
        }
        if (dlen > ackcap) {
            err = LINK_ERR_SIZE;
            continue;
        }

        /* 数据 + CRC */
        if (HAL_UART_Receive(&huart1, &rx[8], (uint16_t)(dlen + 2u),
                             LINK_RX_TIMEOUT_MS) != HAL_OK) {
            err = LINK_ERR_TIMEOUT;
            continue;
        }
        memcpy(rx, hdr, 8u);

        if (!Frame_Parse(rx, (uint16_t)(10u + dlen), &rcmd, &rseq, ack, &rlen)) {
            err = LINK_ERR_FRAME;
            continue;
        }
        if (rcmd != cmd) {
            err = LINK_ERR_FRAME;
            continue;
        }
        if (rseq != s_seq) {
            err = LINK_ERR_SEQ;         /* 上一次重试的迟到应答，丢掉重来 */
            continue;
        }

        if (acklen != NULL) {
            *acklen = rlen;
        }
        return LINK_OK;
    }

    return err;
}
