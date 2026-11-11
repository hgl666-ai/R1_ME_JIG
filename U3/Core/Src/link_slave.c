/**
  ******************************************************************************
  * @file    link_slave.c
  * @brief   U3 从机侧: 解与 PC 侧同款的帧格式, 应答 PING / GET_SNAP
  *
  * 接收: USART1 RXNE 中断把字节塞进环形缓冲 (单生产者 / 单消费者)
  * 处理: 主循环 LINK_Poll() 排空环形缓冲 -> 逐字节解帧 -> 派发 -> 回包
  ******************************************************************************
  */

#include "link_slave.h"
#include "protocol_frame.h"
#include "bsp_adc.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/*===== 接收环形缓冲: 中断写 head, 主循环写 tail =====*/
#define LINK_RX_RING        256u

static volatile uint8_t  s_ring[LINK_RX_RING];
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;

/*===== 解帧缓冲 =====*/
static uint8_t  s_rx[FRAME_BUF_SIZE];
static uint16_t s_rx_len = 0;

/* 解出来的数据区 (静态: 栈只有 1KB, 不能放 256 字节的局部数组) */
static uint8_t  s_rx_data[FRAME_MAX_DATA];

/*===== 发送缓冲: 必须在 HAL_UART_Transmit 返回前一直有效, 所以用静态区 =====*/
static uint8_t  s_tx[FRAME_BUF_SIZE];
static uint8_t  s_payload[FRAME_MAX_DATA];

/*==============================================================================
 * 接收中断
 *============================================================================*/
void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    uint8_t  b;
    uint16_t next;

    if (sr & USART_SR_RXNE) {
        b = (uint8_t)(USART1->DR & 0xFFu);
        next = (uint16_t)((s_head + 1u) % LINK_RX_RING);
        if (next != s_tail) {          /* 满了就丢新字节, 由上层超时兜底 */
            s_ring[s_head] = b;
            s_head = next;
        }
    } else if (sr & USART_SR_ORE) {
        b = (uint8_t)(USART1->DR & 0xFFu);   /* 读 SR+DR 清溢出 */
        (void)b;
    }
}

/*==============================================================================
 * 组包 / 发送
 *============================================================================*/
static void link_reply(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint16_t n = Frame_Build(s_tx, cmd, seq, data, len);

    (void)HAL_UART_Transmit(&huart1, s_tx, n, 20u);
}

/*==============================================================================
 * 命令派发
 *============================================================================*/
static void link_dispatch(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;

    switch (cmd) {

    case LINK_CMD_PING: {
        uint32_t up = HAL_GetTick();

        s_payload[0] = LINK_ST_OK;
        s_payload[1] = LINK_FW_MAJOR;
        s_payload[2] = LINK_FW_MINOR;
        s_payload[3] = LINK_FW_PATCH;
        s_payload[4] = (uint8_t)(up >> 24);
        s_payload[5] = (uint8_t)(up >> 16);
        s_payload[6] = (uint8_t)(up >> 8);
        s_payload[7] = (uint8_t)(up);
        link_reply(cmd, seq, s_payload, 8u);
        break;
    }

    case LINK_CMD_GET_SNAP: {
        BSP_ADC_Snapshot_t snap;
        LINK_Snap_t        ls;

        BSP_ADC_GetSnapshot(&snap);

        ls.flags         = (uint8_t)(BSP_ADC_IsRunning() ? 0x01u : 0x00u);
        ls.mre_last      = snap.mre_last;
        ls.mre_min       = snap.mre_min;
        ls.mre_max       = snap.mre_max;
        ls.mre_mean      = snap.mre_mean;
        ls.mre_mv        = snap.mre_mv;
        ls.vref_last     = snap.vref_last;
        ls.vref_mv       = snap.vref_mv;
        ls.window_size   = snap.window_size;
        ls.total_samples = snap.total_samples;

        s_payload[0] = LINK_ST_OK;
        LINK_SnapPack(&s_payload[1], &ls);
        link_reply(cmd, seq, s_payload, (uint16_t)(1u + LINK_SNAP_LEN));
        break;
    }

    default:
        s_payload[0] = LINK_ST_NIY;
        link_reply(cmd, seq, s_payload, 1u);
        break;
    }
}

/*==============================================================================
 * 解帧 (与 U2 侧 proto_feed 同一套逻辑)
 *============================================================================*/
static void link_feed(uint8_t b)
{
    uint16_t dlen;
    uint16_t need;

    if (s_rx_len == 0u) {
        if (b == FRAME_HDR0) {
            s_rx[s_rx_len++] = b;
        }
        return;
    }

    if (s_rx_len == 1u) {
        if (b == FRAME_HDR1) {
            s_rx[s_rx_len++] = b;
        } else if (b == FRAME_HDR0) {
            s_rx[0] = b;
        } else {
            s_rx_len = 0u;
        }
        return;
    }

    if (s_rx_len >= FRAME_BUF_SIZE) {
        s_rx_len = 0u;
        return;
    }
    s_rx[s_rx_len++] = b;

    if (s_rx_len < 8u) {
        return;
    }

    dlen = (uint16_t)(((uint16_t)s_rx[6] << 8) | s_rx[7]);
    if (dlen > FRAME_MAX_DATA) {
        s_rx_len = 0u;
        return;
    }
    need = (uint16_t)(FRAME_OVERHEAD + dlen);

    if (s_rx_len == need) {
        uint16_t cmd = 0u;
        uint16_t seq = 0u;
        uint16_t len = 0u;

        if (Frame_Parse(s_rx, need, &cmd, &seq, s_rx_data, &len)) {
            link_dispatch(cmd, seq, s_rx_data, len);
        }
        s_rx_len = 0u;
    } else if (s_rx_len > need) {
        s_rx_len = 0u;
    }
}

/*==============================================================================
 * 公开 API
 *============================================================================*/
void LINK_Init(void)
{
    s_head   = 0u;
    s_tail   = 0u;
    s_rx_len = 0u;

    /* 打开 USART1 的 RXNE 中断 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    HAL_NVIC_SetPriority(USART1_IRQn, 2u, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void LINK_Poll(void)
{
    uint8_t b;

    while (s_tail != s_head) {
        b = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) % LINK_RX_RING);
        link_feed(b);
    }
}
