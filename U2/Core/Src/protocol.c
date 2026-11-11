/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   命令分发 + USB CDC 收发
  ******************************************************************************
  */

#include "protocol.h"
#include "protocol_frame.h"
#include "bsp_motor.h"
#include "bsp_adc.h"
#include "bsp_tmc2209.h"
#include "link_u3.h"
#include "usbd_cdc_if.h"
#include <string.h>

/*===== 接收环形缓冲 (USB 中断写, 主循环读) =====*/
#define PROTO_RX_RING_SIZE      1024u

static volatile uint8_t  s_ring[PROTO_RX_RING_SIZE];
static volatile uint16_t s_ring_head = 0;   /* 只由中断上下文修改 */
static volatile uint16_t s_ring_tail = 0;   /* 只由主循环修改 */

/*===== 解帧缓冲 =====*/
static uint8_t  s_rx[FRAME_BUF_SIZE];
static uint16_t s_rx_len = 0;

/*===== 发送缓冲: 必须常驻, CDC_Transmit_FS 返回后 USB 仍在用它 =====*/
static uint8_t  s_tx[FRAME_BUF_SIZE];

/*===== 收/发用的数据区必须分开: 处理命令时 payload 已被应答覆盖 =====*/
static uint8_t  s_rx_data[FRAME_MAX_DATA];
static uint8_t  s_payload[FRAME_MAX_DATA];

/*==============================================================================
 * 小工具
 *============================================================================*/
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}

/*==============================================================================
 * 发送
 *============================================================================*/
static void proto_send(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint16_t n = Frame_Build(s_tx, cmd, seq, data, len);
    uint32_t t0;

    /* CDC 上一次发送还没完成时返回 USBD_BUSY; 短暂等待后重试。
     * 超时就丢弃这一帧 —— 治具是被动应答方, 丢一帧由 PC 侧重发更简单。 */
    t0 = HAL_GetTick();
    while (CDC_Transmit_FS(s_tx, n) == USBD_BUSY) {
        if ((HAL_GetTick() - t0) > 20u) {
            return;
        }
    }
}

/*==============================================================================
 * 命令处理
 *============================================================================*/
static void proto_dispatch(uint16_t cmd, uint16_t seq, const uint8_t *data, uint16_t len)
{
    uint16_t n;

    switch (cmd) {

    /*--- 连通性 ---*/
    case PROTO_CMD_PING:
        s_payload[0] = PROTO_ST_OK;
        proto_send(cmd, seq, s_payload, 1u);
        break;

    /*--- 版本 ---*/
    case PROTO_CMD_GET_VERSION:
        s_payload[0] = PROTO_ST_OK;
        s_payload[1] = PROTO_FW_MAJOR;
        s_payload[2] = PROTO_FW_MINOR;
        s_payload[3] = PROTO_FW_PATCH;
        proto_send(cmd, seq, s_payload, 4u);
        break;

    /*--- 电机: 定长运动 ---*/
    case PROTO_CMD_MOTOR_MOVE:
        if (len != 7u) {
            s_payload[0] = PROTO_ST_ERR_PARAM;
        } else if (data[0] > 1u) {
            s_payload[0] = PROTO_ST_ERR_PARAM;
        } else if (BSP_Motor_IsBusy()) {
            s_payload[0] = PROTO_ST_BUSY;
        } else if (!BSP_Motor_Move(data[0], get_u32(&data[1]), get_u16(&data[5]))) {
            s_payload[0] = PROTO_ST_ERR_PARAM;
        } else {
            s_payload[0] = PROTO_ST_OK;
        }
        proto_send(cmd, seq, s_payload, 1u);
        break;

    /*--- 电机: 停止 ---*/
    case PROTO_CMD_MOTOR_STOP:
        BSP_Motor_Stop();
        s_payload[0] = PROTO_ST_OK;
        proto_send(cmd, seq, s_payload, 1u);
        break;

    /*--- 电机: 运动状态 ---*/
    case PROTO_CMD_MOTOR_STATUS:
        s_payload[0] = PROTO_ST_OK;
        s_payload[1] = BSP_Motor_IsBusy();
        put_u32(&s_payload[2], BSP_Motor_GetDone());
        put_u32(&s_payload[6], BSP_Motor_GetTarget());
        proto_send(cmd, seq, s_payload, 10u);
        break;

    /*--- MRE 采样窗口查询 ---*/
    case PROTO_CMD_MRE_QUERY: {
        BSP_ADC_Snapshot_t snap;

        BSP_ADC_GetSnapshot(&snap);

        n = 0;
        s_payload[n++] = PROTO_ST_OK;
        s_payload[n++] = (uint8_t)((BSP_ADC_IsRunning() ? PROTO_MRE_FLAG_ADC_RUNNING : 0u) |
                                   (BSP_TMC_IsReady()   ? PROTO_MRE_FLAG_TMC_READY   : 0u));
        put_u16(&s_payload[n], snap.mre_last);      n += 2u;
        put_u16(&s_payload[n], snap.mre_min);       n += 2u;
        put_u16(&s_payload[n], snap.mre_max);       n += 2u;
        put_u16(&s_payload[n], snap.mre_mean);      n += 2u;
        put_u16(&s_payload[n], snap.mre_mv);        n += 2u;
        put_u16(&s_payload[n], snap.vref_last);     n += 2u;
        put_u16(&s_payload[n], snap.vref_mv);       n += 2u;
        put_u16(&s_payload[n], snap.window_size);   n += 2u;
        put_u32(&s_payload[n], snap.total_samples); n += 4u;
        proto_send(cmd, seq, s_payload, n);
        break;
    }

    /*--- MRE 整圈极值 (低于下限 / 高于上限即不合格) ---*/
    case PROTO_CMD_MRE_PEAK: {
        uint16_t pmn = 0u;
        uint16_t pmx = 0u;
        uint32_t pcnt = 0u;
        uint8_t  status = PROTO_ST_OK;
        uint8_t  measured = 0u;     /* 1 = 已经取到极值, 需要判定 */

        if ((len != 1u) && (len != 7u)) {
            status = PROTO_ST_ERR_PARAM;
        } else {
            uint8_t mode = data[0];

            if (mode > PROTO_MRE_PEAK_MODE_SELFTEST) {
                status = PROTO_ST_ERR_PARAM;

            } else if (mode == PROTO_MRE_PEAK_MODE_SELFTEST) {
                /* 清零 -> 转一圈 -> 读结果, 一条命令搞定 */
                uint32_t steps = (len == 7u) ? get_u32(&data[1]) : PROTO_MRE_PEAK_STEPS;
                uint32_t freq  = (len == 7u) ? get_u16(&data[5]) : PROTO_MRE_PEAK_FREQ;

                if ((steps == 0u) || (steps > PROTO_MRE_PEAK_STEPS_MAX)) {
                    status = PROTO_ST_ERR_PARAM;
                } else if (BSP_Motor_IsBusy()) {
                    status = PROTO_ST_BUSY;
                } else {
                    uint32_t t_ms;
                    uint32_t t0;
                    uint8_t  timedout = 0u;

                    BSP_ADC_GetPeak(NULL, NULL, NULL, 1u);      /* 清零 */

                    if (!BSP_Motor_Move(PROTO_DIR_CW, steps, freq)) {
                        status = PROTO_ST_ERR_PARAM;
                    } else {
                        /* 超时 = 理论耗时 x2 + 1s, 覆盖加减速段 */
                        t_ms = (uint32_t)(((uint64_t)steps * 1000u) / (uint64_t)freq) * 2u + 1000u;
                        t0 = HAL_GetTick();

                        while (BSP_Motor_IsBusy()) {
                            if ((HAL_GetTick() - t0) > t_ms) {
                                BSP_Motor_Stop();
                                timedout = 1u;
                                break;
                            }
                        }

                        HAL_Delay(25u);     /* 等最后一个采样窗口算完, 尾部不丢 */
                        BSP_ADC_GetPeak(&pmn, &pmx, &pcnt, 0u);
                        measured = 1u;

                        if (timedout != 0u) {
                            status = PROTO_ST_ERR_TIMEOUT;
                        }
                    }
                }

            } else {
                /* 只读 / 读并清零 */
                BSP_ADC_GetPeak(&pmn, &pmx, &pcnt,
                                (uint8_t)(mode == PROTO_MRE_PEAK_MODE_CLEAR ? 1u : 0u));
                measured = 1u;
            }
        }

        /* 判定: 整圈最低值低于下限, 或最高值高于上限 -> 不合格 */
        if ((measured != 0u) && (status == PROTO_ST_OK) && (pcnt != 0u) &&
            ((pmn < PROTO_MRE_PEAK_LO) || (pmx > PROTO_MRE_PEAK_HI))) {
            status = PROTO_ST_ERR_TEST;
        }

        n = 0;
        s_payload[n++] = status;
        put_u16(&s_payload[n], pmn);  n += 2u;
        put_u16(&s_payload[n], pmx);  n += 2u;
        put_u32(&s_payload[n], pcnt); n += 4u;
        proto_send(cmd, seq, s_payload, n);
        break;
    }

    /*--- U2/U3 交叉校验 ---*/
    case PROTO_CMD_CROSS_CHECK: {
        BSP_ADC_Snapshot_t u2adc;
        LINK_Snap_t u2, u3;
        uint8_t  ackbuf[40];
        uint16_t acklen = 0;
        uint8_t  rc;
        uint8_t  peer = 0;
        uint8_t  status;

        /* 取本机快照 */
        BSP_ADC_GetSnapshot(&u2adc);
        u2.flags         = (uint8_t)(BSP_ADC_IsRunning() ? 0x01u : 0x00u);
        u2.mre_last      = u2adc.mre_last;
        u2.mre_min       = u2adc.mre_min;
        u2.mre_max       = u2adc.mre_max;
        u2.mre_mean      = u2adc.mre_mean;
        u2.mre_mv        = u2adc.mre_mv;
        u2.vref_last     = u2adc.vref_last;
        u2.vref_mv       = u2adc.vref_mv;
        u2.window_size   = u2adc.window_size;
        u2.total_samples = u2adc.total_samples;

        memset(&u3, 0, sizeof(u3));

        /* 拉对端快照 */
        rc = LINK_Command(LINK_CMD_GET_SNAP, NULL, 0u, ackbuf, sizeof(ackbuf), &acklen);

        if ((rc != LINK_OK) || (acklen < (uint16_t)(1u + LINK_SNAP_LEN)) ||
            (ackbuf[0] != LINK_ST_OK)) {
            status = PROTO_ST_ERR_TIMEOUT;      /* U3 不在线/应答不合法 */
        } else {
            LINK_SnapUnpack(&u3, &ackbuf[1]);

            peer = PROTO_CROSS_PEER_ALIVE;
            if (u3.flags & 0x01u) {
                peer |= PROTO_CROSS_PEER_ADC;
            }

            {
                int32_t d_mre  = (int32_t)u2.mre_last  - (int32_t)u3.mre_last;
                int32_t d_vref = (int32_t)u2.vref_last - (int32_t)u3.vref_last;

                if (d_mre  < 0) { d_mre  = -d_mre;  }
                if (d_vref < 0) { d_vref = -d_vref; }

                if ((d_mre  > (int32_t)PROTO_CROSS_TOL_MRE_LSB) ||
                    (d_vref > (int32_t)PROTO_CROSS_TOL_VREF_LSB)) {
                    peer  |= PROTO_CROSS_MISMATCH;
                    status = PROTO_ST_ERR_TEST;
                } else {
                    status = PROTO_ST_OK;
                }
            }
        }

        n = 0;
        s_payload[n++] = status;
        s_payload[n++] = peer;
        LINK_SnapPack(&s_payload[n], &u2); n += (uint16_t)LINK_SNAP_LEN;
        LINK_SnapPack(&s_payload[n], &u3); n += (uint16_t)LINK_SNAP_LEN;
        proto_send(cmd, seq, s_payload, n);
        break;
    }

    default:
        s_payload[0] = PROTO_ST_NIY;
        proto_send(cmd, seq, s_payload, 1u);
        break;
    }
}

/*==============================================================================
 * 解帧
 *============================================================================*/
static void proto_feed(uint8_t b)
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
            s_rx[0] = b;                    /* 连续两个 A5, 重新对齐 */
            s_rx_len = 1u;
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

    dlen = get_u16(&s_rx[6]);
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
            proto_dispatch(cmd, seq, s_rx_data, len);
        }
        /* CRC 错帧: 按协议只做诊断, 不回 ACK, 直接丢弃 */
        s_rx_len = 0u;
    } else if (s_rx_len > need) {
        s_rx_len = 0u;
    }
}

/*==============================================================================
 * 公开 API
 *============================================================================*/
void PROTO_Init(void)
{
    s_ring_head = 0u;
    s_ring_tail = 0u;
    s_rx_len    = 0u;
    memset(s_tx, 0, sizeof(s_tx));
    memset(s_rx_data, 0, sizeof(s_rx_data));
    memset(s_payload, 0, sizeof(s_payload));
}

void PROTO_PushRx(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t next;

    if (buf == NULL) {
        return;
    }

    for (i = 0; i < len; i++) {
        next = (uint16_t)((s_ring_head + 1u) % PROTO_RX_RING_SIZE);
        if (next == s_ring_tail) {
            break;                  /* 缓冲满, 丢弃后续字节 */
        }
        s_ring[s_ring_head] = buf[i];
        s_ring_head = next;
    }
}

void PROTO_Run(void)
{
    uint8_t b;

    while (s_ring_tail != s_ring_head) {
        b = s_ring[s_ring_tail];
        s_ring_tail = (uint16_t)((s_ring_tail + 1u) % PROTO_RX_RING_SIZE);
        proto_feed(b);
    }
}

uint16_t PROTO_RxPending(void)
{
    return (uint16_t)((s_ring_head + PROTO_RX_RING_SIZE - s_ring_tail) % PROTO_RX_RING_SIZE);
}
