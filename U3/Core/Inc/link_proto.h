#ifndef __LINK_PROTO_H
#define __LINK_PROTO_H

#include <stdint.h>

/*==============================================================================
 * U2 <-> U3 内部链路的【线上契约】
 *
 * 本文件在 U2 和 U3 两个工程里各放一份**完全相同**的副本。
 * 命令码、状态码、版本号、快照布局、打包顺序全部集中在这里，
 * 只要两边这份文件一致，链路就不会错位。
 *
 * 帧格式与 PC 链路完全一致: A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16
 *   - 编解码复用 protocol_frame.c
 *   - 命令码占 0x01xx 段，与 PC 侧 0x00xx 段分开
 *
 * 物理层: USART1 (PA9=TX / PA10=RX) 115200 8N1
 *         H3 排针必须插 1-2 与 3-4 两个短接帽，否则链路不通。
 *============================================================================*/

/*===== 命令 (只有两条，够用就好) =====*/
#define LINK_CMD_PING       0x0101u   /* REQ L=0 -> ACK L=8 : 状态1 + 版本3 + 运行毫秒4 */
#define LINK_CMD_GET_SNAP   0x0102u   /* REQ L=0 -> ACK L=22: 状态1 + 快照21 */

/*===== 应答数据第 0 字节的状态码 =====*/
#define LINK_ST_OK          0x00u
#define LINK_ST_NIY         0xFFu

/*===== 固件版本：两侧必须一致，不一致说明有一边没重烧 =====*/
#define LINK_FW_MAJOR       1u
#define LINK_FW_MINOR       0u
#define LINK_FW_PATCH       0u

/*===== 快照：线上 21 字节，顺序与 PC 侧 MRE_QUERY 的 D[1..21] 完全相同 =====*/
#define LINK_SNAP_LEN       21u

typedef struct {
    uint8_t  flags;         /* bit0 = ADC/DMA 正在采集 */
    uint16_t mre_last;
    uint16_t mre_min;
    uint16_t mre_max;
    uint16_t mre_mean;
    uint16_t mre_mv;
    uint16_t vref_last;
    uint16_t vref_mv;
    uint16_t window_size;
    uint32_t total_samples;
} LINK_Snap_t;

/** @brief 快照 -> 21 字节线上格式 (大端) */
void LINK_SnapPack(uint8_t *out, const LINK_Snap_t *s);

/** @brief 21 字节线上格式 -> 快照 */
void LINK_SnapUnpack(LINK_Snap_t *s, const uint8_t *in);

#endif /* __LINK_PROTO_H */
