#ifndef __LINK_SLAVE_H
#define __LINK_SLAVE_H

#include "main.h"
#include <stdint.h>
#include "link_proto.h"

/*==============================================================================
 * U3 从机: U2 <-> U3 内部链路 (USART1: PA9=TX / PA10=RX, 115200 8N1)
 *
 * 线上契约（命令码 / 状态码 / 快照布局）在 link_proto.h，两个工程各一份副本，
 * 必须完全一致。本文件只放从机侧的驱动接口。
 *
 * 接收走 RXNE 中断进环形缓冲，主循环里 LINK_Poll() 解帧并处理。
 * 这样即使正在发送应答（阻塞约 3ms），后续字节也不会丢。
 *
 * 物理连接: H3 排针，U2_USART1 与 U3_USART1 交叉相接 (TX<->RX)。
 *           H3 上必须插 1-2、3-4 两个短接帽，否则链路不通。
 *============================================================================*/

/** @brief 初始化: 打开 USART1 的 RXNE 中断 + NVIC。上电调用一次。 */
void LINK_Init(void);

/** @brief 主循环轮询: 从环形缓冲取字节，解帧，处理，回包。 */
void LINK_Poll(void);

#endif /* __LINK_SLAVE_H */
