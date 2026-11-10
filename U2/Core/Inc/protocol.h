#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include <stdint.h>

/*==============================================================================
 * 治具 <-> PC 命令层 (承载在 USB CDC 虚拟串口上)
 *
 * 帧格式见 protocol_frame.h。所有 ACK 的命令码与原命令一致, 帧序号原样回填;
 * 应答数据的第 1 字节固定是状态码 (见下方状态码表, 取自 M1 通讯协议 xlsx)。
 *============================================================================*/

/*===== 固件版本 =====*/
#define PROTO_FW_MAJOR      1u
#define PROTO_FW_MINOR      0u
#define PROTO_FW_PATCH      1u

/*===== 命令码 (2 字节) =====*/
#define PROTO_CMD_PING            0x0001u   /* 连通性测试, ACK 数据仅状态码 */
#define PROTO_CMD_GET_VERSION     0x0002u   /* ACK: 状态码 + 主/次/修订 */
#define PROTO_CMD_MOTOR_MOVE      0x0010u   /* dir(1)+steps(4)+freq_hz(2) */
#define PROTO_CMD_MOTOR_STOP      0x0011u   /* 立即停止 */
#define PROTO_CMD_MOTOR_STATUS    0x0012u   /* ACK: 状态码 + 运行标志(1) + 已走步数(4) + 目标步数(4) */
#define PROTO_CMD_MRE_QUERY       0x0020u   /* 查询最近一个 MRE 采样窗口 */

/*===== 通用状态码 (M1 通讯协议 xlsx 的【通用状态码】表) =====*/
#define PROTO_ST_OK            0x00u
#define PROTO_ST_BUSY          0x01u
#define PROTO_ST_ERR_SIZE      0x02u
#define PROTO_ST_ERR_FLASH     0x03u
#define PROTO_ST_ERR_PROG      0x04u
#define PROTO_ST_ERR_TEST      0x05u
#define PROTO_ST_ERR_NO_FW     0x06u
#define PROTO_ST_ERR_INCOMPLETE 0x07u
#define PROTO_ST_ERR_CRC       0x08u
#define PROTO_ST_ERR_SEQ       0x09u
#define PROTO_ST_ERR_COUNT     0x0Au
#define PROTO_ST_ERR_PARAM     0x0Bu
#define PROTO_ST_ERR_TIMEOUT   0x0Cu
#define PROTO_ST_ERR_OVERFLOW  0x0Du
#define PROTO_ST_NIY           0xFFu

/*===== 方向 =====*/
#define PROTO_DIR_CW       0u
#define PROTO_DIR_CCW      1u

/*===== MRE_QUERY 应答中 flags 的位定义 =====*/
#define PROTO_MRE_FLAG_ADC_RUNNING  0x01u   /* ADC/DMA 正在连续采集 */
#define PROTO_MRE_FLAG_TMC_READY    0x02u   /* TMC2209 配置写入并校验通过 */

/*===== API =====*/

/** @brief 复位协议层状态。上电调用一次。 */
void PROTO_Init(void);

/** @brief 主循环轮询: 从接收环形缓冲取字节, 解帧并处理。 */
void PROTO_Run(void);

/**
  * @brief 把 USB 收到的字节塞进环形缓冲。可在 USB 中断上下文调用。
  * @note  单生产者(中断)/单消费者(主循环) 模型, 不需要关中断。
  */
void PROTO_PushRx(const uint8_t *buf, uint16_t len);

/** @brief 当前接收环形缓冲里的字节数 (诊断用) */
uint16_t PROTO_RxPending(void);

#endif /* __PROTOCOL_H */
