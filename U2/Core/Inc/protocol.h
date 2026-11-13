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
#define PROTO_FW_PATCH      2u

/*===== 命令码 (2 字节) =====*/
#define PROTO_CMD_PING            0x0001u   /* 连通性测试, ACK 数据仅状态码 */
#define PROTO_CMD_GET_VERSION     0x0002u   /* ACK: 状态码 + 主/次/修订 */
#define PROTO_CMD_MOTOR_MOVE      0x0010u   /* dir(1)+steps(4)+freq_hz(2) */
#define PROTO_CMD_MOTOR_STOP      0x0011u   /* 立即停止 */
#define PROTO_CMD_MOTOR_STATUS    0x0012u   /* ACK: 状态码 + 运行标志(1) + 已走步数(4) + 目标步数(4) */
#define PROTO_CMD_MRE_QUERY       0x0020u   /* 查询最近一个 MRE 采样窗口 */
#define PROTO_CMD_CROSS_CHECK     0x0021u   /* U2/U3 交叉校验: 拉 U3 快照并比对 */
#define PROTO_CMD_MRE_PEAK        0x0022u
#define PROTO_CMD_WAVE_CAP        0x0023u   /* 波形抓取：一次传回 120 点原始值 */

/*===== 波形抓取命令的 mode =====*/
#define PROTO_WAVE_MODE_STATIC    0u    /* 不转，抓 120 点静态波形（看噪声底） */
#define PROTO_WAVE_MODE_SPIN      1u    /* 正转 1 圈，抓 120 点 */   /* 整圈极值: 低于下限或高于上限即判不合格 */

/*===== MRE 整圈极值判据 (单位 ADC LSB, 1 LSB ≈ 0.806mV) =====
 * 判法: 转完一圈后, 若 peak_min < LO 或 peak_max > HI → 状态码回 05 ERR_TEST。
 * 当前是宽区间 (便于联调重测), 定产线阈值时再按 DUT 实测范围往里收。 */
#define PROTO_MRE_PEAK_LO         0u        /* 0 = 关闭。当前 MRE 负半周被钳在 0, peak_min 恒为 0,
                                             下限判据没有意义, 判定改用周期数 (见下面 EDGE_PER_REV)。 */
#define PROTO_MRE_PEAK_HI         0u        /* 0 = 关闭。U7(LM358,3.3V) 输出上限约 2620,
                                             当前信号峰值约 2349, 上限判据暂无触发条件。 */

/*===== 周期数判据 (磁编码器一圈的周期数, 即极对数) =====
 * 0 = 未标定, 跳过周期判定。
 * 标定方法: 发 mode=02 转一圈, 读到的 edge_count 就是 N (连测三次确认)。
 * 填好之后: mode=02 转 1 圈要求 edge==N, mode=03 转 3 圈要求 edge==3N。 */
/*===== 整圈最低值的上限 (信号必须扫得下去) =====
 * 2026-11-13 实测:
 *   装磁编码器  peak_min = 0 / 0        (信号被钳到 0, 说明真的在扫)
 *   拆磁编码器  peak_min = 1678 / 1764  (困在高位, 不扫)
 * 差 1678 LSB, 两边都极稳。取 800 作门槛。
 * 0 = 关闭这条判据。 */
#define PROTO_MRE_PEAK_MIN_CEIL   800u

/*===== 峰峰值下限 (信号必须扫得开) =====
 * 2026-11-13 15 组实测:
 *   装磁编码器  max-min = 2104 ~ 2598   (13 组实测)
 *   拿走磁编码器 max-min = 1099 ~ 1391
 * 两者不重叠。门槛取两侧中点的低位 1600, 保证好板和坏件各留 500 左右余量。
 * 这条还能顺带抓住信号被卡死在某个值的坏件(卡死时峰峰值接近 0)。
 * 0 = 关闭这条判据。 */
#define PROTO_MRE_PP_MIN          1600u

/*===== 周期数判据 =====
 * N = 编码器一圈的周期数（极对数），2026-11-13 标定为 2。
 * 用途: 抓幅度正常但极对数不对的坏件 —— 幅度判据看不出来, 只有这条能抓。
 *
 * TOL 取 1: 接受 5~7。注意 20 次实测里有 5 次落在 4 或 8（见下方记录），
 * 这些会被判 FAIL —— 这是已知的误判来源，要压下去得先解决边沿计数的稳定性。
 * 判别力仍然够: 极对数 1 给 3、极对数 3 给 9, 都在 4~8 之外。 */
#define PROTO_MRE_EDGE_PER_REV    2u
#define PROTO_MRE_EDGE_TOL        1u

/*===== MRE_PEAK 请求里的 mode =====*/
#define PROTO_MRE_PEAK_MODE_READ      0u   /* 只读 */
#define PROTO_MRE_PEAK_MODE_CLEAR     1u   /* 读并清零 */
#define PROTO_MRE_PEAK_MODE_SELFTEST  2u   /* 自动测单圈: 清零 -> 正转 1 圈 -> 读结果 */
#define PROTO_MRE_PEAK_MODE_SELFTEST3 3u   /* 自动测三圈: 清零 -> 正转 2 圈 -> 反转 1 圈 -> 读结果 */

/*===== 自动测试的默认参数 (请求 LEN=1 时生效) =====*/
#define PROTO_MRE_PEAK_STEPS         3200u /* 16 微步下 3200 步 = 1 圈 */
#define PROTO_MRE_PEAK_FREQ          4000u /* 4000Hz -> 单圈约 1.9 秒 (含加减速) */
#define PROTO_MRE_PEAK_STEPS_MAX     100000u

/*===== 两段运动之间的停顿 (ms) =====
 * 正转结束到反转开始之间停一下, 让轴停稳再反向。
 * 目的是压掉机械回弹在阈值附近多穿/少穿造成的 ±2 边沿计数误差。
 * 0 = 不停顿。 */
#define PROTO_MRE_SEG_DWELL_MS       100u

/*===== 交叉校验判据 (单位: ADC LSB, 1 LSB ≈ 0.806mV) =====
 * 两路缓冲接同一个输入节点、同一颗 LM358 的两个半边, 差异主要来自
 * 运放失调、两片各自的 ADC 与 VDDA。给足余量, 只抓"明显不一致"。 */
#define PROTO_CROSS_TOL_MRE_LSB   50u    /* MRE 读数差 > 50 LSB (≈40mV) 判超差 */
#define PROTO_CROSS_TOL_VREF_LSB  20u    /* 3.3V 基准读数差 > 20 LSB 判超差 */

/*===== CROSS_CHECK 应答中 peer_flags 的位定义 =====*/
#define PROTO_CROSS_PEER_ALIVE    0x01u   /* U3 有合法应答 */
#define PROTO_CROSS_PEER_ADC      0x02u   /* U3 的 ADC 在采集 */
#define PROTO_CROSS_MISMATCH      0x04u   /* 超出判据 */

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
