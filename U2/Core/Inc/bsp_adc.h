#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "main.h"
#include <stdint.h>

/*==============================================================================
 * MRE 磁编码信号连续采集 (ADC1 + DMA1_Channel1, 循环模式)
 *
 *   通道顺序: rank1 = IN0 (PA0 <- U7 运放 <- 板端 MRE-ADC)
 *             rank2 = IN4 (PA4 <- U5 运放 <- 3V3_ADC0 分压, 用于监视 3.3V 基准)
 *
 *   采样时间 239.5 周期 @ ADC 12MHz -> 单通道 21us, 双通道 42us
 *   -> 每通道约 23.8 kSa/s, 缓冲区 512 半字(256 组) -> 每组 10.7ms 刷新一次统计
 *
 *   统计在 DMA 半传输/传输完成中断里对刚满的那一半做, 主循环读到的永远是
 *   已经算完的整窗数据, 不存在 DMA 写到一半被读走的撕裂问题。
 *============================================================================*/

/* 每个完整窗口包含的 MRE 样本数 (也等于每通道的窗口长度) */
#define BSP_ADC_WINDOW_SAMPLES   256u

typedef struct {
    uint16_t mre_last;      /* 最近一次 MRE 原始值 (12bit) */
    uint16_t mre_min;       /* 最近一个完整窗口内的最小值 */
    uint16_t mre_max;       /* 最近一个完整窗口内的最大值 */
    uint16_t mre_mean;      /* 最近一个完整窗口内的平均值 */
    uint16_t vref_last;     /* 最近一次 3V3_ADC0 通道原始值 */
    uint16_t window_size;   /* 该窗口实际样本数 */
    uint16_t mre_mv;        /* mre_last 换算成 mV (按 VDDA=3300mV) */
    uint16_t vref_mv;       /* vref_last 换算成 mV */
    uint32_t total_samples; /* 上电以来累计的 MRE 样本数 */
} BSP_ADC_Snapshot_t;

/** @brief 配置并启动 ADC 连续转换 + DMA 循环搬运。上电调用一次。
 *         会覆盖 CubeMX 在 MX_ADC1_Init 里生成的单通道单次转换配置。 */
void     BSP_ADC_Init(void);

/** @brief 1=ADC/DMA 正在搬运 */
uint8_t  BSP_ADC_IsRunning(void);

/** @brief 取一份当前快照 (内部做临界区保护, 可在主循环任意位置调用) */
void     BSP_ADC_GetSnapshot(BSP_ADC_Snapshot_t *out);

/** @brief 原始值 -> mV, 使用 VDDA = 3300mV */
uint16_t BSP_ADC_RawToMv(uint16_t raw);

#endif /* __BSP_ADC_H */
