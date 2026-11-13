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

/*===== 周期计数 (过阈值上升沿, 带迟滞) =====
 * 用途: 数一圈里 MRE 波形走了几个周期。
 * 当信号负半周被钳在 0 时, 正半周仍然完整, 所以数沿不受影响。
 * 阈值是 ADC 原始值, 按实际波形调; 400 LSB 的迟滞用来抑制噪声重复计数。 */
/* 2026-11-13 实测对比:
 *   1400/1000 -> 三圈 edges = 6 6 7 5 5 6 (全部在 5~7)
 *   1000/ 500 -> 三圈 edges = 8 5 5 3 3   (噪声开始参与, 反而更差)
 * 所以保持高阈值。 */
#define BSP_ADC_EDGE_HIGH        1400u   /* 超过此值计一个上升沿 (≈1128mV) */
#define BSP_ADC_EDGE_LOW         1000u   /* 回落到此值以下才允许下次计数 (≈805mV) */

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

/**
  * @brief 取"自上次清零以来"累计的极值与周期数
  * @param mn     输出: 最低原始值
  * @param mx     输出: 最高原始值
  * @param cnt    输出: 累计样本数 (0 表示清零后还没采到数据)
  * @param edges  输出: 过阈值上升沿个数 (即波形走过几个周期)
  * @param clear  1 = 读完立即清零, 0 = 只读
  * @note  可以只传关心的指针, 其余传 NULL
  */
void     BSP_ADC_GetPeak(uint16_t *mn, uint16_t *mx, uint32_t *cnt,
                         uint16_t *edges, uint8_t clear);

/** @brief 原始值 -> mV, 使用 VDDA = 3300mV */
uint16_t BSP_ADC_RawToMv(uint16_t raw);

/*===== 波形抓取 =====
 * 每隔 div 个统计窗口(每个 10.75ms)把该窗口的 mre_mean 存进缓冲, 用于把真实
 * 波形一次性传回 PC。抓满 BSP_ADC_CAP_POINTS 点后自动停止。 */
#define BSP_ADC_CAP_POINTS       120u

/** @brief 开始抓波形, div = 每几个窗口存一个 (0 当作 1) */
void            BSP_ADC_CaptureStart(uint8_t div);

/** @brief 停止抓取, 返回已抓到的点数 */
uint16_t        BSP_ADC_CaptureStop(void);

/** @brief 指向抓取缓冲 (只读), 点数由 CaptureStop 给出 */
const uint16_t *BSP_ADC_CaptureBuf(void);

#endif /* __BSP_ADC_H */
