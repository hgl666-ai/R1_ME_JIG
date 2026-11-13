/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   MRE 磁编码信号连续采集 (ADC1 扫描 + DMA 循环)
  *
  * CubeMX 生成的 adc.c 只配了「单通道 / 单次转换 / 软件触发」, 满足不了
  * "连续采样" 的需求。这里在 MX_ADC1_Init() 之后重新把 ADC1 配成
  * 「2 通道扫描 + 连续转换 + DMA 循环」, 并接管 DMA1_Channel1 及其中断。
  ******************************************************************************
  */

#include "bsp_adc.h"
#include "adc.h"

extern ADC_HandleTypeDef hadc1;

/*===== 通道分配 =====*/
#define ADC_CH_MRE              ADC_CHANNEL_0   /* PA0 <- U7 运放 <- 板端 MRE-ADC */
#define ADC_CH_VREF             ADC_CHANNEL_4   /* PA4 <- U5 运放 <- 3V3_ADC0    */

/* 一个统计窗口 = BSP_ADC_WINDOW_SAMPLES 组 (每组 2 个半字: MRE + VREF)
 * DMA 缓冲区容纳两个窗口, 半传输/传输完成各对应一个完整窗口 */
#define ADC_HALF_HALFWORDS      (BSP_ADC_WINDOW_SAMPLES * 2u)
#define ADC_BUF_HALFWORDS       (ADC_HALF_HALFWORDS * 2u)

/*===== 中断优先级 =====*/
#define ADC_DMA_IRQ_PRIORITY    1u

#define ADC_VDDA_MV             3300u
#define ADC_FULL_SCALE          4095u

static DMA_HandleTypeDef s_hdma_adc1;
static uint16_t          s_adc_buf[ADC_BUF_HALFWORDS];
static volatile uint8_t  s_running = 0;

/* 当前统计窗口 (由 DMA 中断更新, 主循环加临界区读取) */
static volatile BSP_ADC_Snapshot_t s_snap;

/* 自清零以来累计的极值: 判"整圈有没有低于下限 / 高于上限"用 */
static volatile uint16_t s_peak_min = 0xFFFFu;
static volatile uint16_t s_peak_max = 0u;
static volatile uint32_t s_peak_cnt = 0u;

/* 周期计数: 过阈值上升沿 (带迟滞)。
 * armed=1 时等信号冲上 EDGE_HIGH 计一次; 计完后要回落到 EDGE_LOW 以下才重新武装,
 * 这样噪声不会在阈值附近反复触发。状态跨窗口保留, 所以窗口边界不会漏计数。 */
static volatile uint16_t s_edge_cnt   = 0u;
static volatile uint8_t  s_edge_armed = 0u;

/* 波形抓取缓冲 */
static uint16_t          s_cap_buf[BSP_ADC_CAP_POINTS];
static volatile uint16_t s_cap_cnt  = 0u;
static volatile uint8_t  s_cap_on   = 0u;
static volatile uint8_t  s_cap_div  = 1u;
static volatile uint8_t  s_cap_skip = 0u;

uint16_t BSP_ADC_RawToMv(uint16_t raw)
{
    /* mV = raw * 3300 / 4095, 用 32 位中间量避免溢出 */
    return (uint16_t)(((uint32_t)raw * ADC_VDDA_MV) / ADC_FULL_SCALE);
}

/*==============================================================================
 * 对 "刚写满的那一半" 做统计
 * @param start  起始下标 (半字为单位)
 * @param count  该段的半字数 (偶数)
 *============================================================================*/
static void adc_accumulate(uint16_t start, uint16_t count)
{
    uint32_t sum = 0;
    uint16_t n   = 0;
    uint16_t i;
    uint16_t mn = 0xFFFFu;
    uint16_t mx = 0u;
    uint16_t vref = 0u;

    for (i = 0; i < count; i += 2u) {
        uint16_t mre = s_adc_buf[start + i];

        vref = s_adc_buf[start + i + 1u];

        if (mre < mn) { mn = mre; }
        if (mre > mx) { mx = mre; }
        sum += mre;
        n++;

        /* 过阈值上升沿计数 (带迟滞) */
        if (s_edge_armed != 0u) {
            if (mre >= BSP_ADC_EDGE_HIGH) {
                if (s_edge_cnt < 0xFFFFu) {
                    s_edge_cnt++;
                }
                s_edge_armed = 0u;
            }
        } else if (mre <= BSP_ADC_EDGE_LOW) {
            s_edge_armed = 1u;
        }
    }

    if (n == 0u) {
        return;
    }

    s_snap.mre_last      = s_adc_buf[start + count - 2u];
    s_snap.vref_last     = vref;
    s_snap.mre_min       = mn;
    s_snap.mre_max       = mx;
    s_snap.mre_mean      = (uint16_t)(sum / n);
    s_snap.window_size   = n;
    s_snap.mre_mv        = BSP_ADC_RawToMv(s_adc_buf[start + count - 2u]);
    s_snap.vref_mv       = BSP_ADC_RawToMv(vref);
    s_snap.total_samples = s_snap.total_samples + n;

    /* 跨窗口累计极值: 把一个窗口的 mn/mx 并进全局极值 */
    if (mn < s_peak_min) { s_peak_min = mn; }
    if (mx > s_peak_max) { s_peak_max = mx; }
    s_peak_cnt += n;

    /* 波形抓取: 每 div 个窗口存一个 mre_mean */
    if (s_cap_on != 0u) {
        if (s_cap_skip == 0u) {
            if (s_cap_cnt < BSP_ADC_CAP_POINTS) {
                s_cap_buf[s_cap_cnt] = s_snap.mre_mean;
                s_cap_cnt++;
                s_cap_skip = (uint8_t)(s_cap_div - 1u);
            } else {
                s_cap_on = 0u;      /* 抓满 */
            }
        } else {
            s_cap_skip--;
        }
    }
}

/*==============================================================================
 * HAL 回调 (DMA1_Channel1 中断 -> HAL_DMA_IRQHandler -> ADC 回调链)
 *============================================================================*/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_accumulate(0u, (uint16_t)ADC_HALF_HALFWORDS);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_accumulate((uint16_t)ADC_HALF_HALFWORDS, (uint16_t)ADC_HALF_HALFWORDS);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_adc1);
}

/*==============================================================================
 * 初始化
 *============================================================================*/
void BSP_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig;

    s_running = 0;

    /*--- 1. ADC1: 2 通道扫描 + 连续转换 ---*/
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2u;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return;
    }

    /* rank1: MRE (PA0) */
    sConfig.Channel      = ADC_CH_MRE;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return;
    }

    /* rank2: 3V3_ADC0 监视 (PA4) */
    sConfig.Channel      = ADC_CH_VREF;
    sConfig.Rank         = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return;
    }

    /*--- 2. DMA1_Channel1 (ADC1 固定映射) ---*/
    __HAL_RCC_DMA1_CLK_ENABLE();

    s_hdma_adc1.Instance                 = DMA1_Channel1;
    s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&s_hdma_adc1) != HAL_OK) {
        return;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, s_hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, ADC_DMA_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /*--- 3. 校准 (F1 必须先校准再用) ---*/
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        return;
    }

    /*--- 4. 启动连续转换 + DMA 循环 ---*/
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_buf, ADC_BUF_HALFWORDS) != HAL_OK) {
        return;
    }

    s_running = 1;
}

uint8_t BSP_ADC_IsRunning(void)
{
    return s_running;
}

void BSP_ADC_GetSnapshot(BSP_ADC_Snapshot_t *out)
{
    uint32_t primask;

    if (out == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_snap;
    __set_PRIMASK(primask);
}

void BSP_ADC_GetPeak(uint16_t *mn, uint16_t *mx, uint32_t *cnt,
                     uint16_t *edges, uint8_t clear)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (cnt != NULL) {
        *cnt = s_peak_cnt;
    }
    if (s_peak_cnt == 0u) {
        /* 清零后还没采到数据: 报 0, 别把 0xFFFF 的初值漏出去 */
        if (mn != NULL) { *mn = 0u; }
        if (mx != NULL) { *mx = 0u; }
    } else {
        if (mn != NULL) { *mn = s_peak_min; }
        if (mx != NULL) { *mx = s_peak_max; }
    }
    if (edges != NULL) {
        *edges = s_edge_cnt;
    }

    if (clear != 0u) {
        s_peak_min   = 0xFFFFu;
        s_peak_max   = 0u;
        s_peak_cnt   = 0u;
        s_edge_cnt   = 0u;
        /* 清零后先解除武装: 要等信号回落到 EDGE_LOW 以下才允许计第一个沿,
         * 否则起点相位若在高位, 会把一个不完整的周期也算进去 (+1 偏差)。 */
        s_edge_armed = 0u;
    }

    __set_PRIMASK(primask);
}

/*==============================================================================
 * 波形抓取
 *============================================================================*/
void BSP_ADC_CaptureStart(uint8_t div)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_cap_on   = 0u;
    s_cap_cnt  = 0u;
    s_cap_div  = (div == 0u) ? 1u : div;
    s_cap_skip = 0u;
    s_cap_on   = 1u;
    __set_PRIMASK(primask);
}

uint16_t BSP_ADC_CaptureStop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_cap_on = 0u;
    __set_PRIMASK(primask);

    return s_cap_cnt;
}

const uint16_t *BSP_ADC_CaptureBuf(void)
{
    return s_cap_buf;
}
