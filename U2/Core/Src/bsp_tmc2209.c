/**
  ******************************************************************************
  * @file    bsp_tmc2209.c
  * @brief   TMC2209 单线 UART 驱动 —— 基于 USART3 (PB10=TX / PB11=RX)。
  *
  * 与参考项目 (diameter_JIG/U7) 的差别: 那边用的是 GPIO + TIM3 软件模拟 UART,
  * 因为它的 UART 引脚被 STEP PWM 占着; 本板 USART3 是独立的, 所以走硬件串口。
  *
  * 单线半双工的要点: MCU 的 TX/RX 与 TMC2209 的 PDN_UART 挂在同一个网络上,
  * 发送时如果接收器还开着, 会把自己发出去的字节收回来当成应答。所以在发送
  * 期间把 USART3 的 RE 位清零, 发完再打开, 这样接收窗口里只有 TMC 的应答。
  ******************************************************************************
  */

#include "bsp_tmc2209.h"
#include "usart.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

#define TMC_UART            USART3
#define TMC_TX_TIMEOUT_MS   20u
#define TMC_RX_TIMEOUT_MS   20u
#define TMC_INIT_TRIES      3u
#define TMC_RETRY_DELAY_MS  50u

/* 单次事务的字节长度 */
#define TMC_DG_WRITE_LEN    8u
#define TMC_DG_READ_LEN     4u
#define TMC_REPLY_LEN       8u

/* 接收窗口里把 TX 引脚(PB10)放开成输入上拉:
 * 板上 PDN_UART 到 MCU 的两根线里有一根串了 1kΩ, 但哪一根串的, 原理图网名和
 * 实物丝印说法不一致。发送结束后释放 TX, TMC2209 应答时无论串阻在哪一侧都不会
 * 和 MCU 的推挽输出对顶。置 0 可退回「TX 一直保持复用推挽」的简单模式。 */
#define TMC_RELEASE_TX_ON_RX    1

static uint8_t s_ready = 0;

/* TX 引脚切回复用推挽 (真正发送时才需要) */
static void tmc_tx_pin_drive(void)
{
    GPIO_InitTypeDef gpio;

    gpio.Pin   = GPIO_PIN_10;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* TX 引脚放开, 只留内部上拉维持总线空闲高电平 */
#if TMC_RELEASE_TX_ON_RX
static void tmc_tx_pin_release(void)
{
    GPIO_InitTypeDef gpio;

    gpio.Pin   = GPIO_PIN_10;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}
#endif

/*==============================================================================
 * CRC8 (Trinamic 位序: LSB-first, 多项式 0x07)
 *   自检向量: CRC([0x05]) = 0x69, CRC([0x05,0x00,0x00]) = 0x48
 *============================================================================*/
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t cur;
    uint8_t i, j;

    for (i = 0; i < len; i++) {
        cur = data[i];
        for (j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (cur & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            cur >>= 1;
        }
    }
    return crc;
}

/*==============================================================================
 * 底层收 / 发
 *============================================================================*/

/* 清掉接收寄存器里的残留 (含溢出标志 ORE, 必须先读 SR 再读 DR 才清得掉) */
static void tmc_rx_flush(void)
{
    volatile uint32_t dump;

    while (TMC_UART->SR & USART_SR_RXNE) {
        dump = TMC_UART->DR;
        (void)dump;
    }
    if (TMC_UART->SR & USART_SR_ORE) {
        dump = TMC_UART->SR;
        dump = TMC_UART->DR;
        (void)dump;
    }
}

/* 发送一个报文。发送期间关闭接收器, 避免回显。成功返回 1 */
static uint8_t tmc_uart_send(const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;

    tmc_tx_pin_drive();                 /* 确认 TX 处于复用推挽 */
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    TMC_UART->CR1 &= ~USART_CR1_RE;     /* 关接收: 不收自己的回显 */
    tmc_rx_flush();

    st = HAL_UART_Transmit(&huart3, buf, len, TMC_TX_TIMEOUT_MS);

    TMC_UART->CR1 |= USART_CR1_RE;      /* 重新打开接收, 准备收应答 */
    tmc_rx_flush();
#if TMC_RELEASE_TX_ON_RX
    tmc_tx_pin_release();               /* 放开总线, 让 TMC2209 能拉低应答 */
#endif

    return (st == HAL_OK) ? 1u : 0u;
}

/* 收固定长度的应答, 成功返回 1 */
static uint8_t tmc_uart_recv(uint8_t *buf, uint16_t len)
{
    return (HAL_UART_Receive(&huart3, buf, len, TMC_RX_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

/*==============================================================================
 * 报文构建
 *
 * 写: 05 | addr | (reg|0x80) | d31..d24 | d23..d16 | d15..d8 | d7..d0 | CRC8(前7字节)
 * 读: 05 | addr | reg | CRC8(前3字节)
 * 应答: 05 | 0xFF | reg | d31..d24 | d23..d16 | d15..d8 | d7..d0 | CRC8(前7字节)
 *============================================================================*/
static uint8_t tmc_build_write(uint8_t reg, uint32_t val, uint8_t *dg)
{
    dg[0] = TMC2209_SYNC_BYTE;
    dg[1] = TMC2209_ADDR_MOTOR;
    dg[2] = (uint8_t)(reg | 0x80);
    dg[3] = (uint8_t)(val >> 24);
    dg[4] = (uint8_t)(val >> 16);
    dg[5] = (uint8_t)(val >> 8);
    dg[6] = (uint8_t)(val);
    dg[7] = tmc_crc8(dg, 7);
    return TMC_DG_WRITE_LEN;
}

static uint8_t tmc_build_read(uint8_t reg, uint8_t *dg)
{
    dg[0] = TMC2209_SYNC_BYTE;
    dg[1] = TMC2209_ADDR_MOTOR;
    dg[2] = reg;
    dg[3] = tmc_crc8(dg, 3);
    return TMC_DG_READ_LEN;
}

/*==============================================================================
 * 公开 API
 *============================================================================*/

uint8_t BSP_TMC_WriteReg(uint8_t reg, uint32_t val)
{
    uint8_t dg[TMC_DG_WRITE_LEN];

    (void)tmc_build_write(reg, val, dg);
    return tmc_uart_send(dg, TMC_DG_WRITE_LEN);
}

uint8_t BSP_TMC_ReadReg(uint8_t reg, uint32_t *val)
{
    uint8_t dg[TMC_DG_READ_LEN];
    uint8_t rx[TMC_REPLY_LEN];

    (void)tmc_build_read(reg, dg);
    if (!tmc_uart_send(dg, TMC_DG_READ_LEN)) {
        return 0u;
    }
    if (!tmc_uart_recv(rx, TMC_REPLY_LEN)) {
        return 0u;
    }
    if (rx[0] != TMC2209_SYNC_BYTE) {
        return 0u;
    }
    if (rx[2] != reg) {                       /* 寄存器回显必须匹配 */
        return 0u;
    }
    if (rx[7] != tmc_crc8(rx, 7)) {
        return 0u;
    }

    if (val != NULL) {
        *val = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
               ((uint32_t)rx[5] << 8)  |  (uint32_t)rx[6];
    }
    return 1u;
}

uint8_t BSP_TMC_WriteRegVerified(uint8_t reg, uint32_t val, uint32_t *readback)
{
    uint32_t rb = 0;
    uint8_t  i;

    for (i = 0; i < 3u; i++) {
        if (!BSP_TMC_WriteReg(reg, val)) {
            continue;
        }
        if (BSP_TMC_ReadReg(reg, &rb) && (rb == val)) {
            if (readback != NULL) {
                *readback = rb;
            }
            return 1u;
        }
    }
    if (readback != NULL) {
        *readback = rb;
    }
    return 0u;
}

uint8_t BSP_TMC_IsAlive(void)
{
    uint32_t dummy = 0;

    return BSP_TMC_ReadReg(TMC2209_REG_GSTAT, &dummy);
}

uint8_t BSP_TMC_GetDrvStatus(uint32_t *val)
{
    return BSP_TMC_ReadReg(TMC2209_REG_DRV_STATUS, val);
}

uint8_t BSP_TMC_SetDefaults(void)
{
    uint8_t ok = 1;
    uint32_t rb = 0;

    GCONF_reg_t gconf;
    IHOLD_IRUN_reg_t ihold;
    TPOWERDOWN_reg_t tpower;
    CHOPCONF_reg_t chop;
    PWMCONF_reg_t pwm;

    /*--- GCONF ---
     * I_scale_analog=0 : 电流基准走内部 5VOUT, 由 IRUN 决定 (VREF 分压只做诊断)
     * internal_Rsense=0: 板上是外部 110mΩ 采样电阻
     * pdn_disable=1    : ★UART 模式必须置 1, 否则 PDN_UART 被当成待机脚
     * mstep_reg_select=1: ★微步由 CHOPCONF.MRES 决定, 不看 MS1/MS2 引脚
     * multistep_filt=1 : 让 STEP 输入带滤波, 抑制脉冲毛刺 */
    gconf.raw = 0;
    gconf.bits.I_scale_analog   = 0;
    gconf.bits.internal_Rsense  = 0;
    gconf.bits.en_SpreadCycle   = 0;   /* StealthChop2 */
    gconf.bits.shaft            = 0;
    gconf.bits.index_otpw       = 1;
    gconf.bits.index_step       = 0;
    gconf.bits.pdn_disable      = 1;
    gconf.bits.mstep_reg_select = 1;
    gconf.bits.multistep_filt   = 1;
    gconf.bits.test_mode        = 0;
    BSP_TMC_WriteReg(TMC2209_REG_GCONF, gconf.raw);
    if (BSP_TMC_ReadReg(TMC2209_REG_GCONF, &rb)) {
        uint32_t key = (1u << 6) | (1u << 7) | (1u << 8);   /* pdn_disable|mstep_reg_select|multistep_filt */
        if ((rb & key) != (gconf.raw & key)) {
            ok = 0;
        }
    } else {
        ok = 0;
    }

    /*--- IHOLD_IRUN (只写寄存器, 读回恒 0, 不做校验) ---*/
    ihold.raw = 0;
    ihold.bits.irun       = TMC_DEFAULT_IRUN;
    ihold.bits.ihold      = TMC_DEFAULT_IHOLD;
    ihold.bits.iholddelay = TMC_DEFAULT_IHOLDDELAY;
    BSP_TMC_WriteReg(TMC2209_REG_IHOLD_IRUN, ihold.raw);

    /*--- TPOWERDOWN (只写): 到位后约 0.4s 降到 IHOLD ---*/
    tpower.raw = 0;
    tpower.bits.tpowerdown = TMC_DEFAULT_TPOWERDOWN;
    BSP_TMC_WriteReg(TMC2209_REG_TPOWERDOWN, tpower.raw);

    /*--- CHOPCONF (关键: 微步 + 斩波) ---*/
    chop.raw = 0;
    chop.bits.toff   = 3;
    chop.bits.hstrt  = 4;
    chop.bits.hend   = 1;
    chop.bits.tbl    = 2;
    chop.bits.vsense = 0;                    /* 高电流量程 */
    chop.bits.mres   = TMC_MRES_16USTEP;     /* 16 微步 -> 3200 脉冲/圈 */
    chop.bits.intpol = 1;
    chop.bits.dedge  = 0;
    chop.bits.diss2g = 0;
    chop.bits.diss2vs = 0;
    if (!BSP_TMC_WriteRegVerified(TMC2209_REG_CHOPCONF, chop.raw, &rb)) {
        ok = 0;
    }

    /*--- PWMCONF (StealthChop2 自动标定) ---*/
    pwm.raw = 0;
    pwm.bits.pwm_ofs       = 36;
    pwm.bits.pwm_grad      = 14;
    pwm.bits.pwm_freq      = 1;
    pwm.bits.pwm_autoscale = 1;
    pwm.bits.pwm_autograd  = 1;
    pwm.bits.freewheel     = 0;              /* 00=普通运行; IHOLD!=0 时 freewheel 本就不生效 */
    pwm.bits.pwm_reg       = 8;
    pwm.bits.pwm_lim       = 12;
    if (!BSP_TMC_WriteRegVerified(TMC2209_REG_PWMCONF, pwm.raw, &rb)) {
        ok = 0;
    }

    return ok;
}

void BSP_TMC_Init(void)
{
    uint8_t i;

    s_ready = 0;

    for (i = 0; i < TMC_INIT_TRIES; i++) {
        if (BSP_TMC_IsAlive()) {
            s_ready = BSP_TMC_SetDefaults();
            if (s_ready) {
                break;
            }
        } else {
            s_ready = 0;
        }
        HAL_Delay(TMC_RETRY_DELAY_MS);
    }
}

uint8_t BSP_TMC_IsReady(void)
{
    return s_ready;
}
