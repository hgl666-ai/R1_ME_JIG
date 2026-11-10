/**
  ******************************************************************************
  * @file    bsp_motor.c
  * @brief   步进电机 STEP/DIR 驱动 (TIM3_CH4 -> PB1, DIR -> PB0)
  *
  * 为什么用寄存器而不是 HAL_TIM: 本工程由 CubeMX 生成时没有使能 HAL_TIM 模块
  * (Drivers 目录里没有 stm32f1xx_hal_tim.c, hal_conf.h 里也没有 HAL_TIM_MODULE_ENABLED)。
  * 一旦手工把该模块加回去, 用户下次用 CubeMX 重新生成工程时又会被裁掉, 编译会突然
  * 断掉。把 TIM3 的全部配置放在本文件里, 重新生成工程不会影响它。
  ******************************************************************************
  */

#include "bsp_motor.h"

/*===== 硬件绑定 =====*/
#define MOTOR_TIM               TIM3
#define MOTOR_STEP_GPIO_PORT    GPIOB
#define MOTOR_STEP_GPIO_PIN     GPIO_PIN_1      /* TIM3_CH4 */
#define MOTOR_DIR_GPIO_PORT     GPIOB
#define MOTOR_DIR_GPIO_PIN      GPIO_PIN_0

/*===== 时基 =====*/
#define MOTOR_TIM_CLK_HZ        72000000u       /* APB1=36MHz, 分频!=1 -> TIM 时钟 x2 */
#define MOTOR_PSC               3u              /* 72MHz / (3+1) = 18MHz */
#define MOTOR_TIM_TICK_HZ       (MOTOR_TIM_CLK_HZ / (MOTOR_PSC + 1u))

/*===== 中断优先级 (数值越小越优先; NVIC_PRIORITYGROUP_4 -> 子优先级必须为 0) =====*/
#define MOTOR_IRQ_PRIORITY      1u

/*===== 状态 =====*/
static volatile uint8_t  s_busy       = 0;
static volatile uint32_t s_target     = 0;
static volatile uint32_t s_done       = 0;
static volatile uint16_t s_ramp       = 0;      /* 加减速段步数, 0=匀速 (最大 MOTOR_RAMP_STEPS=800, 不能是 uint8) */
static volatile uint16_t s_arr_start  = 0;
static volatile int32_t  s_arr_delta  = 0;      /* 每步 ARR 增量, 8 位小数 */

/* 步数上限: 防止上层传进来一个把 int32 距离量撑爆的值 (20kHz 下 1e8 步 ≈ 83 分钟) */
#define MOTOR_MAX_STEPS     100000000u

/*==============================================================================
 * 工具
 *============================================================================*/

/* 频率 -> ARR, 结果夹到 [1, 65535] */
static uint16_t motor_arr_from_hz(uint32_t hz)
{
    uint32_t arr;

    if (hz < MOTOR_F_MIN_HZ) {
        hz = MOTOR_F_MIN_HZ;
    }
    if (hz > MOTOR_F_MAX_HZ) {
        hz = MOTOR_F_MAX_HZ;
    }

    arr = MOTOR_TIM_TICK_HZ / hz;
    if (arr == 0u) {
        arr = 1u;
    }
    if (arr > 0xFFFFu) {
        arr = 0xFFFFu;
    }
    return (uint16_t)(arr - 1u);
}

/* 停止脉冲输出并关掉计数器。调用处需保证在临界区内或中断上下文 */
static void motor_stop_hw(void)
{
    MOTOR_TIM->DIER = 0;
    MOTOR_TIM->CCER &= ~TIM_CCER_CC4E;      /* 释放 PB1, 引脚回落到 ODR=0 (STEP 低) */
    MOTOR_TIM->CR1  &= ~TIM_CR1_CEN;
    MOTOR_TIM->SR    = 0;
    s_busy = 0;
}

/*==============================================================================
 * 更新中断: 每个周期一次 = 一个脉冲走完
 *============================================================================*/
void TIM3_IRQHandler(void)
{
    uint32_t done;
    int32_t  k;
    int32_t  from_end;
    uint16_t arr;

    if ((MOTOR_TIM->SR & TIM_SR_UIF) == 0u) {
        return;
    }
    MOTOR_TIM->SR = (uint16_t)~TIM_SR_UIF;

    if (!s_busy) {
        motor_stop_hw();
        return;
    }

    done = s_done + 1u;

    if (done >= s_target) {
        s_done = s_target;
        motor_stop_hw();
        return;
    }
    s_done = done;

    if (s_ramp != 0u) {
        k = (int32_t)done;
        from_end = (int32_t)s_target - (int32_t)done;
        if (from_end < k) {
            k = from_end;                   /* 减速段: 用离终点的距离 */
        }
        if (k > (int32_t)s_ramp) {
            k = (int32_t)s_ramp;
        }
        arr = (uint16_t)((int32_t)s_arr_start + ((s_arr_delta * k) >> 8));
        if (arr < 2u) {
            arr = 2u;                       /* 再小就没有可用的 50% 占空比了 */
        }
        MOTOR_TIM->ARR  = arr;              /* ARPE=1, 下个更新事件生效 */
        MOTOR_TIM->CCR4 = (uint16_t)(arr >> 1);
    }
}

/*==============================================================================
 * 公开 API
 *============================================================================*/

void BSP_Motor_Init(void)
{
    /* TIM3 时钟 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    MOTOR_TIM->CR1  = 0;
    MOTOR_TIM->DIER = 0;
    MOTOR_TIM->CCER = 0;

    /* CH4: 输出模式, PWM1, 预装载使能 */
    MOTOR_TIM->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

    /* 频率固定为起步频率, 真正的周期在每次 Move 时设定。
     * 注意顺序: 先打开 ARPE, 再写 ARR, 最后用 UG 把预装载寄存器搬进影子
     * 寄存器。否则第一次更新事件可能把 ARR 打回旧值。 */
    MOTOR_TIM->CR1  = TIM_CR1_ARPE;
    MOTOR_TIM->PSC  = MOTOR_PSC;
    MOTOR_TIM->ARR  = motor_arr_from_hz(MOTOR_F_START_HZ);
    MOTOR_TIM->CCR4 = (uint16_t)(MOTOR_TIM->ARR >> 1);

    MOTOR_TIM->EGR = TIM_EGR_UG;
    MOTOR_TIM->SR  = 0;

    HAL_NVIC_SetPriority(TIM3_IRQn, MOTOR_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    /* DIR 引脚在 MX_GPIO_Init 里已配好, 这里只给个确定电平 */
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_GPIO_PIN, GPIO_PIN_RESET);

    s_busy   = 0;
    s_target = 0;
    s_done   = 0;
    s_ramp   = 0;
}

uint8_t BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz)
{
    uint16_t arr_target;
    uint16_t arr_start;
    uint32_t ramp;

    if (steps == 0u) {
        return 0u;
    }
    if (steps > MOTOR_MAX_STEPS) {
        return 0u;
    }
    if (s_busy) {
        return 0u;
    }

    /* 方向先建立, TMC2209 要求 DIR 相对 STEP 上升沿有建立时间 */
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_GPIO_PIN,
                      (dir == MOTOR_DIR_CW) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    {
        /* DIR 建立时间: TMC2209 只需要 ~20ns, 但这里给足 5us 更稳妥 */
        volatile uint32_t d = 400u;
        while (d--) { }
    }

    arr_target = motor_arr_from_hz(freq_hz);
    arr_start  = motor_arr_from_hz(MOTOR_F_START_HZ);
    if (arr_start < arr_target) {
        /* 目标频率低于起步频率 -> 不做加速, 全程按目标频率跑 */
        arr_start = arr_target;
    }

    /* 加减速段长度: 不超过总步数的一半 */
    ramp = steps / 2u;
    if (ramp > MOTOR_RAMP_STEPS) {
        ramp = MOTOR_RAMP_STEPS;
    }
    if (ramp < MOTOR_RAMP_MIN_STEPS) {
        ramp = 0u;
        arr_start = arr_target;
    }

    __disable_irq();

    s_target = steps;
    s_done   = 0;
    s_ramp   = (uint16_t)ramp;
    s_arr_start = arr_start;
    if (ramp != 0u) {
        /* ARR 线性插值: 每步的增量, 8 位小数 */
        s_arr_delta = (((int32_t)arr_target - (int32_t)arr_start) * 256) / (int32_t)ramp;
    } else {
        s_arr_delta = 0;
    }

    MOTOR_TIM->CR1  &= ~TIM_CR1_CEN;
    MOTOR_TIM->DIER  = 0;
    MOTOR_TIM->CCER &= ~TIM_CCER_CC4E;          /* 先确保 STEP 是低的, 下一个脉冲才有上升沿 */
    MOTOR_TIM->CR1  |= TIM_CR1_ARPE;            /* 必须早于 ARR 写入 */
    MOTOR_TIM->PSC   = MOTOR_PSC;
    MOTOR_TIM->ARR   = arr_start;
    MOTOR_TIM->CCR4  = (uint16_t)(arr_start >> 1);
    MOTOR_TIM->CNT   = 0;
    MOTOR_TIM->EGR   = TIM_EGR_UG;
    MOTOR_TIM->SR    = 0;                       /* 清掉 UG 产生的 UIF, 防止多计一步 */

    s_busy = 1;
    MOTOR_TIM->CCER |= TIM_CCER_CC4E;           /* 使能输出: CNT=0 < CCR4 -> STEP 拉高, 出第一个脉冲 */
    MOTOR_TIM->DIER  = TIM_DIER_UIE;
    MOTOR_TIM->CR1  |= (TIM_CR1_ARPE | TIM_CR1_CEN);

    __enable_irq();

    return 1u;
}

void BSP_Motor_Stop(void)
{
    __disable_irq();
    motor_stop_hw();
    __enable_irq();
}

uint8_t BSP_Motor_IsBusy(void)
{
    return s_busy;
}

uint32_t BSP_Motor_GetDone(void)
{
    return s_done;
}

uint32_t BSP_Motor_GetTarget(void)
{
    return s_target;
}

uint32_t BSP_Motor_GetTickHz(void)
{
    return MOTOR_TIM_TICK_HZ;
}
