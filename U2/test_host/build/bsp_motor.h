#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"
#include <stdint.h>

/*==============================================================================
 * 步进电机 STEP/DIR 驱动 (寄存器级, 不依赖 HAL_TIM —— 本工程未编译 HAL_TIM 模块)
 *
 *   STEP = PB1 (TIM3_CH4, 复用推挽)
 *   DIR  = PB0 (普通 GPIO 推挽, 由 MX_GPIO_Init 配置)
 *
 * 时基: TIM3 挂在 APB1, APB1 分频=2 -> TIM3CLK = 72MHz
 *       取 PSC=3 -> 计数时钟 18MHz
 *       步进频率 f = 18MHz / (ARR+1)  =>  f 范围 [275Hz, 20kHz] (16 位 ARR)
 *
 * 加减速: 在 ARR 域线性插值 (f 与 ARR 成反比), 加减速段内每步只做一次
 *         乘加, 中断里不出现除法。直接以目标频率硬启停会丢步, 这是参考
 *         项目 U7 踩过的坑。
 *============================================================================*/

/*===== 方向 =====*/
#define MOTOR_DIR_CW        0u
#define MOTOR_DIR_CCW       1u

/*===== 速度限制 =====*/
#define MOTOR_F_MIN_HZ      300u      /* <275Hz 超出 16 位 ARR 能力 */
#define MOTOR_F_MAX_HZ      20000u    /* 再高 CPU 来不及在中断里换周期 */
#define MOTOR_F_START_HZ    500u      /* 起步频率 */

/*===== 梯形加减速 =====*/
#define MOTOR_RAMP_STEPS    800u      /* 加速段/减速段步数上限 (实际会按总步数对折) */
#define MOTOR_RAMP_MIN_STEPS 4u       /* 总步数小于 2*该值时不做加减速 */

/*===== API =====*/

/** @brief 配置 TIM3_CH4 为 STEP 脉冲源并打开更新中断。上电调用一次。 */
void     BSP_Motor_Init(void);

/**
  * @brief 启动一次定长运动 (非阻塞)
  * @param dir      MOTOR_DIR_CW / MOTOR_DIR_CCW
  * @param steps    步数 (>0)
  * @param freq_hz  巡航频率, 会被夹到 [MOTOR_F_MIN_HZ, MOTOR_F_MAX_HZ]
  * @retval 1=已启动 0=拒绝(正在运动/步数为0)
  */
uint8_t  BSP_Motor_Move(uint8_t dir, uint32_t steps, uint32_t freq_hz);

/** @brief 立即停止 (未走完的步数作废) */
void     BSP_Motor_Stop(void);

/** @brief 1=正在运动 */
uint8_t  BSP_Motor_IsBusy(void);

/** @brief 本次(或最近一次)运动已完成的步数 */
uint32_t BSP_Motor_GetDone(void);

/** @brief 最近一次运动的设定步数 */
uint32_t BSP_Motor_GetTarget(void);

/** @brief 当前实际使用的 TIM3 计数时钟 (Hz), 供上层换算 */
uint32_t BSP_Motor_GetTickHz(void);

#endif /* __BSP_MOTOR_H */
