/* 主机侧编译 bsp_motor.c 用的最小桩头文件
 * 只提供 bsp_motor.c 实际用到的符号，不模拟外设行为。 */
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/* 假的 TIM3 寄存器组: 读写就是普通内存读写 */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t DIER;
    volatile uint32_t CCER;
    volatile uint32_t CCMR2;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t CCR4;
    volatile uint32_t CNT;
    volatile uint32_t EGR;
    volatile uint32_t SR;
} TIM_TypeDef_host;

extern TIM_TypeDef_host g_tim3;
#define TIM3 (&g_tim3)

#define TIM_CCMR2_OC4M_1   (0x1UL << 12)
#define TIM_CCMR2_OC4M_2   (0x1UL << 13)
#define TIM_CCMR2_OC4PE    (0x1UL << 11)
#define TIM_CCER_CC4E      (0x1UL << 12)
#define TIM_CR1_CEN        (0x1UL << 0)
#define TIM_CR1_ARPE       (0x1UL << 7)
#define TIM_EGR_UG         (0x1UL << 0)
#define TIM_SR_UIF         (0x1UL << 0)
#define TIM_DIER_UIE       (0x1UL << 0)

#define TIM3_IRQn          29

#define __HAL_RCC_TIM3_CLK_ENABLE()   do { } while (0)
#define HAL_NVIC_SetPriority(a,b,c)   do { (void)(a); (void)(b); (void)(c); } while (0)
#define HAL_NVIC_EnableIRQ(a)         do { (void)(a); } while (0)

#define GPIOB               ((void *)0x40010C00)
#define GPIO_PIN_0          (0x0001U)
#define GPIO_PIN_1          (0x0002U)
#define GPIO_PIN_RESET      (0U)
#define GPIO_PIN_SET        (1U)
#define HAL_GPIO_WritePin(port, pin, state)  host_gpio_write((pin), (state))

void host_gpio_write(uint16_t pin, uint8_t state);

#define __disable_irq()     do { } while (0)
#define __enable_irq()      do { } while (0)

#endif /* MAIN_H */
