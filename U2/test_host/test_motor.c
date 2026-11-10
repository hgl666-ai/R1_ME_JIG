/* 主机侧验证: bsp_motor.c 的脉冲计数 / 加减速斜坡 / 启停逻辑
 * bsp_motor.c 由 run.ps1 每次从 Core/Src 原样拷进 build\ 再编译，
 * 所以这里测的就是固件里跑的同一份代码。 */
#include <stdio.h>
#include <stdint.h>

#include "main.h"

TIM_TypeDef_host g_tim3;
static uint8_t g_dir_state = 0xFF;   /* 最近一次写进 DIR 的电平 */

void host_gpio_write(uint16_t pin, uint8_t state)
{
    if (pin == GPIO_PIN_0) {
        g_dir_state = state;
    }
}

#include "bsp_motor.c"

static int fails = 0;

static void ck(const char *name, int cond)
{
    printf("  %-58s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) { fails++; }
}

/* 模拟硬件: 计数器跑满一个周期 -> 置 UIF -> 进中断 */
static void tick_n(uint32_t n)
{
    uint32_t i;

    for (i = 0; i < n; i++) {
        if ((g_tim3.CR1 & TIM_CR1_CEN) == 0u) {
            return;
        }
        g_tim3.SR |= TIM_SR_UIF;
        TIM3_IRQHandler();
    }
}

int main(void)
{
    BSP_Motor_Init();

    printf("TIM3 计数时钟 = %u Hz\n", (unsigned)BSP_Motor_GetTickHz());

    printf("== 1. 短行程 (不做加减速) ==\n");
    {
        uint32_t guard = 0;

        ck("Move(3 步) 被接受", BSP_Motor_Move(MOTOR_DIR_CW, 3u, 2000u) == 1);
        ck("DIR 引脚写低 (=CW)", g_dir_state == 0u);
        ck("立刻进入 busy", BSP_Motor_IsBusy() == 1);
        ck("运动期间再来一次 Move 被拒绝", BSP_Motor_Move(MOTOR_DIR_CW, 10u, 1000u) == 0);
        while (BSP_Motor_IsBusy() && (guard < 1000u)) { tick_n(1); guard++; }
        ck("3 个脉冲后自动停止", BSP_Motor_IsBusy() == 0);
        ck("已完成步数 == 3", BSP_Motor_GetDone() == 3u);
        ck("停止后 STEP 输出被关掉 (CC4E=0)", (g_tim3.CCER & TIM_CCER_CC4E) == 0u);
        ck("停止后计数器关掉 (CEN=0)", (g_tim3.CR1 & TIM_CR1_CEN) == 0u);
        tick_n(5);
        ck("停止后的更新中断不会多计步", BSP_Motor_GetDone() == 3u);
    }

    printf("== 2. 长行程 (加速-匀速-减速) ==\n");
    {
        uint32_t target = 5000u;
        uint32_t i;
        uint16_t arr_min = 0xFFFFu, arr_max = 0u;
        uint16_t arr_after_accel = 0u;
        uint16_t arr_at_end;
        uint16_t arr_start_run;
        uint16_t arr_target_run;
        uint16_t one_step;

        BSP_Motor_Move(MOTOR_DIR_CCW, target, 10000u);
        ck("DIR 引脚写高 (=CCW)", g_dir_state == 1u);

        arr_start_run  = (uint16_t)g_tim3.ARR;
        arr_target_run = (uint16_t)((BSP_Motor_GetTickHz() / 10000u) - 1u);

        for (i = 0; i < 2000000u && BSP_Motor_IsBusy(); i++) {
            tick_n(1);
            if (g_tim3.ARR < arr_min) { arr_min = (uint16_t)g_tim3.ARR; }
            if (g_tim3.ARR > arr_max) { arr_max = (uint16_t)g_tim3.ARR; }
            if (BSP_Motor_GetDone() == 800u) { arr_after_accel = (uint16_t)g_tim3.ARR; }
        }

        arr_at_end = (uint16_t)g_tim3.ARR;
        one_step   = (uint16_t)((arr_start_run - arr_target_run) / 800u + 1u);

        printf("  arr_start=%u arr_target=%u arr_min=%u arr_max=%u arr_end=%u (单步增量~%u)\n",
               arr_start_run, arr_target_run, arr_min, arr_max, arr_at_end, one_step);

        ck("走满设定步数后停止", BSP_Motor_GetDone() == target);
        ck("巡航段 ARR 收敛到目标频率", arr_min <= arr_target_run + 2u);
        ck("加速段结束 ARR 已到目标值",
           (arr_after_accel >= arr_target_run) && (arr_after_accel <= arr_target_run + 2u));
        /* 最后一个 ARR 是在 done=target-1 时写的 (k=1), 所以比 arr_start 还差一个斜坡增量 */
        ck("减速段结束 ARR 已回到起步值(差不超过一个斜坡增量)",
           (arr_at_end + one_step + 2u >= arr_start_run) && (arr_at_end <= arr_start_run));
        ck("ARR 从未超出 [目标值-2, 起步值+2]",
           (arr_min + 2u >= arr_target_run) && (arr_max <= arr_start_run + 2u));
        ck("全过程 ARR 未小于 2 (占空比可控)", arr_min >= 2u);
    }

    printf("== 3. 参数边界 ==\n");
    {
        BSP_Motor_Stop();
        ck("0 步被拒绝", BSP_Motor_Move(MOTOR_DIR_CW, 0, 1000) == 0);
        ck("超过步数上限被拒绝", BSP_Motor_Move(MOTOR_DIR_CW, 200000000u, 1000) == 0);
        ck("频率超上限被夹住但仍接受 (200kHz)",
           BSP_Motor_Move(MOTOR_DIR_CW, 50u, 200000u) == 1);
        tick_n(100);
        ck("夹住后仍能正常走完", (BSP_Motor_IsBusy() == 0) && (BSP_Motor_GetDone() == 50u));
        ck("频率低于下限也被接受 (100Hz)", BSP_Motor_Move(MOTOR_DIR_CW, 20u, 100u) == 1);
        tick_n(100);
        ck("低速行程走完", (BSP_Motor_IsBusy() == 0) && (BSP_Motor_GetDone() == 20u));
    }

    printf("== 4. 中途 Stop ==\n");
    {
        BSP_Motor_Move(MOTOR_DIR_CW, 10000u, 5000u);
        tick_n(100);
        BSP_Motor_Stop();
        ck("Stop 后立即退出 busy", BSP_Motor_IsBusy() == 0);
        ck("Stop 后已完成步数停在 100", BSP_Motor_GetDone() == 100u);
        tick_n(10);
        ck("Stop 后不再计数", BSP_Motor_GetDone() == 100u);
    }

    printf("\n%s  (fails=%d)\n", fails ? "*** FAILURES ***" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
