#ifndef __BSP_TMC2209_H
#define __BSP_TMC2209_H

#include "main.h"
#include <stdint.h>

/*==============================================================================
 * TMC2209 单线 UART 驱动 (USART3: PB10=TX / PB11=RX)
 *
 * 硬件连接 (SCH_破碎机编码板-测试治具):
 *   U8 TMC2209-LA-T pin14 PDN_UART  <->  MCU USART3
 *   VS = 24V, VCC_IO = 3.3V, ENN = GND (常使能)
 *   R22/R23 = 110mΩ 采样电阻
 *   STEP = PB1 (TIM3_CH4), DIR = PB0 (见 bsp_motor.c)
 *
 * TMC2209 的 UART 是半双工单线 + 自动波特率 (首个 0x05 SYNC 字节定波特)。
 * 每次事务前关掉 USART3 接收器 (RE=0), 避免把自己的发送回显当成应答。
 *============================================================================*/

/*===== 从机地址 =====*/
#define TMC2209_SLAVE_ADDR0     ((uint8_t)0x00)
#define TMC2209_ADDR_MOTOR      TMC2209_SLAVE_ADDR0  /* 本板仅 1 路电机 */
#define TMC2209_SYNC_BYTE       ((uint8_t)0x05)

/*===== 寄存器地址 =====*/
#define TMC2209_REG_GCONF         ((uint8_t)0x00)
#define TMC2209_REG_GSTAT         ((uint8_t)0x01)
#define TMC2209_REG_IFCNT         ((uint8_t)0x02)
#define TMC2209_REG_SLAVECONF     ((uint8_t)0x03)
#define TMC2209_REG_IOIN          ((uint8_t)0x06)
#define TMC2209_REG_IHOLD_IRUN    ((uint8_t)0x10)
#define TMC2209_REG_TPOWERDOWN    ((uint8_t)0x11)
#define TMC2209_REG_TSTEP         ((uint8_t)0x12)
#define TMC2209_REG_VACTUAL       ((uint8_t)0x22)
#define TMC2209_REG_SGTHRS        ((uint8_t)0x40)
#define TMC2209_REG_SG_RESULT     ((uint8_t)0x41)
#define TMC2209_REG_MSCNT         ((uint8_t)0x6A)
#define TMC2209_REG_MSCURACT      ((uint8_t)0x6B)
#define TMC2209_REG_CHOPCONF      ((uint8_t)0x6C)
#define TMC2209_REG_DRV_STATUS    ((uint8_t)0x6F)
#define TMC2209_REG_PWMCONF       ((uint8_t)0x70)

/*===== 上电缺省配置 =====
 * 步进电机: 1.42 两相, 0.9A/相, 1.8°, MRES=4 -> 16 微步 -> 3200 脉冲/圈
 * 采样电阻 110mΩ, V_FS = 0.325V (vsense=0)
 *   I_RMS_fullscale = 0.325/(0.11+0.02)/sqrt(2) = 1.768 A
 *   I_RMS = (IRUN+1)/32 * 1.768 A
 *   IRUN=14 -> 0.829 A ≈ 电机额定 0.9A 的 92% (与参考项目 U7 的降额比例一致)
 * 若实测力矩不足/发热异常, 只调这两个值, 不要动其它寄存器布局。 */
#define TMC_DEFAULT_IRUN        14u
#define TMC_DEFAULT_IHOLD       7u     /* 保持电流 ≈ IRUN 的 50% */
#define TMC_DEFAULT_IHOLDDELAY  3u
#define TMC_DEFAULT_TPOWERDOWN  20u
#define TMC_MRES_16USTEP        4u     /* CHOPCONF.MRES: 4 -> 16 微步 */

/*===== 寄存器位域 (TMC2209 datasheet Rev1.03, 与参考项目一致) =====*/

typedef union {
    uint32_t raw;
    struct {
        uint32_t I_scale_analog   : 1;
        uint32_t internal_Rsense  : 1;
        uint32_t en_SpreadCycle   : 1;
        uint32_t shaft            : 1;
        uint32_t index_otpw       : 1;
        uint32_t index_step       : 1;
        uint32_t pdn_disable      : 1;
        uint32_t mstep_reg_select : 1;
        uint32_t multistep_filt   : 1;
        uint32_t test_mode        : 1;
        uint32_t reserved         : 22;
    } bits;
} GCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t reset      : 1;
        uint32_t drv_err    : 1;
        uint32_t uv_cp      : 1;
        uint32_t reserved   : 29;
    } bits;
} GSTAT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ifcnt     : 8;
        uint32_t reserved  : 24;
    } bits;
} IFCNT_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t enn       : 1;
        uint32_t reserved1 : 1;
        uint32_t ms1       : 1;
        uint32_t ms2       : 1;
        uint32_t diag      : 1;
        uint32_t reserved2 : 1;
        uint32_t pdn_uart  : 1;
        uint32_t step      : 1;
        uint32_t spread_en : 1;
        uint32_t dir       : 1;
        uint32_t reserved3 : 22;
    } bits;
} IOIN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        /* IHOLD=bit4:0, IRUN=bit12:8, IHOLDDELAY=bit19:16 (TMC2209 布局, 非 TMC2208) */
        uint32_t ihold      : 5;
        uint32_t reserved1  : 3;
        uint32_t irun       : 5;
        uint32_t reserved2  : 3;
        uint32_t iholddelay : 4;
        uint32_t reserved3  : 12;
    } bits;
} IHOLD_IRUN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tpowerdown : 8;
        uint32_t reserved   : 24;
    } bits;
} TPOWERDOWN_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t tstep    : 20;
        uint32_t reserved : 12;
    } bits;
} TSTEP_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t toff      : 4;
        uint32_t hstrt     : 3;
        uint32_t hend      : 4;
        uint32_t reserved1 : 4;
        uint32_t tbl       : 2;
        uint32_t vsense    : 1;
        uint32_t reserved2 : 6;
        uint32_t mres      : 4;
        uint32_t intpol    : 1;
        uint32_t dedge     : 1;
        uint32_t diss2g    : 1;
        uint32_t diss2vs   : 1;
    } bits;
} CHOPCONF_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t ot        : 1;
        uint32_t otpw      : 1;
        uint32_t s2ga      : 1;
        uint32_t s2gb      : 1;
        uint32_t s2vsa     : 1;
        uint32_t s2vsb     : 1;
        uint32_t ola       : 1;
        uint32_t olb       : 1;
        uint32_t t120      : 1;
        uint32_t t143      : 1;
        uint32_t t150      : 1;
        uint32_t t157      : 1;
        uint32_t reserved1 : 4;
        uint32_t cs_actual : 5;
        uint32_t reserved2 : 3;
        uint32_t stealth   : 1;
        uint32_t stst      : 1;
        uint32_t reserved3 : 6;
    } bits;
} DRV_STATUS_reg_t;

typedef union {
    uint32_t raw;
    struct {
        uint32_t pwm_ofs       : 8;
        uint32_t pwm_grad      : 8;
        uint32_t pwm_freq      : 2;
        uint32_t pwm_autoscale : 1;
        uint32_t pwm_autograd  : 1;
        uint32_t freewheel     : 2;
        uint32_t reserved1     : 2;
        uint32_t pwm_reg       : 4;
        uint32_t pwm_lim       : 4;
    } bits;
} PWMCONF_reg_t;

/*===== API =====*/

/** @brief 初始化并配置 TMC2209 (内部含 3 轮重试), 失败时 s_ready 保持 0 */
void     BSP_TMC_Init(void);

/** @brief 1=通信正常且配置写入校验通过; 0=通信或校验失败(电机可能仍能转, 但微步/电流不确定) */
uint8_t  BSP_TMC_IsReady(void);

/** @brief 探测 UART 连通性 (读 GSTAT) */
uint8_t  BSP_TMC_IsAlive(void);

/** @brief 写 32 位寄存器 (无应答), 1=发送成功 */
uint8_t  BSP_TMC_WriteReg(uint8_t reg, uint32_t val);

/** @brief 读 32 位寄存器, 1=应答 CRC 正确 */
uint8_t  BSP_TMC_ReadReg(uint8_t reg, uint32_t *val);

/** @brief 写 + 读回校验 (最多重试 3 次), 1=一致 */
uint8_t  BSP_TMC_WriteRegVerified(uint8_t reg, uint32_t val, uint32_t *readback);

/** @brief 写入本板默认配置 (GCONF/IHOLD_IRUN/TPOWERDOWN/CHOPCONF/PWMCONF)
 *         GCONF 只校验 pdn_disable/mstep_reg_select/multistep_filt 三位;
 *         IHOLD_IRUN/TPOWERDOWN 为只写寄存器, 不做读回校验。
 *  @retval 1=全部通过 0=有关键寄存器校验失败 */
uint8_t  BSP_TMC_SetDefaults(void);

/** @brief 读 DRV_STATUS 供诊断用, 1=读取成功 */
uint8_t  BSP_TMC_GetDrvStatus(uint32_t *val);

#endif /* __BSP_TMC2209_H */
