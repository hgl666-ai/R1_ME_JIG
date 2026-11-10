# R1 破碎机编码板测试治具 —— U2 固件对外接口

本文档描述 U2（主控，USB CDC 接 PC）当前固件实现的命令集。帧格式沿用
`INFO/M1 测试治具PC端通讯协议.xlsx`，命令码是本板新定的。

固件版本 **1.0.1**，Keil 工程 `U2/MDK-ARM/U2.uvprojx`，目标 `U2`。

---

## 1. 硬件映射（U2）

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| USB CDC | PA11 / PA12 | USB FS Device | `MX_USB_DEVICE_Init()`，枚举成虚拟串口 |
| TMC2209 UART | PB10 (TX) / PB11 (RX) | USART3 @115200 8N1 | 单线半双工，见下节 |
| 步进 STEP | PB1 | TIM3_CH4 | PWM 1，占空比固定 50% |
| 步进 DIR | PB0 | GPIO 推挽 | 低=CW，高=CCW |
| MRE 采样 | PA0 | ADC1_IN0 | `MRE-ADC` → U7(LM358 跟随) → 本引脚 |
| 3.3V 监视 | PA4 | ADC1_IN4 | `3V3_ADC0`（3.3V 经 10k/10k 分压）→ U5 跟随 |
| 调试串口 | PA9 / PA10 | USART1 @115200 | 预留（U2↔U3 的 H3 交叉连接） |
| 系统灯 | PC13 | GPIO 推挽 | 低电平点亮 |

### TMC2209 单线串口

`PDN_UART`(U8 pin14) 同时接 MCU 的 TX 和 RX，因此 **MCU 会收到自己的回显**。
驱动里做了两件事规避：

1. 发送期间把 `USART3->CR1` 的 `RE` 清 0，不发不收，发送完成后再置 1；
2. 发送完成后把 PB10 从复用推挽切成输入上拉（`TMC_RELEASE_TX_ON_RX`），
   让 TMC2209 应答时不会和 MCU 的推挽输出对顶。

> 待上板确认：原理图上 `PDN_UART` 直连的是 `MOTO0_RX` 网名（=PB10=USART3_TX），
> 1kΩ 串在 `MOTO0_TX`（=PB11=USART3_RX）一侧；按 USART3 标准引脚理解，
> 1kΩ 应该在 TX 侧。两种接法下第 2 条都能工作，但最好量一下波形确认。

---

## 2. 数据帧格式

```
+--------+---------+---------+---------+--------+----------+
| 帧头 F | 命令 C  | 帧序号N | 长度 L  | 数据 D | CRC16    |
| 2 字节 | 2 字节  | 2 字节  | 2 字节  | L 字节 | 2 字节   |
| A5 5B  | uint16  | uint16  | uint16  |   *    | uint16   |
+--------+---------+---------+---------+--------+----------+
```

- 所有多字节字段 **大端**（高字节在前）
- CRC16 = **MODBUS**（多项式 0xA001 反射，初值 0xFFFF），范围从帧头到数据末尾
- 帧序号：PC 发送从 0x01 递增，到 0xFFFF 回 1；治具 ACK 原样回填；
  治具主动上报固定用 0
- 最短帧 10 字节（L=0）
- USB CDC 的波特率是名义值，PC 端设多少都不影响实际速率

---

## 3. 命令集

应答数据的 **第 1 字节固定为状态码**。

### 3.1 通用状态码

| 值 | 名称 | 含义 |
|----|------|------|
| 0x00 | OK | 成功 |
| 0x01 | BUSY | 系统忙（电机正在运动时收到 MOVE） |
| 0x0B | ERR_PARAM | 参数错误（长度不对 / 方向非法 / 步数为 0） |
| 0x08 | ERR_CRC | 帧 CRC 错（**仅诊断，不回 ACK**） |
| 0xFF | NIY | 命令未实现 |

其余状态码（ERR_SIZE/ERR_FLASH/ERR_PROG/…）沿用 M1 协议表，本版固件暂未使用。

### 3.2 命令表

| 命令 | 值 | PC→治具 数据 | 治具→PC 应答数据 |
|------|----|--------------|------------------|
| PING | 0x0001 | 无 | `状态码` |
| GET_VERSION | 0x0002 | 无 | `状态码` + `主` + `次` + `修订` |
| MOTOR_MOVE | 0x0010 | `dir(1)` + `steps(4)` + `freq_hz(2)` | `状态码` |
| MOTOR_STOP | 0x0011 | 无 | `状态码` |
| MOTOR_STATUS | 0x0012 | 无 | `状态码` + `运行标志(1)` + `已走步数(4)` + `目标步数(4)` |
| MRE_QUERY | 0x0020 | 无 | `状态码` + MRE 快照（下表） |

### 3.3 MOTOR_MOVE 参数

- `dir`：0=CW，1=CCW
- `steps`：1 ~ 100000000，超出返回 ERR_PARAM
- `freq_hz`：巡航步进频率，实际夹到 **300 ~ 20000 Hz**
- 运动为**非阻塞**：立即回 ACK，之后用 MOTOR_STATUS 查进度
- 运动过程中再发 MOVE 返回 BUSY
- 有梯形加减速：起步 500Hz，加减速各最多 800 步（总步数不足时对折）
- 16 微步 → 一圈 = 3200 步

### 3.4 MRE_QUERY 应答数据（共 22 字节）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1 | 状态码 | 0x00 |
| 1 | 1 | flags | bit0=ADC 正在采集，bit1=TMC2209 配置校验通过 |
| 2 | 2 | mre_last | 最近一次 MRE 原始值（12bit） |
| 4 | 2 | mre_min | 最近一个窗口内的最小值 |
| 6 | 2 | mre_max | 最近一个窗口内的最大值 |
| 8 | 2 | mre_mean | 最近一个窗口内的平均值 |
| 10 | 2 | mre_mv | `mre_last` 换算的 mV（VDDA=3300mV） |
| 12 | 2 | vref_last | 3V3_ADC0 通道原始值（理论 ≈2047，即 1650mV） |
| 14 | 2 | vref_mv | 同上，mV |
| 16 | 2 | window_size | 本窗口样本数（固定 256） |
| 18 | 4 | total_samples | 上电以来累计 MRE 样本数 |

采样参数：ADC 12MHz、采样时间 239.5 周期 → 每通道约 **23.8 kSa/s**，
一个窗口 256 点 ≈ **10.7 ms**，每 10.7 ms 刷新一次统计。

---

## 4. 现成样例帧（CRC 已核对）

上板测试的完整流程和"期望应答"见 [TESTING.md](TESTING.md)。这里只列发送帧，
统一用帧序号 `00 01`：

| 用途 | 整帧（十六进制） |
|------|------------------|
| PING | `A5 5B 00 01 00 01 00 00 89 0C` |
| GET_VERSION | `A5 5B 00 02 00 01 00 00 89 48` |
| MOTOR_STATUS | `A5 5B 00 12 00 01 00 00 4A 89` |
| MRE_QUERY | `A5 5B 00 20 00 01 00 00 8E B0` |
| MOVE 200 步 / 1000Hz / 正转 | `A5 5B 00 10 00 01 00 07 00 00 00 00 C8 03 E8 C3 2F` |
| MOVE 3200 步 / 4000Hz / 正转 | `A5 5B 00 10 00 01 00 07 00 00 00 0C 80 0F A0 B3 A9` |
| MOVE 3200 步 / 4000Hz / 反转 | `A5 5B 00 10 00 01 00 07 01 00 00 0C 80 0F A0 73 B9` |
| MOTOR_STOP | `A5 5B 00 11 00 01 00 00 4A CD` |

上面这些帧由 [tools/gen_serial_frames.py](tools/gen_serial_frames.py) 生成。
主机侧回归测试（[test_host/](test_host/)）会把 C 实现组出来的 **PING 帧**和
**MOVE 3200 步帧**与上表逐字节比对，当前一致。

---

## 5. 上电流程

1. `MX_GPIO_Init` / `MX_ADC1_Init` / `MX_USART1` / `MX_USART3` / `MX_USB_DEVICE_Init`
2. `BSP_Motor_Init()` —— 配置 TIM3_CH4、打开更新中断
3. `BSP_ADC_Init()` —— 重配 ADC1 为双通道扫描 + 连续转换 + DMA 循环并启动
4. `BSP_TMC_Init()` —— 探测 TMC2209（读 GSTAT），成功后写 GCONF/IHOLD_IRUN/
   TPOWERDOWN/CHOPCONF/PWMCONF，关键寄存器读回校验；最多重试 3 轮
5. `PROTO_Init()`
6. 主循环只做一件事：`PROTO_Run()`

> **注意**：`BSP_TMC_Init()` 在 TMC2209 不在线时最坏阻塞约 200ms（3 轮 × 50ms
> 加上读超时）。USB 枚举已经在此之前启动，不影响 PC 识别设备。

> **重新生成工程的坑**：`bsp_motor.c` / `bsp_adc.c` / `bsp_tmc2209.c` /
> `protocol*.c` 都是新增文件，放在 `Core/Src` 下，CubeMX 不会动它们；但
> `U2.uvprojx` 里的 `Application/User/BSP` 分组是手工加的，用 CubeMX 重新生成
> 工程后这个分组会丢，Keil 会报链接错误。重新生成后把这一组文件加回去即可
> （6 个 .c，都在 `Core/Src`）。

---

## 6. 默认电机参数（`bsp_tmc2209.h` 可调）

| 项 | 值 | 依据 |
|----|----|------|
| 微步 | 16（CHOPCONF.MRES=4） | 3200 脉冲/圈，与电机 1.8°/步一致 |
| IRUN | 14 | 采样电阻 110mΩ、V_FS=0.325V → 满量程 1.768A；(14+1)/32×1.768 ≈ **0.83A**，约为电机额定 0.9A 的 92% |
| IHOLD | 7 | 保持电流约为运行电流的一半 |
| 斩波 | StealthChop2 | SPREAD 引脚状态未知，先按静音模式跑 |
| TPOWERDOWN | 20 | 到位后约 0.4s 降到 IHOLD |

力矩不够就把 `TMC_DEFAULT_IRUN` 往上调，发热异常就往下调；不要改寄存器位域布局
（`IHOLD_IRUN` 的 IRUN 在 bit12:8，和 TMC2208 不同）。

---

## 7. 已知待上板确认项

1. **TMC2209 串口方向**：见第 1 节末尾。若读寄存器一直失败，先把
   `bsp_tmc2209.c` 里的 `TMC_RELEASE_TX_ON_RX` 改成 0 再试，或反过来。
2. **ADC 通道 2 的采样**：`HAL_ADC_ConfigChannel` 对 rank2 的配置若未生效，
   `vref_last` 会一直是 0 —— 用 MRE_QUERY 一看就知道。
3. **PC13 系统灯**：当前 `MX_GPIO_Init` 把它拉低（=常亮），固件没有闪烁逻辑。
4. **PC10 与 PA4 同网络**：原理图上 `U2_ADC0` 同时接到 PA4 和 PC10。
   PA4 被配成模拟输入，PC10 仍是 `MX_GPIO_Init` 里的浮空输入。如果实测 PC10
   漏电流影响读数，把 PC10 也改成模拟输入。
