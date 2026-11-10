# 主机侧回归测试 (U2)

把 U2 工程里**真实的那份源码**拿到 PC 上编译运行，用来在没有板子的情况下验证
与硬件无关的逻辑。不改变 Keil 工程，不参与固件编译。

| 测试 | 覆盖什么 | 做法 |
|------|----------|------|
| `test_frame.c` | 帧的 CRC16/组帧/解帧/字节序 | 直接编译 `Core/Src/protocol_frame.c` |
| `test_motor.c` | STEP 脉冲计数、加减速斜坡、启停、参数边界 | 直接编译 `Core/Src/bsp_motor.c`，把 `TIM3` 换成内存里的假寄存器组 |

## 跑法

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File U2\test_host\run.ps1
```

`-ExecutionPolicy Bypass` 是必须的：这些脚本没签名，本机默认策略禁止运行。
`run.ps1` 存的是 **UTF-8 带 BOM**，因为 Windows PowerShell 5.1 对无 BOM 的
UTF-8 脚本会按 GBK 解析，中文注释会把语法撑坏（已经踩过一次）。

需要 MinGW 的 gcc，默认找 `C:\Program Files\mingw64\bin\gcc.exe`，
可以用 `-Gcc <路径>` 覆盖。Keil 的 armcc 不能拿来跑这些测试——它们是 x86 程序。

## 说明

- `test_motor.c` 里 `#include "../Core/Src/bsp_motor.c"` 是故意的：这样测的就是
  Keil 编译的同一份代码，不是复制品，不会出现「测试通过但固件里是旧逻辑」。
- `stub/main.h` 只提供 `bsp_motor.c` 用到的那几个符号（TIM3 寄存器、HAL_NVIC_*、
  HAL_GPIO_WritePin），不模拟任何外设行为——寄存器读写就是普通内存读写，
  所以它验证的是**逻辑**（斜坡数学、计数、终止条件），不验证时序。
- `run.ps1` 会从 `Core/Src/bsp_tmc2209.c` 里正则抽出 `tmc_crc8()` 函数体，
  拼进 `build/crc8_extract.c` 供 `test_frame.c` 使用。抽不到会直接报错，
  不会静默跳过。
