# empty1 — MSPM0G3507 循迹小车

基于 TI MSPM0G3507 微控制器的循迹小车项目，集成了 LSM6DSV 六轴 IMU、灰度传感器阵列和双电机 PID 差速转向控制。

## 项目简介

- **MCU**: TI MSPM0G3507 (ARM Cortex-M0+)
- **传感器**: LSM6DSV 六轴 IMU (I2C1, PA16/PA17) + 2 路灰度传感器 (GPIO)
- **执行器**: 双路直流电机，PWM 驱动 (TIMG0 / TIMG6)
- **控制**: SysTick 10ms 中断闭环，PID 差速转向
- **调试**: UART 调试输出
- **构建工具链**: Keil MDK / IAR / TI Clang / GCC (全部支持)

## 目录结构

```
.
├── empty.c                 # 主程序：电机控制、灰度循迹、PID、IMU 初始化
├── lsm6dsv.c / .h          # LSM6DSV 六轴 IMU 驱动
├── uart_debug.c / .h       # UART 调试串口驱动
├── ti_msp_dl_config.c / .h # SysConfig 自动生成的外设配置
├── empty.syscfg            # SysConfig 工程配置文件
├── keil/                   # Keil MDK 工程
├── iar/                    # IAR Embedded Workbench 工程
├── ticlang/                # TI Clang 工程
└── gcc/                    # GCC 工程
```

## 硬件引脚分配

| 外设 | 引脚 | 功能 |
|------|------|------|
| I2C1 SDA | PA16 | IMU 数据线 |
| I2C1 SCL | PA17 | IMU 时钟线 |
| TIMG0 CH0/1 | - | 左电机 PWM (正/反) |
| TIMG6 CH0/1 | - | 右电机 PWM (正/反) |
| GPIO | PA24–PA29 | 6 路灰度传感器 |
| DEBUGSS | PA19/PA20 | SWD 调试接口 |

## 构建与烧录

### Keil MDK
1. 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`
2. 编译工程
3. 连接 LaunchPad，烧录运行

### IAR
1. 打开 `iar/empty_LP_MSPM0G3507_nortos_iar.eww`
2. 编译并下载

### GCC (命令行)
```bash
cd gcc
make
```

## 开发板信息

本项目基于 [LP-MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) LaunchPad 开发。

### 调试引脚配置

| 引脚 | 功能 | LaunchPad 跳线 |
|------|------|----------------|
| PA20 | SWCLK | `J101 15:16 ON` 调试时连接 XDS-110 |
| PA19 | SWDIO | `J101 13:14 ON` 调试时连接 XDS-110 |

## 许可证

本项目基于 BSD-3-Clause 许可证发布，详见源码文件头部版权声明。