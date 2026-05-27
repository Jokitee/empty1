# MSPM0G3507 智能循迹与航向锁死控制小车项目

[![MCU](https://img.shields.io/badge/MCU-MSPM0G3507-blue.svg)](https://www.ti.com/product/MSPM0G3507)
[![Toolchain](https://img.shields.io/badge/Toolchain-Keil%20MDK%20%2F%20IAR-orange.svg)](https://www.keil.com/)
[![IMU](https://img.shields.io/badge/IMU-LSM6DSV-green.svg)](https://www.st.com/en/mems-and-sensors/lsm6dsv.html)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-lightgrey.svg)](LICENSE)

本项目基于德州仪器（TI）最新一代 **MSPM0G3507** (Cortex-M0+) 微控制器，设计并实现了一套用于 2024 年全国大学生电子设计竞赛（H题：自动行驶小车）的高性能、双运动模式控制系统。该系统集成了高性能 LSM6DSV 六轴惯性传感器（IMU）、高灵敏度 6 路灰度传感器阵列，实现了高精度直线姿态锁死、自适应灰度线轨追踪，以及创新的进线主动捕获算法。

---

## 🚀 核心技术与算法

### 1. 灰度加权平均偏差模型 (Grayscale PID)
通过 6 路红外灰度传感器阵列，利用加权平均算法动态解算小车偏离轨道中心线的精确横向距离（范围在 `-2.5` 到 `+2.5` 之间）。通过位置式 PID 控制器调节双侧轮差速，实现流畅稳定的高速巡线。

### 2. 陀螺仪偏航角锁死直线控制 (Yaw-PID Locking)
在白底无轨区域（如直线冲刺段），系统自动切入 `LINE_NONE` 模式。此时，小车将出线瞬间的姿态角（Yaw）作为目标锁定角。通过 LSM6DSV 陀螺仪的实时解算，配合去积分项的 PD 偏航控制算法，消除轮胎打滑与局部机械扭矩偏差，确保小车在无轨区域绝对笔直行驶。

### 3. 主动切线捕获机制 (Active Entry Capture)
为了克服高速过弯与白底切入黑线时的惯性脱轨，系统设计了主动切线捕获机制：
*   **减速增扭**：切入黑线瞬间启动 60 ticks（600ms）保护期，限制基准平动车速并放大转向最大差速限制至 `850`（原限制为 `400`）。
*   **清除历史惯性**：瞬间清空上一周期的 PID 积分量和 `last_err`，消除反向累积惯性。
*   **智能定向锁存**：若在快速摆头纠偏时发生瞬间全白丢线，算法会自动锁存切入方向的横向偏差最大值，使小车以最快速度回正并切入常规循迹。

---

## 📂 项目目录结构

```
.
├── empty.c                 # 系统初始化、主状态机及基础任务控制
├── car_control.c / .h      # 电机驱动、6路灰度控制、PID算法及主动捕获接口
├── lsm6dsv.c / .h          # LSM6DSV 六轴 IMU (I2C) 驱动库
├── uart_debug.c / .h       # 串口重定向非阻塞调试驱动
├── ti_msp_dl_config.c / .h # TI SysConfig 自动生成的外设底层配置
├── empty.syscfg            # SysConfig 硬件资源分配与引脚管理配置文件
├── build_docx_report.py    # Python 自动化编译排版工具 (MD -> 电赛标准 Word)
├── design_report.md        # 竞赛技术设计报告 (Markdown 源码版)
├── keil/                   # Keil MDK 5 编译工程目录
├── iar/                    # IAR Embedded Workbench 工程目录
├── ticlang/                # TI Clang 编译配置目录
└── gcc/                    # GCC 交叉编译工具链工程与 Makefile
```

---

## 🔌 硬件引脚分配

| 外设模块 | 引脚名称 | 硬件引脚 | 描述/功能说明 |
| :--- | :--- | :--- | :--- |
| **I2C1 SDA** | PA16 | 接 LSM6DSV SDA | IMU 传感器双向数据线 |
| **I2C1 SCL** | PA17 | 接 LSM6DSV SCL | IMU 传感器时钟线 |
| **TIMG0 CH0/1** | PA12 / PA13 | 电机A PWM | 左侧电机正反向 PWM 速度驱动控制 |
| **TIMG6 CH0/1** | PA28 / PA31 | 电机B PWM | 右侧电机正反向 PWM 速度驱动控制 |
| **GPIO Inputs** | PA24 ~ PA29 | 6路灰度输入 | 灰度传感器阵列（从左到右：L2, L1, M1, M2, R1, R2） |
| **Buzzer/LED** | PA15 / PA14 | 报警输出 | 报警蜂鸣器与状态指示 LED |
| **DEBUGSS** | PA19 / PA20 | SWD 接口 | `SWDIO` / `SWCLK` 芯片调试仿真口 |

---

## 🛠️ 构建与烧录

### 1. 使用 Keil MDK
1. 进入 `keil/` 目录，双击打开 `empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
2. 在 Keil MDK 中直接进行编译（F7）与烧录（F8）。
3. 确保 LaunchPad 上的 J101 调试跳线（PA19/PA20）连接正常。

### 2. 使用 GCC 工具链
1. 安装 `arm-none-eabi-gcc` 交叉编译器并配置环境变量。
2. 进入 `gcc/` 目录，在命令行中执行 `make` 进行编译。

---

## 📝 自动化生成电赛标准设计报告

本项目附带了一套 **Markdown 到 Word 自动转换排版工具**，专门针对**全国大学生电子设计竞赛（TI杯）官方文档格式要求**进行定制优化。

### 1. 自动转换的格式特性
*   **字体混排**：中文自动应用宋体/黑体，英文字母与数学公式自动应用 Times New Roman。
*   **学术排版**：自动生成标准的**三线表**、公式变量斜体排版、正文首行缩进等。
*   **流程图渲染**：自动调用 Mermaid 渲染服务器，将文档中内嵌的流程图转换为高清 PNG 并优雅嵌入文档。

### 2. 转换方法
只需在安装了 Python 3 的环境中安装 `python-docx` 与 `requests`，并在项目根目录下执行：
```bash
pip install python-docx requests
python build_docx_report.py
```
执行完毕后，项目根目录下将自动生成 **`design_report.docx`**。

---

## ⚖️ 许可证

本项目依据 **BSD 3-Clause License** 许可协议开源，详细版权声明见源码头部。