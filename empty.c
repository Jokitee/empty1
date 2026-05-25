/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "lsm6dsv.h"
#include "uart_debug.h"
#include "car_control.h"

/* ========== 姿态角度全局变量 ========== */
LSM6DSV_Attitude_t g_imu_attitude = {0.0f, 0.0f, 0.0f};
LSM6DSV_Handle_t lsm6dsv_dev;

/* ========== 工作模式与状态变量 ========== */
typedef enum {
		Task_0 = 0,
    Task_1,   		/* 模式 1：任务1 */
    Task_2,       /* 模式 2：任务2 */
    Task_3,       /* 模式 3：任务3 */
    Task_4,       /* 模式 4：任务4 */
    Task_MAX
} TaskMode_t;

volatile TaskMode_t g_selected_mode = Task_0;   /* 当前按键选中的工作模式 */
volatile TaskMode_t g_running_mode  = Task_0;   /* 当前确认运行的工作模式 */
volatile uint8_t    g_task_confirmed = 0;       /* 工作使能标志: 1=工作, 0=停止等待 */
static uint8_t      g_yaw_init = 0;             /* 陀螺仪目标角度锁定状态 */
static volatile uint8_t g_task_success = 0;     /* 任务运行成功完成标志 */
static volatile uint16_t g_idle_led_counter = 0; /* 待机状态下红灯每秒闪烁计数 */
extern volatile int16_t  g_line_base_speed;

/* ========== 红灯闪烁控制 ========== */
void RED_LED_Flash(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        // 点亮红灯
        DL_GPIO_setPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
        delay_ms(150);
        // 熄灭红灯
        DL_GPIO_clearPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
        delay_ms(150);
    }
}

/* ========== 按键中断检测 (MODE与CONF键) ========== */
void GROUP1_IRQHandler(void)
{
    // 检测 GPIO 中断挂起组
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        
        /* ======= PORTA 中断: MODE 按键切换模式 ======= */
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
        {
            uint32_t pins = DL_GPIO_getEnabledInterruptStatus(GPIOA, MODE_GPIO_PIN_1_PIN);
            if (pins & MODE_GPIO_PIN_1_PIN) {
                delay_ms(20); // 软件去抖
                if (DL_GPIO_readPins(GPIOA, MODE_GPIO_PIN_1_PIN) == 0) {
                    // 仅在小车静止未开始工作时允许切换模式
                    if (!g_task_confirmed) {
                        if (g_selected_mode >= Task_4) {
                            g_selected_mode = Task_1;
                        } else {
                            g_selected_mode = (TaskMode_t)((int)g_selected_mode + 1);
                        }
                        // 闪烁红灯次数对应模式 (Task_1 闪1次，Task_2 闪2次，以此类推)
                        RED_LED_Flash((uint8_t)g_selected_mode);
                        g_idle_led_counter = 0; // 重置待机计时器，防止立即再次闪烁
                    }
                }
                DL_GPIO_clearInterruptStatus(GPIOA, MODE_GPIO_PIN_1_PIN);
            }
            break;
        }
        
        /* ======= PORTB 中断: CONF 按键启动/紧急停止 ======= */
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        {
            uint32_t pins = DL_GPIO_getEnabledInterruptStatus(GPIOB, CONF_GPIO_PIN_2_PIN);
            if (pins & CONF_GPIO_PIN_2_PIN) {
                delay_ms(20); // 软件去抖
                if (DL_GPIO_readPins(GPIOB, CONF_GPIO_PIN_2_PIN) == 0) {
                    if (!g_task_confirmed && g_selected_mode != Task_0) {
                        g_running_mode = g_selected_mode;
                        g_task_confirmed = 1;
                        
                        // 确定启动瞬间，将当前偏航角（Yaw）重置为 0，作为后续控制的起始基准
                        g_imu_attitude.yaw = 0.0f;
                        
                        // 确认工作提示：常亮红灯与蜂鸣器并发车延迟 0.5 秒
                        DL_GPIO_setPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
                        Buzzer_On();
                        delay_ms(500);
                        DL_GPIO_clearPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
                        Buzzer_Off();
                    } else {
                        // 如果已经在运行中，再次按下CONF键则作为“急停”信号
                        g_task_confirmed = 0;
                        g_running = 0;
                        g_targetA = 0;
                        g_targetB = 0;
                        Motor_Brake(&MotorA);
                        Motor_Brake(&MotorB);
                        RED_LED_Flash(5); // 快速闪烁5次提示已停机
                    }
                }
                DL_GPIO_clearInterruptStatus(GPIOB, CONF_GPIO_PIN_2_PIN);
            }
            break;
        }
        default:
            break;
    }
}

/* ========== 四种工作模式的实际逻辑 ========== */
void Task1_Process(void)
{
    /* 模式一：从 A 点出发直行到 B 点停车 */
    static uint8_t step = 0;
    Car_TrackLine(g_line_base_speed); // 启动闭环运行 (白区自动使用 Yaw 闭环直跑)
    
    if (!g_yaw_init) {
        step = 0;
        g_yaw_init = 1;
    }
    
    // 步骤 0：如果是从黑线起点出发，先等待小车驶离黑线（进入白区）
    if (step == 0) {
        if (g_line_state == LINE_NONE) {
            step = 1;
        }
    }
    // 步骤 1：在白区直行，直到再次触碰到黑线（终点 B），执行刹车
    else if (step == 1) {
        if (g_line_state == LINE_TRACK) {
            Car_Stop();
            g_task_success = 1;
            g_task_confirmed = 0; // 任务正常结束
            step = 0;
        }
    }
}

void Task2_Process(void)
{
    /* 模式二：圆形大环往返 (A -> B -> C -> D -> A 停车) */
    static uint8_t step = 0;
    static LineState_t last_state = LINE_NONE;
    
    Car_TrackLine(g_line_base_speed);
    
    if (!g_yaw_init) {
        step = 0;
        last_state = g_line_state; // 初始化为开始时的实际状态，避免在起点误触发状态改变
        g_yaw_init = 1;
    }
    
    // 检测状态切换 (白->黑 或 黑->白 判定为一个过渡节点)
    if (g_line_state != last_state) {
        last_state = g_line_state;
        step++;
    }
    
    // 经历4次状态转换（走完一周）回到 A 点白区后刹车停止
    if (step >= 4) {
        Car_Stop();
        g_task_success = 1;
        g_task_confirmed = 0;
    }
}

void Task3_Process(void)
{
    /* 模式三：双半圆 "8" 字往返 (A -> C -> B -> D -> A 停车) */
    static uint8_t step = 0;
    static LineState_t last_state = LINE_NONE;
    
    Car_TrackLine(g_line_base_speed);
    
    if (!g_yaw_init) {
        step = 0;
        last_state = g_line_state; // 初始化为开始时的实际状态，避免在起点误触发状态改变
        g_yaw_init = 1;
    }
    
    if (g_line_state != last_state) {
        last_state = g_line_state;
        step++;
    }
    
    // "8" 字形路径同样包含 4 段切换（两段直行、两段圆弧）
    if (step >= 4) {
        Car_Stop();
        g_task_success = 1;
        g_task_confirmed = 0;
    }
}

void Task4_Process(void)
{
    /* 模式四：连续行驶 4 圈 "8" 字往返 */
    static uint8_t step = 0;
    static LineState_t last_state = LINE_NONE;
    
    Car_TrackLine(g_line_base_speed);
    
    if (!g_yaw_init) {
        step = 0;
        last_state = g_line_state; // 初始化为开始时的实际状态，避免在起点误触发状态改变
        g_yaw_init = 1;
    }
    
    if (g_line_state != last_state) {
        last_state = g_line_state;
        step++;
    }
    
    // 4 圈 * 4 段切换 = 16 次状态改变后完成回 A 停车
    if (step >= 16) {
        Car_Stop();
        g_task_success = 1;
        g_task_confirmed = 0;
    }
}

/* ========== 主函数 ========== */
int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
    
    // 彻底关闭并清除未使用的编码器 B (GPIOA.8 / GPIOA.9) 的 GPIO 中断，防止产生干扰和死锁
    DL_GPIO_disableInterrupt(GPIOA, ENCODER_B_PHASE_A_PIN | ENCODER_B_PHASE_B_PIN);
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_B_PHASE_A_PIN | ENCODER_B_PHASE_B_PIN);
    
    // 开启中断控制
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    DL_SYSTICK_enableInterrupt(); // 开启 SysTick 10ms 中断
    
    /* 等待 LSM6DSV 传感器启动 */
    delay_cycles(32000 * 20);
    
    bool imu_status = LSM6DSV_Init(&lsm6dsv_dev);
    
    // 上电成功提示：闪烁 2 次
    RED_LED_Flash(2);
    
    g_task_confirmed = 0;
    g_running = 0;

    while (1)
    {
        delay_ms(10);
        
        /* 传感器数据更新 */
        Grayscale_ReadAll();
        g_line_pos   = Grayscale_GetLinePosition();
        g_line_state = Grayscale_GetState();
        
        if (imu_status) {
            LSM6DSV_UpdateAttitude(&lsm6dsv_dev, &g_imu_attitude, 0.01f);
        }
        
//        // 每 200ms 通过串口只输出一次 Yaw 的值
//        static uint8_t print_counter = 0;
//        print_counter++;
//        if (print_counter >= 20) {
//            print_counter = 0;
//            if (imu_status) {
//                Debug_UART_PrintYaw(g_imu_attitude.yaw);
//            }
//        }
        
        /* 待机状态下红灯每秒自动闪烁当前任务对应次数 */
        if (!g_task_confirmed && g_selected_mode != Task_0) {
            g_idle_led_counter++;
            if (g_idle_led_counter >= 100) { // 100 * 10ms = 1s
                g_idle_led_counter = 0;
                RED_LED_Flash((uint8_t)g_selected_mode);
            }
        } else {
            g_idle_led_counter = 0;
        }
        
        /* 任务运行与分配器 */
        if (g_task_confirmed) 
        {
            switch (g_running_mode) 
            {
                case Task_1:
                    Task1_Process();
                    break;
                case Task_2:
                    Task2_Process();
                    break;
                case Task_3:
                    Task3_Process();
                    break;
                case Task_4:
                    Task4_Process();
                    break;
                default:
                    break;
            }
        } 
        else 
        {
            // 等待确认或已停机状态下，小车保持静止
            g_running = 0;
            g_targetA = 0;
            g_targetB = 0;
            Motor_Set(&MotorA, 0);
            Motor_Set(&MotorB, 0);
            g_yaw_init = 0; // 重置陀螺仪航向锁定标志
            
            // 如果刚刚是任务正常且成功走完自动停机，发出三短鸣声光提示
            if (g_task_success) {
                g_task_success = 0;
                for (uint8_t i = 0; i < 3; i++) {
                    DL_GPIO_setPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
                    Buzzer_On();
                    delay_ms(500);
                    DL_GPIO_clearPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
                    Buzzer_Off();
                    delay_ms(500);
                }
            }
        }
    }
}