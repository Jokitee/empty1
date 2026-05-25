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

#define PWM_MAX         2100

/* ========== 电机结构体 ========== */
typedef struct {
    GPTIMER_Regs *timer;
    DL_TIMER_CC_INDEX ch_forward;
    DL_TIMER_CC_INDEX ch_backward;
} Motor_t;

/* TIMG0 and TIMG6 are TimerG which only have 2 channels (CC_0 and CC_1) */
Motor_t MotorA = {TIMG0, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX};
Motor_t MotorB = {TIMG6, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX};

///* ========== 编码器A：硬件QEI（TIMG8） ========== */
//#define TIMG8_CTR       (*(volatile uint32_t *)(0x40091008))

//typedef struct {
//    int32_t last_count;
//    int32_t delta;
//} Encoder_Hardware_t;

//Encoder_Hardware_t EncoderA = {0, 0};

///* ========== 编码器B：软件编码器（GPIO中断） ========== */
//typedef struct {
//    volatile int32_t count;
//    volatile uint8_t last_state;
//} Encoder_Software_t;

//Encoder_Software_t EncoderB = {0, 0};

/* ========== 6路灰度传感器 ========== */
#define GRAY_NUM        2

#define GRAY_L2_PIN     DL_GPIO_PIN_24
#define GRAY_L1_PIN     DL_GPIO_PIN_25
#define GRAY_M1_PIN     DL_GPIO_PIN_26
#define GRAY_M2_PIN     DL_GPIO_PIN_27
#define GRAY_R1_PIN     DL_GPIO_PIN_28
#define GRAY_R2_PIN     DL_GPIO_PIN_29

#define GRAY_ALL_PINS   (GRAY_L2_PIN | GRAY_L1_PIN | GRAY_M1_PIN | \
                         GRAY_M2_PIN | GRAY_R1_PIN | GRAY_R2_PIN)
#define DEBUG_UART      DEBUG_UART_INST
/* 灰度值：0=白，1=黑 */
uint8_t gray_value[GRAY_NUM];

/* ========== PID结构体 ========== */
typedef struct {
    float kp, ki, kd;
    float err, last_err, integral;
    int16_t output;
} PID_t;

//PID_t pid_a = {2.0f, 0.8f, 0.0f, 0, 0, 0, 0};
//PID_t pid_b = {2.0f, 0.8f, 0.0f, 0, 0, 0, 0};

/* 循迹转向PID */
PID_t pid_line = {100.0f, 0.0f, 50.0f, 0, 0, 0, 0};

/* 循迹基础速度 */
#define LINE_BASE_SPEED   1500

/* ========== 姿态角度全局变量 ========== */
LSM6DSV_Attitude_t g_imu_attitude = {0.0f, 0.0f, 0.0f};

/* ========== 延时 ========== */
void delay_ms(uint32_t ms)
{
    DL_Common_delayCycles(ms * (32000000 / 1000));
}

/* ========== 电机控制（直接输出） ========== */

/**
 * @brief  电机速度设置（无启动冲击，直接输出）
 */
void Motor_Set(Motor_t *motor, int16_t speed)
{
    /* Clamp speed to valid PWM range */
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;
    
    if (speed > 0) {
        DL_TimerG_setCaptureCompareValue(motor->timer, (uint32_t)speed, motor->ch_forward);
        DL_TimerG_setCaptureCompareValue(motor->timer, 0, motor->ch_backward);
    }
    else if (speed < 0) {
        DL_TimerG_setCaptureCompareValue(motor->timer, 0, motor->ch_forward);
        DL_TimerG_setCaptureCompareValue(motor->timer, (uint32_t)(-speed), motor->ch_backward);
    }
    else {
        DL_TimerG_setCaptureCompareValue(motor->timer, 0, motor->ch_forward);
        DL_TimerG_setCaptureCompareValue(motor->timer, 0, motor->ch_backward);
    }
}

void Motor_Brake(Motor_t *motor)
{
    DL_TimerG_setCaptureCompareValue(motor->timer, PWM_MAX, motor->ch_forward);
    DL_TimerG_setCaptureCompareValue(motor->timer, PWM_MAX, motor->ch_backward);
}

void Motor_Init(void)
{
    DL_TimerG_startCounter(TIMG0);
    DL_TimerG_startCounter(TIMG6);
}

/* ========== 转向控制函数 ========== */

/* 前向声明 */
void PID_Update(PID_t *pid, int32_t target, int32_t actual);

/**
 * @brief  灰度PID转向控制函数
 *
 *   仅使用灰度传感器位置偏差, 通过PID输出转向PWM
 *   单输入(灰度位置) → 单输出(转向PWM)
 *
 * @param  line_pos  灰度传感器线路偏差 (-2.5 ~ +2.5)
 * @return 转向PWM值
 */
int16_t SteerControl(float line_pos)
{
    PID_Update(&pid_line, 0, (int32_t)(line_pos * 100));
    return pid_line.output;
}

///* ========== 编码器A：硬件QEI ========== */
//void EncoderA_Init(void)
//{
//    EncoderA.last_count = (int32_t)DL_TimerG_getTimerCount(ENCODER_A_INST);
//}

//int32_t EncoderA_GetCount(void)
//{
//    return (int32_t)DL_TimerG_getTimerCount(ENCODER_A_INST);
//}

//int32_t EncoderA_Update(void)
//{
//    int32_t current = (int32_t)DL_TimerG_getTimerCount(ENCODER_A_INST);
//    int32_t delta = current - EncoderA.last_count;
//    /* 16-bit QEI timer rollover correction */
//    if (delta > 32768) 
//        delta -= 65536;
//    else if (delta < -32768) 
//        delta += 65536;
//    EncoderA.last_count = current;
//    EncoderA.delta = delta;
//    return delta;
//}

///* ========== 编码器B：软件编码器（GPIO中断） ========== */
//static const int8_t qei_table[16] = {
//    0, -1,  1,  0,
//    1,  0,  0, -1,
//   -1,  0,  0,  1,
//    0,  1, -1,  0
//};

//void GROUP1_IRQHandler(void)
//{
//    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
//        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
//        {
//            uint32_t pins = DL_GPIO_getEnabledInterruptStatus(GPIOA, DL_GPIO_PIN_8 | DL_GPIO_PIN_9);
//            if (pins & (DL_GPIO_PIN_8 | DL_GPIO_PIN_9))
//            {
//                uint8_t a = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_8) ? 1 : 0;
//                uint8_t b = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_9) ? 1 : 0;
//                uint8_t curr = (a << 1) | b;
//                
//                uint8_t index = (EncoderB.last_state << 2) | curr;
//                EncoderB.count += qei_table[index];
//                EncoderB.last_state = curr;
//                
//                DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_8 | DL_GPIO_PIN_9);
//            }
//            break;
//        }
//        default:
//            break;
//    }
//}

//void EncoderB_Init(void)
//{
//    uint8_t a = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_8) ? 1 : 0;
//    uint8_t b = DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_9) ? 1 : 0;
//    EncoderB.last_state = (a << 1) | b;
//    EncoderB.count = 0;
//    
//    DL_GPIO_clearInterruptStatus(GPIOA, DL_GPIO_PIN_8 | DL_GPIO_PIN_9);
//    DL_GPIO_enableInterrupt(GPIOA, DL_GPIO_PIN_8 | DL_GPIO_PIN_9);
//    NVIC_EnableIRQ(GPIOA_INT_IRQn);
//}

//int32_t EncoderB_GetCount(void)
//{
//    return EncoderB.count;
//}

//int32_t EncoderB_Update(void)
//{
//    static int32_t last = 0;
//    int32_t current = EncoderB.count;
//    int32_t delta = current - last;
//    last = current;
//    return delta;
//}

/* ========== 6路灰度传感器 ========== */
void Grayscale_ReadAll(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIOA, GRAY_ALL_PINS);
    
//    gray_value[0] = (pins & GRAY_L2_PIN) ? 0 : 1;   /* 左外 L2 */
//    gray_value[1] = (pins & GRAY_L1_PIN) ? 0 : 1;   /* 左内 L1 */
    gray_value[0] = (pins & GRAY_M1_PIN) ? 0 : 1;   /* 中左 M1 */
    gray_value[1] = (pins & GRAY_M2_PIN) ? 0 : 1;   /* 中右 M2 */
//    gray_value[4] = (pins & GRAY_R1_PIN) ? 0 : 1;   /* 右内 R1 */
//    gray_value[5] = (pins & GRAY_R2_PIN) ? 0 : 1;   /* 右外 R2 */
}

/**
 * @brief  获取黑线状态
 */
typedef enum {
    LINE_STRAIGHT,   /* 中间两路黑，直线 */
    LINE_LEFT,       /* 左路黑，左转 */
    LINE_RIGHT,      /* 右路黑，右转 */
    LINE_CROSS,      /* 十字路口/全黑 */
    LINE_NONE        /* 全白，丢线 */
} LineState_t;

LineState_t Grayscale_GetState(void)
{
//    uint8_t l2 = gray_value[0];
//    uint8_t l1 = gray_value[1];
    uint8_t m1 = gray_value[0];
    uint8_t m2 = gray_value[1];
//    uint8_t r1 = gray_value[4];
//    uint8_t r2 = gray_value[5];
    
    uint8_t total =  m1 + m2 ;
    
    /* 全白 */
    if (total == 0) return LINE_NONE;
    
    /* 中间两路有黑线 → 直线（最高优先级）*/
    if (m1 || m2) return LINE_STRAIGHT;
    
    /* 左路有黑线 → 左转 */
    if (m1 || !m2) return LINE_LEFT;
    
    /* 右路有黑线 → 右转 */
    if (!m1 || m2) return LINE_RIGHT;
    
    /* 默认直线 */
    return LINE_STRAIGHT;
}

/**
 * @brief  计算黑线位置（用于PID微调）
 * @return -2.5 ~ +2.5
 */
float Grayscale_GetLinePosition(void)
{
    const float weights[GRAY_NUM] = {-1.0f,1.0f};
    
    int32_t sum = 0, cnt = 0;
    
    for (uint8_t i = 0; i < GRAY_NUM; i++) {
        if (gray_value[i]) {
            sum += (int32_t)(weights[i] * 10);
            cnt++;
        }
    }
    
    if (cnt == 0) return 0;
    
    return (float)sum / (cnt * 10);
}

/* ========== PID ========== */
void PID_Update(PID_t *pid, int32_t target, int32_t actual)
{
    pid->err = target - actual;
    pid->integral += pid->err;
    
    /* Anti-windup */
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;
    
    float out = pid->kp * pid->err 
              + pid->ki * pid->integral 
              + pid->kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;
    
    /* Output clamping to prevent duty cycle exceeding period */
    if (out > PWM_MAX) out = PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;
    
    pid->output = (int16_t)out;
}

/* ========== 10ms闭环中断 ========== */
volatile int16_t  g_targetA = 0, g_targetB = 0;  // 启动阶段直通目标PWM
volatile float    g_line_pos   = 0.0f;       // 灰度位置, 写入: main loop
volatile uint8_t  g_line_state = LINE_NONE;  // 循迹状态, 写入: main loop
volatile uint8_t  g_running    = 0;          // 运行标志: 1=循迹中, 0=启动/停车

/**
 * @brief  SysTick中断: 执行PID转向控制 + 电机输出
 */
void SysTick_Handler(void)
{
    if (!g_running) {
        /* 启动/停车阶段: 直接输出 g_target */
        Motor_Set(&MotorA, g_targetA);
        Motor_Set(&MotorB, g_targetB);
        return;
    }

    int16_t left_speed, right_speed;

    switch (g_line_state) {
        case LINE_STRAIGHT:
        case LINE_CROSS:
        case LINE_LEFT:
        case LINE_RIGHT:
        {
            /* 所有循迹状态: 灰度PID转向控制 */
            PID_Update(&pid_line, 0, (int32_t)(g_line_pos * 100));
            int16_t steer_pwm = pid_line.output;

            int16_t max_steer = LINE_BASE_SPEED / 2;
            if (steer_pwm >  max_steer) steer_pwm =  max_steer;
            if (steer_pwm < -max_steer) steer_pwm = -max_steer;

            if(steer_pwm >= 0){
							left_speed = LINE_BASE_SPEED - steer_pwm;
							right_speed = LINE_BASE_SPEED;
						}else{
							left_speed = LINE_BASE_SPEED;
							right_speed = LINE_BASE_SPEED + steer_pwm;
						}
            break;
        }

        case LINE_NONE:
        default:
            /* 丢线: 低速前进 */
            left_speed  = 1400;
            right_speed = 1400;
            pid_line.integral = 0;
            pid_line.last_err = 0;
            break;
    }

    Motor_Set(&MotorA, left_speed);
    Motor_Set(&MotorB, right_speed);
}

    /* Initialize LSM6DSV IMU */
    LSM6DSV_Handle_t lsm6dsv_dev;

/* ========== 主函数 ========== */
int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();
//    EncoderA_Init();
//    EncoderB_Init();
    DL_SYSTICK_enableInterrupt(); // Enable SysTick interrupt for PID closed-loop control!
    
    /* Wait for LSM6DSV boot time (~20ms) */
    delay_cycles(32000 * 20);
    

    bool imu_status = LSM6DSV_Init(&lsm6dsv_dev);
    
//    if (imu_status) {
//        UART_SendString("LSM6DSV Init Success!\r\n");
//        #if IMU_USE_PRESET_CALIBRATION
//        LSM6DSV_LoadPresetCalibrate(&lsm6dsv_dev, &g_imu_attitude);
//        UART_SendString("Using preset calibration.\r\n");
//        #else
//        UART_SendString("Auto-calibrating... keep sensor STILL for 1s...\r\n");
//        if (LSM6DSV_BootCalibrate(&lsm6dsv_dev, &g_imu_attitude)) {
//            /* 打印实测偏置值，方便用户更新预设 */
//            int32_t bz = (int32_t)(lsm6dsv_dev.gyro_bias_z * 1000.0f);
//            UART_SendString("Calib OK! bias_z=");
//            UART_SendInt(bz / 1000);
//            UART_SendChar('.');
//            int32_t bz_frac = bz < 0 ? -bz : bz;
//            UART_SendInt(bz_frac % 1000);
//            UART_SendString(" dps  <-- update IMU_PRESET_BIAS_Z\r\n");
//        } else {
//            UART_SendString("Calib FAILED (moving or vibration). Using zero bias.\r\n");
//        }
//        #endif
//    } else {
//        UART_SendString("LSM6DSV Init Failed!\r\n");
//    }
    
    /* ====== 启动阶段: 直接输出PWM克服静摩擦 ====== */
    g_running = 0;
    g_targetA = 1600;
    g_targetB = 1600;
    delay_ms(500);  /* 启动持续500ms */

    /* ====== 正常循迹主循环 (只负责传感器读取) ====== */
    g_running = 1;   /* 告诉中断: 开始循迹控制 */
    while (1)
    {
        delay_ms(10);
        
        /* IMU 姿态更新 */
        if (imu_status) {
            LSM6DSV_UpdateAttitude(&lsm6dsv_dev, &g_imu_attitude, 0.012f);
        }
        
        /* 读取灰度传感器, 更新共享变量给中断使用 */
        Grayscale_ReadAll();
        g_line_pos   = Grayscale_GetLinePosition();
        g_line_state = Grayscale_GetState();
    }
}