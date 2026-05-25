#include "car_control.h"
#include "lsm6dsv.h"

/* TIMG0 and TIMG6 are TimerG which only have 2 channels (CC_0 and CC_1) */
Motor_t MotorA = {TIMG0, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX};
Motor_t MotorB = {TIMG6, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX};

/* 灰度传感器当前数值 */
uint8_t gray_value[GRAY_NUM];

/* 循迹转向PID */
PID_t pid_line = {100.0f, 0.0f, 50.0f, 0, 0, 0, 0};
/* 陀螺仪 Yaw 偏航角控制 PID */
PID_t pid_yaw = {100.0f, 0.0f, 50.0f, 0, 0, 0, 0};

/* 循迹/控制全局变量 */
volatile int16_t  g_targetA = 0;
volatile int16_t  g_targetB = 0;
volatile float    g_line_pos = 0.0f;
volatile uint8_t  g_line_state = LINE_NONE;
volatile uint8_t  g_running = 0;
volatile int16_t  g_line_base_speed = 1500;

extern LSM6DSV_Attitude_t g_imu_attitude;
volatile float    g_target_yaw = 0.0f;
volatile uint8_t  g_yaw_locked = 0;
volatile uint16_t g_beep_ticks = 0;

/**
 * @brief  毫秒级延时函数
 */
void delay_ms(uint32_t ms)
{
    DL_Common_delayCycles(ms * (32000000 / 1000));
}

/**
 * @brief  电机定时器启动初始化
 */
void Motor_Init(void)
{
    DL_TimerG_startCounter(TIMG0);
    DL_TimerG_startCounter(TIMG6);
}

/**
 * @brief  设置电机PWM输出，自动限幅
 */
void Motor_Set(Motor_t *motor, int16_t speed)
{
    /* 速度幅值限幅 */
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

/**
 * @brief  电机紧急刹车控制
 */
void Motor_Brake(Motor_t *motor)
{
    DL_TimerG_setCaptureCompareValue(motor->timer, PWM_MAX, motor->ch_forward);
    DL_TimerG_setCaptureCompareValue(motor->timer, PWM_MAX, motor->ch_backward);
}

/**
 * @brief  根据输入偏差计算转向PWM输出
 */
int16_t SteerControl(float line_pos)
{
    PID_Update(&pid_line, 0, (int32_t)(line_pos * 100));
    return pid_line.output;
}

/**
 * @brief  读取灰度传感器状态 (0=白，1=黑)
 */
void Grayscale_ReadAll(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIOA, GRAY_ALL_PINS);
    gray_value[0] = (pins & GRAY_L1_PIN) ? 0 : 1;   /* 左内 L1 */
    gray_value[1] = (pins & GRAY_M1_PIN) ? 0 : 1;   /* 中左 M1 */
    gray_value[2] = (pins & GRAY_M2_PIN) ? 0 : 1;   /* 中右 M2 */
    gray_value[3] = (pins & GRAY_R1_PIN) ? 0 : 1;   /* 右内 R1 */
}

/**
 * @brief  根据当前灰度传感器状态，判断循迹状态
 */
LineState_t Grayscale_GetState(void)
{
    uint8_t l1 = gray_value[0];
    uint8_t m1 = gray_value[1];
    uint8_t m2 = gray_value[2];
    uint8_t r1 = gray_value[3];
    
    uint8_t total = l1 + m1 + m2 + r1;
    
    if (total == 0) return LINE_NONE;
    if (m1 == 1 || m2 == 1 ||l1 == 1 || r1 ==1) return LINE_TRACK;
    
    return LINE_NONE;
}

/**
 * @brief  计算当前线路位置偏差 (-2.5 ~ +2.5)
 */
float Grayscale_GetLinePosition(void)
{
    const float weights[GRAY_NUM] = {0.0f, -1.0f, 1.0f, 0.0f};
    int32_t sum = 0, cnt = 0;
    
    for (uint8_t i = 1; i < GRAY_NUM - 1; i++) {
        if (gray_value[i]) {
            sum += (int32_t)(weights[i] * 10);
            cnt++;
        }
    }
    
    if (cnt == 0) return 0;
    return (float)sum / (cnt * 10);
}

/**
 * @brief  通用增量/位置式 PID 计算函数
 */
void PID_Update(PID_t *pid, int32_t target, int32_t actual)
{
    pid->err = target - actual;
    pid->integral += pid->err;
    
    /* 抗积分饱和 */
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;
    
    float out = pid->kp * pid->err 
              + pid->ki * pid->integral 
              + pid->kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;
    
    /* 输出幅值限幅 */
    if (out > PWM_MAX) out = PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;
    
    pid->output = (int16_t)out;
}

/**
 * @brief  SysTick中断: 执行PID转向控制 + 电机输出 (10ms)
 */
void SysTick_Handler(void)
{
    // 非运行状态下仅进行蜂鸣器/LED倒计时清除，不执行小车运动控制
    if (!g_running) {
        Motor_Set(&MotorA, g_targetA);
        Motor_Set(&MotorB, g_targetB);
        if (g_beep_ticks > 0) {
            g_beep_ticks--;
            DL_GPIO_setPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
            Buzzer_On();
        } else {
            DL_GPIO_clearPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
            Buzzer_Off();
        }
        return;
    }

    // 状态切换检测（仅在运行时进行，避免待机干扰）
    static uint8_t last_g_line_state = LINE_NONE;
    if (g_line_state != last_g_line_state) {
        last_g_line_state = g_line_state;
        g_beep_ticks = 50; // 状态切换声光提示 50 滴答 = 500ms
    }

    // 状态切换时的声光提示控制
    if (g_beep_ticks > 0) {
        g_beep_ticks--;
        DL_GPIO_setPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
        Buzzer_On();
    } else {
        DL_GPIO_clearPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
        Buzzer_Off();
    }

    int16_t left_speed, right_speed;

    switch (g_line_state) {
        case LINE_TRACK:
        {
            // 在黑线循迹期间，释放 Yaw 锁定，为下一次出黑线锁死偏航角做准备
            g_yaw_locked = 0;

            PID_Update(&pid_line, 0, (int32_t)(g_line_pos * 100));
            int16_t steer_pwm = pid_line.output;

            int16_t max_steer = g_line_base_speed / 2;
            if (steer_pwm > max_steer) steer_pwm = max_steer;
            if (steer_pwm < -max_steer) steer_pwm = -max_steer;

            if (steer_pwm >= 0) {
                left_speed = g_line_base_speed - steer_pwm;
                right_speed = g_line_base_speed;
            } else {
                left_speed = g_line_base_speed;
                right_speed = g_line_base_speed + steer_pwm;
            }
            break;
        }

        case LINE_NONE:
        default:
        {
            // 进入白色无轨区，以进入瞬间的 Yaw 角为目标，使用 Yaw PID 闭环控直行
            if (!g_yaw_locked) {
                g_target_yaw = g_imu_attitude.yaw;
                g_yaw_locked = 1;
            }

            int16_t steer = YawControl(g_target_yaw, g_imu_attitude.yaw);

            left_speed = g_line_base_speed + steer;
            right_speed = g_line_base_speed - steer;

            pid_line.integral = 0;
            pid_line.last_err = 0;
            break;
        }
    }

    Motor_Set(&MotorA, left_speed);
    Motor_Set(&MotorB, right_speed);
}

/* ========== 运动原语接口实现 ========== */

/**
 * @brief  启动闭环循迹运动，允许自定义基础循迹速度
 */
void Car_TrackLine(int16_t base_speed)
{
    g_line_base_speed = base_speed;
    g_running = 1; // 激活 SysTick PID 循迹闭环控制
}

/**
 * @brief  开环直线驱动控制
 */
void Car_DriveStraight(int16_t speed)
{
    g_running = 0; // 关闭循迹
    g_targetA = speed;
    g_targetB = speed;
}

/**
 * @brief  差速转向控制 (正值向前，负值向后)
 */
void Car_Spin(int16_t speed_left, int16_t speed_right)
{
    g_running = 0; // 关闭循迹
    g_targetA = speed_left;
    g_targetB = speed_right;
}

/**
 * @brief  停止运动，执行制动刹车
 */
void Car_Stop(void)
{
    g_running = 0; // 关闭循迹
    g_targetA = 0;
    g_targetB = 0;
    Motor_Brake(&MotorA);
    Motor_Brake(&MotorB);
}

/**
 * @brief  偏航角 (Yaw) PID 控制器转向计算
 * @param  target_yaw  目标角度 (度，-180 ~ +180)
 * @param  current_yaw  当前角度 (度，-180 ~ +180)
 * @return 转向控制 PWM 差值（可以直接加减到左右轮）
 */
int16_t YawControl(float target_yaw, float current_yaw)
{
    float error = target_yaw - current_yaw;
    
    /* 将浮点角度偏差乘以 100 转换为整型误差供 PID 运算，以防浮点数溢出或抖动 */
    PID_Update(&pid_yaw, 0, (int32_t)(-error * 100));
    
    return pid_yaw.output;
}

/**
 * @brief  检测循迹状态是否发生改变
 * @param  current_state 当前的循迹状态
 * @return true = 状态发生切换，false = 状态保持不变
 */
bool Grayscale_StateChanged(LineState_t current_state)
{
    static LineState_t last_state = LINE_NONE;
    if (current_state != last_state) {
        last_state = current_state;
        return true;
    }
    return false;
}

/**
 * @brief  开启蜂鸣器
 */
void Buzzer_On(void)
{
    DL_GPIO_clearPins(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN_3_PIN); // 低电平开启
}

/**
 * @brief  关闭蜂鸣器
 */
void Buzzer_Off(void)
{
    DL_GPIO_setPins(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN_3_PIN); // 高电平关闭
}

/**
 * @brief  蜂鸣器鸣叫指定毫秒数 (采用双向翻转，无论高低电平触发均有效)
 */
void Buzzer_Beep(uint32_t ms)
{
    Buzzer_On();
    delay_ms(ms);
    Buzzer_Off();
}
