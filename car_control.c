#include "car_control.h"
#include "lsm6dsv.h"
#include "uart_debug.h"
#include <stdlib.h>

/* TIMG0 and TIMG6 are TimerG which only have 2 channels (CC_0 and CC_1) */
Motor_t MotorA = {TIMG0, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX};
Motor_t MotorB = {TIMG6, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX};

/* 灰度传感器当前数值 */
uint8_t gray_value[GRAY_NUM];

/* 循迹转向PID */
PID_t pid_line = {2.2f, 0.0f, 1.0f, 0, 0, 0, 0};
/* 陀螺仪 Yaw 偏航角控制 PID 及状态 */
YawController_t g_yaw_ctrl = {
    .pid = {1.4f, 0.0f, 1.8f, 0, 0, 0, 0},
    .target_yaw = 0.0f,
    .locked = 0
};

/* 循迹/控制全局变量 */
volatile int16_t  g_targetA = 0;
volatile int16_t  g_targetB = 0;
volatile float    g_line_pos = 0.0f;
volatile uint8_t  g_line_state = LINE_NONE;
volatile uint8_t  g_running = 0;
volatile int16_t  g_line_base_speed = 1050;
volatile int16_t  g_line_start_speed = 1020;
volatile int16_t  g_max_steer = 400;           /* 最大转向限制 (Steer PWM 限制) */
volatile float    g_speed_drop_factor = 1.2f;  /* 转向大时速度降低系数 */
/* 四路循迹传感器权重（偏移量量化值，正值表示偏左，负值表示偏右） */
volatile float    g_gray_weights[GRAY_NUM] = {1.7f, 0.4f, -0.4f, -1.7f};

volatile uint16_t g_entry_slow_ticks = 0;      /* 从白区切入黑线时的强制减速计数器 */
volatile int16_t  g_entry_slow_speed = 350;     /* 切入黑线瞬间的基准慢速 */
volatile int16_t  g_entry_max_steer = 850;      /* 切入黑线瞬间的最大差速转向限制 */
volatile uint16_t g_entry_hold_ticks = 25;       /* 强制不退出的“真空时间”/死区时间 (8 = 80ms) */
volatile uint16_t g_entry_max_ticks = 60;       /* 抓线捕获期的最大超时时间 (40 = 400ms) */

extern volatile LSM6DSV_Attitude_t g_imu_attitude;
extern LSM6DSV_Handle_t lsm6dsv_dev;
extern volatile bool g_imu_status;
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
    gray_value[0] = (pins & GRAY_L1_PIN) ? 1 : 0;   /* 左内 L1 (白=1, 黑=0) */
    gray_value[1] = (pins & GRAY_M1_PIN) ? 1 : 0;   /* 中左 M1 (白=1, 黑=0) */
    gray_value[2] = (pins & GRAY_M2_PIN) ? 1 : 0;   /* 中右 M2 (白=1, 黑=0) */
    gray_value[3] = (pins & GRAY_R1_PIN) ? 1 : 0;   /* 右内 R1 (白=1, 黑=0) */
}

/**
 * @brief  根据当前灰度传感器状态，判断循迹状态
 */
LineState_t Grayscale_GetState(void)
{
		//uint8_t l2 = gray_value[0];
    uint8_t l1 = gray_value[0];
    uint8_t m1 = gray_value[1];
    uint8_t m2 = gray_value[2];
    uint8_t r1 = gray_value[3];
		//uint8_t r2 = gray_value[5];
    
    uint8_t total = l1 + m1 + m2 + r1;

    if (total == 0) return LINE_NONE;   /* 4路全白：白区 */
    return LINE_TRACK;                  /* 任意一路非白：检测到路标横线 */
}

/**
 * @brief  计算当前线路位置偏差 (-1.5 ~ +1.5)
 */
float Grayscale_GetLinePosition(void)
{
    float sum = 0.0f;
    int32_t cnt = 0;
    
    for (uint8_t i = 0; i < GRAY_NUM; i++) {
        if (gray_value[i]) {
            sum += g_gray_weights[i];
            cnt++;
        }
    }
    
    if (cnt == 0) return 0.0f;
    return sum / cnt;
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
    static uint8_t last_g_line_state = LINE_NONE;
    static float g_entry_dir = 0.0f; // 记录切入方向：+1.7f 表示左，-1.7f 表示右

    // 1. 定时器最前端先采集 IMU 数据并积分 (固定 100Hz = 0.01s)
    if (g_imu_status) {
        LSM6DSV_UpdateAttitude(&lsm6dsv_dev, &g_imu_attitude, 0.01f);
    }

    // 2. 采集灰度传感器数据，确保控制闭环时间同步并防止数据竞争
    Grayscale_ReadAll();
    g_line_pos   = Grayscale_GetLinePosition();
    g_line_state = Grayscale_GetState();

    // 1. 用于串口画图的显示数据：采用较强的低通滤波 (alpha = 0.22f) 消除阶梯感，让波形在上位机上圆润美观
    static float tele_filtered_line_pos = 0.0f;
    float tele_alpha = 0.22f;
    tele_filtered_line_pos = tele_alpha * g_line_pos + (1.0f - tele_alpha) * tele_filtered_line_pos;
    // Debug_UART_PrintFloat(tele_filtered_line_pos);

    // 2. 用于控制的真实反馈数据：采用极轻微的低通滤波 (alpha = 0.8f) 保留高实时性（延迟仅2.5ms），避免PID相位滞后引起震荡
    static float ctrl_filtered_line_pos = 0.0f;
    float ctrl_alpha = 0.8f;
    ctrl_filtered_line_pos = ctrl_alpha * g_line_pos + (1.0f - ctrl_alpha) * ctrl_filtered_line_pos;

    // 非运行状态下仅进行蜂鸣器/LED倒计时清除，不执行小车运动控制
    if (!g_running) {
        Motor_Set(&MotorA, g_targetA);
        Motor_Set(&MotorB, g_targetB);
        
        // 待机状态下重置状态变量，保证下一次按下确认键启动时，状态切换检测 100% 触发
        last_g_line_state = LINE_NONE;
        g_entry_slow_ticks = 0;
        g_entry_dir = 0.0f;
        
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
    if (g_line_state != last_g_line_state) {
        if (last_g_line_state == LINE_NONE && g_line_state == LINE_TRACK) {
            g_entry_slow_ticks = g_entry_max_ticks; // 使用动态可调的捕获最大时长
            
            // 记录切入方向：L1 碰线则为左正，R1 碰线则为右负，否则根据偏移量正负判定
            if (gray_value[0] == 1) g_entry_dir = 1.7f;
            else if (gray_value[3] == 1) g_entry_dir = -1.7f;
            else g_entry_dir = (g_line_pos > 0.0f) ? 1.7f : -1.7f;

            // 重置循迹 PID 状态，消除以前累积的积分 and 微分残留，防止切入瞬间突变
            pid_line.integral = 0.0f;
            pid_line.last_err = 0.0f;
            pid_line.err = 0.0f;
        }
        last_g_line_state = g_line_state;
        g_beep_ticks = 50; // 状态切换声光提示 50 滴答 = 500ms
    }



    // 强行保持：在切入捕捉期内，无条件维持为 LINE_TRACK，且如果短暂丢线（全白），强制锁定偏差为切入方向以持续转弯
    if (g_entry_slow_ticks > 0) {
        g_line_state = LINE_TRACK;
        uint8_t total_sensors = gray_value[0] + gray_value[1] + gray_value[2] + gray_value[3];
        if (total_sensors == 0) {
            g_line_pos = g_entry_dir;
            ctrl_filtered_line_pos = g_entry_dir;
        }
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

    switch (g_line_state){
			case LINE_TRACK:
				{
            // 在黑线循迹期间，释放 Yaw 锁定，为下一次出黑线锁死偏航角做准备。如果是外部预设的目标角 (2)，则保留不修改
            if (g_yaw_ctrl.locked != 2) {
                g_yaw_ctrl.locked = 0;
            }
            
            // 重置 Yaw PID 状态，防止上一次白区控制的残留积分/微分项干扰下一次进入
            g_yaw_ctrl.pid.integral = 0.0f;
            g_yaw_ctrl.pid.last_err = 0.0f;
            g_yaw_ctrl.pid.err = 0.0f;

            PID_Update(&pid_line, 0, (int32_t)(ctrl_filtered_line_pos * 100));
            int16_t steer_pwm = pid_line.output;

            // 若在切入强制减速期间，大幅提升转向输出增益，确保在 300 低速下能产生足够甩头的差速扭矩
            if (g_entry_slow_ticks > 0) {
                steer_pwm = (int16_t)(steer_pwm * 1.8f);
            }

            if (g_entry_slow_ticks > 0 && g_entry_slow_ticks < (g_entry_max_ticks - g_entry_hold_ticks)) {
                if (gray_value[1] == 1 || gray_value[2] == 1) {
                    g_entry_slow_ticks = 0;
                }
            }

            // 限制最大转向控制量（若在切入强制减速期间，允许更大的转向差速上限以提供足够的偏航扭矩）
            int16_t current_max_steer = (g_entry_slow_ticks > 0) ? g_entry_max_steer : g_max_steer;
            if (steer_pwm > current_max_steer) steer_pwm = current_max_steer;
            if (steer_pwm < -current_max_steer) steer_pwm = -current_max_steer;

            // 转向车速分配：如果是刚从白区切入，且尚未完成对齐，强制执行低速过渡，之后恢复正常动态减速
            int16_t current_base_speed;
            if (g_entry_slow_ticks > 0) {
                g_entry_slow_ticks--;
                current_base_speed = g_entry_slow_speed;
            } else {
                current_base_speed = g_line_base_speed - (int16_t)(abs(steer_pwm) * g_speed_drop_factor);
                // 限制最低基础速度，防止降速太多甚至停滞
                int16_t min_speed = g_line_base_speed / 2;
                if (current_base_speed < min_speed) {
                    current_base_speed = min_speed;
                }
            }

            left_speed = current_base_speed - steer_pwm;
            right_speed = current_base_speed + steer_pwm;
						
            break;
        }
		  case LINE_NONE:
			default:
        {
            if (g_yaw_ctrl.locked == 0) {
                g_yaw_ctrl.target_yaw = g_imu_attitude.yaw;  // 记录进入瞬间的 Yaw 角作为直行目标
                g_yaw_ctrl.locked = 1;
                // 锁定时清零 PID 状态，确保平滑过渡
                g_yaw_ctrl.pid.integral = 0.0f;
                g_yaw_ctrl.pid.last_err = 0.0f;
                g_yaw_ctrl.pid.err = 0.0f;
            } else if (g_yaw_ctrl.locked == 2) {
                g_yaw_ctrl.locked = 1; // 转换为普通锁定状态，以便下一次出黑线时能正常工作
                // 清零 PID 状态，确保平滑过渡（保留外部预设好的 target_yaw）
                g_yaw_ctrl.pid.integral = 0.0f;
                g_yaw_ctrl.pid.last_err = 0.0f;
                g_yaw_ctrl.pid.err = 0.0f;
            }

            int16_t steer = YawControl(g_yaw_ctrl.target_yaw, g_imu_attitude.yaw);

            int16_t max_steer = g_line_base_speed / 3;
            if (steer > max_steer) steer = max_steer;
            if (steer < -max_steer) steer = -max_steer;
            
            // 同样，如果在直道纠偏时偏差过大，也适当降低行驶速度
            int16_t current_base_speed = g_line_base_speed - (int16_t)(abs(steer / 2) * g_speed_drop_factor);
            int16_t min_speed = g_line_base_speed / 3;
            if (current_base_speed < min_speed) {
                current_base_speed = min_speed;
            }
            
            // 根据输出的steer调整两轮差速，steer>0时左轮减速右轮加速，小车向左转
            left_speed  = current_base_speed + steer / 2;
            right_speed = current_base_speed - steer / 2;
            
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

// 偏航角控制极性翻转标志。如果发现小车不仅不纠偏反而加速偏离目标方向，请将此宏改为 -1.0f 
#define YAW_PID_POLARITY (1.0f)

/**
 * @brief  偏航角 (Yaw) PID 控制器转向计算
 * @param  target_yaw  目标角度 (度，-180 ~ +180)
 * @param  current_yaw  当前角度 (度，-180 ~ +180)
 * @return 转向控制 PWM 差值（可以直接加减到左右轮）
 */
int16_t YawControl(float target_yaw, float current_yaw)
{
    /* 1. 计算角度偏差并归一化到 [-180, +180] */
    float err = target_yaw - current_yaw;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    /* 应用控制极性，如果出现正反馈画圈，通过改变极性解决 */
    err *= YAW_PID_POLARITY;

    /* 2. 更新误差项 (直接使用真实的物理角度差，不强制转int，保留浮点精度) */
    g_yaw_ctrl.pid.err = err;
    
    /* 3. 积分累加及抗饱和限幅 (积分阈值设为 20.0 度) */
    g_yaw_ctrl.pid.integral += g_yaw_ctrl.pid.err;
    if (g_yaw_ctrl.pid.integral >  20.0f) g_yaw_ctrl.pid.integral =  20.0f;
    if (g_yaw_ctrl.pid.integral < -20.0f) g_yaw_ctrl.pid.integral = -20.0f;
    
    /* 4. 计算 PID 输出 
       (为了兼容原有 PID 参数 kp=3.0 等的调参手感，原有逻辑中将角度放大了 100 倍，
        这里将最终乘积放大 100 倍，以保持对电机的控制输出增益等效) */
    float out = (g_yaw_ctrl.pid.kp * g_yaw_ctrl.pid.err 
               + g_yaw_ctrl.pid.ki * g_yaw_ctrl.pid.integral 
               + g_yaw_ctrl.pid.kd * (g_yaw_ctrl.pid.err - g_yaw_ctrl.pid.last_err)) * 100.0f;
               
    /* 5. 记录上一次误差 */
    g_yaw_ctrl.pid.last_err = g_yaw_ctrl.pid.err;

    /* 6. 输出幅值限幅 */
    if (out > PWM_MAX) out = PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;

    g_yaw_ctrl.pid.output = (int16_t)out;
    return g_yaw_ctrl.pid.output;
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
