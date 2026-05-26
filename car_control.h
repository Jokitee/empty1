#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include "ti_msp_dl_config.h"

/* ========== 物理/控制常数 ========== */
#define PWM_MAX             1400

/* ========== 电机控制结构体与声明 ========== */
typedef struct {
    GPTIMER_Regs *timer;
    DL_TIMER_CC_INDEX ch_forward;
    DL_TIMER_CC_INDEX ch_backward;
} Motor_t;

extern Motor_t MotorA;
extern Motor_t MotorB;

/* ========== 6路灰度传感器相关 ========== */
#define GRAY_NUM            4

#define GRAY_L2_PIN         DL_GPIO_PIN_24
#define GRAY_L1_PIN         DL_GPIO_PIN_25
#define GRAY_M1_PIN         DL_GPIO_PIN_26
#define GRAY_M2_PIN         DL_GPIO_PIN_27
#define GRAY_R1_PIN         DL_GPIO_PIN_28
#define GRAY_R2_PIN         DL_GPIO_PIN_29

#define GRAY_ALL_PINS       (GRAY_L2_PIN | GRAY_L1_PIN | GRAY_M1_PIN | \
                             GRAY_M2_PIN | GRAY_R1_PIN | GRAY_R2_PIN)

extern uint8_t gray_value[GRAY_NUM];

typedef enum {
		LINE_NONE	 = 0,		/* 全白，丢线 */
    LINE_TRACK   /* 中间有黑线，循迹中 */
} LineState_t;

/* ========== PID 控制结构体 ========== */
typedef struct {
    float kp, ki, kd;
    float err, last_err, integral;
    int16_t output;
} PID_t;

extern PID_t pid_line;

/* ========== Yaw 控制结构体 ========== */
typedef struct {
    PID_t pid;
    volatile float target_yaw;
    volatile uint8_t locked;
} YawController_t;

extern YawController_t g_yaw_ctrl;

/* ========== 循迹/控制全局变量 ========== */
extern volatile int16_t  g_targetA;         /* 启动阶段或开环控制下的电机 A 目标 PWM */
extern volatile int16_t  g_targetB;         /* 启动阶段或开环控制下的电机 B 目标 PWM */
extern volatile float    g_line_pos;        /* 灰度计算得到的偏差值 (-2.5 ~ +2.5) */
extern volatile uint8_t  g_line_state;      /* 循迹状态 */
extern volatile uint8_t  g_running;         /* 闭环运行标志位: 1=循迹运行, 0=开环直通/停止 */
extern volatile int16_t  g_line_base_speed; /* 闭环循迹时的基础速度 (可由外部任务动态修改) */
extern volatile int16_t  g_max_steer;        /* 循迹时的最大转向控制量 */
extern volatile float    g_speed_drop_factor;/* 转向大时速度降低系数 */
extern volatile float    g_gray_weights[GRAY_NUM]; /* 四路循迹传感器权重 */
extern volatile uint16_t g_beep_ticks;      /* 非阻塞蜂鸣器/LED报警滴答计数 */

/* ========== 系统与控制核心接口 ========== */
void delay_ms(uint32_t ms);

void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Beep(uint32_t ms);

void Motor_Init(void);
void Motor_Set(Motor_t *motor, int16_t speed);
void Motor_Brake(Motor_t *motor);

void Grayscale_ReadAll(void);
LineState_t Grayscale_GetState(void);
float Grayscale_GetLinePosition(void);
bool Grayscale_StateChanged(LineState_t current_state);

void PID_Update(PID_t *pid, int32_t target, int32_t actual);
int16_t SteerControl(float line_pos);
int16_t YawControl(float target_yaw, float current_yaw);

/* ========== 运动原语（供不同任务调用） ========== */
void Car_TrackLine(int16_t base_speed);             /* 闭环循迹运动 */
void Car_DriveStraight(int16_t speed);              /* 开环/直行运动 */
void Car_Spin(int16_t speed_left, int16_t speed_right); /* 原地旋转或差速转向 */
void Car_Stop(void);                                /* 紧急制动刹车 */

#endif /* CAR_CONTROL_H */
