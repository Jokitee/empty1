#include "ti_msp_dl_config.h"
#include "lsm6dsv.h"
#include "uart_debug.h"
#include "car_control.h"

/* ========== 姿态角度全局变量 ========== */
volatile LSM6DSV_Attitude_t g_imu_attitude = {0.0f, 0.0f, 0.0f};
LSM6DSV_Handle_t lsm6dsv_dev;
volatile bool g_imu_status = false;
extern YawController_t g_yaw_ctrl;

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
static volatile uint8_t g_yaw_reset_req = 0;    /* 主循环执行 yaw 清零的请求标志（避免ISR直接写float） */
static volatile uint8_t g_mode_switch_req = 0;  /* MODE键按下请求标志 */
static volatile uint8_t g_confirm_req = 0;      /* CONF键按下请求标志 */
extern volatile int16_t  g_line_start_speed;

/* ========== 红灯闪烁控制 ========== */
void RED_LED_Flash(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        // 点亮红灯
        DL_GPIO_setPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
        delay_ms(200);
        // 熄灭红灯
        DL_GPIO_clearPins(RED_GPIO_PORT, RED_GPIO_PIN_0_PIN);
        delay_ms(200);
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
                g_mode_switch_req = 1;
                DL_GPIO_clearInterruptStatus(GPIOA, MODE_GPIO_PIN_1_PIN);
            }
            break;
        }
        
        /* ======= PORTB 中断: CONF 按键启动/紧急停止 ======= */
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        {
            uint32_t pins = DL_GPIO_getEnabledInterruptStatus(GPIOB, CONF_GPIO_PIN_2_PIN);
            if (pins & CONF_GPIO_PIN_2_PIN) {
                g_confirm_req = 1;
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

    if (!g_yaw_init) {
        step = 0;
        g_yaw_init = 1;
    }

    Car_TrackLine(g_line_start_speed); // 启动（白区 Yaw 闭环直跑）

    // 步骤 0：等待小车进入白区（适配起点在黑线或白区两种情况）
    if (step == 0) {
        if (g_line_state == LINE_NONE) {
            step = 1;
        }
    }
    // 步骤 1：白区直行，LINE_TRACK 可靠判定路标 B，执行停车
    else if (step == 1) {
        if (g_line_state == LINE_TRACK) {
            Car_Stop();
            g_task_success = 1;
            g_task_confirmed = 0;
            step = 0;
        }
    }
}

void Task2_Process(void)
{
    /* 模式二：圆形大环往返 (A -> B -> C -> D -> A 停车) */
    static uint8_t step = 0;
    static LineState_t last_state = LINE_NONE;
    
    Car_TrackLine(g_line_start_speed);
    
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
		
		if(step == 2) {
            g_yaw_ctrl.target_yaw = 180.0;
            g_yaw_ctrl.locked = 2; // 预设目标角，不允许 SysTick 覆盖
        }
    
    // 经历8次状态转换（4个黑线标记点各进出一次：B,C,D,A）
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
    
    Car_TrackLine(g_line_start_speed);
    
    if (!g_yaw_init) {
        step = 0;
        last_state = g_line_state; // 初始化为开始时的实际状态，避免在起点误触发状态改变
        g_yaw_init = 1;
    }
		
    
    if (g_line_state != last_state) {
        last_state = g_line_state;
        step++;
    }
		
		if(step == 2) {
            g_yaw_ctrl.target_yaw = -100.0;
            g_yaw_ctrl.locked = 2; // 预设目标角，不允许 SysTick 覆盖
        }
		
    // "8" 字形路径：C,B,D,A
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
		static uint8_t circle = 0;
    static LineState_t last_state = LINE_NONE;
    
    Car_TrackLine(g_line_start_speed);
    
    if (!g_yaw_init) {
        step = 0;
        last_state = g_line_state; // 初始化为开始时的实际状态，避免在起点误触发状态改变
        g_yaw_init = 1;
    }
    
    if (g_line_state != last_state) {
        last_state = g_line_state;
        step++;
    }
		
		switch(step){
			case 2:
				g_yaw_ctrl.target_yaw = -100.0;
				g_yaw_ctrl.locked = 2; // 预设目标角
				break;
			case 0:
				g_yaw_ctrl.target_yaw = 0.0;
				g_yaw_ctrl.locked = 2; // 预设目标角
				break;
			default:
				break;
		}
		
		if (step >= 4){
				step = 0;
				circle++;	
		}
    
    // 4 圈后完成回 A 停车
    if (circle >= 4) {
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
    
    // 开启中断控制
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    DL_SYSTICK_enableInterrupt(); // 开启 SysTick 10ms 中断
    
    /* 等待 LSM6DSV 传感器启动 */
    delay_cycles(32000 * 20);
    
    g_imu_status = LSM6DSV_Init(&lsm6dsv_dev);
    
    // 上电成功提示：闪烁 2 次
    RED_LED_Flash(2);
    
    g_task_confirmed = 0;
    g_running = 0;

    while (1)
    {
			
        delay_ms(10);
        
        /* 处理来自 MODE 按键 ISR 的请求 */
        if (g_mode_switch_req) {
            g_mode_switch_req = 0;
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
        }

        /* 处理来自 CONF 按键 ISR 的请求 */
        if (g_confirm_req) {
            g_confirm_req = 0;
            delay_ms(20); // 软件去抖
            if (DL_GPIO_readPins(GPIOB, CONF_GPIO_PIN_2_PIN) == 0) {
                if (!g_task_confirmed && g_selected_mode != Task_0) {
                    g_running_mode = g_selected_mode;
                    g_task_confirmed = 1;
                    
                    // 请求主循环清零Yaw（避免ISR直接写float导致数据竞争）
                    g_yaw_reset_req = 1;
                    // 启动蜂鸣+红灯：交由 SysTick 非阻塞驱动（g_running=0 分支）
                    g_beep_ticks = 50;
                    // 取消500ms阻塞等待，直接开始运行
                } else {
                    // 如果已经在运行中，再次按下CONF键则作为“急停”信号
                    g_task_confirmed = 0;
                    g_running = 0;
                    g_targetA = 0;
                    g_targetB = 0;
                    Motor_Brake(&MotorA);
                    Motor_Brake(&MotorB);
                    RED_LED_Flash(3); // 快速闪烁3次提示已停机
                }
            }
        }
        

        /* 再处理 Yaw 清零请求 */
        if (g_yaw_reset_req) {
            g_yaw_reset_req = 0;
            g_imu_attitude.yaw = 0.0f; // 清零陀螺仪积分 yaw
            g_yaw_ctrl.target_yaw = 0.0f; // 同步清零目标角，防止旧锁定值残留导致固定PID偏差
            g_yaw_ctrl.locked = 0;     // yaw 和 target 均已归零后，才允许 SysTick 重新锁定
                                       // → SysTick 下一帧在 LINE_NONE 时将以 0° 为起点锁定
        }
				
        
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
            g_yaw_ctrl.locked = 0; // 重置航向锁定状态，允许下一次运行时重新锁定
            g_line_state = LINE_NONE; // 重置为初始状态
            g_line_pos = 0.0f;
            
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