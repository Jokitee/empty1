/**
 * @file    lsm6dsv.c
 * @brief   LSM6DSV 六轴 IMU 驱动实现
 * @note    底层 I2C 通信使用 MSPM0 DriverLib (DL_I2C_*)
 *          I2C1 外设，SDA: PA16, SCL: PA17
 */

#include "lsm6dsv.h"
#include "ti_msp_dl_config.h"
#include "uart_debug.h"
#include <math.h>

/* I2C1 实例（由 SysConfig 配置生成） */
#define LSM6DSV_I2C_INST    I2C1

/* 超时计数 */
#define I2C_TIMEOUT         500000

/* ========== 底层 I2C 读写 ========== */

/**
 * @brief  等待 TX FIFO 有空位
 */
static bool _waitTxFree(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(LSM6DSV_I2C_INST)) {
        if (--timeout == 0) return false;
    }
    return true;
}

/**
 * @brief  向指定寄存器写入一个字节
 */
bool LSM6DSV_WriteReg(LSM6DSV_Handle_t *handle, uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2];
    tx_buf[0] = reg;
    tx_buf[1] = val;
    
    DL_I2C_flushControllerTXFIFO(LSM6DSV_I2C_INST);
    DL_I2C_fillControllerTXFIFO(LSM6DSV_I2C_INST, tx_buf, 2);
    
    /* Wait for I2C to be Idle */
    uint32_t timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    
    DL_I2C_startControllerTransfer(LSM6DSV_I2C_INST,
                                    handle->i2c_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX,
                                    2);
                                    
    timeout = I2C_TIMEOUT;
    while (DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0) return false;
    }
    
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    
    DL_I2C_flushControllerTXFIFO(LSM6DSV_I2C_INST);
    return true;
}

/**
 * @brief  从指定寄存器读取多个字节
 */
bool LSM6DSV_ReadReg(LSM6DSV_Handle_t *handle, uint8_t reg,
                      uint8_t *buf, uint8_t len)
{
    if (len == 0) return true;
    
    DL_I2C_flushControllerTXFIFO(LSM6DSV_I2C_INST);
    DL_I2C_fillControllerTXFIFO(LSM6DSV_I2C_INST, &reg, 1);
    
    /* Wait for I2C to be Idle */
    uint32_t timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    
    DL_I2C_startControllerTransfer(LSM6DSV_I2C_INST,
                                    handle->i2c_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_TX,
                                    1);
                                    
    timeout = I2C_TIMEOUT;
    while (DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0) return false;
    }
    
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    
    DL_I2C_flushControllerTXFIFO(LSM6DSV_I2C_INST);
    
    /* Send a read request to Target */
    DL_I2C_flushControllerRXFIFO(LSM6DSV_I2C_INST);
    DL_I2C_startControllerTransfer(LSM6DSV_I2C_INST,
                                    handle->i2c_addr,
                                    DL_I2C_CONTROLLER_DIRECTION_RX,
                                    len);
                                    
    for (uint8_t i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT;
        while (DL_I2C_isControllerRXFIFOEmpty(LSM6DSV_I2C_INST)) {
            if (--timeout == 0) return false;
        }
        buf[i] = DL_I2C_receiveControllerData(LSM6DSV_I2C_INST);
    }
    
    // Wait for transfer to complete and bus to go idle
    timeout = I2C_TIMEOUT;
    while (DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (--timeout == 0) return false;
    }
    
    timeout = I2C_TIMEOUT;
    while (!(DL_I2C_getControllerStatus(LSM6DSV_I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return false;
    }
    
    return true;
}

/* ========== API 函数 ========== */

uint8_t LSM6DSV_ReadWhoAmI(LSM6DSV_Handle_t *handle)
{
    uint8_t val = 0;
    LSM6DSV_ReadReg(handle, LSM6DSV_REG_WHO_AM_I, &val, 1);
    return val;
}

void LSM6DSV_Reset(LSM6DSV_Handle_t *handle)
{
    /* CTRL3 寄存器 bit[0] = SW_RESET */
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL3, 0x01)) {
        return;
    }

    /* 等待复位完成（SW_RESET 位自动清零） */
    uint8_t val = 0x01;
    uint32_t timeout = 200;
    while (val & 0x01) {
        if (!LSM6DSV_ReadReg(handle, LSM6DSV_REG_CTRL3, &val, 1)) {
            break;
        }
        if (--timeout == 0) break;
        delay_cycles(32000 * 1); // 1ms delay
    }
}

bool LSM6DSV_Init(LSM6DSV_Handle_t *handle)
{
    handle->i2c_addr = LSM6DSV_I2C_ADDR;
    handle->xl_fs    = LSM6DSV_XL_FS_4G;
    handle->g_fs     = LSM6DSV_G_FS_2000DPS;
    handle->gyro_bias_x = 0.0f;
    handle->gyro_bias_y = 0.0f;
    handle->gyro_bias_z = 0.0f;
    handle->accel_bias_roll = 0.0f;
    handle->accel_bias_pitch = 0.0f;
    handle->filtered_gx = 0.0f;
    handle->filtered_gy = 0.0f;
    handle->filtered_gz = 0.0f;
    /* 诊断字段清零 */
    handle->diag_read_total = 0;
    handle->diag_read_fail  = 0;
    handle->diag_raw_gx = 0.0f;
    handle->diag_raw_gy = 0.0f;
    handle->diag_raw_gz = 0.0f;

    /* 软件复位 */
    LSM6DSV_Reset(handle);

    /* 等待传感器启动就绪（datasheet 要求 BOOT_TIME >= 10ms，陀螺仪 turn-on 需 30ms） */
    delay_cycles(32000 * 50);  /* ~50ms @32MHz */

    /* WHO_AM_I 校验 */
    uint8_t id = LSM6DSV_ReadWhoAmI(handle);
    UART_SendString("WHO_AM_I=0x"); UART_SendHex(id); UART_SendString("\r\n");
    if (id != LSM6DSV_WHO_AM_I_VALUE) {
        UART_SendString("[ERR] WHO_AM_I mismatch! Expected 0x70\r\n");
        return false;
    }

    /* 启用 BDU（Block Data Update）+ 地址自增 */
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL3, 0x44)) {
        UART_SendString("LSM6DSV: Write CTRL3 Failed!\r\n");
        return false;
    }

    /* 默认配置：加速度计 120Hz, ±4g；陀螺仪 120Hz, ±2000dps */
    if (!LSM6DSV_ConfigAccel(handle, LSM6DSV_ODR_120HZ, handle->xl_fs)) {
        UART_SendString("LSM6DSV: ConfigAccel Failed!\r\n");
        return false;
    }

    if (!LSM6DSV_ConfigGyro(handle, LSM6DSV_ODR_120HZ, handle->g_fs)) {
        UART_SendString("LSM6DSV: ConfigGyro Failed!\r\n");
        return false;
    }

    /* 启用 DRDY_MASK */
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL4, 0x04)) {
        UART_SendString("LSM6DSV: Write CTRL4 Failed!\r\n");
        return false;
    }

    /* 配置陀螺仪 LPF1 滤波器 */
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL7, 0x01)) {
        UART_SendString("LSM6DSV: Write CTRL7 Failed!\r\n");
        return false;
    }

    /* 配置加速度计 LPF2 滤波器 */
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL9, 0x02)) {
        UART_SendString("LSM6DSV: Write CTRL9 Failed!\r\n");
        return false;
    }

    /* 等待滤波器建立完成 */
    delay_cycles(32000 * 20);  /* ~20ms */

    /* 回读 CTRL1/CTRL2/CTRL6，打印诊断（不作为 Init 成败判断）*/
    uint8_t r1 = 0, r2 = 0, r6 = 0;
    LSM6DSV_ReadReg(handle, LSM6DSV_REG_CTRL1, &r1, 1);
    LSM6DSV_ReadReg(handle, LSM6DSV_REG_CTRL2, &r2, 1);
    LSM6DSV_ReadReg(handle, LSM6DSV_REG_CTRL6, &r6, 1);
    UART_SendString("CTRL1=0x"); UART_SendHex(r1);
    UART_SendString(" CTRL2=0x"); UART_SendHex(r2);
    UART_SendString(" CTRL6=0x"); UART_SendHex(r6);
    if (r2 == 0x00) {
        UART_SendString(" [WARN:G_ODR=0]");
    }
    UART_SendString("\r\n");

    return true;
}

bool LSM6DSV_ConfigAccel(LSM6DSV_Handle_t *handle,
                          LSM6DSV_ODR_t odr, LSM6DSV_XL_FS_t fs)
{
    handle->xl_fs = fs;

    /* 计算灵敏度 (mg/LSB) */
    switch (fs) {
        case LSM6DSV_XL_FS_2G:  handle->xl_sensitivity = 0.061f; break;
        case LSM6DSV_XL_FS_4G:  handle->xl_sensitivity = 0.122f; break;
        case LSM6DSV_XL_FS_8G:  handle->xl_sensitivity = 0.244f; break;
        case LSM6DSV_XL_FS_16G: handle->xl_sensitivity = 0.488f; break;
        default:                 handle->xl_sensitivity = 0.122f; break;
    }

    /* CTRL1 (10h): Bit7=0, [6:4]=OP_MODE_XL (000=HP), [3:0]=ODR_XL */
    uint8_t ctrl1 = ((uint8_t)odr & 0x0F);
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL1, ctrl1)) return false;

    /* CTRL8 (17h): HP_LPF2_XL_BW[7:5]=011 (ODR/45), [1:0]=FS_XL (accelerometer full-scale) */
    uint8_t ctrl8 = (0x03 << 5) | ((uint8_t)fs & 0x03);
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL8, ctrl8)) return false;

    return true;
}

bool LSM6DSV_ConfigGyro(LSM6DSV_Handle_t *handle,
                          LSM6DSV_ODR_t odr, LSM6DSV_G_FS_t fs)
{
    handle->g_fs = fs;

    /* 计算灵敏度 (mdps/LSB) */
    switch (fs) {
        case LSM6DSV_G_FS_125DPS:  handle->g_sensitivity = 4.375f;  break;
        case LSM6DSV_G_FS_250DPS:  handle->g_sensitivity = 8.75f;   break;
        case LSM6DSV_G_FS_500DPS:  handle->g_sensitivity = 17.50f;  break;
        case LSM6DSV_G_FS_1000DPS: handle->g_sensitivity = 35.0f;   break;
        case LSM6DSV_G_FS_2000DPS: handle->g_sensitivity = 70.0f;   break;
        case LSM6DSV_G_FS_4000DPS: handle->g_sensitivity = 140.0f;  break;
        default:                    handle->g_sensitivity = 70.0f;   break;
    }

    /* CTRL2 (11h): Bit7=0, [6:4]=OP_MODE_G (000=HP), [3:0]=ODR_G */
    uint8_t ctrl2 = ((uint8_t)odr & 0x0F);
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL2, ctrl2)) return false;

    /* CTRL6 (15h): LPF1_G_BW[6:4]=000, [3:0]=FS_G (gyroscope full-scale) */
    uint8_t ctrl6 = ((uint8_t)fs & 0x0F);
    if (!LSM6DSV_WriteReg(handle, LSM6DSV_REG_CTRL6, ctrl6)) return false;

    return true;
}


/* ---------- 一次性全部读取（已精简为仅读取陀螺仪 6 字节） ---------- */

bool LSM6DSV_ReadAll(LSM6DSV_Handle_t *handle,
                      LSM6DSV_Axes_t *accel,
                      LSM6DSV_Axes_t *gyro,
                      float *temp)
{
    uint8_t buf[14];

    /* 直接从 OUT_TEMP_L (20h) 连续读取 14 字节
     * BDU 模式已保证同一帧数据的高低字节不会混用，无需额外等待 DRDY */
    if (!LSM6DSV_ReadReg(handle, LSM6DSV_REG_OUT_TEMP_L, buf, 14)) {
        return false;
    }

    /* 仅排除全 0xFF（I2C 总线拉高故障） */
    bool all_ff = true;
    for (int i = 0; i < 14; i++) {
        if (buf[i] != 0xFF) { all_ff = false; break; }
    }
    if (all_ff) return false;

    /* 解析温度 */
    int16_t raw_temp = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    *temp = (raw_temp / 256.0f) + 25.0f;

    /* 解析陀螺仪 (buf[2..7]) */
    int16_t gx = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    int16_t gy = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    int16_t gz = (int16_t)((uint16_t)buf[7] << 8 | buf[6]);

    /* 解析加速度计 (buf[8..13]) */
    int16_t ax = (int16_t)((uint16_t)buf[9]  << 8 | buf[8]);
    int16_t ay = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);
    int16_t az = (int16_t)((uint16_t)buf[13] << 8 | buf[12]);

    float g_scale = handle->g_sensitivity / 1000.0f;
    float a_scale = handle->xl_sensitivity / 1000.0f;

    gyro->x = gx * g_scale;
    gyro->y = gy * g_scale;
    gyro->z = gz * g_scale;

    accel->x = ax * a_scale;
    accel->y = ay * a_scale;
    accel->z = az * a_scale;

    /* 物理量程合理性过滤：上限设为量程 95%（2000dps * 0.95 ≈ 1900dps） */
    if (fabsf(gyro->x) > 1900.0f || fabsf(gyro->y) > 1900.0f || fabsf(gyro->z) > 1900.0f) {
        return false;
    }

    return true;
}

/* ========== 滤波算法与姿态解算实现 ========== */


bool LSM6DSV_BootCalibrate(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude)
{
    // 1. 等待 300ms 确保传感器上电和内部滤波器完全稳定
    delay_cycles(32000 * 300);

    LSM6DSV_Axes_t accel;
    LSM6DSV_Axes_t gyro;
    float temp;

    // 2. 连续读取并丢弃前 50 帧数据，清除滤波器建立期的暂态大误差
    for (uint8_t i = 0; i < 50; i++) {
        LSM6DSV_ReadAll(handle, &accel, &gyro, &temp);
        delay_cycles(32000 * 5); // 5ms delay
    }

    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    
    // 采样 200 次，每次延时 5ms，总共耗时约 1000ms = 1 秒
    uint16_t samples = 200;
    uint16_t valid_count = 0;  // 记录有效采样帧数
    
    // 用于检测静态稳定度：记录陀螺仪的最大、最小值
    float gyro_max_x = -9999.0f, gyro_min_x = 9999.0f;
    float gyro_max_y = -9999.0f, gyro_min_y = 9999.0f;
    float gyro_max_z = -9999.0f, gyro_min_z = 9999.0f;

    for (uint16_t i = 0; i < samples; i++) {
        if (LSM6DSV_ReadAll(handle, &accel, &gyro, &temp)) {
            valid_count++;
            sum_gx += gyro.x;
            sum_gy += gyro.y;
            sum_gz += gyro.z;
            
            if (gyro.x > gyro_max_x) gyro_max_x = gyro.x;
            if (gyro.x < gyro_min_x) gyro_min_x = gyro.x;
            if (gyro.y > gyro_max_y) gyro_max_y = gyro.y;
            if (gyro.y < gyro_min_y) gyro_min_y = gyro.y;
            if (gyro.z > gyro_max_z) gyro_max_z = gyro.z;
            if (gyro.z < gyro_min_z) gyro_min_z = gyro.z;
        }
        delay_cycles(32000 * 5); // 5ms delay
    }

    // 有效帧过少（少于总数的 50%）则视为 I2C 通信异常，校准失败
    if (valid_count < (samples / 2)) {
        return false;
    }

    float mean_gx = (float)(sum_gx / valid_count);
    float mean_gy = (float)(sum_gy / valid_count);
    float mean_gz = (float)(sum_gz / valid_count);

    // 3. 车辆抖动校验 (Vibration Check): 采样期间陀螺仪波动最大差值不能超过 6.0 dps
    if ((gyro_max_x - gyro_min_x) > 6.0f || 
        (gyro_max_y - gyro_min_y) > 6.0f || 
        (gyro_max_z - gyro_min_z) > 6.0f) {
        return false;
    }

    // 4. 校验通过，存储零偏参数并初始化姿态角
    handle->gyro_bias_x = mean_gx;
    handle->gyro_bias_y = mean_gy;
    handle->gyro_bias_z = mean_gz;
    handle->accel_bias_roll = 0.0f;
    handle->accel_bias_pitch = 0.0f;

    attitude->roll = 0.0f;
    attitude->pitch = 0.0f;
    attitude->yaw = 0.0f; // Yaw 轴初始清零

    // 5. 串口打印校准结果以供调试
    UART_SendString("IMU Bias Calibrated: X=");
    UART_SendInt((int32_t)(mean_gx * 1000.0f));
    UART_SendString(" Y=");
    UART_SendInt((int32_t)(mean_gy * 1000.0f));
    UART_SendString(" Z=");
    UART_SendInt((int32_t)(mean_gz * 1000.0f));
    UART_SendString(" (mdps)\r\n");

    return true;
}

void LSM6DSV_LoadPresetCalibrate(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude)
{
    handle->gyro_bias_x = IMU_PRESET_BIAS_X;
    handle->gyro_bias_y = IMU_PRESET_BIAS_Y;
    handle->gyro_bias_z = IMU_PRESET_BIAS_Z;
    handle->accel_bias_roll = IMU_PRESET_BIAS_ROLL;
    handle->accel_bias_pitch = IMU_PRESET_BIAS_PITCH;

    attitude->roll = 0.0f;
    attitude->pitch = 0.0f;
    attitude->yaw = 0.0f;
}

void LSM6DSV_UpdateAttitude(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude, float dt)
{
    LSM6DSV_Axes_t accel;
    LSM6DSV_Axes_t gyro;
    float temp;

    // 1. 读取传感器数据，失败时记录失败计数并跳过
    handle->diag_read_total++;
    if (!LSM6DSV_ReadAll(handle, &accel, &gyro, &temp)) {
        handle->diag_read_fail++;
        return;
    }

    // 诊断：保存原始陀螺仪数据（减偏置前）供外部打印
    handle->diag_raw_gx = gyro.x;
    handle->diag_raw_gy = gyro.y;
    handle->diag_raw_gz = gyro.z;

    // 2. 减去陀螺仪的静态漂移
    float gx = gyro.x - handle->gyro_bias_x;
    float gy = gyro.y - handle->gyro_bias_y;
    float gz = gyro.z - handle->gyro_bias_z;

    // 3. 一阶低通滤波（仅用于 Roll/Pitch，Yaw 路径不经过 LPF 以避免响应迟滞）
    const float alpha = 0.20f;
    handle->filtered_gx = alpha * gx + (1.0f - alpha) * handle->filtered_gx;
    handle->filtered_gy = alpha * gy + (1.0f - alpha) * handle->filtered_gy;
    handle->filtered_gz = alpha * gz + (1.0f - alpha) * handle->filtered_gz;

    // 4. 死区处理（仅防止静止漂移，0.10 dps 以内视为静止）
    float gx_final = handle->filtered_gx;
    float gy_final = handle->filtered_gy;
    const float gyro_deadband = 0.10f;
    if (fabsf(gx_final) < gyro_deadband) gx_final = 0.0f;
    if (fabsf(gy_final) < gyro_deadband) gy_final = 0.0f;

    // Yaw 死区略大于 Roll/Pitch（吸收残余偏置，防止零飘）
    float gz_yaw = gz;
    const float yaw_deadband = 0.15f;  /* 增加死区以吸收残余偏置并彻底消除静态零漂 */
    if (fabsf(gz_yaw) < yaw_deadband) gz_yaw = 0.0f;

    // 5. 陀螺仪积分 Roll / Pitch
    float max_change = 2000.0f * dt;
    float delta_roll  = -gx_final * dt;
    float delta_pitch = -gy_final * dt;

    if (delta_roll  >  max_change) delta_roll  =  max_change;
    if (delta_roll  < -max_change) delta_roll  = -max_change;
    if (delta_pitch >  max_change) delta_pitch =  max_change;
    if (delta_pitch < -max_change) delta_pitch = -max_change;

    // 6. 加速度计辅助修正 Roll / Pitch（互补滤波）
    float acc_roll  = atan2f(accel.y, accel.z) * 180.0f / 3.14159265f;
    float acc_pitch = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z)) * 180.0f / 3.14159265f;

    acc_roll  -= handle->accel_bias_roll;
    acc_pitch -= handle->accel_bias_pitch;

    attitude->roll  = 0.98f * (attitude->roll  + delta_roll)  + 0.02f * acc_roll;
    attitude->pitch = 0.98f * (attitude->pitch + delta_pitch) + 0.02f * acc_pitch;

    // 7. Yaw 直接由 gz 积分（对于水平放置的机器人，Z 轴 = 偏航轴）
    //    若传感器 Z 轴不垂直向上，请修改此处轴映射（可改用 gx 或 gy）
    float delta_yaw = gz_yaw * dt;
    if (delta_yaw >  max_change) delta_yaw =  max_change;
    if (delta_yaw < -max_change) delta_yaw = -max_change;

    attitude->yaw += delta_yaw;
}