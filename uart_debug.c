#include "uart_debug.h"

void UART_SendChar(char ch)
{
    uint32_t timeout = 5000;
    while (DL_UART_isTXFIFOFull(DEBUG_UART_INST)) {
        if (--timeout == 0) return; // Skip if UART is stuck
    }
    DL_UART_transmitData(DEBUG_UART_INST, ch);
}

void UART_SendString(const char *str)
{
    while (*str) {
        UART_SendChar(*str++);
    }
}

void UART_SendInt(int32_t num)
{
    char buf[12];
    int i = 0;
    
    if (num == 0) {
        UART_SendChar('0');
        return;
    }
    
    if (num < 0) {
        UART_SendChar('-');
        num = -num;
    }
    
    while (num > 0) {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    
    while (i > 0) {
        UART_SendChar(buf[--i]);
    }
}

void UART_SendHex(uint8_t num)
{
    const char hex_chars[] = "0123456789ABCDEF";
    UART_SendChar(hex_chars[(num >> 4) & 0x0F]);
    UART_SendChar(hex_chars[num & 0x0F]);
}

void Debug_UART_PrintIMU(LSM6DSV_Handle_t *handle, bool imu_status)
{
    if (!imu_status) {
        UART_SendString("LSM6DSV Connection Error!\r\n");
        return;
    }
    
    LSM6DSV_Axes_t accel_data;
    LSM6DSV_Axes_t gyro_data;
    float temp_data;
    
    LSM6DSV_ReadAll(handle, &accel_data, &gyro_data, &temp_data);
    
    // Convert floats to fixed-point integers (x100 for IMU axes, x10 for temperature)
    int32_t ax = (int32_t)(accel_data.x * 100.0f);
    int32_t ay = (int32_t)(accel_data.y * 100.0f);
    int32_t az = (int32_t)(accel_data.z * 100.0f);
    int32_t gx = (int32_t)(gyro_data.x * 100.0f);
    int32_t gy = (int32_t)(gyro_data.y * 100.0f);
    int32_t gz = (int32_t)(gyro_data.z * 100.0f);
    int32_t temp = (int32_t)(temp_data * 10.0f);
    
    // Print Accel
    UART_SendString("ACC: X=");
    UART_SendInt(ax / 100);
    UART_SendChar('.');
    int32_t ax_frac = (ax < 0 ? -ax : ax) % 100;
    if (ax_frac < 10) UART_SendChar('0');
    UART_SendInt(ax_frac);
    
    UART_SendString(", Y=");
    UART_SendInt(ay / 100);
    UART_SendChar('.');
    int32_t ay_frac = (ay < 0 ? -ay : ay) % 100;
    if (ay_frac < 10) UART_SendChar('0');
    UART_SendInt(ay_frac);
    
    UART_SendString(", Z=");
    UART_SendInt(az / 100);
    UART_SendChar('.');
    int32_t az_frac = (az < 0 ? -az : az) % 100;
    if (az_frac < 10) UART_SendChar('0');
    UART_SendInt(az_frac);
    
    // Print Gyro
    UART_SendString(" | GYRO: X=");
    UART_SendInt(gx / 100);
    UART_SendChar('.');
    int32_t gx_frac = (gx < 0 ? -gx : gx) % 100;
    if (gx_frac < 10) UART_SendChar('0');
    UART_SendInt(gx_frac);
    
    UART_SendString(", Y=");
    UART_SendInt(gy / 100);
    UART_SendChar('.');
    int32_t gy_frac = (gy < 0 ? -gy : gy) % 100;
    if (gy_frac < 10) UART_SendChar('0');
    UART_SendInt(gy_frac);
    
    UART_SendString(", Z=");
    UART_SendInt(gz / 100);
    UART_SendChar('.');
    int32_t gz_frac = (gz < 0 ? -gz : gz) % 100;
    if (gz_frac < 10) UART_SendChar('0');
    UART_SendInt(gz_frac);
    
    // Print Temp
    UART_SendString(" | TEMP: ");
    UART_SendInt(temp / 10);
    UART_SendChar('.');
    UART_SendInt((temp < 0 ? -temp : temp) % 10);
    UART_SendString(" C\r\n");
}

void Debug_UART_PrintAttitude(LSM6DSV_Handle_t *handle, LSM6DSV_Attitude_t *attitude)
{
    /* Roll */
    int32_t roll_i = (int32_t)(attitude->roll * 10.0f);
    UART_SendString("R:");
    UART_SendInt(roll_i / 10);
    UART_SendChar('.');
    UART_SendInt((roll_i < 0 ? -roll_i : roll_i) % 10);

    /* Pitch */
    int32_t pitch_i = (int32_t)(attitude->pitch * 10.0f);
    UART_SendString(" P:");
    UART_SendInt(pitch_i / 10);
    UART_SendChar('.');
    UART_SendInt((pitch_i < 0 ? -pitch_i : pitch_i) % 10);

    /* Yaw */
    int32_t yaw_i = (int32_t)(attitude->yaw * 10.0f);
    UART_SendString(" Y:");
    UART_SendInt(yaw_i / 10);
    UART_SendChar('.');
    UART_SendInt((yaw_i < 0 ? -yaw_i : yaw_i) % 10);

    /* 诊断：原始 gz（减偏置前） */
    int32_t gz_i = (int32_t)(handle->diag_raw_gz * 10.0f);
    UART_SendString(" gz:");
    UART_SendInt(gz_i / 10);
    UART_SendChar('.');
    UART_SendInt((gz_i < 0 ? -gz_i : gz_i) % 10);

    /* 诊断：ReadAll 失败率 */
    UART_SendString(" F:");
    UART_SendInt((int32_t)handle->diag_read_fail);
    UART_SendChar('/');
    UART_SendInt((int32_t)handle->diag_read_total);

    UART_SendString("\r\n");
}
