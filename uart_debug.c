#include "uart_debug.h"
#include <stdio.h>
#include <rt_sys.h>


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

void Debug_UART_PrintYaw(float yaw)
{
    int32_t yaw_i = (int32_t)(yaw * 10.0f);
    UART_SendString("Yaw:");
    UART_SendInt(yaw_i / 10);
    UART_SendChar('.');
    UART_SendInt((yaw_i < 0 ? -yaw_i : yaw_i) % 10);
    UART_SendString("\r\n");
}

/* 范定义输出函数 */
void Debug_UART(float number)
{
    int32_t yaw_i = (int32_t)(number * 10.0f);
    UART_SendString("Sta:");
    UART_SendInt(yaw_i / 10);
    UART_SendChar('.');
    UART_SendInt((yaw_i < 0 ? -yaw_i : yaw_i) % 10);
    UART_SendString("\r\n");
}

//// Keil MDK (AC5 / AC6) printf 重定向
//#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
//int fputc(int ch, FILE *f)
//{
//    UART_SendChar(ch);
//    return ch;
//}

//#if !defined(__MICROLIB)
//// 非 MicroLib 且禁用半主机模式下，必须重写低级 I/O 接口 (避免 L6915E 错误)
//__asm(".global __use_no_semihosting\n\t");

//FILEHANDLE _sys_open(const char *name, int openmode)
//{
//    return 1; // 返回一个虚拟非0句柄表示成功
//}

//int _sys_close(FILEHANDLE fh)
//{
//    return 0;
//}

//int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)
//{
//    unsigned int i;
//    for (i = 0; i < len; i++)
//    {
//        UART_SendChar(buf[i]);
//    }
//    return 0; // 返回0表示成功写入
//}

//int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode)
//{
//    return -1;
//}

//int _sys_istty(FILEHANDLE fh)
//{
//    return 1;
//}

//int _sys_seek(FILEHANDLE fh, long pos)
//{
//    return -1;
//}

//long _sys_flen(FILEHANDLE fh)
//{
//    return 0;
//}

//void _sys_exit(int return_code)
//{
//    while (1);
//}

//void _ttywrch(int ch)
//{
//    UART_SendChar(ch);
//}
//#endif

//// GCC 编译器 (CCS / GCC) printf 重定向
//#elif defined(__GNUC__)
//int _write(int fd, char *ptr, int len)
//{
//    int i;
//    for (i = 0; i < len; i++)
//    {
//        UART_SendChar(ptr[i]);
//    }
//    return len;
//}
//#endif
