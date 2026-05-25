#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#include "ti_msp_dl_config.h"
#include "lsm6dsv.h"

// Send a single character via UART (non-blocking with timeout)
void UART_SendChar(char ch);

// Send a string via UART
void UART_SendString(const char *str);

// Send a signed integer via UART
void UART_SendInt(int32_t num);

// Send a byte as a hexadecimal string via UART
void UART_SendHex(uint8_t num);

// Print IMU data via UART
void Debug_UART_PrintIMU(LSM6DSV_Handle_t *handle, bool imu_status);

// Print only yaw value via UART
void Debug_UART_PrintYaw(float yaw);

#endif /* UART_DEBUG_H */
