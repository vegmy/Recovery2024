#ifndef GPS_H
#define GPS_H
#include "stm32g4xx_hal.h" // Include the HAL library for STM32G431KB
#include <string.h>        // Include string library for string operations
void processGPSMessage(char* gpggaMessage, char* resultBuffer, size_t bufferSize);
HAL_StatusTypeDef readGPSMessage(I2C_HandleTypeDef *hi2c, uint8_t *message, uint16_t *size);
#endif