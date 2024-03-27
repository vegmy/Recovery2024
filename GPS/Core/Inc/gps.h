#ifndef GPS_H
#define GPS_H
#include "stm32g4xx_hal.h" // Include the HAL library for STM32G431KB
#include <string.h>        // Include string library for string operations
void parseGPGGA(char *nmeaSentence);
void sendUBX(uint8_t* msg, uint8_t len);
void checksumcalc(uint8_t *buffer, uint16_t size);
#endif