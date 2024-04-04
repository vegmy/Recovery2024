#include <stdio.h>  // For sscanf
#include <string.h> // For strtok
#include <stdint.h> // For uint8_t
#include "i2c.h"
#define I2C_ADDRESS_GPS 0x42 << 1



void processGPSMessage(char* gpggaMessage, char* resultBuffer, size_t bufferSize) {
    // Variables to hold the extracted data
    char latitude[12] = {0};
    char longitude[13] = {0};
    char gpsFix[2] = {0};
    char numSatellites[3] = {0};
    memset(resultBuffer, 0, bufferSize);
    

    // Tokenize the GPGGA message using comma as a delimiter
    char* token = strtok(gpggaMessage, ",");
    int field = 0; // Field counter

    while (token != NULL) {
        // Increment field counter to move to the next field in the message
        field++;

        // Extract fields based on their position in the GPGGA message
        switch (field) {
            case 3: // Latitude
                strncpy(latitude, token, sizeof(latitude) - 1);
                break;
            case 4: // N or S indicator for latitude
                strncat(latitude, token, 1);
                break;
            case 5: // Longitude
                strncpy(longitude, token, sizeof(longitude) - 1);
                break;
            case 6: // E or W indicator for longitude
                strncat(longitude, token, 1);
                break;
            case 7: // GPS Fix
                strncpy(gpsFix, token, sizeof(gpsFix) -1);
                break;
            case 8: // Number of satellites
                strncpy(numSatellites, token, sizeof(numSatellites) -1);
                break;
            default:
                break;
        }

        // Get next token
        token = strtok(NULL, ",");
    }
    snprintf(resultBuffer, bufferSize, "%s,%s,%s,%s", latitude, longitude, gpsFix, numSatellites);
}


HAL_StatusTypeDef readGPSMessage(I2C_HandleTypeDef *hi2c, uint8_t *message, uint16_t *size) {
    uint8_t availableBytesHigh = 0, availableBytesLow = 0;
    uint16_t availableBytes = 0;
    
    // Read the high part of the available bytes
    HAL_I2C_Mem_Read(hi2c, I2C_ADDRESS_GPS, 0xFD, I2C_MEMADD_SIZE_8BIT, &availableBytesHigh, 1, HAL_MAX_DELAY);

    // Read the low part of the available bytes
    HAL_I2C_Mem_Read(hi2c, I2C_ADDRESS_GPS, 0xFE, I2C_MEMADD_SIZE_8BIT, &availableBytesLow, 1, HAL_MAX_DELAY);

    // Combine the high and low parts of the available bytes
    availableBytes = (uint16_t)((availableBytesHigh << 8) | availableBytesLow);

    // Assume buffer size is always sufficient and read the available bytes from the message stream
    if(availableBytes > 0) {
        HAL_I2C_Mem_Read(hi2c, I2C_ADDRESS_GPS, 0xFF, I2C_MEMADD_SIZE_8BIT, message, availableBytes, HAL_MAX_DELAY);
        *size = availableBytes; // Set the actual size read
        return HAL_OK; // Indicate data was read
    } else {
        *size = 0; // Set size to 0 to indicate no data was read
        return HAL_OK; // Still return HAL_OK indicating the operation was successful but no data
    }
}


// Function to read the message stream from the GPS
// HAL_StatusTypeDef readGPSMessage(I2C_HandleTypeDef *hi2c, uint8_t *message, uint16_t *size) {
//     HAL_StatusTypeDef ret;
//     uint8_t availableBytesHigh = 0, availableBytesLow = 0;
//     uint16_t availableBytes = 0;
    

//     // Read the high part of the available bytes
//     ret = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS_GPS, 0xFD, I2C_MEMADD_SIZE_8BIT, &availableBytesHigh, 1, 1000);
//     if(ret != HAL_OK) return ret; // Return error if transmission failed
//     //HAL_Delay(1);
//     // Read the low part of the available bytes
//     ret = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS_GPS, 0xFE, I2C_MEMADD_SIZE_8BIT, &availableBytesLow, 1, 1000);
//     if(ret != HAL_OK) return ret; // Return error if transmission failed
//     //HAL_Delay(1);
//     // Combine the high and low parts of the available bytes
//     availableBytes = (uint16_t)((availableBytesHigh << 8) | availableBytesLow);

//     // Check if there are bytes available to read
//     if(availableBytes > 0) {
//         // Ensure your buffer size is large enough to hold the data
//         if (*size < availableBytes) {
//             // Handle the case where buffer provided is smaller than the available data
//             *size = 0; // Indicate buffer was too small
//             return HAL_ERROR;// Buffer too small error
//         }
        
//         // Read the available bytes from the message stream
//         ret = HAL_I2C_Mem_Read(&hi2c1, I2C_ADDRESS_GPS, 0xFF, I2C_MEMADD_SIZE_8BIT, message, availableBytes, 1000);
//         *size = availableBytes; // Set the actual size read
//         //HAL_Delay(5);
//         return ret; // Return the result of the transmission
//     } else {
//         // No data available to read
//         *size = 0; // Set size to 0 to indicate no data was read
//         return HAL_OK; // Operation was successful, just no data to read
//     }
// }

