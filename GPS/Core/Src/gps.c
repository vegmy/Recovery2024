#include <stdio.h>  // For sscanf
#include <string.h> // For strtok
#include <stdint.h> // For uint8_t
#include "i2c.h"
#define I2C_ADDRESS_GPS 0x42 << 1

void parseGPGGA(char *nmeaSentence)
{
    char *token;
    int count = 0;
    char latitude[20] = {0};
    char longitude[20] = {0};
    int fixQuality = 0;
    int numSatellites = 0;

    // Tokenize the sentence by commas
    token = strtok(nmeaSentence, ",");
    while (token != NULL)
    {
        count++;
        switch (count)
        {
        case 3: // Latitude
            strncpy(latitude, token, sizeof(latitude) - 1);
            break;
        case 4: // N or S
            strncat(latitude, token, 1);
            break;
        case 5: // Longitude
            strncpy(longitude, token, sizeof(longitude) - 1);
            break;
        case 6: // E or W
            strncat(longitude, token, 1);
            break;
        case 7: // Fix quality
            // fixQuality = atoi(token);
            break;
        case 8: // Number of satellites
            // numSatellites = atoi(token);
            break;
        default:
            break; // Other fields are not processed
        }
        token = strtok(NULL, ",");
    }

    // Print the extracted data
    printf("Latitude: %s\n", latitude);
    printf("Longitude: %s\n", longitude);
    printf("Fix Quality: %d\n", fixQuality);
    printf("Number of Satellites: %d\n", numSatellites);
}

void sendUBX(uint8_t *msg, uint8_t len)
{
    HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS_GPS, msg, len, HAL_MAX_DELAY); // Write the GPS module's I2C address with write bit
}

void checksumcalc(uint8_t *buffer, uint16_t size)
{
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for (int I = 0; I < size - 2; I++)
    {
        CK_A = CK_A + buffer[I];
        CK_B = CK_B + CK_A;
    }
    buffer[size-2] = CK_A;
    buffer[size-1] = CK_B;
}