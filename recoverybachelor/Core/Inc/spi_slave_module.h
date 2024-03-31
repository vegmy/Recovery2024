#ifndef __SPI_SLAVE_MODULE_H
#define __SPI_SLAVE_MODULE_H

#include "stdint.h"

typedef enum // fix to struct
{
    READ_GPS            = 0xC1,
    GET_STATUS          = 0xC3,
    RELASE_DROUGE_CHUTE = 0xC5,
    RELEASE_MAIN_CHUTE  = 0xC9,
} rec_commands_t;

typedef enum
{
    IDLE       = 0xA1,
    SEPARATION = 0xA3,
    E_MATCH_M  = 0xA5,
    E_MATCH_R  = 0xA7,
    TENDER_DSC = 0xA9,
    
} rec_flags_t;

void check_command(uint8_t* rx_data, uint8_t* command_flag);


#endif