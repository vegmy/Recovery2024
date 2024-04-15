#ifndef __SPI_SLAVE_MODULE_H
#define __SPI_SLAVE_MODULE_H

#include "stdint.h"

/*----- Flight computer commands -----*/
#define READ_GPS            0xC0
#define GET_STATUS          0xC3
#define RELASE_DROUGE_CHUTE 0xC5
#define RELEASE_MAIN_CHUTE  0xC9

/*----- Recovery status values -----*/
#define PASSIVE    0xB0
#define SEPARATION 0xB3
#define E_MATCH_M  0xB5
#define E_MATCH_R  0xB7
#define TENDER_DSC 0xB9
#define MAIN_CHUTE 0xBC

/*----- Handshakes -----*/
#define FC_HANDSHAKE  0xAC
#define REC_HANDSHAKE 0xDC // not a coincidence ( ͡° ͜ʖ ͡°)

/*----- Undecided code ----*/

// typedef enum
// {
//     READ_GPS            = 0xC0,
//     GET_STATUS          = 0xC3,
//     RELASE_DROUGE_CHUTE = 0xC5,
//     RELEASE_MAIN_CHUTE  = 0xC9,
// } rec_commands_t;

// typedef enum
// {
//     PASSIVE    = 0xA0,
//     SEPARATION = 0xA3,
//     E_MATCH_M  = 0xA5,
//     E_MATCH_R  = 0xA7,
//     TENDER_DSC = 0xA9,
    
// } rec_status_t;

void check_command(uint8_t* rx_data, uint8_t* command_flag);


#endif