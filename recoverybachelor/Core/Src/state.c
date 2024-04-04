#include "main.h"
#include "stdint.h" // For uint8_t
#include <string.h> // For memcpy

    
void pressTobytes(float pressure, uint8_t presbuf[4]) {
    memcpy(presbuf,&pressure, sizeof(float));
}

void stateControll( volatile uint8_t *tilstandsreg1, volatile uint8_t *tilstandsreg2, char *buffer, size_t buffersize) {

    extern volatile GPIO_PinState dio1_em1;
    extern volatile GPIO_PinState dio2_em2;
    extern volatile GPIO_PinState dio3_td1;
    extern volatile GPIO_PinState dio4_td2;
    extern volatile GPIO_PinState foto;
    extern volatile GPIO_PinState magn;
    extern volatile GPIO_PinState heli;
    extern volatile uint8_t trans2_flagg;
    extern volatile uint8_t trans4_flagg;
    extern volatile uint8_t trans6_flagg;
    extern volatile uint8_t eagle_flagg;
    extern volatile uint8_t separation_conf;
    extern volatile uint8_t e_main;
    extern volatile uint8_t e_redundant;
    extern volatile uint8_t tender_decender;
    



    if(dio1_em1 == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x80;
    }
    else
    {
      *tilstandsreg1 &= (~0x80);
    }
    if(dio2_em2 == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x40;
    }
    else
    {
      *tilstandsreg1 &= (~0x40);
    }
    if(dio3_td1 == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x20;
    }
    else
    {
      *tilstandsreg1 &= (~0x20);
    }
    if(dio4_td2 == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x10;
    }
    else
    {
      *tilstandsreg1 &= (~0x10);
    }
    if(foto == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x08;
    }
    else
    {
      *tilstandsreg1 &= (~0x08);
    }
    if(magn == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x04;
    }
    else
    {
      *tilstandsreg1 &= (~0x04);
    }
    if(heli == GPIO_PIN_SET)
    {
      *tilstandsreg1 |=  0x02;
    }
    else
    {
      *tilstandsreg1 &= (~0x02);
    }
    if(separation_conf)
    {
      *tilstandsreg2 |=  0x80;
    }
    else
    {
      *tilstandsreg2 &= (~0x80);
    }
    if(eagle_flagg)
    {
      *tilstandsreg2 |=  0x40;
    }
    else
    {
      *tilstandsreg2 &= (~0x40);
    }
    if(e_main)
    {
      *tilstandsreg2 |=  0x20;
    }
    else
    {
      *tilstandsreg2 &= (~0x20);
    }
    if(e_redundant)
    {
      *tilstandsreg2 |=  0x10;
    }
    else
    {
      *tilstandsreg2 &= (~0x10);
    }
    if(tender_decender)
    {
      *tilstandsreg2 |=  0x08;
    }
    else
    {
      *tilstandsreg2 &= (~0x08);
    }
    strncpy(buffer, tilstandsreg1, buffersize - 1);
    strncpy(buffer, tilstandsreg2, buffersize);
}