
#ifndef STATE_H
#define STATE_H
void pressTobytes(float pressure, uint8_t presbuf[4]);
void stateControll( volatile uint8_t *tilstandsreg1, volatile uint8_t *tilstandsreg2, char *buffer, size_t buffersize);
#endif
