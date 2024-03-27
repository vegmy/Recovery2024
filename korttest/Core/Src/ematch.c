
#include "ematch.h"

void pin_matching(GPIO_TypeDef *first_gpio, uint16_t first_pin,GPIO_TypeDef *second_gpio, uint16_t second_pin){
    if (HAL_GPIO_ReadPin(first_gpio, first_pin) == GPIO_PIN_SET)
    {
        HAL_GPIO_WritePin(second_gpio, second_pin, GPIO_PIN_SET);
    }
    

}