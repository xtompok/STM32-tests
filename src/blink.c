/**
 * STM32F103C8 Blink Demonstration
 *
 * Kevin Cuzner
 */
 
#include "stm32f0xx.h"
 
int main(void)
{
    //Step 1: Enable the clock to PORT B
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
 
    //Step 2: Change PB0's mode to 0x3 (output) and cfg to 0x0 (push-pull)
    GPIOC->MODER = GPIO_MODER_MODER9_0;
 
    while (1)
    {
        //Step 3: Set PB0 high
        GPIOC->BSRR = GPIO_BSRR_BS_9;
        for (uint16_t i = 0; i != 0xffff; i++) { }
        //Step 4: Reset PB0 low
        GPIOC->BSRR = GPIO_BSRR_BR_9;
        for (uint16_t i = 0; i != 0xffff; i++) { }
    }
 
    return 0;
}
