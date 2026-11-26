#ifndef MSP_BLINK_MIN_H
#define MSP_BLINK_MIN_H

#include "stm32u5xx_hal.h"


/**
 * @brief MSP init for HAL
 * 
 * MSP = MCU specific package
 */
void HAL_MspInit(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    // __HAL_RCC_SYSCFG_CLK_ENABLE();
}

#endif // MSP_BLINK_MIN_H
