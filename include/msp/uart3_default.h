#ifndef MSP_UART3_DEFAULT_H
#define MSP_UART3_DEFAULT_H

#include "stm32u5xx_hal.h"

#include "handlers/error.h"


/* 
USART3:
    - PC10, TX, AF7, CN7-1
    - PC11, RX, AF7, CN7-2
*/

void hal_msp_uart3_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /** USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
};


void hal_msp_uart3_deinit()
{
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /** USART3 GPIO Configuration
        PC10     ------> USART3_TX
        PC11     ------> USART3_RX
    */
   HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);
}



#endif // MSP_UART3_DEFAULT_H
