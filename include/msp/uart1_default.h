#ifndef MSP_UART1_DEFAULT_H
#define MSP_UART1_DEFAULT_H

#include "stm32u5xx_hal.h"

#include "handlers/error.h"

/* 
USART1:
    - PA9,  TX, AF7, CN10-2, VCP
    - PA10, RX, AF7, CN10-4, VCP
*/


// MSP = MCU Specific Package, setup
void hal_msp_uart1_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /** USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10    ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
};


void hal_msp_uart1_deinit()
{
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /** USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10    ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
}



#endif // MSP_UART1_DEFAULT_H
