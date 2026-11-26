#ifndef MSP_UART_GENERIC_H
#define MSP_UART_GENERIC_H

#include "stm32u5xx_hal.h"

#include "handlers/error.h"

// Define week symbols without a default implementation
//  The compile will only call a strong implementation, otherwise insert a 'nop'
__attribute__((weak)) void hal_msp_uart1_init();
__attribute__((weak)) void hal_msp_uart3_init();
__attribute__((weak)) void hal_msp_uart4_init();
__attribute__((weak)) void hal_msp_uart5_init();

__attribute__((weak)) void hal_msp_uart1_deinit();
__attribute__((weak)) void hal_msp_uart3_deinit();
__attribute__((weak)) void hal_msp_uart4_deinit();
__attribute__((weak)) void hal_msp_uart5_deinit();


/**
 * @brief MSP UART INIT
 * 
 * @param huart 
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance == USART1)
    {
        hal_msp_uart1_init();
    }
    else if(huart->Instance == USART3)
    {
        hal_msp_uart3_init();
    }
    else if(huart->Instance == UART4)
    {
        hal_msp_uart4_init();
    }
    else if(huart->Instance == UART5)
    {
        hal_msp_uart5_init();
    }
}



/**
 * @brief MSP UART Deinit
 * 
 * @param huart 
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance == USART1)
    {
        hal_msp_uart1_deinit();
    }
    else if(huart->Instance == USART3)
    {
        hal_msp_uart3_deinit();
    }
    else if(huart->Instance == UART4)
    {
        hal_msp_uart4_deinit();
    }
    else if(huart->Instance == UART5)
    {
        hal_msp_uart5_deinit();
    }
}


#endif // MSP_UART_GENERIC_H
