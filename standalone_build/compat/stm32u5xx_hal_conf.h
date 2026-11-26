#ifndef STM32U5xx_HAL_CONF_H
#define STM32U5xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAL_MODULE_ENABLED
#define HAL_MODULE_ENABLED
#endif
#ifndef HAL_CORTEX_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#endif
#ifndef HAL_DMA_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#endif
#ifndef HAL_EXTI_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#endif
#ifndef HAL_FLASH_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#endif
#ifndef HAL_GPIO_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#endif
#ifndef HAL_ICACHE_MODULE_ENABLED
#define HAL_ICACHE_MODULE_ENABLED
#endif
#ifndef HAL_PWR_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#endif
#ifndef HAL_RCC_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#endif
#ifndef HAL_SPI_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#endif
#ifndef HAL_TIM_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#endif
#ifndef HAL_UART_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#endif
#ifndef HAL_USART_MODULE_ENABLED
#define HAL_USART_MODULE_ENABLED
#endif

#if !defined(HSE_VALUE)
#define HSE_VALUE              16000000UL
#endif
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    100UL
#endif
#if !defined(MSI_VALUE)
#define MSI_VALUE              4000000UL
#endif
#if !defined(HSI_VALUE)
#define HSI_VALUE              16000000UL
#endif
#if !defined(HSI48_VALUE)
#define HSI48_VALUE            48000000UL
#endif
#if !defined(LSI_VALUE)
#define LSI_VALUE              32000UL
#endif
#if !defined(LSI_STARTUP_TIMEOUT)
#define LSI_STARTUP_TIMEOUT    130UL
#endif
#if !defined(LSE_VALUE)
#define LSE_VALUE              32768UL
#endif
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    5000UL
#endif
#if !defined(EXTERNAL_SAI1_CLOCK_VALUE)
#define EXTERNAL_SAI1_CLOCK_VALUE 48000UL
#endif

#define VDD_VALUE                         3300UL
#define TICK_INT_PRIORITY                 ((1UL << __NVIC_PRIO_BITS) - 1UL)
#define USE_RTOS                          0U
#define PREFETCH_ENABLE                   1U

#define USE_HAL_DMA_REGISTER_CALLBACKS    0U
#define USE_HAL_FLASH_REGISTER_CALLBACKS  0U
#define USE_HAL_GPIO_REGISTER_CALLBACKS   0U
#define USE_HAL_ICACHE_REGISTER_CALLBACKS 0U
#define USE_HAL_PWR_REGISTER_CALLBACKS    0U
#define USE_HAL_RCC_REGISTER_CALLBACKS    0U
#define USE_HAL_SPI_REGISTER_CALLBACKS    0U
#define USE_HAL_TIM_REGISTER_CALLBACKS    0U
#define USE_HAL_UART_REGISTER_CALLBACKS   0U
#define USE_HAL_USART_REGISTER_CALLBACKS  0U

#define USE_SPI_CRC                       0U

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32u5xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32u5xx_hal_gpio.h"
#endif
#ifdef HAL_ICACHE_MODULE_ENABLED
#include "stm32u5xx_hal_icache.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32u5xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32u5xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32u5xx_hal_flash.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32u5xx_hal_pwr.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32u5xx_hal_spi.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32u5xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32u5xx_hal_uart.h"
#endif
#ifdef HAL_USART_MODULE_ENABLED
#include "stm32u5xx_hal_usart.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32u5xx_hal_exti.h"
#endif

#define assert_param(expr) ((void)0U)

#ifdef __cplusplus
}
#endif

#endif /* STM32U5xx_HAL_CONF_H */
