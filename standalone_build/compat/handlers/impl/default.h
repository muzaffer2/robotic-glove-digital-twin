#ifndef COMPAT_HANDLERS_IMPL_DEFAULT_H
#define COMPAT_HANDLERS_IMPL_DEFAULT_H

#include "stm32u5xx_hal.h"

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

#endif
