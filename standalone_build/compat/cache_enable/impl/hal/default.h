#ifndef COMPAT_CACHE_ENABLE_IMPL_HAL_DEFAULT_H
#define COMPAT_CACHE_ENABLE_IMPL_HAL_DEFAULT_H

#include "stm32u5xx_hal.h"

void CACHE_Enable(void)
{
    (void)HAL_ICACHE_Enable();
}

#endif
