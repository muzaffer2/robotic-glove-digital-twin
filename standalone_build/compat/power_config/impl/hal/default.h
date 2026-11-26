#ifndef COMPAT_POWER_CONFIG_IMPL_HAL_DEFAULT_H
#define COMPAT_POWER_CONFIG_IMPL_HAL_DEFAULT_H

#include "stm32u5xx_hal.h"

void SystemPower_Config(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    (void)HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
}

#endif
