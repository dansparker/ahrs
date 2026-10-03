#!/bin/sh
# Fetches the ST CMSIS device files, CMSIS core and the STM32F4 HAL into firmware/Drivers.
set -e
cd "$(dirname "$0")"
mkdir -p Drivers
[ -d Drivers/cmsis_device_f4 ] || git clone --depth 1 --branch v2.6.9 https://github.com/STMicroelectronics/cmsis_device_f4.git Drivers/cmsis_device_f4
[ -d Drivers/stm32f4xx_hal_driver ] || git clone --depth 1 --branch v1.8.5 https://github.com/STMicroelectronics/stm32f4xx_hal_driver.git Drivers/stm32f4xx_hal_driver
[ -d Drivers/cmsis_core ] || git clone --depth 1 --branch v5.9.0_20250520 https://github.com/STMicroelectronics/cmsis_core.git Drivers/cmsis_core
