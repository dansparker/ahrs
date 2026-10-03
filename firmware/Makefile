# AHRS firmware for the STM32F446RE. Drivers: ./fetch_drivers.sh
PREFIX ?= arm-none-eabi-
CC = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE = $(PREFIX)size

D = Drivers
HAL = $(D)/stm32f4xx_hal_driver/Src
BUILD = build

HAL_SRC = stm32f4xx_hal.c stm32f4xx_hal_cortex.c stm32f4xx_hal_rcc.c stm32f4xx_hal_rcc_ex.c stm32f4xx_hal_gpio.c \
          stm32f4xx_hal_dma.c stm32f4xx_hal_pwr.c stm32f4xx_hal_pwr_ex.c stm32f4xx_hal_i2c.c stm32f4xx_hal_spi.c \
          stm32f4xx_hal_uart.c stm32f4xx_hal_can.c stm32f4xx_hal_flash.c stm32f4xx_hal_flash_ex.c stm32f4xx_hal_iwdg.c

APP_SRC = src/main.c src/sensors.c src/comms.c \
          ../core/eskf.c ../core/magcal.c ../core/ahrs.c ../core/ubx.c ../core/canas.c ../core/can_out.c ../core/baro.c

SRC = $(APP_SRC) $(addprefix $(HAL)/,$(HAL_SRC)) $(D)/cmsis_device_f4/Source/Templates/system_stm32f4xx.c
ASM = $(D)/cmsis_device_f4/Source/Templates/gcc/startup_stm32f446xx.s

MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS = -DSTM32F446xx -DUSE_HAL_DRIVER
INC = -Isrc -I../core -I$(D)/stm32f4xx_hal_driver/Inc -I$(D)/cmsis_device_f4/Include -I$(D)/cmsis_core/Include
CFLAGS = $(MCU) $(DEFS) $(INC) -std=c11 -O2 -g -ffunction-sections -fdata-sections \
         -Wall -Wextra
# our own sources are held to -Werror; ST's HAL is not
APP_CFLAGS = -Werror
LDFLAGS = $(MCU) -TSTM32F446RETx_FLASH.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/ahrs.map --specs=nano.specs --specs=nosys.specs -lm

OBJ = $(patsubst %.c,$(BUILD)/%.o,$(subst ../,,$(SRC))) $(BUILD)/startup.o

all: $(BUILD)/ahrs.bin
	$(SIZE) $(BUILD)/ahrs.elf

$(BUILD)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(APP_CFLAGS) -c $< -o $@
$(BUILD)/core/%.o: ../core/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(APP_CFLAGS) -c $< -o $@
$(BUILD)/$(D)/%.o: $(D)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -w -c $< -o $@
$(BUILD)/startup.o: $(ASM)
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -c $< -o $@

$(BUILD)/ahrs.elf: $(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) -o $@
$(BUILD)/ahrs.bin: $(BUILD)/ahrs.elf
	$(OBJCOPY) -O binary $< $@
	$(OBJCOPY) -O ihex $< $(BUILD)/ahrs.hex

clean:
	rm -rf $(BUILD)
.PHONY: all clean
