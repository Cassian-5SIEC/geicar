################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FLASH_PAGE_F1.c \
../Src/adc.c \
../Src/calibrate.c \
../Src/can.c \
../Src/cassian_i2c.c \
../Src/control.c \
../Src/dma.c \
../Src/gpio.c \
../Src/main.c \
../Src/power.c \
../Src/steering.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/tim.c \
../Src/timer.c \
../Src/usart.c \
../Src/wheels.c 

OBJS += \
./Src/FLASH_PAGE_F1.o \
./Src/adc.o \
./Src/calibrate.o \
./Src/can.o \
./Src/cassian_i2c.o \
./Src/control.o \
./Src/dma.o \
./Src/gpio.o \
./Src/main.o \
./Src/power.o \
./Src/steering.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/tim.o \
./Src/timer.o \
./Src/usart.o \
./Src/wheels.o 

C_DEPS += \
./Src/FLASH_PAGE_F1.d \
./Src/adc.d \
./Src/calibrate.d \
./Src/can.d \
./Src/cassian_i2c.d \
./Src/control.d \
./Src/dma.d \
./Src/gpio.d \
./Src/main.d \
./Src/power.d \
./Src/steering.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/tim.d \
./Src/timer.d \
./Src/usart.d \
./Src/wheels.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103xB -DUSE_HAL_DRIVER -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/FLASH_PAGE_F1.cyclo ./Src/FLASH_PAGE_F1.d ./Src/FLASH_PAGE_F1.o ./Src/FLASH_PAGE_F1.su ./Src/adc.cyclo ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/calibrate.cyclo ./Src/calibrate.d ./Src/calibrate.o ./Src/calibrate.su ./Src/can.cyclo ./Src/can.d ./Src/can.o ./Src/can.su ./Src/cassian_i2c.cyclo ./Src/cassian_i2c.d ./Src/cassian_i2c.o ./Src/cassian_i2c.su ./Src/control.cyclo ./Src/control.d ./Src/control.o ./Src/control.su ./Src/dma.cyclo ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/power.cyclo ./Src/power.d ./Src/power.o ./Src/power.su ./Src/steering.cyclo ./Src/steering.d ./Src/steering.o ./Src/steering.su ./Src/stm32f1xx_hal_msp.cyclo ./Src/stm32f1xx_hal_msp.d ./Src/stm32f1xx_hal_msp.o ./Src/stm32f1xx_hal_msp.su ./Src/stm32f1xx_it.cyclo ./Src/stm32f1xx_it.d ./Src/stm32f1xx_it.o ./Src/stm32f1xx_it.su ./Src/system_stm32f1xx.cyclo ./Src/system_stm32f1xx.d ./Src/system_stm32f1xx.o ./Src/system_stm32f1xx.su ./Src/tim.cyclo ./Src/tim.d ./Src/tim.o ./Src/tim.su ./Src/timer.cyclo ./Src/timer.d ./Src/timer.o ./Src/timer.su ./Src/usart.cyclo ./Src/usart.d ./Src/usart.o ./Src/usart.su ./Src/wheels.cyclo ./Src/wheels.d ./Src/wheels.o ./Src/wheels.su

.PHONY: clean-Src

