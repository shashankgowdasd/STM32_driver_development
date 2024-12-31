################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f401re_gpio_driver.c \
../drivers/Src/stm32f401re_spi_driver.c 

OBJS += \
./drivers/Src/stm32f401re_gpio_driver.o \
./drivers/Src/stm32f401re_spi_driver.o 

C_DEPS += \
./drivers/Src/stm32f401re_gpio_driver.d \
./drivers/Src/stm32f401re_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"G:/STM32/STM32_WORKSPACE/Driver_development/STM32F4xx_device_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f401re_gpio_driver.d ./drivers/Src/stm32f401re_gpio_driver.o ./drivers/Src/stm32f401re_spi_driver.d ./drivers/Src/stm32f401re_spi_driver.o

.PHONY: clean-drivers-2f-Src

