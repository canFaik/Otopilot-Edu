################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LIS3DSH/Src/MY_LIS3DSH.c 

OBJS += \
./LIS3DSH/Src/MY_LIS3DSH.o 

C_DEPS += \
./LIS3DSH/Src/MY_LIS3DSH.d 


# Each subdirectory must supply rules for building sources it contributes
LIS3DSH/Src/%.o LIS3DSH/Src/%.su LIS3DSH/Src/%.cyclo: ../LIS3DSH/Src/%.c LIS3DSH/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/Pilot/ACCE_TEST/Core" -I"D:/Pilot/ACCE_TEST/LIS3DSH/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LIS3DSH-2f-Src

clean-LIS3DSH-2f-Src:
	-$(RM) ./LIS3DSH/Src/MY_LIS3DSH.cyclo ./LIS3DSH/Src/MY_LIS3DSH.d ./LIS3DSH/Src/MY_LIS3DSH.o ./LIS3DSH/Src/MY_LIS3DSH.su

.PHONY: clean-LIS3DSH-2f-Src

