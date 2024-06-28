################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../TEST/Src/MY_LIS3DSH.c 

OBJS += \
./TEST/Src/MY_LIS3DSH.o 

C_DEPS += \
./TEST/Src/MY_LIS3DSH.d 


# Each subdirectory must supply rules for building sources it contributes
TEST/Src/%.o TEST/Src/%.su TEST/Src/%.cyclo: ../TEST/Src/%.c TEST/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/f-a-i/STM32CubeIDE/workspace_1.14.0/ACCE_TEST/TEST/Inc" -I"C:/Users/f-a-i/STM32CubeIDE/workspace_1.14.0/ACCE_TEST/Core" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-TEST-2f-Src

clean-TEST-2f-Src:
	-$(RM) ./TEST/Src/MY_LIS3DSH.cyclo ./TEST/Src/MY_LIS3DSH.d ./TEST/Src/MY_LIS3DSH.o ./TEST/Src/MY_LIS3DSH.su

.PHONY: clean-TEST-2f-Src

