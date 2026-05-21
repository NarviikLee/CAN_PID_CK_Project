################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/src/PID_M.c 

OBJS += \
./Core/lib/src/PID_M.o 

C_DEPS += \
./Core/lib/src/PID_M.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/src/%.o Core/lib/src/%.su: ../Core/lib/src/%.c Core/lib/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"H:/STM32CubeIDE_1.9.0/Stm32_WorkSpace/OJ_CODING_EMBEDED/Part4/CAN_PID_CK_Project/Core/lib/inc" -I"H:/STM32CubeIDE_1.9.0/Stm32_WorkSpace/OJ_CODING_EMBEDED/Part4/CAN_PID_CK_Project/Core/lib" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lib-2f-src

clean-Core-2f-lib-2f-src:
	-$(RM) ./Core/lib/src/PID_M.d ./Core/lib/src/PID_M.o ./Core/lib/src/PID_M.su

.PHONY: clean-Core-2f-lib-2f-src

