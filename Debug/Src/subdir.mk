################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ADC.c \
../Src/ADE7953.c \
../Src/GSM.c \
../Src/LCD.c \
../Src/LCD1.c \
../Src/RTC.c \
../Src/Relay.c \
../Src/main.c \
../Src/motor.c \
../Src/myString.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_hal_timebase_TIM.c \
../Src/stm32f0xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f0xx.c 

OBJS += \
./Src/ADC.o \
./Src/ADE7953.o \
./Src/GSM.o \
./Src/LCD.o \
./Src/LCD1.o \
./Src/RTC.o \
./Src/Relay.o \
./Src/main.o \
./Src/motor.o \
./Src/myString.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_hal_timebase_TIM.o \
./Src/stm32f0xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f0xx.o 

C_DEPS += \
./Src/ADC.d \
./Src/ADE7953.d \
./Src/GSM.d \
./Src/LCD.d \
./Src/LCD1.d \
./Src/RTC.d \
./Src/Relay.d \
./Src/main.d \
./Src/motor.d \
./Src/myString.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_hal_timebase_TIM.d \
./Src/stm32f0xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F072xB -I"C:/Users/Jeefo/System_Workbench/workspace/IMCv4_07_1/IMCv4_07_1/Inc" -I"C:/Users/Jeefo/System_Workbench/workspace/IMCv4_07_1/IMCv4_07_1/Drivers/STM32F0xx_HAL_Driver/Inc" -I"C:/Users/Jeefo/System_Workbench/workspace/IMCv4_07_1/IMCv4_07_1/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Jeefo/System_Workbench/workspace/IMCv4_07_1/IMCv4_07_1/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"C:/Users/Jeefo/System_Workbench/workspace/IMCv4_07_1/IMCv4_07_1/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


