################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/events.c \
../src/functions.c \
../src/gps.c \
../src/init.c \
../src/main.c \
../src/modbus.c \
../src/powerSensor.c \
../src/sensors.c \
../src/stm32f4xx_it.c \
../src/system_stm32f4xx.c \
../src/tiny_printf.c \
../src/var.c 

S_UPPER_SRCS += \
../src/startup_stm32f4xx.S 

OBJS += \
./src/events.o \
./src/functions.o \
./src/gps.o \
./src/init.o \
./src/main.o \
./src/modbus.o \
./src/powerSensor.o \
./src/sensors.o \
./src/startup_stm32f4xx.o \
./src/stm32f4xx_it.o \
./src/system_stm32f4xx.o \
./src/tiny_printf.o \
./src/var.o 

C_DEPS += \
./src/events.d \
./src/functions.d \
./src/gps.d \
./src/init.d \
./src/main.d \
./src/modbus.d \
./src/powerSensor.d \
./src/sensors.d \
./src/stm32f4xx_it.d \
./src/system_stm32f4xx.d \
./src/tiny_printf.d \
./src/var.d 

S_UPPER_DEPS += \
./src/startup_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\src" -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\Libraries\CMSIS\Include" -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\Libraries\Device\STM32F4xx\Include" -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\Libraries\STM32F4xx_StdPeriph_Driver\inc" -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\src" -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\Libraries\STM32F4xx_StdPeriph_Driver\inc" -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\Libraries\Device\STM32F4xx\Include" -I"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\Libraries\CMSIS\Include" -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


