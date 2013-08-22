################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AudioComm.c \
../src/adc.c \
../src/ahrs.c \
../src/ahrs_math.c \
../src/compass.c \
../src/events.c \
../src/flight.c \
../src/functions.c \
../src/gps.c \
../src/init.c \
../src/main.c \
../src/math_custom.c \
../src/modbus.c \
../src/powerSensor.c \
../src/sensors.c \
../src/spi_sd.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f4xx.c \
../src/tiny_printf.c \
../src/usb.c \
../src/usb_bsp.c \
../src/usbd_desc.c \
../src/usbd_usr.c \
../src/var.c 

S_UPPER_SRCS += \
../src/startup_stm32f4xx.S 

OBJS += \
./src/AudioComm.o \
./src/adc.o \
./src/ahrs.o \
./src/ahrs_math.o \
./src/compass.o \
./src/events.o \
./src/flight.o \
./src/functions.o \
./src/gps.o \
./src/init.o \
./src/main.o \
./src/math_custom.o \
./src/modbus.o \
./src/powerSensor.o \
./src/sensors.o \
./src/spi_sd.o \
./src/startup_stm32f4xx.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f4xx.o \
./src/tiny_printf.o \
./src/usb.o \
./src/usb_bsp.o \
./src/usbd_desc.o \
./src/usbd_usr.o \
./src/var.o 

C_DEPS += \
./src/AudioComm.d \
./src/adc.d \
./src/ahrs.d \
./src/ahrs_math.d \
./src/compass.d \
./src/events.d \
./src/flight.d \
./src/functions.d \
./src/gps.d \
./src/init.d \
./src/main.d \
./src/math_custom.d \
./src/modbus.d \
./src/powerSensor.d \
./src/sensors.d \
./src/spi_sd.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f4xx.d \
./src/tiny_printf.d \
./src/usb.d \
./src/usb_bsp.d \
./src/usbd_desc.d \
./src/usbd_usr.d \
./src/var.d 

S_UPPER_DEPS += \
./src/startup_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DUSE_USB_OTG_FS -DUSE_STM32F4_DISCOVERY -DSTM32F4XX -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\CMSIS\Include" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\Device\STM32F4xx\Include" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\STM32F4xx_StdPeriph_Driver\inc" -I"D:\Jure\Projekti\Git\autopilot_sw\src" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\STM32_USB_Device_Library\Class\hid_custom\inc" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\STM32_USB_Device_Library\Core\inc" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\STM32_USB_OTG_Driver\inc" -I"D:\Jure\Projekti\Git\autopilot_sw\FatFS" -I"D:\Jure\Projekti\Git\autopilot_sw\FatFS\option" -O0 -ffunction-sections -fdata-sections -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC Assembler'
	arm-none-eabi-gcc -x assembler-with-cpp -I"D:\Jure\Projekti\Git\autopilot_sw\src" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\CMSIS\Include" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\STM32F4xx_StdPeriph_Driver\inc" -I"D:\Jure\Projekti\Git\autopilot_sw\Libraries\Device\STM32F4xx\Include" -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -g -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


