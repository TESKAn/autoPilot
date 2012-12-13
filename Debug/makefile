################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Libraries/STM32F4xx_StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
autopilot_sw.hex \

SECONDARY_LIST += \
autopilot_sw.lst \

SECONDARY_SIZE += \
autopilot_sw.siz \


# All Target
all: autopilot_sw.elf secondary-outputs

# Tool invocations
autopilot_sw.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc -T"D:\Jure\Projekti\Git\AutoPilot\autopilot_sw\stm32_flash.ld" -Xlinker --gc-sections -Wl,-Map,autopilot_sw.map -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "autopilot_sw.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

autopilot_sw.hex: autopilot_sw.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex autopilot_sw.elf  "autopilot_sw.hex"
	@echo 'Finished building: $@'
	@echo ' '

autopilot_sw.lst: autopilot_sw.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S autopilot_sw.elf > "autopilot_sw.lst"
	@echo 'Finished building: $@'
	@echo ' '

autopilot_sw.siz: autopilot_sw.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley autopilot_sw.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) autopilot_sw.elf
	-@echo ' '

post-build:
	-@echo 'Binary'
	-arm-none-eabi-objcopy -S  -O binary  "autopilot_sw.elf" "autopilot_sw.bin"
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
