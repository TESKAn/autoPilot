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
AutoPilot_SW.hex \

SECONDARY_LIST += \
AutoPilot_SW.lst \

SECONDARY_SIZE += \
AutoPilot_SW.siz \


# All Target
all: AutoPilot_SW.elf secondary-outputs

# Tool invocations
AutoPilot_SW.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc -T"C:\Users\Jure\workspace\TemplateProject\stm32_flash.ld" -Xlinker --gc-sections -Wl,-Map,AutoPilot_SW.map -mcpu=cortex-m4 -mthumb -g -gdwarf-2 -o "AutoPilot_SW.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

AutoPilot_SW.hex: AutoPilot_SW.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex AutoPilot_SW.elf  "AutoPilot_SW.hex"
	@echo 'Finished building: $@'
	@echo ' '

AutoPilot_SW.lst: AutoPilot_SW.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S AutoPilot_SW.elf > "AutoPilot_SW.lst"
	@echo 'Finished building: $@'
	@echo ' '

AutoPilot_SW.siz: AutoPilot_SW.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley AutoPilot_SW.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) AutoPilot_SW.elf
	-@echo ' '

post-build:
	-@echo 'Binary'
	-arm-none-eabi-objcopy -S  -O binary  "AutoPilot_SW.elf" "AutoPilot_SW.bin"
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
