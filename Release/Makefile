include config.mk

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

OBJS += ti_drivers_config.o vkart.o main.o 

DBG := -g -gdwarf-3 -gstrict-dwarf
COPT := -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Wall

RTOS := -I"$(SL_TOOL_ROOT)/kernel/nortos" -I"$(SL_TOOL_ROOT)/kernel/nortos/posix"

ORDERED_OBJS += \
	"./ti_drivers_config.o" \
	"./main.o" \
	"./vkart.o" \
	-Wl,-T"../vkart.lds" \
	$(GEN_CMDS__FLAG) \
	-l:"third_party/spiffs/lib/gcc/m4f/spiffs.a" \
	-l:"ti/drivers/lib/drivers_msp432p401x.am4fg" \
	-l:"third_party/fatfs/lib/gcc/m4f/fatfs.a" \
	-l:"lib/nortos_msp432p401x.am4fg" \
	-l:"ti/devices/msp432p4xx/driverlib/gcc/msp432p4xx_driverlib.a" \
	-lgcc \
	-lc \
	-lm \
	-lnosys \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All Target
all: vkart.out

ti_drivers_config.o: ../ti_drivers_config.c 
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	$(CC) -c $(COPT) -I"$(SL_TOOL_ROOT)/source" -I"$(SL_TOOL_ROOT)/source/third_party/CMSIS/Include" $(RTOS) -I"$(CG_TOOL_ROOT)/arm-none-eabi/include/newlib-nano" -I"$(CG_TOOL_ROOT)/arm-none-eabi/include" -ffunction-sections -fdata-sections -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

vkart.o: ../vkart.c 
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"$(CG_TOOL_ROOT)/bin/arm-none-eabi-gcc-7.2.1" -c $(COPT) -I"$(SL_TOOL_ROOT)/source" -I"$(SL_TOOL_ROOT)/source/third_party/CMSIS/Include" $(RTOS) -I"$(SL_TOOL_ROOT)/kernel/nortos/posix" -I"$(CG_TOOL_ROOT)/arm-none-eabi/include/newlib-nano" -I"$(CG_TOOL_ROOT)/arm-none-eabi/include" -ffunction-sections -fdata-sections -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

main.o: ../main.c 
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"$(CG_TOOL_ROOT)/bin/arm-none-eabi-gcc-7.2.1" -c $(COPT) -I"$(SL_TOOL_ROOT)/source" -I"$(SL_TOOL_ROOT)/source/third_party/CMSIS/Include" $(RTOS) -I"$(CG_TOOL_ROOT)/arm-none-eabi/include/newlib-nano" -I"$(CG_TOOL_ROOT)/arm-none-eabi/include" -ffunction-sections -fdata-sections -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(basename\ $(<F)).o" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += vkart.out 

EXE_OUTPUTS__QUOTED += "vkart.out" 

BIN_OUTPUTS += vkart.hex 

BIN_OUTPUTS__QUOTED += "vkart.hex" 



# Tool invocations
vkart.out: $(OBJS) 
	@echo 'Building target: "$@"'
	@echo 'Invoking: GNU Linker'
	"$(CG_TOOL_ROOT)/bin/arm-none-eabi-gcc-7.2.1" -ffunction-sections -fdata-sections $(COPT) -Wl,-Map,"vkart.map" --entry resetISR -nostartfiles -static -Wl,--gc-sections -L"$(SL_TOOL_ROOT)/source" -L"$(SL_TOOL_ROOT)/kernel/nortos" -L"$(CG_TOOL_ROOT)/arm-none-eabi/lib/thumb/v7e-m/fpv4-sp/hard" $(COPT) --specs=nano.specs -o"vkart.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

vkart.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: GNU Objcopy Utility'
	"$(CG_TOOL_ROOT)/bin/arm-none-eabi-objcopy"  $(EXE_OUTPUTS__QUOTED) "vkart.hex"
	@echo 'Finished building secondary target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) vkart.out
	-$(RM) "ti_drivers_config.o" "vkart.o" "main.o" 
	-$(RM) "ti_drivers_config.d" "vkart.d" "main.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

