################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./empty.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"../Debug - 副本/Src/angle_control.o" \
"../Debug - 副本/Src/attitude.o" \
"../Debug - 副本/Src/board_notify.o" \
"../Debug - 副本/Src/delay.o" \
"../Debug - 副本/Src/encoder.o" \
"../Debug - 副本/Src/gray_serial.o" \
"../Debug - 副本/Src/icm42688.o" \
"../Debug - 副本/Src/line_track.o" \
"../Debug - 副本/Src/motor.o" \
"../Debug - 副本/Src/pid.o" \
"../Debug - 副本/Src/square_track.o" \
"../Debug - 副本/Src/task1_ab.o" \
"../Debug - 副本/Src/task2_abcd.o" \
"../Debug - 副本/Src/task3_acbd.o" \
"../Debug - 副本/Src/uart_cmd.o" \
"../Debug - 副本/Src/vofa.o" \
"../Debug - 副本/empty.o" \
"../Debug - 副本/startup_mspm0g350x_ticlang.o" \
"./Debug - 副本/ti_msp_dl_config.o" \
"../Debug - 副本/ti_msp_dl_config.o" \
"./Src/angle_control.o" \
"./Src/attitude.o" \
"./Src/board_notify.o" \
"./Src/delay.o" \
"./Src/encoder.o" \
"./Src/gray_serial.o" \
"./Src/icm42688.o" \
"./Src/line_track.o" \
"./Src/motor.o" \
"./Src/pid.o" \
"./Src/square_track.o" \
"./Src/task1_ab.o" \
"./Src/task2_abcd.o" \
"./Src/task3_acbd.o" \
"./Src/uart_cmd.o" \
"./Src/vofa.o" \
"../Debug - 副本/device_linker.cmd" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Debug\ -\ 副本/Src/subdir_vars.mk
-include Debug\ -\ 副本/subdir_vars.mk
-include Src/subdir_vars.mk
-include subdir_rules.mk
-include Debug\ -\ 副本/Src/subdir_rules.mk
-include Debug\ -\ 副本/subdir_rules.mk
-include Src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
C07A.out 

EXE_OUTPUTS__QUOTED += \
"C07A.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "C07A.out"

# Tool invocations
C07A.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"D:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -gdwarf-3 -Wl,-m"C07A.map" -Wl,-i"C:/TI/mspm0_sdk_2_10_00_04/source" -Wl,-i"C:/Users/gaofu/workspace_ccstheia/C07A" -Wl,-i"C:/Users/gaofu/workspace_ccstheia/C07A/Debug/syscfg" -Wl,-i"D:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="C07A_linkInfo.xml" -Wl,--rom_model -o "C07A.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "empty.o" "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "Debug - 副本\ti_msp_dl_config.o" "Src\angle_control.o" "Src\attitude.o" "Src\board_notify.o" "Src\delay.o" "Src\encoder.o" "Src\gray_serial.o" "Src\icm42688.o" "Src\line_track.o" "Src\motor.o" "Src\pid.o" "Src\square_track.o" "Src\task1_ab.o" "Src\task2_abcd.o" "Src\task3_acbd.o" "Src\uart_cmd.o" "Src\vofa.o" 
	-$(RM) "empty.d" "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "Debug - 副本\ti_msp_dl_config.d" "Src\angle_control.d" "Src\attitude.d" "Src\board_notify.d" "Src\delay.d" "Src\encoder.d" "Src\gray_serial.d" "Src\icm42688.d" "Src\line_track.d" "Src\motor.d" "Src\pid.d" "Src\square_track.d" "Src\task1_ab.d" "Src\task2_abcd.d" "Src\task3_acbd.d" "Src\uart_cmd.d" "Src\vofa.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

