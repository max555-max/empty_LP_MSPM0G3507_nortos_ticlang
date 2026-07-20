################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Debug\ -\ 副本/ti_msp_dl_config.o: ../Debug\ -\ 副本/ti_msp_dl_config.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/gaofu/workspace_ccstheia/C07A" -I"C:/Users/gaofu/workspace_ccstheia/C07A/Debug" -I"C:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/TI/mspm0_sdk_2_10_00_04/source" -I"C:/Users/gaofu/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/Src" -I"C:/Users/gaofu/workspace_ccstheia/empty_LP_MSPM0G3507_nortos_ticlang/Inc" -gdwarf-3 -MMD -MP -MF"Debug - 副本/ti_msp_dl_config.d_raw" -MT"Debug\ -\ 副本/ti_msp_dl_config.o"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


