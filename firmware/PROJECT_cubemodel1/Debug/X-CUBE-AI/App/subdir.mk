################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../X-CUBE-AI/App/mnv1.c \
../X-CUBE-AI/App/mnv1_data.c \
../X-CUBE-AI/App/mnv1_data_params.c 

C_DEPS += \
./X-CUBE-AI/App/mnv1.d \
./X-CUBE-AI/App/mnv1_data.d \
./X-CUBE-AI/App/mnv1_data_params.d 

OBJS += \
./X-CUBE-AI/App/mnv1.o \
./X-CUBE-AI/App/mnv1_data.o \
./X-CUBE-AI/App/mnv1_data_params.o 


# Each subdirectory must supply rules for building sources it contributes
X-CUBE-AI/App/%.o X-CUBE-AI/App/%.su X-CUBE-AI/App/%.cyclo: ../X-CUBE-AI/App/%.c X-CUBE-AI/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../X-CUBE-AI/App -I"../Drivers/CMSIS/Device/ST/STM32F7xx/Include " -I../Middlewares/ST/AI/Inc -I../Drivers/CMSIS/Include -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-X-2d-CUBE-2d-AI-2f-App

clean-X-2d-CUBE-2d-AI-2f-App:
	-$(RM) ./X-CUBE-AI/App/mnv1.cyclo ./X-CUBE-AI/App/mnv1.d ./X-CUBE-AI/App/mnv1.o ./X-CUBE-AI/App/mnv1.su ./X-CUBE-AI/App/mnv1_data.cyclo ./X-CUBE-AI/App/mnv1_data.d ./X-CUBE-AI/App/mnv1_data.o ./X-CUBE-AI/App/mnv1_data.su ./X-CUBE-AI/App/mnv1_data_params.cyclo ./X-CUBE-AI/App/mnv1_data_params.d ./X-CUBE-AI/App/mnv1_data_params.o ./X-CUBE-AI/App/mnv1_data_params.su

.PHONY: clean-X-2d-CUBE-2d-AI-2f-App

