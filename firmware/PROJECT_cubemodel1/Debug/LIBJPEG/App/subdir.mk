################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LIBJPEG/App/libjpeg.c 

C_DEPS += \
./LIBJPEG/App/libjpeg.d 

OBJS += \
./LIBJPEG/App/libjpeg.o 


# Each subdirectory must supply rules for building sources it contributes
LIBJPEG/App/%.o LIBJPEG/App/%.su LIBJPEG/App/%.cyclo: ../LIBJPEG/App/%.c LIBJPEG/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../LIBJPEG/App -I../LIBJPEG/Target -I../Middlewares/Third_Party/LibJPEG/include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LIBJPEG-2f-App

clean-LIBJPEG-2f-App:
	-$(RM) ./LIBJPEG/App/libjpeg.cyclo ./LIBJPEG/App/libjpeg.d ./LIBJPEG/App/libjpeg.o ./LIBJPEG/App/libjpeg.su

.PHONY: clean-LIBJPEG-2f-App

