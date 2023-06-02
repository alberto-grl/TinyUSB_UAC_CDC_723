################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.c 

OBJS += \
./tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.o 

C_DEPS += \
./tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb-master/src/portable/espressif/esp32sx/%.o tinyusb-master/src/portable/espressif/esp32sx/%.su tinyusb-master/src/portable/espressif/esp32sx/%.cyclo: ../tinyusb-master/src/portable/espressif/esp32sx/%.c tinyusb-master/src/portable/espressif/esp32sx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I"C:/Users/alberto/Documents/USBCompositeWorkspace/723TinyUSB3/tinyusb-master/src" -I"C:/Users/alberto/Documents/USBCompositeWorkspace/723TinyUSB3/tinyusb-master/src/device" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"${workspace_loc:/$(ProjName)/hw/bsp}" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2d-master-2f-src-2f-portable-2f-espressif-2f-esp32sx

clean-tinyusb-2d-master-2f-src-2f-portable-2f-espressif-2f-esp32sx:
	-$(RM) ./tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.cyclo ./tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.d ./tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.o ./tinyusb-master/src/portable/espressif/esp32sx/dcd_esp32sx.su

.PHONY: clean-tinyusb-2d-master-2f-src-2f-portable-2f-espressif-2f-esp32sx

