################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb-master/src/class/vendor/vendor_device.c \
../tinyusb-master/src/class/vendor/vendor_host.c 

OBJS += \
./tinyusb-master/src/class/vendor/vendor_device.o \
./tinyusb-master/src/class/vendor/vendor_host.o 

C_DEPS += \
./tinyusb-master/src/class/vendor/vendor_device.d \
./tinyusb-master/src/class/vendor/vendor_host.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb-master/src/class/vendor/%.o tinyusb-master/src/class/vendor/%.su tinyusb-master/src/class/vendor/%.cyclo: ../tinyusb-master/src/class/vendor/%.c tinyusb-master/src/class/vendor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I"C:/Users/alberto/Documents/USBCompositeWorkspace/TinyUSB_UAC_CDC_723/tinyusb-master/src" -I"C:/Users/alberto/Documents/USBCompositeWorkspace/TinyUSB_UAC_CDC_723/tinyusb-master/src/device" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"${workspace_loc:/$(ProjName)/hw/bsp}" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2d-master-2f-src-2f-class-2f-vendor

clean-tinyusb-2d-master-2f-src-2f-class-2f-vendor:
	-$(RM) ./tinyusb-master/src/class/vendor/vendor_device.cyclo ./tinyusb-master/src/class/vendor/vendor_device.d ./tinyusb-master/src/class/vendor/vendor_device.o ./tinyusb-master/src/class/vendor/vendor_device.su ./tinyusb-master/src/class/vendor/vendor_host.cyclo ./tinyusb-master/src/class/vendor/vendor_host.d ./tinyusb-master/src/class/vendor/vendor_host.o ./tinyusb-master/src/class/vendor/vendor_host.su

.PHONY: clean-tinyusb-2d-master-2f-src-2f-class-2f-vendor

