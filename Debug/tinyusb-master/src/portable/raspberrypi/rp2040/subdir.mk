################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.c \
../tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.c \
../tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.c 

OBJS += \
./tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.o \
./tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.o \
./tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.o 

C_DEPS += \
./tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.d \
./tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.d \
./tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb-master/src/portable/raspberrypi/rp2040/%.o tinyusb-master/src/portable/raspberrypi/rp2040/%.su tinyusb-master/src/portable/raspberrypi/rp2040/%.cyclo: ../tinyusb-master/src/portable/raspberrypi/rp2040/%.c tinyusb-master/src/portable/raspberrypi/rp2040/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I"C:/Users/alberto/Documents/USBCompositeWorkspace/TinyUSB_UAC_CDC_723/tinyusb-master/src" -I"C:/Users/alberto/Documents/USBCompositeWorkspace/TinyUSB_UAC_CDC_723/tinyusb-master/src/device" -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"${workspace_loc:/$(ProjName)/hw/bsp}" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2d-master-2f-src-2f-portable-2f-raspberrypi-2f-rp2040

clean-tinyusb-2d-master-2f-src-2f-portable-2f-raspberrypi-2f-rp2040:
	-$(RM) ./tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.cyclo ./tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.d ./tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.o ./tinyusb-master/src/portable/raspberrypi/rp2040/dcd_rp2040.su ./tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.cyclo ./tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.d ./tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.o ./tinyusb-master/src/portable/raspberrypi/rp2040/hcd_rp2040.su ./tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.cyclo ./tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.d ./tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.o ./tinyusb-master/src/portable/raspberrypi/rp2040/rp2040_usb.su

.PHONY: clean-tinyusb-2d-master-2f-src-2f-portable-2f-raspberrypi-2f-rp2040

