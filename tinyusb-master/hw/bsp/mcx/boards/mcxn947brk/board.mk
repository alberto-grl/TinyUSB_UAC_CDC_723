MCU_VARIANT = MCXN947
MCU_CORE = MCXN947_cm33_core0
PORT ?= 1

CFLAGS += -DCPU_MCXN947VDF_cm33_core0

JLINK_DEVICE = LPC55S69
PYOCD_TARGET = LPC55S69

# flash using pyocd
flash: flash-pyocd