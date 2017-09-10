##############################################################################################
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified to suit our purposes by Hussam Al-Hertani
##############################################################################################

CCPREFIX = arm-none-eabi-
CC = $(CCPREFIX)gcc
CP = $(CCPREFIX)objcopy
AS = $(CCPREFIX)gcc -x assembler-with-cpp
GDBTUI = $(CCPREFIX)gdbtui
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
MCU = cortex-m3

# List all C defines here
DDEFS = -DSTM32F10X -DUSE_STDPERIPH_DRIVER -DSTM32F105RC -DSTM32F10X_CL -D__ASSEMBLY__
# Define project name and Ram/Flash mode here
PROJECT = rys-beacon

# List C source files here
CMSISDIR = Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x
CORELIBDIR = Libraries/CMSIS/CM3/CoreSupport

STMSPSRCDDIR = Libraries/STM32F10x_StdPeriph_Driver/src
STMSPINCDDIR = Libraries/STM32F10x_StdPeriph_Driver/inc

# List of src files to include in build process

SRC = ./main.c
SRC += ./platform/deca_mutex.c
SRC += ./platform/deca_sleep.c
SRC += ./platform/deca_spi.c
SRC += ./platform/lcd.c
SRC += ./platform/port.c
SRC += ./platform/stm32f10x_it.c
SRC += ./platform/syscalls.c
SRC += ./decadriver/deca_device.c
SRC += ./decadriver/deca_params_init.c
SRC += $(CMSISDIR)/system_stm32f10x.c
SRC += $(CORELIBDIR)/core_cm3.c

## used parts of the STM-Library
#SRC += $(STMSPSRCDDIR)/stm32f10x_adc.c
SRC += $(STMSPSRCDDIR)/stm32f10x_bkp.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_can.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_cec.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_crc.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_dac.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_dbgmcu.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_dma.c
SRC += $(STMSPSRCDDIR)/stm32f10x_exti.c
SRC += $(STMSPSRCDDIR)/stm32f10x_flash.c
# SRC += $(STMSPSRCDDIR)/stm32f10x_fsmc.c
SRC += $(STMSPSRCDDIR)/stm32f10x_gpio.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_i2c.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_iwdg.c
SRC += $(STMSPSRCDDIR)/stm32f10x_pwr.c
SRC += $(STMSPSRCDDIR)/stm32f10x_rcc.c
SRC += $(STMSPSRCDDIR)/stm32f10x_rtc.c
# SRC += $(STMSPSRCDDIR)/stm32f10x_sdio.c
SRC += $(STMSPSRCDDIR)/stm32f10x_spi.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_tim.c
SRC += $(STMSPSRCDDIR)/stm32f10x_usart.c
#SRC += $(STMSPSRCDDIR)/stm32f10x_wwdg.c
SRC += $(STMSPSRCDDIR)/misc.c

# List assembly startup source file here
STARTUP = ${CMSISDIR}/startup/startup_stm32f10x_cl.s

# List all directories here
INCDIRS = $(CMSISDIR) \
	$(CORELIBDIR) \
	$(STMSPINCDDIR) \
	decadriver \
	platform

# List the user directory to look for the libraries here
LIBDIRS += Libraries

# List all user libraries here
LIBS =
# LIBS = -Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -Wl,--end-group

# Define optimisation level here
OPT = -Os

# Define linker script file here
LINKER_SCRIPT = ./Linkers/stm32_flash_256k_ram_64k.ld

INCDIR = $(patsubst %,-I%, $(INCDIRS))
LIBDIR = $(patsubst %,-L%, $(LIBDIRS))
##reference only flags for run from ram...not used here
##DEFS = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM

## run from Flash
DEFS = $(DDEFS) -DRUN_FROM_FLASH=1

OBJS = $(STARTUP:.s=.o) $(SRC:.c=.o)
MCFLAGS = -mcpu=$(MCU) -mthumb

ASFLAGS = $(MCFLAGS)
CCFLAGS = $(MCFLAGS) $(OPT) -fomit-frame-pointer -Wall $(DEFS)
LDFLAGS = $(MCFLAGS) -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIBS)

LD_FLAGS :=-Wl,--gc-sections --specs=nosys.specs -u _printf_float -u _scanf_float

###
# makefile rules
###

all: $(OBJS) $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin
	@echo "Size of generated binary:"
	@size $(PROJECT).elf

%.o: %.c
	@echo "CC: $@"
	$(CC) -c $(CCFLAGS) -I . $(INCDIR) $< -o $@

%.o: %.s
	@echo "ASM: $@"
	$(AS) -c $(ASFLAGS) $< -o $@

%.elf: $(OBJS)
	@echo "ELF: $@"
	$(CC) $(OBJS) $(LDFLAGS) -o $@

%.hex: %.elf
	@echo "HEx: $@"
	$(HEX) $< $@

%.bin: %.elf
	@echo "BIN: $@"
	$(BIN) $< $@

flash_stlink: $(PROJECT).bin
	st-flash write $(PROJECT).bin 0x8000000

erase_stlink:
	st-flash erase

clean:
	-rm -rf $(OBJS)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(STARTUP:.s=.lst)

.PHONY: all flash_stlink erase_stlink clean
