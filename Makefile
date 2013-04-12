##########################################################################
# User configuration and firmware specific object files	
##########################################################################

PROJECT=firmware

TARGET = lpc13u
ifeq (lpc11u,$(TARGET))
  CORE = cortex-m0
  LDSCRIPT = cmsis/lpc11U37.ld
else
  CORE = cortex-m3
  LDSCRIPT = cmsis/lpc1347.ld
endif

OPTIMIZATION = s

##########################################################################
# Output directories
##########################################################################

BIN_PATH = bin
OBJ_PATH = bin/obj

##########################################################################
# Source Files
##########################################################################

VPATH = cmsis
ifeq (lpc11u,$(TARGET))
  OBJS   = $(OBJ_PATH)/startup_lpc11u.o 
  OBJS  += $(OBJ_PATH)/system_LPC11Uxx.o
else
  OBJS   = $(OBJ_PATH)/startup_lpc13u.o 
  OBJS  += $(OBJ_PATH)/system_LPC13Uxx.o
endif

VPATH += src
OBJS  += $(OBJ_PATH)/main.o
OBJS  += $(OBJ_PATH)/messages.o
OBJS  += $(OBJ_PATH)/printf-retarget.o

VPATH += src/boards/lpcxpresso1347
OBJS  += $(OBJ_PATH)/board_lpcxpresso1347.o

# VPATH += src/boards/rf1ghznode
# OBJS  += $(OBJ_PATH)/board_rf1ghznode.o

VPATH += src/boards/rf1ghzusb
OBJS  += $(OBJ_PATH)/board_rf1ghzusb.o

VPATH += src/cli
OBJS  += $(OBJ_PATH)/cli.o 
OBJS  += $(OBJ_PATH)/commands.o

VPATH += src/cli/commands
OBJS  += $(OBJ_PATH)/cmd_chibi_addr.o 
OBJS  += $(OBJ_PATH)/cmd_chibi_tx.o 
OBJS  += $(OBJ_PATH)/cmd_dbg_memrd.o
OBJS  += $(OBJ_PATH)/cmd_eeprom_read.o 
OBJS  += $(OBJ_PATH)/cmd_eeprom_write.o 
OBJS  += $(OBJ_PATH)/cmd_i2c_read.o
OBJS  += $(OBJ_PATH)/cmd_i2c_scan.o 
OBJS  += $(OBJ_PATH)/cmd_i2c_write.o 
OBJS  += $(OBJ_PATH)/cmd_nfc_mfc_ndef.o
OBJS  += $(OBJ_PATH)/cmd_nfc_mifareclassic_memdump.o
OBJS  += $(OBJ_PATH)/cmd_nfc_mifareclassic_valueblock.o 
OBJS  += $(OBJ_PATH)/cmd_nfc_mifareultralight_memdump.o 
OBJS  += $(OBJ_PATH)/cmd_rtc_read.o
OBJS  += $(OBJ_PATH)/cmd_rtc_write.o
OBJS  += $(OBJ_PATH)/cmd_sd_dir.o
OBJS  += $(OBJ_PATH)/cmd_sysinfo.o

VPATH += src/core/adc
OBJS  += $(OBJ_PATH)/adc.o

VPATH += src/core/eeprom
OBJS  += $(OBJ_PATH)/eeprom.o

VPATH += src/core/fifo
OBJS  += $(OBJ_PATH)/fifo.o

VPATH += src/core/gpio
OBJS  += $(OBJ_PATH)/gpio.o

VPATH += src/core/i2c
OBJS  += $(OBJ_PATH)/i2c.o

VPATH += src/core/iap
OBJS  += $(OBJ_PATH)/iap.o

VPATH += src/core/libc
OBJS  += $(OBJ_PATH)/stdio.o 
OBJS  += $(OBJ_PATH)/string.o

VPATH += src/core/pmu
OBJS  += $(OBJ_PATH)/pmu.o

VPATH += src/core/ssp0
OBJS  += $(OBJ_PATH)/ssp0.o

VPATH += src/core/ssp1
OBJS  += $(OBJ_PATH)/ssp1.o

VPATH += src/core/systick
OBJS  += $(OBJ_PATH)/systick.o

VPATH += src/core/timer32
OBJS  += $(OBJ_PATH)/timer32.o

VPATH += src/core/uart
OBJS  += $(OBJ_PATH)/uart.o 
OBJS  += $(OBJ_PATH)/uart_buf.o

VPATH += src/core/usb
OBJS  += $(OBJ_PATH)/descriptors.o 
OBJS  += $(OBJ_PATH)/usb_cdc.o 
OBJS  += $(OBJ_PATH)/usb_hid.o 
OBJS  += $(OBJ_PATH)/usb_msc.o 
OBJS  += $(OBJ_PATH)/usbd.o

VPATH += src/drivers
OBJS  += $(OBJ_PATH)/timespan.o

VPATH += src/drivers/displays
OBJS  += $(OBJ_PATH)/smallfonts.o

VPATH += src/drivers/displays/bitmap/ssd1306
OBJS  += $(OBJ_PATH)/ssd1306_i2c.o

VPATH += src/drivers/displays/graphic 
OBJS  += $(OBJ_PATH)/aafonts.o 
OBJS  += $(OBJ_PATH)/colors.o 
OBJS  += $(OBJ_PATH)/drawing.o 
OBJS  += $(OBJ_PATH)/fonts.o 
OBJS  += $(OBJ_PATH)/theme.o

VPATH += src/drivers/displays/graphic/aafonts/aa2 
OBJS  += $(OBJ_PATH)/DejaVuSansCondensed14_AA2.o 
OBJS  += $(OBJ_PATH)/DejaVuSansCondensedBold14_AA2.o 
OBJS  += $(OBJ_PATH)/DejaVuSansMono10_AA2.o 
OBJS  += $(OBJ_PATH)/DejaVuSansMono13_AA2.o 
OBJS  += $(OBJ_PATH)/DejaVuSansMono14_AA2.o 
OBJS  += $(OBJ_PATH)/FontCalibri18_AA2.o 
OBJS  += $(OBJ_PATH)/FontCalibriBold18_AA2.o 
OBJS  += $(OBJ_PATH)/FontCalibriItalic18_AA2.o 
OBJS  += $(OBJ_PATH)/FontFranklinGothicBold99_Numbers_AA2.o

VPATH += src/drivers/displays/graphic/aafonts/aa4 
OBJS  += $(OBJ_PATH)/FontCalibri18_AA4.o

VPATH += src/drivers/displays/graphic/fonts 
OBJS  += $(OBJ_PATH)/dejavusans9.o 
OBJS  += $(OBJ_PATH)/dejavusansbold9.o 
OBJS  += $(OBJ_PATH)/dejavusanscondensed9.o 
OBJS  += $(OBJ_PATH)/dejavusansmono8.o 
OBJS  += $(OBJ_PATH)/dejavusansmonobold8.o 
OBJS  += $(OBJ_PATH)/veramono9.o 
OBJS  += $(OBJ_PATH)/veramono11.o 
OBJS  += $(OBJ_PATH)/veramonobold9.o 
OBJS  += $(OBJ_PATH)/veramonobold11.o 
OBJS  += $(OBJ_PATH)/verdana9.o 
OBJS  += $(OBJ_PATH)/verdana14.o 
OBJS  += $(OBJ_PATH)/verdanabold14.o

VPATH += src/drivers/displays/graphic/hw
OBJS  += $(OBJ_PATH)/hx8340b.o 
# OBJS  += $(OBJ_PATH)/hx8347g.o

VPATH += src/drivers/displays/segment/ht16k33
OBJS  += $(OBJ_PATH)/ht16k33.o

VPATH += src/drivers/motor/stepper
OBJS  += $(OBJ_PATH)/stepper.o

VPATH += src/drivers/pwm/pca9685
OBJS  += $(OBJ_PATH)/pca9685.o

VPATH += src/drivers/rf/chibi
OBJS  += $(OBJ_PATH)/chb.o 
OBJS  += $(OBJ_PATH)/chb_buf.o 
OBJS  += $(OBJ_PATH)/chb_drvr.o 
OBJS  += $(OBJ_PATH)/chb_eeprom.o 
OBJS  += $(OBJ_PATH)/chb_spi.o

VPATH += src/drivers/rf/pn532
OBJS  += $(OBJ_PATH)/pn532.o 
OBJS  += $(OBJ_PATH)/pn532_bus_i2c.o 
OBJS  += $(OBJ_PATH)/pn532_bus_uart.o

VPATH += src/drivers/rf/pn532/helpers
OBJS  += $(OBJ_PATH)/pn532_config.o 
OBJS  += $(OBJ_PATH)/pn532_gpio.o 
OBJS  += $(OBJ_PATH)/pn532_mifare_classic.o 
OBJS  += $(OBJ_PATH)/pn532_mifare_ultralight.o 
OBJS  += $(OBJ_PATH)/pn532_ndef.o 
OBJS  += $(OBJ_PATH)/pn532_ndef_cards.o 

VPATH += src/drivers/rf/pn532/mem_allocator
OBJS  += $(OBJ_PATH)/bget.o 
OBJS  += $(OBJ_PATH)/pn532_mem.o 

VPATH += src/drivers/rtc
OBJS  += $(OBJ_PATH)/rtc.o

VPATH += src/drivers/rtc/pcf2129
OBJS  += $(OBJ_PATH)/pcf2129.o

VPATH += src/drivers/sensors
OBJS  += $(OBJ_PATH)/sensors.o

VPATH += src/drivers/sensors/accelerometers
OBJS  += $(OBJ_PATH)/adxl345.o
OBJS  += $(OBJ_PATH)/lis3dh.o
OBJS  += $(OBJ_PATH)/lsm303accel.o

VPATH += src/drivers/sensors/gyroscopes
OBJS  += $(OBJ_PATH)/l3gd20.o

VPATH += src/drivers/sensors/light
OBJS  += $(OBJ_PATH)/tsl2561.o

VPATH += src/drivers/sensors/magnetometers
OBJS  += $(OBJ_PATH)/lsm303mag.o

VPATH += src/drivers/sensors/pressure
OBJS  += $(OBJ_PATH)/bmp085.o
OBJS  += $(OBJ_PATH)/mpl115a2.o

VPATH += src/drivers/sensors/temperature
OBJS  += $(OBJ_PATH)/lm75b.o

VPATH += src/drivers/statistics
OBJS  += $(OBJ_PATH)/avg_d.o 
OBJS  += $(OBJ_PATH)/avg_f.o 
OBJS  += $(OBJ_PATH)/avg_i.o
OBJS  += $(OBJ_PATH)/iir_f.o

VPATH += src/drivers/storage/fatfs
OBJS  += $(OBJ_PATH)/ff.o 
OBJS  += $(OBJ_PATH)/mmc.o

VPATH += src/localisation
OBJS  += $(OBJ_PATH)/localisation.o

##########################################################################
# Include paths
##########################################################################

ROOT_PATH = src
INCLUDE_PATHS = -I$(ROOT_PATH) -Icmsis

##########################################################################
# GNU GCC compiler prefix
##########################################################################

# Use the default toolchain (based on the PATH variable, etc.)
CROSS_COMPILE ?= arm-none-eabi-

# Use a toolchain at a specific location
# CROSS_COMPILE = C:/code_red/RedSuiteNXP_5.0.12_1048/redsuite/tools/bin/arm-none-eabi-
# CROSS_COMPILE = C:/arm/gnu4.7.2012.q4/bin/arm-none-eabi-

AS      = $(CROSS_COMPILE)gcc
CC      = $(CROSS_COMPILE)gcc
LD      = $(CROSS_COMPILE)gcc
SIZE    = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
OUTFILE = $(BIN_PATH)/$(PROJECT)
LPCRC   ?= ./lpcrc
REMOVE  = rm -f
MOUNT_POINT ?= /media/CRP DISABLD

##########################################################################
# Compiler settings, parameters and flags
##########################################################################

# Compiler Options
GCFLAGS  = -c 
GCFLAGS += -std=gnu99 
GCFLAGS += -g 
GCFLAGS += -O$(OPTIMIZATION) 
GCFLAGS += $(INCLUDE_PATHS) 
GCFLAGS += -Wall 
GCFLAGS += -mthumb 
GCFLAGS += -ffunction-sections 
GCFLAGS += -fdata-sections 
GCFLAGS += -fmessage-length=0 
GCFLAGS += -fno-builtin 
GCFLAGS += -mcpu=$(CORE) 
GCFLAGS += -DTARGET=$(TARGET)
# For use with the GCC ARM Embedded toolchain
# GCFLAGS += --specs=nano.specs
# For use with the LPCXpresso toolchain
# GCFLAGS += -D__REDLIB__ -D__CODE_RED

# Assembler Options
ASFLAGS  = -c 
ASFLAGS += -g 
ASFLAGS += -O$(OPTIMIZATION) 
ASFLAGS += $(INCLUDE_PATHS) 
ASFLAGS += -Wall 
ASFLAGS += -mthumb 
ASFLAGS += -ffunction-sections 
ASFLAGS += -fdata-sections 
ASFLAGS += -fmessage-length=0 
ASFLAGS += -mcpu=$(CORE) 
ASFLAGS += -D__ASSEMBLY__ 
ASFLAGS += -x assembler-with-cpp

# Linker Options
LDFLAGS  = -nostartfiles 
LDFLAGS += -mcpu=$(CORE) 
LDFLAGS += -mthumb 
LDFLAGS += -O$(OPTIMIZATION) 
LDFLAGS += -Wl,--gc-sections 
LDFLAGS += -T $(LDSCRIPT)

# External Libraries
LDLIBS   = -lm
# The following libraries are required with the LPCXpresso toolchain
# LDLIBS  += -lcr_c -lcr_eabihelpers

OCFLAGS = --strip-unneeded

##########################################################################
# Rules
##########################################################################

all: firmware

$(OBJ_PATH)/%.o : %.c
	@mkdir -p $(dir $@)
	-@echo "COMPILING $(@F)"
	@$(CC) $(GCFLAGS) -o $@ $<

$(OBJ_PATH)/%.o : %.s
	@mkdir -p $(dir $@)
	-@echo "ASSEMBLING $(@F)"
	@$(AS) $(ASFLAGS) -o $@ $<

firmware: $(OBJS) $(SYS_OBJS)
	@mkdir -p $(BIN_PATH)
	-@echo ""
	-@echo "LINKING $(OUTFILE).elf ($(CORE) -O$(OPTIMIZATION))"
	@$(LD) $(LDFLAGS) -o $(OUTFILE).elf $(LDLIBS) $(OBJS) $(LDLIBS)
	-@echo ""
	@$(SIZE) $(OUTFILE).elf
	-@echo ""
	-@echo "Generating $(OUTFILE).hex"
	@$(OBJCOPY) $(OCFLAGS) -O ihex $(OUTFILE).elf $(OUTFILE).hex
	-@echo "Generating $(OUTFILE).bin"
	@$(OBJCOPY) $(OCFLAGS) -O binary $(OUTFILE).elf $(OUTFILE).bin
	-@echo ""
	@$(LPCRC) $(OUTFILE).bin

flash: firmware
	-@echo -n "Flashing..."
	-@[ -e "$(MOUNT_POINT)/firmware.bin" ] && dd if=bin/firmware.bin of="$(MOUNT_POINT)/firmware.bin" conv=nocreat,notrunc && umount "$(MOUNT_POINT)" || echo "Error, no device?!"

clean:
	@$(REMOVE) $(OBJS) $(OUTFILE).elf $(OUTFILE).bin $(OUTFILE).hex

#########################################################################
