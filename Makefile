################################################################################
# User Definitions
#

CROSS_COMPILE = arm-none-eabi-

GIT  = git
CC   = $(CROSS_COMPILE)gcc
CP   = $(CROSS_COMPILE)objcopy
AS   = $(CROSS_COMPILE)gcc -x assembler-with-cpp
BIN  = $(CP) -O $(FORMAT)
LIBS = -lstm32l0 -ltinyusb

STD_PERIPH_LIB = libraries
HAL_DIR = $(STD_PERIPH_LIB)/STM32L0xx_HAL_Driver
CMSIS_DIR = $(STD_PERIPH_LIB)/CMSIS
TINYUSB_DIR = tinyusb

TARGET = main
LDSCRIPT_ROM = stm32l0.ld

SRCDIR = src
ASMDIR = asm
OBJDIR = obj
INCDIR = include
LIBDIR = $(STD_PERIPH_LIB) $(TINYUSB_DIR) device/ldscripts

INCDIR += include/config
INCDIR += $(STD_PERIPH_LIB)/ 
INCDIR += $(HAL_DIR)/inc
INCDIR += $(CMSIS_DIR)/Include
INCDIR += $(CMSIS_DIR)/Device/ST/STM32L0xx/Include
INCDIR += $(TINYUSB_DIR)
INCDIR += $(TINYUSB_DIR)/src

DEVICE = STM32L082xx
MCU  = cortex-m0plus
FORMAT = binary

OPT = s

CDEFS = -D$(DEVICE) -DUSE_HAL_DRIVER -DCFG_TUSB_MCU=OPT_MCU_STM32L0 -DDEBUG


################################################################################
# Default section
#
#ASFLAGS  = -mcpu=$(MCU) -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fstack-usage

CFLAGS   = -mlittle-endian -mthumb -mcpu=$(MCU) -march=armv6s-m
CFLAGS  += -std=c99 -g -O$(OPT) $(CDEFS) -fomit-frame-pointer -Wall -fstack-usage
CFLAGS  += -fverbose-asm -mlittle-endian -ffunction-sections -fdata-sections 
CFLAGS  += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map
LDFLAGS  = -mcpu=$(MCU) -mthumb  -T$(LDSCRIPT_ROM) -Wl,-Map=$(TARGET).map,--cref,--no-warn-mismatch  -lc_nano

CFLAGS  += $(patsubst %,-I%, $(INCDIR))
LDFLAGS += $(patsubst %,-L%, $(LIBDIR))

LDFLAGS += $(LIBS)

CSRC := $(foreach FILE,$(shell find $(SRCDIR) -name *.c | xargs), \
	$(subst $(SRCDIR)/, , $(FILE)))
ASRC := $(foreach FILE,$(shell find $(ASMDIR) -name *.S | xargs), \
	$(subst $(ASMDIR)/, , $(FILE)))

# Define all object files.
OBJ = $(addprefix $(OBJDIR)/,$(CSRC:.c=.o)) $(addprefix $(OBJDIR)/,$(ASRC:.S=.o))

VERSION = $(shell $(GIT) describe --always)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

################################################################################
# Targets
#

all: build

build: init lib elf bin hex

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin

init:
	@if [ ! -e $(OBJDIR) ]; then mkdir -p $(OBJDIR); fi;
	@$(foreach DIR,$(sort $(dir $(CSRC))), if [ ! -e $(OBJDIR)/$(DIR) ]; \
		then mkdir -p $(OBJDIR)/$(DIR); fi; )

version:
	sed 's/".*";$$/"$(VERSION)";/' src/version.template > src/version.c

# Link: create ELF output file from object files.
$(TARGET).elf: $(OBJ) $(TINYUSB_DIR)/libtinyusb.a
	$(CC) $(CFLAGS) $(OBJ) -o $@ $(LDFLAGS)

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

$(TINYUSB_DIR)/libtinyusb.a:
	$(MAKE) -C $(TINYUSB_DIR)
	
%hex: %elf
	@echo
	$(CP) -O ihex $< $@
	@if [ -f $(TARGET).elf ]; then echo; echo Size after:; $(CROSS_COMPILE)size -A $(TARGET).elf; fi

%bin: %elf
	@echo
	$(CP) -O binary $< $@
	@if [ -f $(TARGET).elf ]; then echo; echo Size after:; $(CROSS_COMPILE)size -A $(TARGET).elf; fi

# Compile: create object files from C source files.
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c $(CFLAGS) $< -o $@ 


# Compile: create assembler files from C source files.
$(OBJDIR)/%.s: $(SRCDIR)/%.c
	$(CC) -S $(CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
$(OBJDIR)/%.o: $(ASMDIR)/%.S
	$(CC) -c $(CFLAGS) $< -o $@

program: $(TARGET).bin
	dfu-util -a 0 -s 0x08000000 -D $(TARGET).bin

clean:
	@echo clean:
	-rm -rf $(OBJDIR)
	-rm -f $(TARGET).elf
	-rm -f $(TARGET).hex

distclean: clean
	$(MAKE) -C $(TINYUSB_DIR) clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean


# Target: .tar.gz Archive erstellen.
archive: elf hex
	tar -czvf $(TARGET)_`date +%Y%m%d_%H%M`.tgz $(SRCDIR) $(ASMDIR) $(INCDIR) $(LIBDIR) $(TARGET).elf $(TARGET).hex

test:
	@echo $(CFLAGS)
