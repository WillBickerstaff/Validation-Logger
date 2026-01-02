# ---------------------------------------------------------------------------
# Toolchain selection
# ---------------------------------------------------------------------------
# Explicitly name the AVR toolchain components to avoid accidental use of
# host compilers or wrapper scripts.
CC      := avr-gcc
OBJCOPY := avr-objcopy
SIZE    := avr-size

# ---------------------------------------------------------------------------
# Target hardware configuration
# ---------------------------------------------------------------------------
# MCU type must match the physical device fitted on the validation logger PCB.
# This affects register definitions, linker scripts, and interrupt vectors.
MCU     := atmega328p

# CPU clock frequency in Hz.
# Must match fuse configuration and oscillator source on the hardware.
# This value is used by util/delay, UART baud calculations, and later timers.
F_CPU   := 8000000UL

# ---------------------------------------------------------------------------
# Project structure
# ---------------------------------------------------------------------------
# Logical name of the firmware image.
# Used to name build artefacts (.elf, .hex).
TARGET  := logger

# Source files for this stage.
# Kept deliberately minimal for initial bring-up.
SRC     := main.c timer1_capture.c
OBJ     := $(SRC:.c=.o)

# ---------------------------------------------------------------------------
# Compiler and linker flags
# ---------------------------------------------------------------------------
# -mmcu      : select correct AVR architecture
# -DF_CPU    : ensure compiler sees the same clock value as the source
# -Os        : optimise for size (predictable, commonly used on AVR)
# -std=c11   : explicit C standard
# -Wall/...  : treat warnings seriously during validation work
#              Enable (1) or disable (0) Timer1 input capture noise canceller
#              (ICNC1). This affects edge filtering and timing fidelity and
#              is intentionally controlled at build time for reproducibility.
CFLAGS  := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=c11 \
           -Wall -Wextra -Werror \
           -DTIMER1_CAPTURE_USE_NOISE_CANCEL=1

# Linker must also know the MCU type to select the correct memory layout.
LDFLAGS := -mmcu=$(MCU)

# ---------------------------------------------------------------------------
# Output artefacts
# ---------------------------------------------------------------------------
# ELF is the linked binary with symbols (useful for inspection).
# HEX is the format actually flashed to the device.
ELF     := $(TARGET).elf
HEX     := $(TARGET).hex

# ---------------------------------------------------------------------------
# Build targets
# ---------------------------------------------------------------------------

# Default target: build the firmware image.
all: $(HEX)

# Link object files into an ELF binary.
$(ELF): $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^

# Compile C source into object files.
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Convert ELF to Intel HEX format and report code size.
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	$(SIZE) --mcu=$(MCU) --format=avr $<

# ---------------------------------------------------------------------------
# Housekeeping
# ---------------------------------------------------------------------------

# Remove all generated build artefacts.
# Does not touch source files or documentation.
clean:
	rm -f $(OBJ) $(ELF) $(HEX)

# Explicit size target for quick inspection.
size: $(ELF)
	$(SIZE) --mcu=$(MCU) --format=avr $(ELF)
