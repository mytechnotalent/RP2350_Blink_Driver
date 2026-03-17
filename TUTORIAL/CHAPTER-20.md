# Chapter 20: The Build System

## Introduction

Our build system transforms human-readable assembly source files into a UF2 firmware image that the RP2350 can execute.  The process has three stages: assemble, link, and convert.  This chapter walks through every command in build.bat and clean.bat, explaining what each tool does, what flags control, and how the output of one stage feeds the next.

## The Build Pipeline

```
Source Files (.s)
      |
      v
  [arm-none-eabi-as]     Stage 1: Assemble
      |
      v
Object Files (.o)
      |
      v
  [arm-none-eabi-ld]     Stage 2: Link
      |
      v
  blink.elf
      |
      v
  [arm-none-eabi-objcopy] Stage 3: Extract binary
      |
      v
  blink.bin
      |
      v
  [python uf2conv.py]     Stage 4: Convert to UF2
      |
      v
  blink.uf2
```

## Stage 1: Assembly

Each source file is assembled independently:

```bat
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb vector_table.s -o vector_table.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb reset_handler.s -o reset_handler.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb stack.s -o stack.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb xosc.s -o xosc.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb reset.s -o reset.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb coprocessor.s -o coprocessor.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb gpio.s -o gpio.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb delay.s -o delay.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb main.s -o main.o
arm-none-eabi-as -g -mcpu=cortex-m33 -mthumb image_def.s -o image_def.o
```

### Assembler Flags

| Flag | Meaning |
|------|---------|
| `-g` | Generate debug information (DWARF) |
| `-mcpu=cortex-m33` | Target the Cortex-M33 processor |
| `-mthumb` | Generate Thumb instructions |

Each `.s` file produces a corresponding `.o` (object) file containing:

- Machine code with unresolved relocations
- A symbol table (local and global symbols)
- Section headers (.text, .data, .bss, etc.)
- Debug information (source line mappings)

### Ten Source Files

| Source File | Purpose |
|-------------|---------|
| vector_table.s | Vector table (stack pointer + reset vector) |
| reset_handler.s | Boot sequence calling all init functions |
| stack.s | Stack pointer initialization |
| xosc.s | Crystal oscillator init and clock enable |
| reset.s | Peripheral reset release |
| coprocessor.s | CP0 (SIO) access enable |
| gpio.s | GPIO pad config, set, clear functions |
| delay.s | Millisecond delay function |
| main.s | Application entry point and blink loop |
| image_def.s | PICOBIN boot metadata |

## Stage 2: Linking

```bat
arm-none-eabi-ld -g -T linker.ld ^
  vector_table.o reset_handler.o stack.o xosc.o reset.o ^
  coprocessor.o gpio.o delay.o main.o image_def.o ^
  -o blink.elf
```

### Linker Flags

| Flag | Meaning |
|------|---------|
| `-g` | Preserve debug information |
| `-T linker.ld` | Use our linker script for memory layout |
| `-o blink.elf` | Output filename |

The linker:

1. Reads all ten object files
2. Merges matching sections (all `.text` sections combine into one)
3. Assigns absolute addresses according to linker.ld
4. Resolves all symbol references (e.g., `bl GPIO_Set` gets patched with the correct offset)
5. Produces an ELF (Executable and Linkable Format) file

### ELF File Contents

The ELF file contains:

- All machine code at final addresses
- Section headers describing memory layout
- Symbol table with resolved addresses
- Debug information mapping code to source lines

## Stage 3: Binary Extraction

```bat
arm-none-eabi-objcopy -O binary blink.elf blink.bin
```

`objcopy` strips all ELF metadata and produces a raw binary — the exact bytes that will be written to flash starting at `0x10000000`.  This file contains no headers, no symbol table, no debug info — just machine code and data.

## Stage 4: UF2 Conversion

```bat
python uf2conv.py -b 0x10000000 -f 0xe48bff59 -o blink.uf2 blink.bin
```

### UF2 Flags

| Flag | Meaning |
|------|---------|
| `-b 0x10000000` | Base address (flash start) |
| `-f 0xe48bff59` | RP2350 family ID |
| `-o blink.uf2` | Output filename |

UF2 (USB Flashing Format) wraps the binary in a format that the RP2350's USB bootloader understands.  Each 512-byte UF2 block contains:

- Magic numbers for identification
- Target address for that block's data
- Up to 256 bytes of payload
- Family ID to prevent flashing the wrong chip

## Error Handling

Every command in build.bat is followed by:

```bat
if errorlevel 1 goto error
```

If any stage fails (syntax error, undefined symbol, missing file), the build stops immediately with an error message.  This prevents cascading failures where a later stage operates on corrupt or missing input.

## Flashing the Firmware

build.bat prints instructions for two flashing methods:

### UF2 (USB Mass Storage)

1. Hold BOOTSEL button on the Pico 2
2. Connect USB cable
3. Copy blink.uf2 to the RP2350 drive

### OpenOCD (Debug Probe)

```bat
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg ^
  -c "adapter speed 5000" ^
  -c "program blink.elf verify reset exit"
```

This uses a CMSIS-DAP debug probe to program the ELF file directly, verify the contents, reset the chip, and exit.

## The Clean Script

```bat
@echo off
echo Cleaning...
del *.o *.elf *.bin *.uf2 2>nul
echo Clean complete.
```

clean.bat removes all build artifacts:

| Pattern | Files Removed |
|---------|---------------|
| `*.o` | All object files |
| `*.elf` | The linked ELF binary |
| `*.bin` | The raw binary |
| `*.uf2` | The UF2 firmware image |

The `2>nul` suppresses error messages if files do not exist.  After cleaning, only source files remain.

## Summary

- The build pipeline has four stages: assemble → link → extract binary → convert to UF2.
- `arm-none-eabi-as` assembles each source file into an object file with `-mcpu=cortex-m33 -mthumb`.
- `arm-none-eabi-ld` links all object files using linker.ld to produce blink.elf.
- `arm-none-eabi-objcopy` strips the ELF to a raw binary.
- `uf2conv.py` wraps the binary in UF2 format with the RP2350 family ID.
- Error checking after each stage prevents cascading build failures.
- clean.bat removes all generated files, leaving only source code.
