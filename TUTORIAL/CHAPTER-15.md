# Chapter 15: Sections, Memory Layout, and the Linker Script

## Introduction

Assembly source files organize code and data into **sections**.  The linker script maps those sections to physical memory regions — flash and RAM.  Together, sections and the linker script determine where every byte of our blink firmware lives at runtime.  This chapter explains how our linker.ld places code in flash, reserves the stack in RAM, and enforces alignment and size constraints.

## What Are Sections?

A section is a named block of content in an object file.  The assembler places each instruction or data item into the current section:

```asm
.section .text, "ax"                             // code goes here
  push  {r4-r12, lr}                             // this instruction is in .text

.section .rodata, "a"                            // read-only data goes here
```

Standard sections used in our firmware:

| Section | Content | Flags | Location |
|---------|---------|-------|----------|
| `.picobin_block` | Boot image metadata | alloc | Flash start |
| `.vectors` | Vector table | alloc | Flash (after picobin) |
| `.text` | Executable code | alloc, exec | Flash |
| `.rodata` | Read-only constants | alloc | Flash |
| `.data` | Initialized variables | alloc, write | RAM (copied from flash) |
| `.bss` | Uninitialized variables | alloc, write, nobits | RAM |
| `.stack` | Stack space | — | RAM (top of SRAM) |

## Our Linker Script

The linker script (linker.ld) defines memory regions and section placement:

### Memory Regions

```ld
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x10000000, LENGTH = 32M
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}
```

| Region | Start Address | Size | Permissions |
|--------|--------------|------|-------------|
| FLASH | `0x10000000` | 32 MB | Read, execute |
| RAM | `0x20000000` | 512 KB | Read, write, execute |

### Section Placement

The `SECTIONS` block maps each section to a memory region:

```ld
SECTIONS
{
  .embedded_block : {
    KEEP(*(.picobin_block))
  } > FLASH

  .vectors : ALIGN(128) {
    *(.vectors)
  } > FLASH

  ASSERT(SIZEOF(.vectors) <= 4096, "Vector table too large")

  .text : {
    *(.text*)
    *(.rodata*)
  } > FLASH

  .stack (NOLOAD) : {
    . = ALIGN(8);
    . = . + STACK_SIZE;
  } > RAM
}
```

## Section-by-Section Walkthrough

### `.embedded_block` — PICOBIN Metadata

```ld
  .embedded_block : {
    KEEP(*(.picobin_block))
  } > FLASH
```

The `KEEP` directive prevents the linker from discarding this section during garbage collection.  The PICOBIN block must be at the start of flash for the RP2350 boot ROM to recognize the image.

### `.vectors` — Vector Table

```ld
  .vectors : ALIGN(128) {
    *(.vectors)
  } > FLASH
```

`ALIGN(128)` ensures the vector table starts on a 128-byte boundary.  The Cortex-M33 requires the vector table to be aligned to at least the number of implemented vectors rounded up to the nearest power of two.

The `ASSERT` checks that the vector table does not exceed 4096 bytes — a safety constraint.

### `.text` and `.rodata` — Code and Constants

```ld
  .text : {
    *(.text*)
    *(.rodata*)
  } > FLASH
```

All executable code and read-only data are placed in flash.  The wildcard `*(.text*)` matches `.text`, `.text.main`, `.text.GPIO_Config`, etc.

### `.stack` — Stack Reservation

```ld
  STACK_SIZE = 32K;

  .stack (NOLOAD) : {
    . = ALIGN(8);
    . = . + STACK_SIZE;
  } > RAM
```

`NOLOAD` means this section occupies address space but is not loaded from flash — the stack is runtime-only.  The 32 KB reservation provides ample space for our firmware's modest call depth.

## The Final Memory Map

After linking, the firmware occupies flash starting at `0x10000000`:

```
Flash (0x10000000):
+----------------------------+
| .picobin_block (metadata)  |  ~20 bytes
+----------------------------+
| .vectors (SP, Reset_Handler)| 8 bytes (2 words)
+----------------------------+
| .text (all code)           |  ~1 KB
| .rodata (constants)        |
+----------------------------+

RAM (0x20000000):
+----------------------------+
| .data / .bss               |  (unused in our firmware)
+----------------------------+
| .stack (32 KB)             |
+----------------------------+
|        ...                 |
| STACK_TOP = 0x20082000     |
+----------------------------+
```

## `ENTRY` Directive

```ld
ENTRY(Reset_Handler)
```

This tells the linker which symbol is the program entry point.  Debuggers and flash tools use this information.  On Cortex-M, actual execution starts from the reset vector in the vector table, but `ENTRY` provides metadata for the toolchain.

## Symbol Exports from the Linker Script

The linker script can define symbols accessible from assembly:

```ld
  _stack_top = ORIGIN(RAM) + LENGTH(RAM);
```

While our firmware uses `.equ` constants in constants.s instead, linker-defined symbols are common in larger projects for separating configuration from code.

## Why Sections Matter

Without sections, the linker would not know:

- Which bytes are executable code (flash) vs. data (RAM)
- What must appear first (PICOBIN block)
- What alignment is required (vector table)
- What is runtime-only and should not consume flash (stack)

The section system lets ten separate assembly files contribute code to a single, correctly-ordered binary.

## Summary

- Sections organize code and data into named blocks with specific attributes.
- The linker script maps sections to physical memory regions (flash and RAM).
- `.embedded_block` (PICOBIN) must be first in flash for the boot ROM.
- `.vectors` is aligned to 128 bytes for hardware requirements.
- `.text` contains all executable code, placed in flash.
- `.stack` reserves 32 KB in RAM with `NOLOAD` (no flash content).
- The `ENTRY` directive identifies the program entry point for tools.
