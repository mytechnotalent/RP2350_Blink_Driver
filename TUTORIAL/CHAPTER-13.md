# Chapter 13: Assembler Directives

## Introduction

Assembler directives are commands to the assembler itself — they do not produce machine instructions.  Instead, they control how the assembler organizes code, allocates data, defines symbols, and structures the output object file.  Our blink driver relies on directives to set up sections, export symbols, align data, and reserve space.  This chapter catalogs every directive used in our firmware and explains its purpose.

## Syntax and Instruction Set Directives

### `.syntax unified`

```asm
.syntax unified                                  // use unified ARM/Thumb syntax
```

Every source file begins with this directive.  It tells the assembler to accept the unified instruction syntax where both 16-bit and 32-bit Thumb-2 encodings are selected automatically.  Without it, the assembler would expect the older divided syntax.

### `.thumb`

```asm
.thumb                                           // generate Thumb instructions
```

Specifies that all following code should be assembled as Thumb (16/32-bit) instructions.  The Cortex-M33 only executes Thumb code, so this directive is mandatory.

## Section Directives

### `.section`

```asm
.section .text, "ax"                             // executable code section
.section .rodata, "a"                            // read-only data section
.section .data, "aw"                             // read-write data section
.section .bss, "aw", %nobits                     // uninitialized data section
```

`.section` places subsequent code or data into a named section.  The flags indicate:

| Flag | Meaning |
|------|---------|
| `a` | Allocatable (occupies memory) |
| `x` | Executable |
| `w` | Writable |
| `%nobits` | No initial content (BSS) |

Our firmware uses several sections:

| Section | Purpose | Used In |
|---------|---------|---------|
| `.text` | Executable code | All .s files |
| `.rodata` | Read-only constants | main.s |
| `.data` | Initialized variables | main.s |
| `.bss` | Uninitialized variables | main.s |
| `.picobin_block` | Boot image metadata | image_def.s |
| `.vectors` | Interrupt vector table | vector_table.s |

### `.align`

```asm
.align 2                                         // align to 4-byte boundary
```

`.align N` pads with zeros until the current address is a multiple of 2^N bytes.  `.align 2` means 4-byte alignment (2^2 = 4).

In vector_table.s:

```asm
.section .vectors, "a"                           // vector table section
.align 2                                         // 4-byte aligned
```

The vector table must be aligned because the Cortex-M33 hardware reads it as 32-bit words.

## Symbol Directives

### `.global`

```asm
.global GPIO_Config                              // export symbol
.global GPIO_Set                                 // export symbol
.global GPIO_Clear                               // export symbol
```

`.global` makes a symbol visible to the linker.  Without it, the symbol exists only within that assembly file.  Every function called from another file must be exported:

| Symbol | Defined In | Called From |
|--------|-----------|------------|
| `Reset_Handler` | reset_handler.s | vector_table.s |
| `Init_Stack` | stack.s | reset_handler.s |
| `Init_XOSC` | xosc.s | reset_handler.s |
| `Enable_XOSC_Peri_Clock` | xosc.s | reset_handler.s |
| `Init_Subsystem` | reset.s | reset_handler.s |
| `Enable_Coprocessor` | coprocessor.s | reset_handler.s |
| `main` | main.s | reset_handler.s |
| `GPIO_Config` | gpio.s | main.s |
| `GPIO_Set` | gpio.s | main.s |
| `GPIO_Clear` | gpio.s | main.s |
| `Delay_MS` | delay.s | main.s |

### `.extern`

```asm
.extern GPIO_Config                              // import external symbol
```

`.extern` declares that a symbol is defined in another file.  While technically optional (the assembler assumes unknown symbols are external), it documents intent and aids readability.

### `.equ`

```asm
.equ STACK_TOP, 0x20082000                       // define constant
.equ XOSC_BASE, 0x40048000                       // define constant
```

`.equ` defines a named constant.  It produces no code or data — the symbol is replaced by its value wherever it appears.  Our constants.s file is entirely `.equ` directives:

```asm
.equ IO_BANK0_BASE, 0x40028000                   // GPIO control base
.equ PADS_BANK0_BASE, 0x40038000                 // GPIO pad base
.equ IO_BANK0_GPIO16_CTRL_OFFSET, 0x84           // GPIO16 control offset
.equ PADS_BANK0_GPIO16_OFFSET, 0x44              // GPIO16 pad offset
```

## Data Directives

### `.word`

```asm
.word STACK_TOP                                  // emit 32-bit value
.word Reset_Handler + 1                          // emit address with Thumb bit
```

`.word` emits a 32-bit value into the current section.  The vector table uses `.word` to place the initial stack pointer and the reset handler address:

```asm
_vectors:
  .word STACK_TOP                                // initial SP value
  .word Reset_Handler + 1                        // reset vector (Thumb)
```

The `+ 1` sets the Thumb bit, telling the processor to execute in Thumb mode.

### `.byte`

```asm
.byte 0x42                                       // emit 8-bit value
```

Used in image_def.s for byte-level fields in the PICOBIN block structure.

### `.hword`

```asm
.hword 0x2101                                    // emit 16-bit value
```

Used in image_def.s for halfword (16-bit) fields.

## Function Directives

### `.type` and `.thumb_func`

```asm
.type Reset_Handler, %function                   // mark as function symbol
.thumb_func                                      // set Thumb bit in symbol address
```

`.type` tells the linker that the symbol is a function (not data).  `.thumb_func` ensures the symbol's address has bit 0 set, which is required for correct branching on Cortex-M.

### `.size`

```asm
.size Reset_Handler, .-Reset_Handler             // record function size
```

`.size` records the function's byte size in the symbol table.  The expression `.-Reset_Handler` calculates the distance from the current position to the function's start.

## Include Directive

### `.include`

```asm
.include "constants.s"                           // include shared constants
```

`.include` textually inserts another file at the current position.  Every source file includes constants.s to access shared `.equ` definitions.

## Summary

- `.syntax unified` and `.thumb` establish the instruction set context.
- `.section` organizes code and data into named regions for the linker.
- `.global` and `.extern` control symbol visibility across files.
- `.equ` defines named constants without emitting code.
- `.word`, `.byte`, and `.hword` emit raw data values.
- `.type`, `.thumb_func`, and `.size` provide metadata for the linker and debugger.
- `.include` shares constants across all source files.
