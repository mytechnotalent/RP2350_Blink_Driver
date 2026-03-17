# Chapter 14: Labels, Symbols, and the Symbol Table

## Introduction

Labels give names to memory addresses.  Every function entry point, every loop target, and every data item in our blink driver is identified by a label.  The assembler records these labels in a symbol table, which the linker uses to resolve cross-file references and compute branch targets.  This chapter examines how labels and the symbol table work in practice.

## Defining Labels

A label is any identifier followed by a colon:

```asm
main:                                            // function entry point
  push  {r4-r12, lr}                             // save registers
```

The assembler records the current address as the value of the label `main`.  When another instruction references `main`, the assembler or linker substitutes this address.

## Global vs. Local Labels

### Global Labels

Global labels are visible across all object files after linking.  They must be declared with `.global`:

```asm
.global main                                     // visible to linker
main:                                            // address recorded globally
```

Every function called from another file must be global.  Our firmware has these global symbols:

```
Reset_Handler    main
Init_Stack       GPIO_Config       GPIO_Set
Init_XOSC        GPIO_Clear        Delay_MS
Enable_XOSC_Peri_Clock
Init_Subsystem   Enable_Coprocessor
```

### Local Labels

Labels without `.global` are file-local — invisible outside the object file:

```asm
.Loop:                                           // local to this file
  mov   r0, #16
  bl    GPIO_Set
  ...
  b     .Loop                                    // branch within file
```

By convention, our firmware prefixes local labels with a dot (`.Loop`, `.Wait_XOSC`, `.Delay_Loop`).  This makes the distinction visually clear.

## Label Types in Our Firmware

| Label | File | Type | Purpose |
|-------|------|------|---------|
| `main` | main.s | Global | Program entry |
| `.Loop` | main.s | Local | Infinite blink loop |
| `GPIO_Config` | gpio.s | Global | GPIO initialization function |
| `.Wait_XOSC` | xosc.s | Local | XOSC polling loop |
| `.Wait_Reset` | reset.s | Local | Reset polling loop |
| `.Delay_Loop` | delay.s | Local | Delay count loop |
| `.Delay_Done` | delay.s | Local | Early exit target |
| `_vectors` | vector_table.s | Local | Vector table start |

## The Symbol Table

The assembler creates a symbol table in each object file (.o).  Each entry contains:

| Field | Description |
|-------|-------------|
| Name | The label string |
| Value | Address or constant value |
| Section | Which section contains it |
| Binding | Local or global |
| Type | Function, object, or notype |

After assembly, the symbol table for gpio.o would contain:

```
Symbol           Value  Section  Bind    Type
GPIO_Config      0x00   .text    GLOBAL  FUNC
GPIO_Set         0x38   .text    GLOBAL  FUNC
GPIO_Clear       0x48   .text    GLOBAL  FUNC
```

The values are section-relative until the linker assigns final addresses.

## `.equ` Constants in the Symbol Table

`.equ` definitions also create symbol table entries, but with absolute values:

```asm
.equ IO_BANK0_BASE, 0x40028000                   // absolute symbol
```

These symbols have no section — they are pure numeric constants.  The assembler substitutes the value at assembly time wherever the symbol appears.

## Cross-File Resolution

When main.s references `GPIO_Set`:

```asm
  bl    GPIO_Set                                 // reference to external symbol
```

The assembler cannot resolve this because `GPIO_Set` is defined in gpio.s.  It creates a **relocation entry** in main.o:

```
Relocation: offset=0x10, type=R_ARM_THM_CALL, symbol=GPIO_Set
```

The linker reads both main.o and gpio.o, finds `GPIO_Set` in gpio.o's symbol table, computes the final address, and patches the `bl` instruction with the correct offset.

## The Linking Process

```
+----------+       +----------+       +----------+
| main.o   |       | gpio.o   |       | delay.o  |
| (symbols |       | (symbols |       | (symbols |
|  + reloc)|       |  + reloc)|       |  + reloc)|
+----+-----+       +----+-----+       +----+-----+
     |                   |                  |
     +------- LINKER ----+---------+--------+
              |
        +-----+------+
        | blink.elf   |
        | (all symbols|
        |  resolved)  |
        +-------------+
```

The linker:

1. Collects all sections from all object files
2. Assigns final addresses according to the linker script
3. Resolves all relocations using the merged symbol table
4. Produces the final ELF binary

## The Thumb Bit

On Cortex-M, all function addresses must have bit 0 set to indicate Thumb mode.  The assembler and linker handle this automatically for `bl` targets.  In the vector table, we set it explicitly:

```asm
  .word Reset_Handler + 1                        // Thumb bit set manually
```

Hardware reads this vector and uses bit 0 to select Thumb execution.  If bit 0 were clear, a HardFault would occur.

## Summary

- Labels assign names to addresses; they are the foundation of all control flow and data references.
- Global labels (`.global`) are visible across files; local labels are file-private.
- The symbol table maps every label to its address, section, and type.
- `.equ` creates absolute symbols — numeric constants with no section.
- The linker resolves cross-file references by merging symbol tables and patching relocations.
- The Thumb bit (bit 0) must be set on function addresses for correct execution on Cortex-M.
