# Chapter 23: vector_table.s and stack.s — Boot Foundation

## Introduction

When the Cortex-M33 comes out of reset, the very first thing it does is read two 32-bit words from the vector table: the initial stack pointer and the address of the reset handler.  These two values — defined in vector_table.s — determine where the stack lives and where execution begins.  stack.s then refines the stack configuration with limit registers that catch overflow.  Together, these two files form the boot foundation of our firmware.

## vector_table.s — Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

.section .vectors, "ax"                          // vector table section
.align 2                                         // align to 4-byte boundary

.global _vectors                                 // export _vectors symbol
_vectors:
  .word STACK_TOP                                // initial stack pointer
  .word Reset_Handler + 1                        // reset handler (Thumb bit set)
```

### Section and Alignment

```asm
.section .vectors, "ax"                          // vector table section
.align 2                                         // align to 4-byte boundary
```

The vector table goes in its own section (`.vectors`) so the linker can place it precisely.  The `"ax"` flags mean allocatable and executable.  `.align 2` ensures 4-byte alignment (2^2 = 4), required because the hardware reads these as 32-bit words.

The linker script aligns this section to 128 bytes and verifies it is within the first 4 KB of flash:

```ld
.vectors ALIGN(128) : { KEEP(*(.vectors)) } > FLASH :text
ASSERT(((ADDR(.vectors) - ORIGIN(FLASH)) < 0x1000), ...)
```

### Vector Table Entries

```asm
_vectors:
  .word STACK_TOP                                // initial stack pointer
  .word Reset_Handler + 1                        // reset handler (Thumb bit set)
```

| Index | Content | Value | Purpose |
|-------|---------|-------|---------|
| 0 | Initial SP | `0x20082000` | Loaded into MSP on reset |
| 1 | Reset Handler | `Reset_Handler + 1` | First instruction to execute |

**Entry 0 — Initial Stack Pointer:** The hardware loads this value into the Main Stack Pointer (MSP) before executing any instruction.  This means the stack is functional from the very first instruction of Reset_Handler.

**Entry 1 — Reset Vector:** The hardware loads this address into the Program Counter (PC).  The `+ 1` sets bit 0, which tells the processor to execute in Thumb mode.  Without the Thumb bit, a HardFault would occur immediately.

### Why Only Two Entries?

A full Cortex-M33 vector table can have up to 496 entries (16 system exceptions + 480 external interrupts).  Our firmware uses no interrupts, so we only need the two mandatory entries.  The hardware reads the rest of the table only when an interrupt occurs.

## stack.s — Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

.global Init_Stack
.type Init_Stack, %function
Init_Stack:
  ldr   r0, =STACK_TOP                           // load stack top
  msr   PSP, r0                                  // set PSP
  ldr   r0, =STACK_LIMIT                         // load stack limit
  msr   MSPLIM, r0                               // set MSP limit
  msr   PSPLIM, r0                               // set PSP limit
  ldr   r0, =STACK_TOP                           // reload stack top
  msr   MSP, r0                                  // set MSP
  bx    lr                                       // return
```

### Line-by-Line Walkthrough

**Set Process Stack Pointer:**

```asm
  ldr   r0, =STACK_TOP                           // r0 = 0x20082000
  msr   PSP, r0                                  // PSP = 0x20082000
```

The Process Stack Pointer (PSP) is used when running in Thread mode with the CONTROL register configured for process stack.  Our firmware uses MSP, but setting PSP ensures a clean state.

**Set Stack Limits:**

```asm
  ldr   r0, =STACK_LIMIT                         // r0 = 0x2007a000
  msr   MSPLIM, r0                               // MSPLIM = 0x2007a000
  msr   PSPLIM, r0                               // PSPLIM = 0x2007a000
```

Stack limit registers are an ARMv8-M feature.  If SP ever drops below the limit, the processor generates a UsageFault.  This prevents the stack from silently overwriting data in lower SRAM.

The limit `0x2007a000` is 32 KB below `STACK_TOP` (`0x20082000`), giving us a 32 KB stack.

**Set Main Stack Pointer:**

```asm
  ldr   r0, =STACK_TOP                           // r0 = 0x20082000
  msr   MSP, r0                                  // MSP = 0x20082000
```

Although the hardware already loaded MSP from the vector table, we set it again to ensure a known state after any debug probe manipulation or warm reset.

**Return:**

```asm
  bx    lr                                       // return to Reset_Handler
```

Init_Stack is a leaf function — it calls no other functions, so LR is preserved.

### Stack Layout

```
RAM:
+----------------------------+ 0x20000000
| (available SRAM)           |
+----------------------------+
|                            |
| ...                        |
|                            |
+----------------------------+ 0x2007a000  ← STACK_LIMIT
| Stack (32 KB)              |
|   grows downward ↓         |
|                            |
+----------------------------+ 0x20082000  ← STACK_TOP (MSP/PSP initial)
```

## The Boot Sequence

```
Power On / Reset
  |
  v
Hardware reads _vectors[0] → loads MSP = 0x20082000
Hardware reads _vectors[1] → loads PC = Reset_Handler
  |
  v
Reset_Handler executes
  |
  +→ bl Init_Stack
  |    Sets PSP, MSPLIM, PSPLIM, MSP
  |    Returns via bx lr
  v
  (continues initialization...)
```

## Summary

- vector_table.s defines the two mandatory vector table entries: initial stack pointer and reset vector.
- The `+ 1` on Reset_Handler sets the Thumb bit, required for Cortex-M execution.
- The linker aligns the vector table to 128 bytes within the first 4 KB of flash.
- stack.s initializes PSP, MSP, MSPLIM, and PSPLIM for a 32 KB stack at the top of SRAM.
- Stack limit registers (ARMv8-M feature) catch stack overflow with a UsageFault.
- Together, these files ensure the processor has a valid stack from the very first instruction.
