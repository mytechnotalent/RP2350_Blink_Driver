# Chapter 10: ARM Memory Access Instructions

## Introduction

The Cortex-M33 is a load-store architecture: arithmetic and logic operate only on registers.  To interact with the outside world — RAM, peripheral registers, the stack — the processor must explicitly load from and store to memory.  This chapter explores every memory-access instruction and addressing mode used in our blink driver.

## `ldr` — Load Register

`ldr` reads a 32-bit word from memory into a register:

```asm
  ldr   r5, [r3]                                 // r5 = memory[r3]
```

The source address is held in a base register.  Several addressing modes exist:

### Immediate Offset

```asm
  ldr   r1, [r0, #4]                             // r1 = memory[r0 + 4]
```

The offset is added to the base register.  The base register itself is not modified.  In our firmware, we typically compute the full address first and use zero offset:

```asm
  add   r3, r3, r0                               // r3 = PADS_BANK0_BASE + offset
  ldr   r5, [r3]                                 // read pad register
```

### Zero Offset

```asm
  ldr   r1, [r0]                                 // r1 = memory[r0]
```

This is the most common form in our firmware.  xosc.s, reset.s, gpio.s, and coprocessor.s all load peripheral registers this way.

### PC-Relative (Literal Pool)

```asm
  ldr   r0, =0x40048000                          // assembled as ldr r0, [pc, #offset]
```

The assembler generates a PC-relative load from the literal pool.  This is how every `ldr Rd, =constant` works internally.

## `str` — Store Register

`str` writes a 32-bit word from a register to memory:

```asm
  str   r5, [r3]                                 // memory[r3] = r5
```

Store completes the read-modify-write cycle.  After modifying bits with `orr`/`bic`, the result is written back:

```asm
  ldr   r5, [r3]                                 // read
  bic   r5, r5, #(1<<7)                          // modify
  str   r5, [r3]                                 // write back
```

Every hardware register configuration in our firmware follows this pattern.

## `push` and `pop` — Stack Operations

`push` and `pop` are aliases for store-multiple and load-multiple using the stack pointer (SP):

```asm
  push  {r4-r12, lr}                             // save registers to stack
  pop   {r4-r12, lr}                             // restore registers from stack
```

### How `push` Works

`push {r4-r12, lr}` performs:

```
SP = SP - (10 * 4)        // decrement by 40 bytes (10 registers)
memory[SP + 0]  = r4
memory[SP + 4]  = r5
...
memory[SP + 36] = lr
```

The stack grows downward (toward lower addresses).

### How `pop` Works

`pop {r4-r12, lr}` performs:

```
r4  = memory[SP + 0]
r5  = memory[SP + 4]
...
lr  = memory[SP + 36]
SP  = SP + 40              // restore stack pointer
```

### Register Lists

Our firmware uses several different register lists:

| Function | Push/Pop Registers | Count |
|----------|--------------------|-------|
| `main` | `{r4-r12, lr}` | 10 registers |
| `GPIO_Config` | `{r0-r12, lr}` | 13 registers |
| `GPIO_Set` | `{r0-r12, lr}` | 13 registers |
| `GPIO_Clear` | `{r0-r12, lr}` | 13 registers |
| `Delay_MS` | `{r4-r12, lr}` | 10 registers |

Functions that receive parameters in r0–r3 and need to preserve them for the caller push the full range r0–r12.

## Memory Map and Peripheral Access

Our firmware accesses these memory regions:

```
+------------------------------+
| 0x10000000  Flash (code)     |
+------------------------------+
| 0x20000000  SRAM (data)      |
+------------------------------+
| 0x40010000  CLOCKS           |
| 0x40020000  RESETS           |
| 0x40028000  IO_BANK0         |
| 0x40038000  PADS_BANK0       |
| 0x40048000  XOSC             |
+------------------------------+
| 0xE0000000  PPB (system)     |
+------------------------------+
```

Every peripheral access follows the same steps:

1. `ldr` the base address into a register
2. Optionally `add` an offset
3. `ldr` to read the current value
4. `bic`/`orr` to modify bits
5. `str` to write back

## Alignment Requirements

The Cortex-M33 requires word-aligned access for `ldr` and `str`:

| Access | Alignment | Size |
|--------|-----------|------|
| `ldr` / `str` (word) | 4-byte aligned | 32 bits |
| `ldrh` / `strh` (halfword) | 2-byte aligned | 16 bits |
| `ldrb` / `strb` (byte) | No alignment | 8 bits |

All peripheral registers in our firmware are at 4-byte-aligned addresses, so word access is safe.  Unaligned word access on Cortex-M33 can be configured to fault or succeed with a performance penalty.

## `msr` and `mrs` — Special Register Access

The Cortex-M33 has special-purpose registers accessed via `msr` and `mrs`:

```asm
  msr   MSP, r0                                  // set Main Stack Pointer
  msr   PSP, r0                                  // set Process Stack Pointer
  msr   MSPLIM, r0                               // set MSP limit
  msr   PSPLIM, r0                               // set PSP limit
```

These are used in stack.s to initialize the stack pointers.  Unlike `ldr`/`str`, these instructions move data between core registers and the special register bank.

## Summary

- `ldr` and `str` are the only instructions that access memory, supporting immediate offset and PC-relative addressing.
- `push` and `pop` manage the call stack, saving and restoring registers across function calls.
- Peripheral registers are accessed at fixed addresses in the `0x4xxx_xxxx` range using the read-modify-write pattern.
- `msr`/`mrs` access special-purpose registers like the stack pointer and its limits.
