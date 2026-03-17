# Chapter 8: ARM Immediate and Move Instructions

## Introduction

Before a processor can operate on data it must get that data into a register.  On a RISC machine, transferring constants into registers is one of the most common operations.  This chapter examines how the Cortex-M33 loads immediate values, the constraints the encoding imposes, and the role of the assembler's `ldr Rd, =value` pseudo-instruction that our blink firmware relies on heavily.

## The `mov` Instruction

`mov` copies a value into the destination register:

```asm
  mov   r0, #16                                  // r0 = 16 (GPIO number)
```

Thumb-2 encodes an immediate inside the instruction word itself.  The available range depends on the encoding:

| Encoding | Range | Notes |
|----------|-------|-------|
| 16-bit `mov` | 0–255 | Low registers r0–r7 only |
| 32-bit `mov` | Modified immediate | 8-bit value rotated within 32-bit word |
| 32-bit `movw` | 0–65535 | Full 16-bit immediate |
| `movt` | 0–65535 into upper half | Pairs with `movw` for 32-bit values |

The "modified immediate" encoding can represent values like `0xFF`, `0xFF00`, or `0xFF000000`, but not arbitrary 32-bit constants.

## The `ldr Rd, =value` Pseudo-Instruction

When a constant does not fit any `mov` encoding, the assembler provides a pseudo-instruction:

```asm
  ldr   r0, =PADS_BANK0_GPIO16_OFFSET            // r0 = 0x44
  ldr   r1, =IO_BANK0_GPIO16_CTRL_OFFSET          // r1 = 0x84
```

The assembler handles this by placing the 32-bit constant in a "literal pool" — a data area near the code — and generating a PC-relative `ldr`:

```
Before assembly:
  ldr   r0, =0x40038000                           // pseudo-instruction

After assembly:
  ldr   r0, [pc, #offset]                         // actual instruction
  ...
  .word 0x40038000                                 // literal pool entry
```

This is critical for our firmware because peripheral base addresses like `0x40028000` (IO_BANK0_BASE) and `0x40038000` (PADS_BANK0_BASE) are 32-bit values that cannot fit in any immediate encoding.

## Literal Pool Placement

The literal pool must be within ±4095 bytes of the `ldr` instruction (for 32-bit Thumb-2 encoding).  The assembler typically places it after the next unconditional branch or at the end of the section:

```
+-----------------+
| ldr r0, [pc, #] | -----+
| ldr r1, [pc, #] | ---+ |
| ...             |    | |
| b  .Loop        |    | |
+-----------------+    | |
| .word 0x40038000| <--|-+  (literal pool)
| .word 0x40028000| <--+
+-----------------+
```

## Our Firmware's Use of Immediates

In our blink driver, constants flow into registers in several ways:

1. **`ldr Rd, =constant`** for peripheral addresses — used in every source file:

```asm
  ldr   r0, =XOSC_BASE                           // r0 = 0x40048000
  ldr   r1, =RESETS_BASE                          // r1 = 0x40020000
```

2. **Immediate operands** for bit manipulation — common in gpio.s:

```asm
  bic   r5, r5, #(1<<7)                          // clear OD bit 7
  orr   r5, r5, #(1<<6)                          // set IE bit 6
```

Here `(1<<7)` = 0x80 and `(1<<6)` = 0x40, both fitting in modified immediate encoding.

3. **Small immediates in `mov`** — used in delay.s:

```asm
  mov   r4, #3600                                // loops per millisecond
```

The value 3600 (0xE10) fits in a 16-bit `movw` encoding.

4. **`ldr` of full 32-bit constants** for control register values:

```asm
  ldr   r1, =0x00FAB000                          // XOSC_CTRL value
  orr   r1, r1, #0xAA0                           // form 0x00FABAA0
```

## Why Not Always Use `mov`?

Most of our peripheral addresses are 32-bit values in the `0x4xxx_xxxx` or `0x2xxx_xxxx` ranges.  These cannot be represented by any Thumb-2 modified immediate constant, so `ldr Rd, =value` is mandatory.  The assembler silently handles the literal pool, making the code clean while the binary contains the necessary indirection.

## Summary

- `mov` loads small immediates directly into registers, constrained by encoding limits.
- `ldr Rd, =value` is a pseudo-instruction that uses a literal pool for arbitrary 32-bit constants.
- Our firmware relies on `ldr Rd, =value` extensively for peripheral base addresses.
- Immediate operands in `bic`/`orr`/`tst` use the modified immediate encoding for single-bit and small-mask operations.
