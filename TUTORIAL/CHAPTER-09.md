# Chapter 9: ARM Arithmetic and Logic Instructions

## Introduction

Arithmetic and logic instructions form the computational core of any processor.  On the Cortex-M33, these instructions operate exclusively on registers — never directly on memory — consistent with the load-store philosophy.  Our blink driver uses arithmetic for delay timing and logic for bit manipulation of hardware registers.  This chapter examines every arithmetic and logic instruction that appears in our firmware.

## Arithmetic Instructions

### `add` — Addition

```asm
  add   r3, r0, r1                               // r3 = r0 + r1
```

Adds two values.  In our firmware, `add` calculates addresses from a base register plus an offset:

```asm
  ldr   r3, =PADS_BANK0_BASE                     // r3 = 0x40038000
  add   r3, r3, r0                               // r3 = base + pad offset
```

This is how gpio.s reaches the pad control register for GPIO16.

### `sub` and `subs` — Subtraction

```asm
  sub   r0, r0, #1                               // r0 = r0 - 1
  subs  r5, r5, #1                               // r5 = r5 - 1, update flags
```

The `s` suffix is critical.  Without it, the APSR flags remain unchanged.  With it, the processor updates N, Z, C, and V flags.  Our delay loop in delay.s depends on `subs` to set the Zero flag:

```asm
.Delay_Loop:
  subs  r5, r5, #1                               // decrement, set Z when 0
  bne   .Delay_Loop                              // loop until Z = 1
```

### `mul` — Multiplication

```asm
  mul   r5, r0, r4                               // r5 = r0 * r4
```

In delay.s, `mul` computes the total number of loop iterations:

```asm
  mov   r4, #3600                                // loops per millisecond
  mul   r5, r0, r4                               // total = ms * 3600
```

At 14.5 MHz XOSC clock, 3600 inner iterations approximate one millisecond.

### `cmp` — Compare

```asm
  cmp   r0, #0                                   // set flags for r0 - 0
```

`cmp` subtracts the second operand from the first, discards the result, and updates all four APSR flags.  delay.s uses this to validate the input:

```asm
  cmp   r0, #0                                   // is delay <= 0?
  ble   .Delay_Done                              // if so, skip
```

## Logic Instructions

### `orr` — Bitwise OR

```asm
  orr   r5, r5, #(1<<6)                          // set bit 6
```

`orr` sets specific bits while preserving all others.  This is the standard idiom for setting a single bit in a hardware register.  In gpio.s, it enables input:

```asm
  orr   r5, r5, #(1<<6)                          // set IE (input enable)
```

In xosc.s, it enables the peripheral clock:

```asm
  orr   r1, r1, #(1<<11)                         // set ENABLE bit
```

### `bic` — Bit Clear

```asm
  bic   r5, r5, #(1<<7)                          // clear bit 7
```

`bic` (Bit Clear) performs `Rd = Rn AND NOT imm`.  It clears specific bits.  In gpio.s, it disables output disable and isolation:

```asm
  bic   r5, r5, #(1<<7)                          // clear OD (output disable)
  bic   r5, r5, #(1<<8)                          // clear ISO (isolation)
```

And clears the FUNCSEL field before setting the new value:

```asm
  bic   r5, r5, #0x1f                            // clear FUNCSEL [4:0]
  orr   r5, r5, #0x05                            // set FUNCSEL = 5 (SIO)
```

### `tst` — Test Bits

```asm
  tst   r1, #(1<<31)                             // test bit 31
```

`tst` performs `Rn AND imm`, discards the result, and updates flags.  If the tested bit is zero, Z=1.  Our XOSC initialization polls the STABLE bit:

```asm
.Wait_XOSC:
  ldr   r1, [r0]                                 // read XOSC_STATUS
  tst   r1, #(1<<31)                             // STABLE bit set?
  beq   .Wait_XOSC                              // no — keep waiting
```

## The APSR Flags

All flag-setting instructions modify the Application Program Status Register (APSR):

| Flag | Name | Set When |
|------|------|----------|
| N | Negative | Result bit 31 is 1 |
| Z | Zero | Result is zero |
| C | Carry | Unsigned overflow or borrow |
| V | oVerflow | Signed overflow |

Conditional branches test these flags:

| Branch | Condition | Flags |
|--------|-----------|-------|
| `beq` | Equal / zero | Z=1 |
| `bne` | Not equal / non-zero | Z=0 |
| `ble` | Less than or equal (signed) | Z=1 or N!=V |
| `bge` | Greater or equal (signed) | N=V |

## Read-Modify-Write Pattern

Hardware register manipulation follows a consistent three-step pattern throughout our firmware:

```asm
  ldr   r5, [r3]                                 // READ current value
  bic   r5, r5, #(1<<7)                          // MODIFY: clear bit
  orr   r5, r5, #(1<<6)                          // MODIFY: set bit
  str   r5, [r3]                                 // WRITE back
```

This pattern appears in gpio.s (pad configuration, control register), xosc.s (clock enable), and coprocessor.s (CPACR modification).  It preserves all bits we do not intend to change.

## Summary

- Arithmetic instructions (`add`, `sub`, `subs`, `mul`, `cmp`) handle addressing, delay loops, and input validation.
- Logic instructions (`orr`, `bic`, `tst`) manipulate individual bits in hardware registers.
- The `s` suffix on `subs` sets APSR flags, enabling conditional branches.
- The read-modify-write pattern (ldr → bic/orr → str) is the fundamental hardware register manipulation idiom.
