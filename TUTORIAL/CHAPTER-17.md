# Chapter 17: Bit Manipulation Patterns

## Introduction

Bare-metal firmware is fundamentally about controlling hardware, and hardware is controlled by setting, clearing, and testing individual bits in memory-mapped registers.  Every peripheral on the RP2350 exposes its functionality through bit fields — groups of bits with specific meanings.  This chapter catalogs the bit manipulation patterns used throughout our blink driver and builds your fluency in reading and writing them.

## The Fundamental Operations

There are three primitive bit operations:

| Operation | Instruction | Pattern | Effect |
|-----------|-------------|---------|--------|
| Set bit | `orr` | `orr Rd, Rd, #(1<<N)` | Force bit N to 1 |
| Clear bit | `bic` | `bic Rd, Rd, #(1<<N)` | Force bit N to 0 |
| Test bit | `tst` | `tst Rn, #(1<<N)` | Set Z flag if bit N is 0 |

## Set a Single Bit

To set bit N without affecting other bits, OR with a mask that has only bit N set:

```asm
  orr   r5, r5, #(1<<6)                          // set bit 6 (IE)
```

Truth table for OR:

```
  Original bit:  0  1  0  1
  Mask bit:      1  1  0  0
  Result:        1  1  0  1
```

Bits where the mask is 0 are unchanged.  Bits where the mask is 1 are forced to 1.

### In Our Firmware

gpio.s sets the Input Enable bit in the pad register:

```asm
  orr   r5, r5, #(1<<6)                          // set IE (input enable)
```

xosc.s enables the peripheral clock:

```asm
  orr   r1, r1, #(1<<11)                         // set ENABLE bit
```

coprocessor.s enables CP0 access:

```asm
  orr   r1, r1, #(1<<1)                          // set CP0 access bit 1
  orr   r1, r1, #(1<<0)                          // set CP0 access bit 0
```

## Clear a Single Bit

To clear bit N without affecting other bits, BIC (bit clear) with a mask:

```asm
  bic   r5, r5, #(1<<7)                          // clear bit 7 (OD)
```

BIC performs `Rd = Rn AND NOT mask`:

```
  Original bit:  0  1  0  1
  NOT Mask:      0  0  1  1    (mask was 1  1  0  0)
  Result:        0  0  0  1
```

Bits where the mask is 0 are unchanged.  Bits where the mask is 1 are forced to 0.

### In Our Firmware

gpio.s clears Output Disable and Isolation in the pad register:

```asm
  bic   r5, r5, #(1<<7)                          // clear OD (output disable)
  bic   r5, r5, #(1<<8)                          // clear ISO (isolation)
```

## Clear a Multi-Bit Field

To clear several contiguous bits (a "field"), use a wider mask:

```asm
  bic   r5, r5, #0x1f                            // clear bits [4:0]
```

The mask `0x1f` = `0b00011111` clears the lowest 5 bits.  This is used in gpio.s to clear the FUNCSEL field before writing a new value:

```asm
  bic   r5, r5, #0x1f                            // clear FUNCSEL [4:0]
  orr   r5, r5, #0x05                            // set FUNCSEL = 5 (SIO)
```

This two-step pattern — clear then set — is standard for writing a multi-bit field without affecting surrounding bits.

## Test a Bit

To check whether a bit is set without modifying any register:

```asm
  tst   r1, #(1<<31)                             // test STABLE bit
  beq   .Wait_XOSC                              // branch if bit is 0
```

`tst` performs `AND` and discards the result, updating only the flags:

- If bit 31 is 0: result is 0, Z=1, `beq` branches
- If bit 31 is 1: result is non-zero, Z=0, `beq` falls through

### Polling Loops

Both hardware polling loops use `tst` + `beq`:

**XOSC Stabilization:**

```asm
.Wait_XOSC:
  ldr   r1, [r0]                                 // read XOSC_STATUS
  tst   r1, #(1<<31)                             // STABLE bit?
  beq   .Wait_XOSC                              // not yet — keep polling
```

**Reset Completion:**

```asm
.Wait_Reset:
  ldr   r1, [r0]                                 // read RESET_DONE
  tst   r1, #(1<<6)                              // IO_BANK0 done?
  beq   .Wait_Reset                             // not yet — keep polling
```

## Combined Patterns

### Read-Modify-Write (Single Bit)

```asm
  ldr   r1, [r0]                                 // READ
  orr   r1, r1, #(1<<11)                         // MODIFY: set bit
  str   r1, [r0]                                 // WRITE
```

### Read-Modify-Write (Multiple Bits)

```asm
  ldr   r5, [r3]                                 // READ
  bic   r5, r5, #(1<<7)                          // MODIFY: clear OD
  orr   r5, r5, #(1<<6)                          // MODIFY: set IE
  bic   r5, r5, #(1<<8)                          // MODIFY: clear ISO
  str   r5, [r3]                                 // WRITE
```

### Clear Field Then Set Value

```asm
  ldr   r5, [r6]                                 // READ
  bic   r5, r5, #0x1f                            // clear field [4:0]
  orr   r5, r5, #0x05                            // set value 5
  str   r5, [r6]                                 // WRITE
```

## Bit Fields in Our Registers

### PADS_BANK0 Pad Register (GPIO16)

```
Bit 8: ISO  (Isolation)         — clear to 0
Bit 7: OD   (Output Disable)    — clear to 0
Bit 6: IE   (Input Enable)      — set to 1
Bit 5: DRIVE[1]                  — unchanged
Bit 4: DRIVE[0]                  — unchanged
Bit 3: PUE  (Pull-Up Enable)    — unchanged
Bit 2: PDE  (Pull-Down Enable)  — unchanged
Bit 1: SCHMITT                   — unchanged
Bit 0: SLEWFAST                  — unchanged
```

### IO_BANK0 Control Register (GPIO16)

```
Bits [4:0]: FUNCSEL — cleared to 0, then set to 5 (SIO)
```

### CPACR Coprocessor Access

```
Bits [1:0]: CP0 access — set to 0b11 (full access)
```

### CLK_PERI_CTRL

```
Bit 11:    ENABLE   — set to 1
Bits [7:5]: AUXSRC  — set to 4 (XOSC)
```

## Why This Matters

Understanding bit manipulation is the single most important skill for bare-metal programming.  Every peripheral on the RP2350 — GPIO, UART, SPI, I2C, timers, DMA — is controlled by reading and writing bit fields in memory-mapped registers.  The patterns in this chapter (`orr` to set, `bic` to clear, `tst` to test, clear-then-set for fields) apply to every register on the chip.

## Summary

- `orr Rd, Rd, #(1<<N)` sets bit N: the fundamental enable operation.
- `bic Rd, Rd, #(1<<N)` clears bit N: the fundamental disable operation.
- `tst Rn, #(1<<N)` tests bit N: used in all polling loops.
- Multi-bit fields are written with a clear-then-set pattern: `bic` with the field mask, then `orr` with the value.
- The read-modify-write pattern (ldr → modify → str) preserves bits we do not intend to change.
- These three operations form the complete vocabulary for hardware register control.
