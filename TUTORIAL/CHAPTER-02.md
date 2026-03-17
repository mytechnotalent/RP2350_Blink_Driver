# Chapter 2: Number Systems — Binary, Hexadecimal, and Decimal

## Introduction

Every value in our firmware — addresses, register contents, bit masks, constants — is stored as a pattern of ones and zeros.  This chapter teaches the three number systems you will encounter on every page of this tutorial: decimal, binary, and hexadecimal.  You will learn to convert between them, understand bit numbering, and recognize the specific constants used in our blink driver.

## Decimal — Base 10

Decimal is the number system humans use every day.  It has ten digits: 0 through 9.  Each position represents a power of 10:

```
  4   2   3
  |   |   |
  |   |   +-- 3 × 10^0 = 3
  |   +------ 2 × 10^1 = 20
  +---------- 4 × 10^2 = 400
                         ----
                          423
```

Decimal appears in our firmware for delay values, GPIO pin numbers, and loop counts.

## Binary — Base 2

Binary has two digits: 0 and 1.  Each digit is called a **bit**.  Each position represents a power of 2:

```
  1   1   0   1
  |   |   |   |
  |   |   |   +-- 1 × 2^0 = 1
  |   |   +------ 0 × 2^1 = 0
  |   +---------- 1 × 2^2 = 4
  +-------------- 1 × 2^3 = 8
                             --
                             13 (decimal)
```

The processor operates entirely in binary.  Every register is 32 bits wide — 32 individual ones and zeros.

In GNU assembly, binary literals use the `0b` prefix:

```asm
  .hword 0b0001000000100001                      // SECURE mode (0x1021)
```

## Hexadecimal — Base 16

Hexadecimal (hex) has sixteen digits: 0–9 and A–F (where A=10, B=11, C=12, D=13, E=14, F=15).  Each hex digit represents exactly four bits:

| Hex | Binary | Decimal |
|-----|--------|---------|
| 0 | 0000 | 0 |
| 1 | 0001 | 1 |
| 2 | 0010 | 2 |
| 3 | 0011 | 3 |
| 4 | 0100 | 4 |
| 5 | 0101 | 5 |
| 6 | 0110 | 6 |
| 7 | 0111 | 7 |
| 8 | 1000 | 8 |
| 9 | 1001 | 9 |
| A | 1010 | 10 |
| B | 1011 | 11 |
| C | 1100 | 12 |
| D | 1101 | 13 |
| E | 1110 | 14 |
| F | 1111 | 15 |

This makes hex the preferred notation for memory addresses and register values because each hex digit maps directly to four bits.

## The 0x Prefix

In assembly and C, hexadecimal numbers are written with a `0x` prefix:

```
0x40028000 = 0100 0000 0000 0010 1000 0000 0000 0000 (binary)
```

Every memory-mapped address in our firmware is written in hex:

```asm
  .equ XOSC_BASE,  0x40048000                    // crystal oscillator base
  .equ RESETS_BASE, 0x40020000                    // reset controller base
```

## Bit Numbering

Bits in a 32-bit register are numbered 0 (least significant, rightmost) to 31 (most significant, leftmost):

```
Bit:  31 30 29 28 ... 3  2  1  0
       |  |  |  |      |  |  |  |
MSB ---+  |  |  |      |  |  |  +--- LSB
          |  |  |      |  |  |
          v  v  v      v  v  v
```

When we write `#(1<<6)`, we mean a 32-bit value with only bit 6 set:

```
0000 0000 0000 0000 0000 0000 0100 0000 = 0x00000040
```

This notation appears throughout our firmware for setting and clearing individual hardware control bits.

## Common Bit Patterns in Our Firmware

| Pattern | Hex | Binary (relevant bits) | Used For |
|---------|-----|----------------------|----------|
| `1<<6` | 0x00000040 | bit 6 set | IO_BANK0 reset bit |
| `1<<7` | 0x00000080 | bit 7 set | OD (output disable) pad bit |
| `1<<8` | 0x00000100 | bit 8 set | ISO (isolation) pad bit |
| `1<<11` | 0x00000800 | bit 11 set | CLK_PERI enable bit |
| `1<<31` | 0x80000000 | bit 31 set | XOSC STABLE status bit |
| `0x1f` | 0x0000001F | bits 4:0 set | FUNCSEL mask (5 bits) |
| `0x05` | 0x00000005 | bits 2,0 set | FUNCSEL = SIO (GPIO function) |

## Two's Complement — Signed Numbers

ARM Cortex-M33 uses **two's complement** for signed integers.  In a 32-bit register:

- Bit 31 is the sign bit: 0 = positive, 1 = negative.
- Positive numbers are represented normally.
- Negative numbers are represented by inverting all bits and adding 1.

| Decimal | 32-bit Hex | Binary (MSB shown) |
|---------|-----------|-------------------|
| +1 | 0x00000001 | 0000...0001 |
| 0 | 0x00000000 | 0000...0000 |
| -1 | 0xFFFFFFFF | 1111...1111 |
| -2 | 0xFFFFFFFE | 1111...1110 |

In our blink driver we use only unsigned values, but understanding two's complement is essential for interpreting condition codes and branch instructions.

## Data Sizes on ARM Cortex-M33

| Name | Size | ARM Instruction Suffix |
|------|------|----------------------|
| Byte | 8 bits | `b` (ldrb, strb) |
| Halfword | 16 bits | `h` (ldrh, strh) |
| Word | 32 bits | (default: ldr, str) |

Registers are 32 bits wide.  Our firmware works exclusively with word-sized (32-bit) loads and stores because all peripheral registers are 32 bits wide.

## Summary

- Binary (base 2) is the native language of the processor — every register is 32 bits.
- Hexadecimal (base 16) condenses binary into a readable format — each hex digit is 4 bits.
- The `0x` prefix marks hexadecimal; `0b` marks binary.
- Bits are numbered 0 (LSB) to 31 (MSB).
- The shift expression `1<<N` creates a mask with only bit N set.
- ARM Cortex-M33 uses two's complement for signed arithmetic.
