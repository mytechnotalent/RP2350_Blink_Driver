# Chapter 22: constants.s — Memory Addresses and Constants

## Introduction

Every peripheral on the RP2350 is accessed at a fixed memory address.  Rather than scattering magic numbers throughout our source files, we centralize all addresses and constants in constants.s using `.equ` directives.  Every other source file includes this file, creating a single source of truth for the entire firmware.  This chapter examines every constant and explains what it references in hardware.

## Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

/**
 * Memory addresses and constants.
 */
.equ STACK_TOP,                   0x20082000
.equ STACK_LIMIT,                 0x2007a000
.equ XOSC_BASE,                   0x40048000
.equ XOSC_CTRL,                   XOSC_BASE + 0x00
.equ XOSC_STATUS,                 XOSC_BASE + 0x04
.equ XOSC_STARTUP,                XOSC_BASE + 0x0c
.equ PPB_BASE,                    0xe0000000
.equ CPACR,                       PPB_BASE + 0x0ed88
.equ CLOCKS_BASE,                 0x40010000
.equ CLK_PERI_CTRL,               CLOCKS_BASE + 0x48
.equ RESETS_BASE,                 0x40020000
.equ RESETS_RESET,                RESETS_BASE + 0x0
.equ RESETS_RESET_CLEAR,          RESETS_BASE + 0x3000
.equ RESETS_RESET_DONE,           RESETS_BASE + 0x8
.equ IO_BANK0_BASE,               0x40028000
.equ IO_BANK0_GPIO16_CTRL_OFFSET, 0x84
.equ PADS_BANK0_BASE,             0x40038000
.equ PADS_BANK0_GPIO16_OFFSET,    0x44
```

## Preamble

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set
```

Every source file that `.include`s constants.s inherits these directives.  They establish the instruction set context for the assembler.

## Stack Constants

```asm
.equ STACK_TOP,                   0x20082000
.equ STACK_LIMIT,                 0x2007a000
```

| Constant | Value | Description |
|----------|-------|-------------|
| `STACK_TOP` | `0x20082000` | Top of stack (highest address) |
| `STACK_LIMIT` | `0x2007a000` | Bottom of stack (32 KB below top) |

SRAM on the RP2350 extends from `0x20000000` to `0x20082000`.  The stack occupies the top 32 KB.  The limit register triggers a fault if SP drops below this value, preventing silent stack overflow.

## Crystal Oscillator Constants

```asm
.equ XOSC_BASE,                   0x40048000
.equ XOSC_CTRL,                   XOSC_BASE + 0x00
.equ XOSC_STATUS,                 XOSC_BASE + 0x04
.equ XOSC_STARTUP,                XOSC_BASE + 0x0c
```

| Constant | Address | Description |
|----------|---------|-------------|
| `XOSC_BASE` | `0x40048000` | XOSC peripheral base |
| `XOSC_CTRL` | `0x40048000` | Control register |
| `XOSC_STATUS` | `0x40048004` | Status register (bit 31 = STABLE) |
| `XOSC_STARTUP` | `0x4004800C` | Startup delay register |

These addresses are defined in the RP2350 datasheet.  The assembler computes derived addresses at assembly time (e.g., `XOSC_BASE + 0x04` = `0x40048004`).

## System Registers

```asm
.equ PPB_BASE,                    0xe0000000
.equ CPACR,                       PPB_BASE + 0x0ed88
```

| Constant | Address | Description |
|----------|---------|-------------|
| `PPB_BASE` | `0xE0000000` | Private Peripheral Bus base |
| `CPACR` | `0xE000ED88` | Coprocessor Access Control Register |

The PPB region contains ARM-defined system registers.  CPACR controls which coprocessors the firmware can access.  Our firmware sets bits [1:0] to enable CP0 (SIO).

## Clock Constants

```asm
.equ CLOCKS_BASE,                 0x40010000
.equ CLK_PERI_CTRL,               CLOCKS_BASE + 0x48
```

| Constant | Address | Description |
|----------|---------|-------------|
| `CLOCKS_BASE` | `0x40010000` | Clocks controller base |
| `CLK_PERI_CTRL` | `0x40010048` | Peripheral clock control |

CLK_PERI_CTRL selects the clock source for peripherals and enables the clock output.

## Reset Controller Constants

```asm
.equ RESETS_BASE,                 0x40020000
.equ RESETS_RESET,                RESETS_BASE + 0x0
.equ RESETS_RESET_CLEAR,          RESETS_BASE + 0x3000
.equ RESETS_RESET_DONE,           RESETS_BASE + 0x8
```

| Constant | Address | Description |
|----------|---------|-------------|
| `RESETS_BASE` | `0x40020000` | Reset controller base |
| `RESETS_RESET` | `0x40020000` | Reset control register |
| `RESETS_RESET_CLEAR` | `0x40023000` | Atomic clear alias |
| `RESETS_RESET_DONE` | `0x40020008` | Reset done status |

Bit 6 in these registers corresponds to IO_BANK0 (GPIO controller).  Clearing bit 6 in RESETS_RESET releases GPIO from reset.

## GPIO Constants

```asm
.equ IO_BANK0_BASE,               0x40028000
.equ IO_BANK0_GPIO16_CTRL_OFFSET, 0x84
.equ PADS_BANK0_BASE,             0x40038000
.equ PADS_BANK0_GPIO16_OFFSET,    0x44
```

| Constant | Value | Description |
|----------|-------|-------------|
| `IO_BANK0_BASE` | `0x40028000` | IO control base |
| `IO_BANK0_GPIO16_CTRL_OFFSET` | `0x84` | GPIO16 control register offset |
| `PADS_BANK0_BASE` | `0x40038000` | Pad control base |
| `PADS_BANK0_GPIO16_OFFSET` | `0x44` | GPIO16 pad register offset |

The control register offset `0x84` comes from the formula: each GPIO has an 8-byte register pair (status + ctrl), and GPIO16 is at offset `16 * 8 + 4 = 0x84` (the +4 selects the ctrl register).

The pad offset `0x44` comes from: each GPIO has a 4-byte pad register, starting at offset 4 (GPIO0 is at 0x04), so GPIO16 is at `4 + 16 * 4 = 0x44`.

## How `.include` Works

Every source file begins with:

```asm
.include "constants.s"
```

The assembler textually inserts the entire contents of constants.s at that point.  This means every file has access to all constants with no runtime cost — `.equ` definitions produce no code.

## Design Principle

Centralizing constants provides three benefits:

1. **Single source of truth** — change an address in one place, all files see the update
2. **Readability** — `ldr r0, =XOSC_BASE` is clearer than `ldr r0, =0x40048000`
3. **Correctness** — derived addresses (base + offset) are computed by the assembler, eliminating manual arithmetic errors

## Summary

- constants.s defines 19 `.equ` constants covering stack, XOSC, system, clock, reset, and GPIO addresses.
- All addresses come directly from the RP2350 datasheet.
- Derived addresses use assembler arithmetic (e.g., `XOSC_BASE + 0x04`).
- `.include "constants.s"` in every source file provides universal access with zero runtime cost.
- GPIO16 offsets are computed from the GPIO number using the register layout formulas in the datasheet.
