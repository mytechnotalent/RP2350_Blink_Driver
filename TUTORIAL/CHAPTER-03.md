# Chapter 3: Memory — Addresses, Bytes, Words, and Endianness

## Introduction

Every instruction our processor executes and every hardware register it accesses lives at a numeric address in a single, flat address space.  This chapter explains how the RP2350 organizes its 4 GB address space, how data is stored in memory, and why alignment and endianness matter for bare-metal programming.

## The Address Space

The ARM Cortex-M33 uses 32-bit addresses, giving a theoretical range of 0x00000000 to 0xFFFFFFFF — 4 GB.  Most of that space is unoccupied.  The RP2350 maps real hardware into specific regions:

| Address Range | Size | Contents |
|--------------|------|----------|
| 0x00000000–0x0FFFFFFF | 256 MB | Boot ROM, internal ROM |
| 0x10000000–0x11FFFFFF | 32 MB | External flash (XIP) |
| 0x20000000–0x20081FFF | 520 KB | SRAM |
| 0x40000000–0x4FFFFFFF | 256 MB | Peripheral registers (APB/AHB) |
| 0xE0000000–0xE00FFFFF | 1 MB | Private Peripheral Bus (PPB) — NVIC, SysTick, SCB |

Our firmware occupies flash starting at 0x10000000 and uses SRAM from 0x20000000 for the stack.

## Bytes, Halfwords, and Words

| Unit | Size | Example |
|------|------|---------|
| Byte | 8 bits (1 byte) | A single ASCII character |
| Halfword | 16 bits (2 bytes) | A Thumb 16-bit instruction |
| Word | 32 bits (4 bytes) | A register value, a memory address |

The ARM Cortex-M33 is a 32-bit architecture.  All general-purpose registers are one word (32 bits) wide.

## Alignment

A word-sized access must target an address divisible by 4.  A halfword access must target an address divisible by 2.  A byte access can target any address.

| Access Size | Valid Addresses | Invalid Addresses |
|-------------|----------------|-------------------|
| Word (4 bytes) | 0x00, 0x04, 0x08 | 0x01, 0x02, 0x03 |
| Halfword (2 bytes) | 0x00, 0x02, 0x04 | 0x01, 0x03 |
| Byte (1 byte) | Any | None |

An unaligned word access triggers a HardFault exception on Cortex-M33.  All peripheral registers in the RP2350 are word-aligned by design.

## Little-Endian Byte Order

The ARM Cortex-M33 stores multi-byte values in **little-endian** order: the least significant byte occupies the lowest address.

The 32-bit value 0x12345678 stored at address 0x20000000:

```
Address:  0x20000000  0x20000001  0x20000002  0x20000003
Content:     0x78        0x56        0x34        0x12
              LSB                                 MSB
```

This matters when examining memory in a debugger — the bytes appear "reversed" compared to how we write the number.

## Memory-Mapped Registers

The RP2350 exposes hardware control through **memory-mapped registers**.  Reading from or writing to a specific address directly controls hardware:

```asm
  ldr   r0, =XOSC_CTRL                           // load XOSC_CTRL address (0x40048000)
  ldr   r1, =0x00FABAA0                          // value to configure oscillator
  str   r1, [r0]                                 // write value — hardware starts oscillator
```

The `str` instruction does not just store data — it triggers real hardware behavior.  The crystal oscillator physically begins oscillating because we wrote to that address.

## The Stack

The stack is a region of SRAM used for:

- Saving and restoring registers across function calls (`push`/`pop`)
- Storing local variables
- Saving processor state during interrupts

The ARM Cortex-M33 stack grows **downward**: pushing data decreases the stack pointer (SP), popping increases it.

In our firmware:

```asm
  .equ STACK_TOP,   0x20082000                    // top of stack (highest address)
  .equ STACK_LIMIT, 0x2007a000                    // bottom of stack (lowest address)
```

This gives us 32 KB of stack space (0x20082000 − 0x2007A000 = 0x8000 = 32,768 bytes).

## Flash Memory (XIP)

Our firmware is stored in external SPI flash.  The RP2350 maps this flash into the address space starting at 0x10000000 through a mechanism called **Execute-In-Place (XIP)**: the processor fetches instructions directly from flash as if it were regular memory.

The linker script places our code at 0x10000000:

```
FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 32M
```

## SRAM

The RP2350 provides 520 KB of on-chip SRAM starting at 0x20000000.  Our firmware uses this exclusively for the stack.  No global variables are needed for a simple blink driver.

```
RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
```

## Reading the Address Map

When you see an address like 0x40038044 in the firmware:

1. **0x4003xxxx** — falls in the peripheral region, specifically PADS_BANK0 (base 0x40038000).
2. **Offset 0x44** — this is the PADS_BANK0_GPIO16 register (0x40038000 + 0x44).

Learning to decompose addresses into base + offset is essential for understanding every hardware access in our code.

## Summary

- The RP2350 uses a flat 32-bit address space — flash, SRAM, and peripherals all share one map.
- Data sizes are byte (8-bit), halfword (16-bit), and word (32-bit).
- Word accesses must be 4-byte aligned.
- Little-endian: least significant byte at the lowest address.
- Memory-mapped I/O means writing to an address controls real hardware.
- The stack lives in SRAM and grows downward.
- Flash is mapped at 0x10000000 via XIP.
