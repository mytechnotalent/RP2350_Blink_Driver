# Chapter 25: xosc.s — Crystal Oscillator and Clock Configuration

## Introduction

The RP2350 starts running from an internal ring oscillator at approximately 14.5 MHz.  For accurate timing and stable peripheral operation, our firmware activates the external 12 MHz crystal oscillator (XOSC) and routes it to the peripheral clock.  xosc.s contains two functions: `Init_XOSC` configures and waits for the crystal, and `Enable_XOSC_Peri_Clock` connects it to the peripheral clock domain.

## Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

.global Init_XOSC
.type Init_XOSC, %function
Init_XOSC:
  ldr   r0, =XOSC_STARTUP                        // load XOSC_STARTUP address
  ldr   r1, =0x00c4                              // set delay 50,000 cycles
  str   r1, [r0]                                 // store value into XOSC_STARTUP
  ldr   r0, =XOSC_CTRL                           // load XOSC_CTRL address
  ldr   r1, =0x00FABAA0                          // set 1_15MHz, freq range, actual 14.5MHz
  str   r1, [r0]                                 // store value into XOSC_CTRL
.Init_XOSC_Wait:
  ldr   r0, =XOSC_STATUS                         // load XOSC_STATUS address
  ldr   r1, [r0]                                 // read XOSC_STATUS value
  tst   r1, #(1<<31)                             // test STABLE bit
  beq   .Init_XOSC_Wait                          // wait until stable bit is set
  bx    lr                                       // return

.global Enable_XOSC_Peri_Clock
.type Enable_XOSC_Peri_Clock, %function
Enable_XOSC_Peri_Clock:
  ldr   r0, =CLK_PERI_CTRL                       // load CLK_PERI_CTRL address
  ldr   r1, [r0]                                 // read CLK_PERI_CTRL value
  orr   r1, r1, #(1<<11)                         // set ENABLE bit
  orr   r1, r1, #(4<<5)                          // set AUXSRC: XOSC_CLKSRC bit
  str   r1, [r0]                                 // store value into CLK_PERI_CTRL
  bx    lr                                       // return
```

## Init_XOSC — Line-by-Line

### Configure Startup Delay

```asm
  ldr   r0, =XOSC_STARTUP                        // r0 = 0x4004800C
  ldr   r1, =0x00c4                              // r1 = 196
  str   r1, [r0]                                 // XOSC_STARTUP = 196
```

The XOSC_STARTUP register sets the number of clock cycles the oscillator must run before being declared stable.  The actual delay is:

```
delay = value × 256 = 196 × 256 = 50,176 cycles
```

At the ring oscillator's ~14.5 MHz, this is approximately 3.5 ms — enough time for a 12 MHz crystal to stabilize.

### Enable the Oscillator

```asm
  ldr   r0, =XOSC_CTRL                           // r0 = 0x40048000
  ldr   r1, =0x00FABAA0                          // r1 = control word
  str   r1, [r0]                                 // XOSC_CTRL = 0x00FABAA0
```

The control word `0x00FABAA0` encodes two fields:

| Field | Bits | Value | Meaning |
|-------|------|-------|---------|
| ENABLE | [23:12] | `0xFAB` | Enable the oscillator |
| FREQ_RANGE | [11:0] | `0xAA0` | 1–15 MHz range |

The ENABLE field uses a magic value (`0xFAB`) rather than a simple bit — this is a safety mechanism to prevent accidental modification.

### Wait for Stability

```asm
.Init_XOSC_Wait:
  ldr   r0, =XOSC_STATUS                         // r0 = 0x40048004
  ldr   r1, [r0]                                 // r1 = current status
  tst   r1, #(1<<31)                             // test bit 31 (STABLE)
  beq   .Init_XOSC_Wait                          // Z=1 means not stable
  bx    lr                                       // stable — return
```

This is a polling loop.  `tst` ANDs the status register with the mask `(1<<31)`.  If bit 31 is zero (not stable), the result is zero, Z=1, and `beq` loops back.  Once the crystal stabilizes, bit 31 goes high, the `tst` result is non-zero, Z=0, and `beq` falls through.

The polling loop flow:

```
  +-->  ldr r1, [r0]       read status
  |     tst r1, #(1<<31)   check STABLE
  |     beq --------+
  |                 |
  +-----------------+      (loop while not stable)
        |
        v
  bx    lr                 (stable — return)
```

## Enable_XOSC_Peri_Clock — Line-by-Line

```asm
  ldr   r0, =CLK_PERI_CTRL                       // r0 = 0x40010048
  ldr   r1, [r0]                                 // read current value
  orr   r1, r1, #(1<<11)                         // set ENABLE bit (bit 11)
  orr   r1, r1, #(4<<5)                          // set AUXSRC = 4 (bits [7:5])
  str   r1, [r0]                                 // write back
  bx    lr                                       // return
```

### CLK_PERI_CTRL Register

| Bit(s) | Field | Value Set | Meaning |
|--------|-------|-----------|---------|
| 11 | ENABLE | 1 | Enable the peripheral clock |
| [7:5] | AUXSRC | 4 (`100`) | Select XOSC as clock source |

The `orr r1, r1, #(4<<5)` expression shifts 4 left by 5 positions, producing `0x80` (`0b10000000`).  This sets bits [7:5] to `100` binary = 4, selecting XOSC_CLKSRC.

Available AUXSRC values:

| Value | Source |
|-------|--------|
| 0 | CLK_SYS |
| 1 | PLL_SYS |
| 2 | PLL_USB |
| 3 | ROSC_CLKSRC_PH |
| 4 | XOSC_CLKSRC |
| 5 | GPIN0 |
| 6 | GPIN1 |

We select 4 (XOSC) because we just configured and stabilized the crystal oscillator.

## Both Functions Are Leaf Functions

Neither `Init_XOSC` nor `Enable_XOSC_Peri_Clock` calls another function.  They do not push/pop registers or save LR.  They use only r0 and r1, which are caller-saved scratch registers per the AAPCS — no preservation needed.

## Clock Domain After Configuration

```
               +--------+
  12 MHz       |        |    CLK_PERI
  Crystal ---->| XOSC   |--------------> Peripherals
               |        |                (GPIO, etc.)
               +--------+
                    |
                    +---> STABLE bit 31

  ~14.5 MHz    +--------+
  Ring Osc --->| ROSC   |--------------> CPU core
               +--------+                (default after reset)
```

The CPU core continues running from the ring oscillator.  The peripheral clock now runs from XOSC.  Our delay.s calibration (3600 loops/ms) is based on the ~14.5 MHz ring oscillator speed that the CPU runs at.

## Summary

- `Init_XOSC` configures the startup delay, enables the crystal oscillator, and polls until stable.
- The XOSC_CTRL enable field uses a magic value (`0xFAB`) as a safety mechanism.
- `Enable_XOSC_Peri_Clock` routes the XOSC output to the peripheral clock domain.
- Both are leaf functions using only scratch registers — no stack frame needed.
- After these functions complete, peripherals have a stable 12 MHz clock from the external crystal.
