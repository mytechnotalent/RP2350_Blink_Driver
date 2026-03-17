# Chapter 18: RP2350 Hardware Architecture

## Introduction

The RP2350 is a dual-core microcontroller from Raspberry Pi, capable of running ARM Cortex-M33 or RISC-V Hazard3 cores.  Our blink driver targets the ARM Cortex-M33 core and uses a subset of the chip's peripherals: the crystal oscillator (XOSC), the clock system, the reset controller, the GPIO pads and IO bank, and the SIO coprocessor.  This chapter maps the hardware our firmware touches and explains the initialization sequence that brings the chip from reset to a blinking LED.

## RP2350 Block Diagram

```
+-----------------------------------------------------+
|                      RP2350                          |
|                                                      |
|  +-------------+  +-------------+                    |
|  | Cortex-M33  |  | Cortex-M33  |                    |
|  |   Core 0    |  |   Core 1    |                    |
|  +------+------+  +------+------+                    |
|         |                |                           |
|         +-------+--------+                           |
|                 |                                     |
|          +------+------+                              |
|          |   Bus Fabric |                             |
|          +------+------+                              |
|                 |                                     |
|    +----+----+--+--+----+----+----+                   |
|    |    |    |     |    |    |    |                   |
|  FLASH SRAM XOSC CLOCKS RESETS GPIO SIO              |
|  32MB  512K                                          |
+-----------------------------------------------------+
```

Our firmware operates entirely on Core 0.  Core 1 remains in its default reset state.

## Memory Map

The RP2350 uses a unified memory map where peripherals are accessed at fixed addresses:

| Address Range | Peripheral | Our Constant |
|---------------|-----------|-------------|
| `0x10000000` – `0x11FFFFFF` | External Flash | — |
| `0x20000000` – `0x2007FFFF` | SRAM (512 KB) | STACK_TOP, STACK_LIMIT |
| `0x40010000` | Clocks Controller | CLOCKS_BASE |
| `0x40020000` | Reset Controller | RESETS_BASE |
| `0x40028000` | IO Bank 0 | IO_BANK0_BASE |
| `0x40038000` | Pads Bank 0 | PADS_BANK0_BASE |
| `0x40048000` | Crystal Oscillator | XOSC_BASE |
| `0xE0000000` | Private Peripheral Bus | PPB_BASE |

## Crystal Oscillator (XOSC)

The RP2350 Pico 2 board has a 12 MHz crystal.  The XOSC peripheral drives this crystal and provides a stable clock reference.

### XOSC Registers

| Register | Offset | Purpose |
|----------|--------|---------|
| XOSC_CTRL | +0x00 | Control: enable and frequency range |
| XOSC_STATUS | +0x04 | Status: STABLE bit (bit 31) |
| XOSC_STARTUP | +0x0C | Startup delay count |

### Initialization Sequence

1. Write startup delay (0x00C4 = 196 cycles × 256 = ~4 ms at ring oscillator speed)
2. Write control word (0x00FABAA0 = ENABLE + frequency range 1–15 MHz)
3. Poll STATUS bit 31 until STABLE = 1

## Clock System

The clock system distributes clock signals to all peripherals.  Our firmware configures only the peripheral clock (CLK_PERI):

### CLK_PERI_CTRL Register

| Bit | Field | Value | Meaning |
|-----|-------|-------|---------|
| 11 | ENABLE | 1 | Enable the peripheral clock |
| 7:5 | AUXSRC | 4 | Select XOSC as clock source |

After this configuration, all peripherals (including GPIO) run from the 12 MHz XOSC.  The processor core runs at approximately 14.5 MHz from the on-chip ring oscillator (the default after reset), which is why our delay calibration uses 3600 loops per millisecond.

## Reset Controller

The reset controller holds peripherals in reset until software explicitly releases them.  This saves power and prevents bus conflicts during boot.

### Reset Registers

| Register | Address | Purpose |
|----------|---------|---------|
| RESETS_RESET | BASE + 0x00 | Reset control (1 = held in reset) |
| RESETS_RESET_CLEAR | BASE + 0x3000 | Atomic clear (write 1 to release) |
| RESETS_RESET_DONE | BASE + 0x08 | Status (1 = reset complete) |

### Our Firmware's Reset Sequence

We release IO_BANK0 from reset (bit 6):

```
1. Read RESETS_RESET
2. Clear bit 6 (release IO_BANK0)
3. Write back
4. Poll RESETS_RESET_DONE bit 6 until set
```

This enables the GPIO control registers.  Without this step, any access to IO_BANK0 would read as zero.

## GPIO Architecture

The RP2350 has 48 GPIO pins.  Each pin has two control points:

### Pads Bank 0

The pad register controls the electrical characteristics of the physical pin:

```
+----------------------------------+
| Bit 8: ISO (Isolation)           |
| Bit 7: OD  (Output Disable)     |
| Bit 6: IE  (Input Enable)       |
| Bit 5-4: DRIVE (strength)       |
| Bit 3: PUE (Pull-Up Enable)     |
| Bit 2: PDE (Pull-Down Enable)   |
| Bit 1: SCHMITT (trigger)        |
| Bit 0: SLEWFAST                 |
+----------------------------------+
```

For output, we clear OD (allow output), set IE (enable input — required for readback), and clear ISO (remove isolation).

### IO Bank 0

The IO control register selects which internal peripheral drives the pin:

```
Bits [4:0]: FUNCSEL — Function Select
  0 = SPI
  1 = UART
  2 = I2C
  3 = PWM
  4 = SIO (Single-cycle IO) ← reserved for PIO
  5 = SIO ← our selection
  ...
  31 = NULL (disabled)
```

We set FUNCSEL = 5 to connect GPIO16 to the SIO block.

### SIO (Single-Cycle IO) via Coprocessor

Once FUNCSEL routes the pin to SIO, we control it through coprocessor 0:

```
mcrr p0, #4, Rn, Rv, c4   → Output Enable (Rv: 1=enable, 0=disable)
mcrr p0, #4, Rn, Rv, c0   → Output Value  (Rv: 1=high, 0=low)
```

Where Rn holds the GPIO number and Rv holds the value.  This provides fast, deterministic GPIO access.

## GPIO16 and the LED

On the Raspberry Pi Pico 2, GPIO16 is not connected to the on-board LED (which is on GPIO25).  For our blink driver, we wire an external LED with a current-limiting resistor to GPIO16:

```
GPIO16 (Pin 21) ---[330 ohm]---[LED]--- GND (Pin 23)
```

When GPIO16 is driven high (3.3V), current flows through the resistor and LED to ground, illuminating the LED.  When driven low (0V), no current flows and the LED is off.

## Boot Sequence

After power-on, the RP2350 executes the following sequence:

```
1. Boot ROM runs (internal to chip)
2. Boot ROM finds PICOBIN block in flash
3. Boot ROM validates image and jumps to Reset_Handler
4. Reset_Handler: Init_Stack
5. Reset_Handler: Init_XOSC (crystal oscillator)
6. Reset_Handler: Enable_XOSC_Peri_Clock (peripheral clock)
7. Reset_Handler: Init_Subsystem (release GPIO from reset)
8. Reset_Handler: Enable_Coprocessor (allow CP0 access)
9. Reset_Handler: branch to main
10. main: configure GPIO16, enter blink loop
```

Each step builds on the previous one — the clock must be stable before configuring peripherals, peripherals must be out of reset before accessing their registers, and the coprocessor must be enabled before using `mcrr`.

## Summary

- The RP2350 uses memory-mapped peripherals at fixed addresses in the `0x4xxx_xxxx` range.
- XOSC provides a stable 12 MHz clock from an external crystal.
- The clock system routes XOSC to peripherals via CLK_PERI_CTRL.
- The reset controller must release peripherals before they can be used.
- GPIO pins are configured through two register banks: pads (electrical) and IO (function select).
- The SIO coprocessor (CP0) provides fast GPIO set/clear via `mcrr` instructions.
- The boot sequence proceeds: boot ROM → Reset_Handler → init peripherals → main → blink loop.
