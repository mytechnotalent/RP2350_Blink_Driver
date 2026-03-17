# Chapter 26: reset.s — Releasing Peripherals from Reset

## Introduction

After power-on, most peripherals on the RP2350 are held in reset to save power and prevent bus conflicts.  Before our firmware can configure GPIO registers, it must explicitly release IO_BANK0 from reset and wait for the release to complete.  reset.s contains a single function, `Init_Subsystem`, that does exactly this.

## Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

.global Init_Subsystem
.type Init_Subsystem, %function
Init_Subsystem:
.GPIO_Subsystem_Reset:
  ldr   r0, =RESETS_RESET                        // load RESETS->RESET address
  ldr   r1, [r0]                                 // read RESETS->RESET value
  bic   r1, r1, #(1<<6)                          // clear IO_BANK0 bit
  str   r1, [r0]                                 // store value into RESETS->RESET address
.GPIO_Subsystem_Reset_Wait:
  ldr   r0, =RESETS_RESET_DONE                   // load RESETS->RESET_DONE address
  ldr   r1, [r0]                                 // read RESETS->RESET_DONE value
  tst   r1, #(1<<6)                              // test IO_BANK0 reset done
  beq   .GPIO_Subsystem_Reset_Wait               // wait until done
  bx    lr                                       // return
```

## Line-by-Line Walkthrough

### Release IO_BANK0 from Reset

```asm
  ldr   r0, =RESETS_RESET                        // r0 = 0x40020000
  ldr   r1, [r0]                                 // r1 = current reset register value
  bic   r1, r1, #(1<<6)                          // clear bit 6 (IO_BANK0)
  str   r1, [r0]                                 // write back
```

The RESETS_RESET register has one bit per peripheral.  When a bit is 1, that peripheral is held in reset (inactive).  When cleared to 0, the peripheral begins its reset release sequence.

Bit 6 corresponds to IO_BANK0, which controls the GPIO function select and control registers.

The read-modify-write pattern (`ldr` → `bic` → `str`) preserves all other bits.  Only IO_BANK0 is released; all other peripherals remain in their current state.

### Wait for Reset Completion

```asm
  ldr   r0, =RESETS_RESET_DONE                   // r0 = 0x40020008
  ldr   r1, [r0]                                 // r1 = current done status
  tst   r1, #(1<<6)                              // test IO_BANK0 done bit
  beq   .GPIO_Subsystem_Reset_Wait               // loop if not done
  bx    lr                                       // done — return
```

Clearing the reset bit starts the release process, but it is not instantaneous.  The peripheral needs clock cycles to initialize its internal state.  The RESETS_RESET_DONE register reports completion: bit 6 goes high when IO_BANK0 is fully operational.

The polling pattern is identical to the XOSC stabilization loop:

```
  +-->  ldr r1, [r0]       read RESET_DONE
  |     tst r1, #(1<<6)    check bit 6
  |     beq --------+
  |                 |
  +-----------------+      (loop while not done)
        |
        v
  bx    lr                 (done — return)
```

## Reset Register Bit Map

The RESETS_RESET register controls many peripherals.  Bit 6 is the only one we use:

| Bit | Peripheral | Our Usage |
|-----|-----------|-----------|
| 0 | ADC | Not used |
| 1 | BUSCTRL | Not used |
| 2 | DMA | Not used |
| 3 | HSTX | Not used |
| 4 | I2C0 | Not used |
| 5 | I2C1 | Not used |
| **6** | **IO_BANK0** | **Released** |
| 7 | IO_QSPI | Not used |
| 8 | JTAG | Not used |
| 9 | PADS_BANK0 | Not used |
| 10 | PADS_QSPI | Not used |
| ... | ... | ... |

## Why the Wait Is Necessary

Without polling RESET_DONE, the firmware might immediately try to read GPIO registers before IO_BANK0 has completed initialization.  The result would be:

- Reads return zero (registers not yet initialized)
- Writes are silently ignored
- GPIO configuration appears to succeed but has no effect

The polling loop guarantees that IO_BANK0 is fully functional before any GPIO register access.

## Function Characteristics

`Init_Subsystem` is a leaf function:

- **No function calls** — does not use `bl`
- **No stack frame** — does not push/pop registers
- **Uses only r0, r1** — both are scratch registers (caller-saved)
- **Returns via `bx lr`** — LR is preserved

## Local Labels

The function uses two local labels:

| Label | Purpose |
|-------|---------|
| `.GPIO_Subsystem_Reset` | Marks the reset release code |
| `.GPIO_Subsystem_Reset_Wait` | Polling loop target |

These are file-local (prefixed with `.`) and serve as documentation anchors.

## Summary

- `Init_Subsystem` releases IO_BANK0 from reset by clearing bit 6 in RESETS_RESET.
- It then polls RESETS_RESET_DONE bit 6 until the release is complete.
- This must happen before any GPIO register access — otherwise reads return zero.
- The read-modify-write pattern preserves other peripherals' reset state.
- This is a leaf function using only scratch registers, requiring no stack frame.
