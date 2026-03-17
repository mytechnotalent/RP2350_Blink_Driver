# Chapter 24: reset_handler.s — The Boot Sequence

## Introduction

After the hardware loads the stack pointer and jumps to the reset vector, `Reset_Handler` takes control.  It is the first code we write that actually executes.  Its job is simple but critical: call each initialization function in the correct order, then hand control to the main application.  This chapter examines the boot sequence and explains why the order matters.

## Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

.global Reset_Handler                            // export Reset_Handler symbol
.type Reset_Handler, %function
Reset_Handler:
  bl    Init_Stack                               // initialize MSP/PSP and limits
  bl    Init_XOSC                                // initialize external crystal oscillator
  bl    Enable_XOSC_Peri_Clock                   // enable XOSC peripheral clock
  bl    Init_Subsystem                           // initialize subsystems
  bl    Enable_Coprocessor                       // enable CP0 coprocessor
  b     main                                     // branch to main loop
.size Reset_Handler, . - Reset_Handler
```

## Symbol Metadata

```asm
.global Reset_Handler                            // export Reset_Handler symbol
.type Reset_Handler, %function
```

`Reset_Handler` is global because vector_table.s references it.  The `.type` directive marks it as a function for the linker and debugger.

```asm
.size Reset_Handler, . - Reset_Handler
```

`.size` calculates the function's byte size (current address minus start address) and records it in the symbol table.  This aids debuggers in displaying function boundaries.

## The Initialization Sequence

Each `bl` (branch with link) call does one initialization step:

### Step 1: Init_Stack (stack.s)

```asm
  bl    Init_Stack                               // initialize MSP/PSP and limits
```

Sets MSP, PSP, MSPLIM, and PSPLIM.  This must be first because every subsequent function uses the stack (push/pop).

**Why first?** The hardware already loaded MSP from the vector table, so the stack technically works.  But Init_Stack also sets the limit registers, which catch overflow.  Calling it first ensures overflow protection is active for all subsequent calls.

### Step 2: Init_XOSC (xosc.s)

```asm
  bl    Init_XOSC                                // initialize external crystal oscillator
```

Configures the external crystal oscillator and waits for it to stabilize.  The chip initially runs on the internal ring oscillator (~14.5 MHz).  XOSC provides a stable 12 MHz reference.

**Why second?** The XOSC must be stable before we can use it as a clock source for peripherals.

### Step 3: Enable_XOSC_Peri_Clock (xosc.s)

```asm
  bl    Enable_XOSC_Peri_Clock                   // enable XOSC peripheral clock
```

Configures CLK_PERI to use XOSC as its source and enables the clock output.

**Why third?** The XOSC must be stable (step 2) before we route it to peripherals.  Peripherals need a clock to function.

### Step 4: Init_Subsystem (reset.s)

```asm
  bl    Init_Subsystem                           // initialize subsystems
```

Releases IO_BANK0 from reset and waits for the reset to complete.

**Why fourth?** The peripheral clock must be running (step 3) before we release peripherals from reset.  A peripheral without a clock cannot complete its reset sequence.

### Step 5: Enable_Coprocessor (coprocessor.s)

```asm
  bl    Enable_Coprocessor                       // enable CP0 coprocessor
```

Grants full access to coprocessor 0 (SIO) by modifying CPACR and executing barrier instructions.

**Why fifth?** The coprocessor is used for GPIO operations in main.  It must be enabled before any `mcrr` instruction executes.

### Step 6: Branch to main

```asm
  b     main                                     // branch to main loop
```

Note the use of `b` (branch) instead of `bl` (branch with link).  `b` does not save a return address because main never returns — it contains an infinite loop.  Using `b` also means Reset_Handler does not need to save LR, since it never returns either.

## Dependency Chain

The initialization order is not arbitrary — each step depends on the previous one:

```
Init_Stack
  |  (stack works)
  v
Init_XOSC
  |  (crystal stable)
  v
Enable_XOSC_Peri_Clock
  |  (peripheral clock running)
  v
Init_Subsystem
  |  (GPIO released from reset)
  v
Enable_Coprocessor
  |  (CP0 accessible)
  v
main
  |  (GPIO set/clear via mcrr)
```

If any step were skipped or reordered:

| Missing Step | Consequence |
|-------------|-------------|
| Init_Stack | Stack overflow is undetected |
| Init_XOSC | Peripheral clock has no stable source |
| Enable_XOSC_Peri_Clock | Peripherals have no clock |
| Init_Subsystem | GPIO registers read as zero |
| Enable_Coprocessor | `mcrr` triggers UsageFault |

## Reset_Handler Is Not a Normal Function

Reset_Handler has several special properties:

1. **Never called by software** — only by hardware via the reset vector
2. **Never returns** — it branches to main, which loops forever
3. **Does not save/restore registers** — there is no caller to preserve state for
4. **Uses `bl` for init calls** — each init function returns via `bx lr`
5. **Uses `b` for main** — tail call, not a function call

## Summary

- Reset_Handler is the first code executed after the hardware reads the vector table.
- It calls five initialization functions in strict dependency order.
- Each `bl` calls a function that returns via `bx lr`.
- The final `b main` is a one-way branch — the firmware never returns past this point.
- Reordering or skipping any step would cause hardware faults or incorrect behavior.
- `.size` records the function size for debugger support.
