# Chapter 16: System Registers and Coprocessor Interface

## Introduction

Beyond the sixteen general-purpose registers and the APSR, the Cortex-M33 provides system-level special registers and a coprocessor interface.  Our blink driver uses `msr` to configure the stack pointers, `dsb`/`isb` to synchronize processor state, and `mcrr` to control GPIO through the RP2350's SIO coprocessor.  This chapter covers these system-level instructions and the hardware interface they expose.

## Special-Purpose Registers

The Cortex-M33 has several special-purpose registers accessed via `msr` (write) and `mrs` (read):

| Register | Purpose | Used In |
|----------|---------|---------|
| MSP | Main Stack Pointer | stack.s |
| PSP | Process Stack Pointer | stack.s |
| MSPLIM | Main Stack Pointer Limit | stack.s |
| PSPLIM | Process Stack Pointer Limit | stack.s |
| PRIMASK | Interrupt mask | (not used) |
| CONTROL | Execution mode / stack select | (not used) |

### Stack Pointer Initialization

stack.s configures all stack registers:

```asm
Init_Stack:
  ldr   r0, =STACK_TOP                           // r0 = 0x20082000
  msr   PSP, r0                                  // process stack pointer
  ldr   r0, =STACK_LIMIT                         // r0 = 0x2007a000
  msr   MSPLIM, r0                               // main stack limit
  msr   PSPLIM, r0                               // process stack limit
  ldr   r0, =STACK_TOP                           // r0 = 0x20082000
  msr   MSP, r0                                  // main stack pointer
  bx    lr                                       // return
```

**MSP** is the active stack pointer after reset.  **MSPLIM** and **PSPLIM** are ARMv8-M features that trigger a fault if the stack pointer goes below the limit, preventing silent stack overflow.

## Memory-Mapped System Registers

Some system configuration is done through memory-mapped registers in the Private Peripheral Bus (PPB) region at `0xE000_0000`:

### CPACR — Coprocessor Access Control Register

Address: `PPB_BASE + 0x0ED88` = `0xE000ED88`

The CPACR controls access to coprocessors.  Our firmware enables coprocessor 0 (CP0), which provides access to the RP2350's SIO GPIO interface:

```asm
Enable_Coprocessor:
  ldr   r0, =CPACR                               // r0 = 0xE000ED88
  ldr   r1, [r0]                                 // read current CPACR
  orr   r1, r1, #(1<<1)                          // set CP0 full access bit 1
  orr   r1, r1, #(1<<0)                          // set CP0 full access bit 0
  str   r1, [r0]                                 // write back
  dsb                                            // synchronize
  isb                                            // flush pipeline
  bx    lr                                       // return
```

Bits [1:0] of CPACR control CP0 access:

| Bits [1:0] | Access Level |
|------------|-------------|
| `00` | No access (default) |
| `01` | Privileged access only |
| `11` | Full access (our setting) |

## Barrier Instructions

### `dsb` — Data Synchronization Barrier

```asm
  dsb                                            // wait for all memory accesses to complete
```

`dsb` ensures that all preceding memory accesses (loads and stores) have completed before any subsequent instruction executes.  This is critical after modifying the CPACR because the processor must see the new coprocessor configuration before attempting a coprocessor instruction.

### `isb` — Instruction Synchronization Barrier

```asm
  isb                                            // flush pipeline
```

`isb` flushes the processor pipeline, ensuring that all subsequent instructions are fetched and decoded using the current system state.  After changing CPACR, any `mcrr` instructions in the pipeline must be re-fetched with the new access permissions.

The `dsb` + `isb` sequence is an ARM-mandated pattern after modifying system control registers:

```
str  → dsb → isb → (now safe to use CP0)
```

## The Coprocessor Interface

### What Is a Coprocessor?

On the RP2350, coprocessor 0 (CP0) provides fast access to the Single-cycle IO (SIO) block.  The SIO contains GPIO set/clear/toggle registers that bypass the normal peripheral bus — providing faster GPIO control.

### `mcrr` — Move to Coprocessor from Two Registers

The `mcrr` instruction transfers data from two ARM registers to a coprocessor:

```asm
  mcrr  p0, #4, r2, r4, c4                      // CP0: enable OE for GPIO
```

The instruction format is:

```
mcrr  pN, #opc, Rt, Rt2, CRm
```

| Field | Meaning | In Our Firmware |
|-------|---------|-----------------|
| `pN` | Coprocessor number | `p0` (SIO) |
| `#opc` | Operation code | `#4` (GPIO operation) |
| `Rt` | First source register | GPIO number or value |
| `Rt2` | Second source register | Value or mask |
| `CRm` | Coprocessor register | Operation selector |

### GPIO Operations via CP0

Our firmware uses `mcrr` for three distinct GPIO operations:

**1. Enable Output (gpio.s — GPIO_Config):**

```asm
  mov   r4, #1                                   // r4 = 1 (enable)
  mcrr  p0, #4, r2, r4, c4                      // OE enable for GPIO r2
```

This sets the output enable bit for the specified GPIO pin.

**2. Set GPIO High (gpio.s — GPIO_Set):**

```asm
  mov   r4, #1                                   // r4 = 1 (set high)
  mcrr  p0, #4, r0, r4, c0                      // set GPIO r0 high
```

This drives the GPIO pin to logic high (3.3V), turning the LED on.

**3. Clear GPIO Low (gpio.s — GPIO_Clear):**

```asm
  mov   r4, #0                                   // r4 = 0 (set low)
  mcrr  p0, #4, r0, r4, c0                      // set GPIO r0 low
```

This drives the GPIO pin to logic low (0V), turning the LED off.

### Why Use a Coprocessor for GPIO?

Direct register writes through the peripheral bus take multiple clock cycles due to bus arbitration.  The coprocessor interface provides a dedicated path to the SIO block, enabling single-cycle GPIO operations.  For a blink driver this performance difference is negligible, but for high-speed bit-banging it is significant.

## The Complete Coprocessor Flow

Before any `mcrr` instruction can execute:

```
1. Enable_Coprocessor: CPACR |= 0x3 → dsb → isb
2. GPIO_Config: pad setup → ctrl setup → mcrr OE enable
3. Main loop: mcrr set → delay → mcrr clear → delay → repeat
```

If step 1 is omitted, any `mcrr` instruction triggers a UsageFault because CP0 access is denied by default.

## Summary

- `msr`/`mrs` access special-purpose registers like MSP, PSP, and their limits.
- The CPACR (memory-mapped at 0xE000ED88) controls coprocessor access permissions.
- `dsb` and `isb` synchronize memory and pipeline state after system register changes.
- `mcrr` transfers data to coprocessor 0, providing fast access to the SIO GPIO interface.
- Three `mcrr` operations control GPIO: output enable, set high, and clear low.
- Coprocessor access must be enabled before any `mcrr` instruction, or a fault occurs.
