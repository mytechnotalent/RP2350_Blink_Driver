# Chapter 28: gpio.s Part 2, delay.s, and coprocessor.s — Output Control and Timing

## Introduction

With GPIO16 configured by `GPIO_Config`, three more functions complete the blink driver's runtime operation: `GPIO_Set` drives the pin high, `GPIO_Clear` drives it low, and `Delay_MS` creates a timed pause between transitions.  A fourth function, `Enable_Coprocessor`, makes all coprocessor-based GPIO operations possible.  This chapter covers all four functions.

## GPIO_Set — Drive Pin High

```asm
.global GPIO_Set
.type GPIO_Set, %function
GPIO_Set:
.GPIO_Set_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Set_Execute:
  ldr   r4, =1                                   // enable output
  mcrr  p0, #4, r0, r4, c0                       // gpioc_bit_out_put(GPIO, 1)
.GPIO_Set_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return
```

### Function Signature

```
GPIO_Set(r0 = GPIO_NUMBER)
```

### How It Works

```asm
  ldr   r4, =1                                   // r4 = 1 (high)
  mcrr  p0, #4, r0, r4, c0                       // set GPIO output high
```

The `mcrr` instruction fields:

| Field | Value | Meaning |
|-------|-------|---------|
| `p0` | Coprocessor 0 | SIO block |
| `#4` | Opcode | GPIO bit operation |
| `r0` | GPIO number | 16 |
| `r4` | Output value | 1 (high = 3.3V) |
| `c0` | Register | Output Value control |

When r4 = 1, the GPIO pin is driven to 3.3V.  Current flows through the LED and resistor to ground — the LED lights up.

## GPIO_Clear — Drive Pin Low

```asm
.global GPIO_Clear
.type GPIO_Clear, %function
GPIO_Clear:
.GPIO_Clear_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Clear_Execute:
  ldr   r4, =0                                   // disable output
  mcrr  p0, #4, r0, r4, c0                       // gpioc_bit_out_put(GPIO, 0)
.GPIO_Clear_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return
```

### Difference from GPIO_Set

The only difference is r4:

| Function | r4 Value | Pin State | LED |
|----------|----------|-----------|-----|
| GPIO_Set | 1 | High (3.3V) | ON |
| GPIO_Clear | 0 | Low (0V) | OFF |

Both use `c0` (output value register) instead of `c4` (output enable register used in GPIO_Config).

## Delay_MS — Millisecond Delay

```asm
.global Delay_MS
.type Delay_MS, %function
Delay_MS:
.Delay_MS_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.Delay_MS_Check:
  cmp   r0, #0                                   // if MS is not valid, return
  ble   .Delay_MS_Done                           // branch if less or equal to 0
.Delay_MS_Setup:
  ldr   r4, =3600                                // loops per MS based on 14.5MHz clock
  mul   r5, r0, r4                               // MS * 3600
.Delay_MS_Loop:
  subs  r5, r5, #1                               // decrement counter
  bne   .Delay_MS_Loop                           // branch until zero
.Delay_MS_Done:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return
```

### Function Signature

```
Delay_MS(r0 = milliseconds)
```

### Input Validation

```asm
  cmp   r0, #0                                   // compare ms to 0
  ble   .Delay_MS_Done                           // if <= 0, skip to return
```

`cmp` subtracts 0 from r0 and sets the flags.  `ble` branches if the result is less than or equal to zero (Z=1 or N!=V).  This prevents:

- Zero delay: `mul` would produce 0 iterations, but the check makes intent explicit
- Negative values: if r0 were interpreted as signed negative, `mul` could produce a huge unsigned count

### Calculate Total Iterations

```asm
  ldr   r4, =3600                                // loops per millisecond
  mul   r5, r0, r4                               // total iterations = ms * 3600
```

At approximately 14.5 MHz, with `subs` + `bne` taking ~4 cycles per iteration:

```
iterations per ms = 14,500,000 / 4 ≈ 3,625 ≈ 3,600 (rounded)
```

For 500 ms: `500 × 3600 = 1,800,000` iterations.

### The Delay Loop

```asm
.Delay_MS_Loop:
  subs  r5, r5, #1                               // r5 = r5 - 1, set flags
  bne   .Delay_MS_Loop                           // loop if r5 != 0
```

This is the tightest possible loop on ARM: two instructions, executing approximately 1,800,000 times for a 500 ms delay.  The `s` suffix on `subs` sets the Zero flag when r5 reaches zero.  `bne` loops while Z=0 (non-zero).

### Timing Accuracy

The delay is approximate because:

1. The ring oscillator frequency varies with temperature and voltage
2. Pipeline effects add small variations per iteration
3. The loop overhead (setup, push/pop) adds a few microseconds

For blinking an LED, this accuracy is more than sufficient.

## Enable_Coprocessor — CP0 Access

```asm
.global Enable_Coprocessor
.type Enable_Coprocessor , %function
Enable_Coprocessor:
  ldr   r0, =CPACR                               // load CPACR address
  ldr   r1, [r0]                                 // read CPACR value
  orr   r1, r1, #(1<<1)                          // set CP0: Ctrl access priv coproc 0 bit
  orr   r1, r1, #(1<<0)                          // set CP0: Ctrl access priv coproc 0 bit
  str   r1, [r0]                                 // store value into CPACR
  dsb                                            // data sync barrier
  isb                                            // instruction sync barrier
  bx    lr                                       // return
```

### CPACR Modification

```asm
  ldr   r0, =CPACR                               // r0 = 0xE000ED88
  ldr   r1, [r0]                                 // read current value
  orr   r1, r1, #(1<<1)                          // set bit 1
  orr   r1, r1, #(1<<0)                          // set bit 0
  str   r1, [r0]                                 // write back
```

Setting bits [1:0] to `0b11` grants full (privileged + unprivileged) access to coprocessor 0.  Without this, any `mcrr p0, ...` instruction triggers a UsageFault.

### Barrier Instructions

```asm
  dsb                                            // all memory accesses complete
  isb                                            // flush instruction pipeline
```

The `dsb` (Data Synchronization Barrier) ensures the CPACR write has reached the register before proceeding.  The `isb` (Instruction Synchronization Barrier) flushes the pipeline so any subsequent `mcrr` instructions are fetched with the updated access permissions.

This sequence is required by the ARM Architecture Reference Manual after any modification to CPACR.

### Leaf Function

`Enable_Coprocessor` is a leaf function using only scratch registers (r0, r1).  No push/pop is needed.

## The Runtime Flow

With all four functions defined, the blink loop operates:

```
main:
  GPIO_Config(0x44, 0x84, 16)    ← configure GPIO16
  loop:
    GPIO_Set(16)                  ← pin high (LED on)
    Delay_MS(500)                 ← wait 500ms
    GPIO_Clear(16)                ← pin low (LED off)
    Delay_MS(500)                 ← wait 500ms
    goto loop
```

The LED blinks at 1 Hz (500 ms on, 500 ms off).

## Summary

- `GPIO_Set` drives a pin high via `mcrr p0, #4, r0, r4, c0` with r4=1.
- `GPIO_Clear` drives a pin low via the same instruction with r4=0.
- `Delay_MS` validates input, computes iterations (ms × 3600), and counts down with `subs`/`bne`.
- `Enable_Coprocessor` grants CP0 access via CPACR modification, followed by mandatory `dsb`/`isb` barriers.
- The delay calibration of 3600 loops/ms assumes the ~14.5 MHz ring oscillator clock speed.
