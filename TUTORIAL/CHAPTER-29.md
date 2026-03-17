# Chapter 29: main.s — The Blink Loop

## Introduction

main.s is the application entry point — the code that ties every driver function together into an LED that blinks.  After Reset_Handler completes all initialization, it branches to `main`, which configures GPIO16 once and then enters an infinite loop: set, delay, clear, delay, repeat.  This chapter walks through every line.

## Complete Source Code

```asm
.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

.global main                                     // export main
.type main, %function                            // mark as function
main:
.Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO16_Config:
  ldr   r0, =PADS_BANK0_GPIO16_OFFSET            // load PADS_BANK0_GPIO16_OFFSET
  ldr   r1, =IO_BANK0_GPIO16_CTRL_OFFSET         // load IO_BANK0_GPIO16_CTRL_OFFSET
  ldr   r2, =16                                  // load GPIO number
  bl    GPIO_Config                              // call GPIO_Config
.Loop:
  ldr   r0, =16                                  // load GPIO number
  bl    GPIO_Set                                 // call GPIO_Set
  ldr   r0, =500                                 // 500ms
  bl    Delay_MS                                 // call Delay_MS
  ldr   r0, =16                                  // load GPIO number
  bl    GPIO_Clear                               // call GPIO_Clear
  ldr   r0, =500                                 // 500ms
  bl    Delay_MS                                 // call Delay_MS
  b     .Loop                                    // loop forever
.Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return to caller

.section .rodata                                 // read-only data section

.section .data                                   // data section

.section .bss                                    // BSS section
```

## Function Metadata

```asm
.global main                                     // export main
.type main, %function                            // mark as function
```

`main` is global because reset_handler.s references it with `b main`.  The `.type` directive marks it as a function for the linker and debugger.

## Register Save

```asm
.Push_Registers:
  push  {r4-r12, lr}                             // save 10 registers (40 bytes)
```

main is a non-leaf function — it calls `GPIO_Config`, `GPIO_Set`, `GPIO_Clear`, and `Delay_MS`, all of which overwrite LR.  Pushing LR preserves the return address, and pushing r4-r12 follows the AAPCS callee-save convention.

In practice, main never returns (due to the infinite loop), but the push/pop pair maintains correct function structure.

## GPIO16 Configuration (One-Time Setup)

```asm
.GPIO16_Config:
  ldr   r0, =PADS_BANK0_GPIO16_OFFSET            // r0 = 0x44 (pad offset)
  ldr   r1, =IO_BANK0_GPIO16_CTRL_OFFSET         // r1 = 0x84 (ctrl offset)
  ldr   r2, =16                                  // r2 = 16 (GPIO number)
  bl    GPIO_Config                              // configure GPIO16
```

This runs once, before the loop.  It passes three parameters:

| Register | Value | Purpose |
|----------|-------|---------|
| r0 | `0x44` | Pad register offset for GPIO16 |
| r1 | `0x84` | Control register offset for GPIO16 |
| r2 | `16` | GPIO pin number |

After `GPIO_Config` returns, GPIO16 is fully configured: pad connected, function select set to SIO, output enabled.

## The Infinite Blink Loop

```asm
.Loop:
  ldr   r0, =16                                  // r0 = 16 (GPIO number)
  bl    GPIO_Set                                 // drive GPIO16 high (LED on)
  ldr   r0, =500                                 // r0 = 500 (milliseconds)
  bl    Delay_MS                                 // wait 500ms
  ldr   r0, =16                                  // r0 = 16 (GPIO number)
  bl    GPIO_Clear                               // drive GPIO16 low (LED off)
  ldr   r0, =500                                 // r0 = 500 (milliseconds)
  bl    Delay_MS                                 // wait 500ms
  b     .Loop                                    // repeat forever
```

### Iteration Timeline

```
Time (ms):  0     500    1000   1500   2000   2500
            |------|------|------|------|------|
LED:        ON     OFF    ON     OFF    ON     OFF
            Set    Clear  Set    Clear  Set    Clear
```

The LED blinks at 1 Hz: 500 ms on, 500 ms off, total period = 1 second.

### Why Load r0 Every Iteration?

```asm
  ldr   r0, =16                                  // reload GPIO number each time
```

r0 is a scratch register (caller-saved).  Each `bl` call may modify r0 internally.  We must reload it before each function call.

### The Loop Never Exits

```asm
  b     .Loop                                    // unconditional branch back
```

`b .Loop` is an unconditional backward branch.  The processor will execute this loop until:

- Power is removed
- The chip is reset
- A debugger halts execution

## Unreachable Code

```asm
.Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return to caller
```

Because of the infinite loop, these two instructions never execute.  They are present for structural completeness — if the loop were removed or exited, the function would return correctly.

## Data Sections

```asm
.section .rodata                                 // read-only data section
.section .data                                   // data section
.section .bss                                    // BSS section
```

These sections are declared but empty.  Our blink driver uses no global variables, no initialized data, and no uninitialized data.  The sections are present as placeholders for future expansion.

## Complete Execution Flow

```
Power On
  |
  v
Boot ROM → finds PICOBIN → jumps to Reset_Handler
  |
  v
Reset_Handler
  bl Init_Stack            → stack configured
  bl Init_XOSC             → crystal running
  bl Enable_XOSC_Peri_Clock → peripheral clock on
  bl Init_Subsystem        → GPIO released from reset
  bl Enable_Coprocessor    → CP0 accessible
  b  main
  |
  v
main
  push {r4-r12, lr}
  GPIO_Config(0x44, 0x84, 16)  → GPIO16 ready
  |
  +--→ GPIO_Set(16)        → LED ON
  |    Delay_MS(500)        → wait 500ms
  |    GPIO_Clear(16)       → LED OFF
  |    Delay_MS(500)        → wait 500ms
  |    b .Loop ─────────────+
```

## Summary

- main.s configures GPIO16 once, then enters an infinite blink loop.
- The loop calls GPIO_Set, Delay_MS, GPIO_Clear, Delay_MS in sequence.
- r0 is reloaded before each function call because it is a scratch register.
- The `b .Loop` creates an infinite cycle — the LED blinks at 1 Hz.
- The pop/bx instructions after the loop are structurally correct but never reached.
- Empty `.rodata`, `.data`, and `.bss` sections are declared for completeness.
