# Chapter 12: ARM Calls, Returns, and the Stack Frame

## Introduction

Structured software is built from functions that call other functions.  The Cortex-M33 implements calls with `bl` (branch with link) and returns with `bx lr` (branch exchange via the link register).  When functions are nested — a caller invokes a callee that invokes yet another callee — the link register must be saved to the stack.  This chapter examines how calls, returns, and the stack frame work together in our blink driver.

## The Link Register

When `bl` executes, the processor stores the address of the next instruction in the Link Register (LR, also named r14):

```
Before bl:
  PC = 0x10000100   (bl GPIO_Set instruction)
  LR = (previous)

After bl:
  PC = GPIO_Set     (target address)
  LR = 0x10000104   (return address, next instruction)
```

The callee returns by executing `bx lr`, which loads LR back into PC.

## Leaf Functions vs. Non-Leaf Functions

A **leaf function** does not call any other function.  It can use LR directly without saving it:

```asm
Init_Stack:
  ldr   r0, =STACK_TOP                           // r0 = stack top
  msr   PSP, r0                                  // set process stack pointer
  ldr   r0, =STACK_LIMIT                         // r0 = stack limit
  msr   MSPLIM, r0                               // set main stack limit
  msr   PSPLIM, r0                               // set process stack limit
  ldr   r0, =STACK_TOP                           // r0 = stack top
  msr   MSP, r0                                  // set main stack pointer
  bx    lr                                       // return (LR preserved)
```

A **non-leaf function** calls other functions, which overwrites LR.  It must save LR on entry and restore it on exit:

```asm
main:
  push  {r4-r12, lr}                             // save LR (and work registers)
  ...
  bl    GPIO_Config                              // LR overwritten
  ...
  bl    GPIO_Set                                 // LR overwritten again
  ...
  pop   {r4-r12, lr}                             // restore original LR
  bx    lr                                       // return to caller
```

## The Call Chain

Our blink driver has this call hierarchy:

```
Reset_Handler
  +-- Init_Stack          (leaf)
  +-- Init_XOSC           (leaf)
  +-- Enable_XOSC_Peri_Clock  (leaf)
  +-- Init_Subsystem      (leaf)
  +-- Enable_Coprocessor  (leaf)
  +-- main                (non-leaf)
        +-- GPIO_Config   (leaf)
        +-- GPIO_Set      (leaf)
        +-- Delay_MS      (leaf)
        +-- GPIO_Clear    (leaf)
```

Reset_Handler uses `bl` to call each initialization function and then `b main` (branch, not branch-with-link) because main never returns.  Within main, `bl` calls GPIO and delay functions.

## The Stack Frame

When `push {r4-r12, lr}` executes, the stack pointer decreases and registers are saved:

```
High addresses (STACK_TOP = 0x20082000)
+------------------+
| lr               |  SP + 36
| r12              |  SP + 32
| r11              |  SP + 28
| r10              |  SP + 24
| r9               |  SP + 20
| r8               |  SP + 16
| r7               |  SP + 12
| r6               |  SP + 8
| r5               |  SP + 4
| r4               |  SP + 0   <-- SP after push
+------------------+
Low addresses
```

When the function returns, `pop {r4-r12, lr}` restores every register and advances SP back up.

## Nested Calls

When `main` calls `GPIO_Set`, and GPIO_Set pushes its own registers:

```
+------------------+
| main's lr        |  main's frame
| main's r12       |
| ...              |
| main's r4        |
+------------------+
| GPIO_Set's lr    |  GPIO_Set's frame
| GPIO_Set's r12   |
| ...              |
| GPIO_Set's r0    |
+------------------+  <-- SP
```

Each function gets its own frame.  When GPIO_Set returns, its frame is removed.  When main's loop calls GPIO_Clear, a new frame is created at the same location.

## The ARM Calling Convention (AAPCS)

Our firmware follows the ARM Architecture Procedure Call Standard (AAPCS):

| Register | Role | Caller/Callee Saved |
|----------|------|-------------------|
| r0–r3 | Arguments and return values | Caller-saved (scratch) |
| r4–r11 | General purpose | Callee-saved (must preserve) |
| r12 | Intra-procedure scratch | Caller-saved |
| r13 (SP) | Stack pointer | Callee-saved |
| r14 (LR) | Link register | Callee-saved (if non-leaf) |
| r15 (PC) | Program counter | — |

Our functions demonstrate this convention:

- `main` passes GPIO number in r0 and delay milliseconds in r0
- `GPIO_Config` receives pad offset (r0), ctrl offset (r1), GPIO number (r2)
- `GPIO_Set` and `GPIO_Clear` receive GPIO number in r0
- `Delay_MS` receives milliseconds in r0

## Parameter Passing Example

When main calls GPIO_Config:

```asm
  ldr   r0, =PADS_BANK0_GPIO16_OFFSET            // r0 = pad offset
  ldr   r1, =IO_BANK0_GPIO16_CTRL_OFFSET          // r1 = ctrl offset
  mov   r2, #16                                  // r2 = GPIO number
  bl    GPIO_Config                              // call
```

The callee receives these in r0, r1, r2 with no additional mechanism needed.

When main calls GPIO_Set:

```asm
  mov   r0, #16                                  // r0 = GPIO number
  bl    GPIO_Set                                 // call
```

## Reset_Handler: A Special Case

Reset_Handler never returns.  It calls initialization functions with `bl` and then jumps to main with `b` (not `bl`):

```asm
Reset_Handler:
  bl    Init_Stack                               // returns via bx lr
  bl    Init_XOSC                                // returns via bx lr
  bl    Enable_XOSC_Peri_Clock                   // returns via bx lr
  bl    Init_Subsystem                           // returns via bx lr
  bl    Enable_Coprocessor                       // returns via bx lr
  b     main                                     // never returns
```

Using `b` instead of `bl` means main's LR value is the return address from the last `bl` — but since main contains an infinite loop, this is irrelevant.  The system never returns past main.

## Summary

- `bl` saves the return address in LR and branches to the target function.
- `bx lr` returns to the caller by loading LR into PC.
- Non-leaf functions save LR with `push` and restore it with `pop`.
- The stack grows downward; each function creates a frame for its saved registers.
- The AAPCS defines r0–r3 as argument/return registers and r4–r11 as callee-saved.
- Reset_Handler uses `b main` (not `bl`) because the system never returns.
