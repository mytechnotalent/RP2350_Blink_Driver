# Chapter 4: What Is a Register?

## Introduction

Registers are the fastest storage in the processor — small, 32-bit holding cells built directly into the CPU silicon.  Every computation happens in registers: loading values from memory, performing arithmetic, testing conditions, and storing results back.  This chapter maps out the complete ARM Cortex-M33 register file and shows exactly which registers our blink driver uses.

## The ARM Cortex-M33 Register File

The Cortex-M33 provides 16 general-purpose registers plus several special-purpose registers:

| Register | ABI Name | Purpose |
|----------|----------|---------|
| r0 | a1 | Argument 1 / return value / scratch |
| r1 | a2 | Argument 2 / scratch |
| r2 | a3 | Argument 3 / scratch |
| r3 | a4 | Argument 4 / scratch |
| r4 | v1 | Callee-saved variable |
| r5 | v2 | Callee-saved variable |
| r6 | v3 | Callee-saved variable |
| r7 | v4 | Callee-saved variable |
| r8 | v5 | Callee-saved variable |
| r9 | v6 | Callee-saved variable (platform-specific) |
| r10 | v7 | Callee-saved variable |
| r11 | v8 | Callee-saved variable (frame pointer) |
| r12 | IP | Intra-procedure scratch |
| r13 | SP | Stack Pointer |
| r14 | LR | Link Register |
| r15 | PC | Program Counter |

## Registers r0-r3: Arguments and Scratch

The first four registers pass function arguments and return values.  They are **caller-saved**: a called function is free to overwrite them without restoring their previous values.

In our firmware:

```asm
  ldr   r0, =PADS_BANK0_GPIO16_OFFSET            // argument 1: pad offset
  ldr   r1, =IO_BANK0_GPIO16_CTRL_OFFSET         // argument 2: ctrl offset
  ldr   r2, =16                                  // argument 3: GPIO number
  bl    GPIO_Config                              // call GPIO_Config
```

Three arguments are loaded into r0, r1, r2 before the call.

## Registers r4-r11: Callee-Saved

These registers must be preserved across function calls.  If a function uses them, it must save them on entry and restore them on exit:

```asm
  push  {r4-r12, lr}                             // save callee-saved registers
  // ... use r4, r5 freely ...
  pop   {r4-r12, lr}                             // restore callee-saved registers
```

Our GPIO_Config function uses r4 and r5 as working registers for address calculations and read-modify-write operations.

## Register r12 (IP): Intra-Procedure Scratch

r12 is designated as the Intra-Procedure call scratch register.  The linker may use it for long-range branch veneers.  Our firmware saves and restores it as part of the `{r4-r12, lr}` push/pop block.

## Register r13 (SP): Stack Pointer

The Stack Pointer holds the address of the top of the stack.  ARM Cortex-M33 actually has two stack pointers:

- **MSP (Main Stack Pointer)** — used in handler mode (interrupts) and by default in thread mode
- **PSP (Process Stack Pointer)** — optionally used in thread mode

Our Init_Stack function initializes both:

```asm
  ldr   r0, =STACK_TOP                           // load stack top
  msr   PSP, r0                                  // set PSP
  msr   MSP, r0                                  // set MSP
```

## Register r14 (LR): Link Register

When a `bl` (branch-with-link) instruction calls a function, the processor stores the return address in LR.  The function returns by branching to this address:

```asm
  bl    GPIO_Set                                 // LR = address of next instruction
  // ... GPIO_Set runs ...
  bx    lr                                       // return to caller
```

For nested calls, LR must be saved on the stack because the inner call will overwrite it.

## Register r15 (PC): Program Counter

The PC holds the address of the current instruction being fetched.  You rarely write to it directly — branches, calls, and returns update it implicitly.

On Cortex-M33, the PC always contains the address of the current instruction plus 4 (due to the pipeline).

## Special Registers

Beyond the general-purpose file, the Cortex-M33 has special registers accessible only through the `msr` and `mrs` instructions:

| Register | Purpose |
|----------|---------|
| MSP | Main Stack Pointer |
| PSP | Process Stack Pointer |
| MSPLIM | MSP limit (stack overflow detection) |
| PSPLIM | PSP limit (stack overflow detection) |
| PRIMASK | Interrupt mask (1 = all interrupts disabled) |
| CONTROL | Stack pointer selection, privilege level |
| xPSR | Combined program status register |

Our Init_Stack function uses four of these:

```asm
  msr   PSP, r0                                  // set Process Stack Pointer
  msr   MSPLIM, r0                               // set MSP lower limit
  msr   PSPLIM, r0                               // set PSP lower limit
  msr   MSP, r0                                  // set Main Stack Pointer
```

## The Program Status Register (xPSR)

The xPSR is actually three registers combined:

| Sub-register | Bits | Contents |
|-------------|------|----------|
| APSR | 31:28 | Condition flags: N (negative), Z (zero), C (carry), V (overflow) |
| IPSR | 8:0 | Exception number (0 = thread mode, nonzero = handler) |
| EPSR | 24 | T bit (always 1 for Thumb mode) |

The condition flags in APSR are set by instructions with the `s` suffix (like `subs`, `tst`, `cmp`) and tested by conditional branches (`beq`, `bne`, `ble`, etc.).

In our delay loop:

```asm
  subs  r5, r5, #1                               // decrement counter (sets Z flag)
  bne   .Delay_MS_Loop                           // branch if Z=0 (not zero)
```

The `subs` instruction updates the Z flag.  When r5 reaches zero, Z is set to 1, and `bne` falls through to end the loop.

## Register Usage in Our Firmware

| Register | Where Used | Purpose |
|----------|-----------|---------|
| r0 | Everywhere | Addresses, arguments, GPIO pin numbers |
| r1 | Everywhere | Register values, second arguments |
| r2 | main.s | GPIO number argument |
| r4 | gpio.s, delay.s | Working register for address math |
| r5 | gpio.s, delay.s | Working register for values |
| SP | Implicitly | Stack operations (push/pop) |
| LR | Implicitly | Return addresses |
| PC | Implicitly | Instruction fetch |

## Summary

- The Cortex-M33 has 16 general-purpose registers (r0–r15) plus special registers.
- r0–r3 are argument/scratch registers; r4–r11 are callee-saved.
- SP (r13) points to the top of the stack; LR (r14) holds the return address.
- The xPSR contains condition flags (N, Z, C, V) set by arithmetic instructions.
- Special registers (MSP, PSP, MSPLIM, PSPLIM) are accessed via `msr`/`mrs`.
- Our firmware uses r0–r2 for function arguments and r4–r5 for internal computation.
