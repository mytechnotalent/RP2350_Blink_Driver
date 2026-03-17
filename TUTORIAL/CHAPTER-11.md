# Chapter 11: ARM Branch Instructions

## Introduction

Branch instructions alter the flow of execution.  Without branches, the processor would execute instructions sequentially from the reset vector to the end of flash — never looping, never calling a function, never making a decision.  Our blink driver relies on branches for its infinite blink loop, function calls, hardware polling, and input validation.  This chapter covers every branch instruction in our firmware.

## Unconditional Branches

### `b` — Branch

```asm
  b     .Loop                                    // jump to .Loop forever
```

`b` changes the program counter (PC) to the target address.  Execution continues at the new location.  The target is encoded as a signed offset from the current PC.

In our main.s, the infinite blink loop uses `b`:

```asm
.Loop:
  mov   r0, #16                                  // GPIO 16
  bl    GPIO_Set                                 // LED on
  ldr   r0, =500                                 // 500 ms
  bl    Delay_MS                                 // wait
  mov   r0, #16                                  // GPIO 16
  bl    GPIO_Clear                               // LED off
  ldr   r0, =500                                 // 500 ms
  bl    Delay_MS                                 // wait
  b     .Loop                                    // repeat forever
```

The `b .Loop` at the end creates an infinite cycle — the LED blinks until power is removed.

### `bl` — Branch with Link

```asm
  bl    GPIO_Config                              // call GPIO_Config
```

`bl` performs two operations atomically:

1. Saves the return address (next instruction) in the Link Register (LR / r14)
2. Branches to the target

This is how function calls work on ARM.  The caller executes `bl`, and the callee returns with `bx lr`.

### `bx` — Branch Exchange

```asm
  bx    lr                                       // return to caller
```

`bx` branches to the address in a register.  The lowest bit of the address determines the instruction set state (1 = Thumb, 0 = ARM).  On the Cortex-M33, which only supports Thumb, bit 0 must always be 1.

Every function in our firmware ends with:

```asm
  pop   {r4-r12, lr}                             // restore saved registers
  bx    lr                                       // return
```

Or equivalently, some functions pop directly into PC:

```asm
  pop   {r4-r12, pc}                             // restore and return
```

## Conditional Branches

### `beq` — Branch if Equal (Z=1)

```asm
  beq   .Wait_XOSC                              // loop if bit not set
```

`beq` branches only if the Zero flag is set.  In xosc.s, this polls the XOSC_STATUS register:

```asm
.Wait_XOSC:
  ldr   r1, [r0]                                 // read XOSC_STATUS
  tst   r1, #(1<<31)                             // test STABLE bit
  beq   .Wait_XOSC                              // Z=1 means bit is 0: not stable
```

The `tst` instruction ANDs r1 with the mask.  If bit 31 is zero, the result is zero, Z=1, and `beq` loops back.

### `bne` — Branch if Not Equal (Z=0)

```asm
  bne   .Delay_Loop                              // loop until counter reaches 0
```

`bne` branches if the Zero flag is clear.  The delay loop uses this:

```asm
.Delay_Loop:
  subs  r5, r5, #1                               // decrement counter
  bne   .Delay_Loop                              // non-zero: continue
```

When `r5` reaches zero, `subs` sets Z=1, `bne` falls through, and the delay is complete.

### `ble` — Branch if Less or Equal (Z=1 or N!=V)

```asm
  ble   .Delay_Done                              // skip if r0 <= 0
```

`ble` is a signed comparison branch.  In delay.s, it validates the input parameter:

```asm
  cmp   r0, #0                                   // compare ms value to 0
  ble   .Delay_Done                              // if <= 0, do nothing
```

This guards against zero or negative delay values that would produce incorrect behavior.

## Branch Encoding and Range

| Instruction | Encoding | Range |
|-------------|----------|-------|
| `b` (narrow) | 16-bit | ±2 KB |
| `b` (wide) | 32-bit | ±16 MB |
| `bl` | 32-bit | ±16 MB |
| `bx Rn` | 16-bit | Any address in register |
| `beq`/`bne` (narrow) | 16-bit | ±256 bytes |
| `beq`/`bne` (wide) | 32-bit | ±1 MB |

The assembler automatically selects the appropriate encoding based on the distance to the target.

## Condition Codes

The ARM architecture defines 15 condition codes, though our firmware uses only a few:

| Code | Meaning | Flags | Used In |
|------|---------|-------|---------|
| EQ | Equal / zero | Z=1 | xosc.s, reset.s |
| NE | Not equal / non-zero | Z=0 | delay.s |
| LE | Signed less or equal | Z=1 or N!=V | delay.s |
| AL | Always (default) | — | `b`, `bl` |

## Polling Loops

Our firmware contains two hardware-polling loops:

**XOSC Stabilization (xosc.s):**

```
  tst  → beq → tst → beq → ... → tst (bit set) → fall through
```

**Reset Completion (reset.s):**

```
  tst  → beq → tst → beq → ... → tst (bit set) → fall through
```

Both poll a status register bit until hardware signals readiness.  This is "busy waiting" — appropriate for bare-metal firmware where no operating system scheduler exists.

## Summary

- `b` provides unconditional jumps and creates our infinite blink loop.
- `bl` calls functions by saving the return address in LR.
- `bx lr` returns from functions by branching to the saved address.
- Conditional branches (`beq`, `bne`, `ble`) enable polling loops, delay counting, and input validation.
- The condition codes test APSR flags set by prior arithmetic or logic instructions.
