# Chapter 5: Load-Store Architecture — How ARM Accesses Memory

## Introduction

ARM Cortex-M33 is a **load-store architecture**: the only instructions that touch memory are loads (read from memory into a register) and stores (write from a register into memory).  All arithmetic and logic happens exclusively between registers.  This chapter explains why this design exists, the key load and store instructions, and how our blink driver uses them to control hardware.

## Why Load-Store?

In a load-store architecture, there is a clear separation:

1. **Load** data from memory into registers.
2. **Process** data using arithmetic/logic instructions (registers only).
3. **Store** results from registers back to memory.

This simplifies the processor pipeline because only two instruction types access memory.  Every other instruction operates at register speed — one cycle.

## The Load Instruction: ldr

The `ldr` (Load Register) instruction reads a 32-bit word from memory into a register:

```asm
  ldr   r1, [r0]                                 // r1 = Memory[r0]
```

This reads four bytes starting at the address in r0 and places the value in r1.

With an immediate offset:

```asm
  ldr   r1, [r0, #4]                             // r1 = Memory[r0 + 4]
```

The pseudo-instruction form loads a constant or address:

```asm
  ldr   r0, =XOSC_CTRL                           // r0 = address of XOSC_CTRL
```

The assembler generates a PC-relative load from a literal pool to place the 32-bit constant into r0.

## The Store Instruction: str

The `str` (Store Register) instruction writes a 32-bit word from a register to memory:

```asm
  str   r1, [r0]                                 // Memory[r0] = r1
```

When the target address is a peripheral register, this write triggers hardware behavior:

```asm
  ldr   r0, =XOSC_CTRL                           // load XOSC_CTRL address
  ldr   r1, =0x00FABAA0                          // oscillator config value
  str   r1, [r0]                                 // write to hardware — oscillator starts
```

## The Load-Modify-Store Pattern

Most hardware configuration follows a three-step pattern:

1. **Load** the current register value.
2. **Modify** specific bits using bitwise operations.
3. **Store** the modified value back.

From our GPIO pad configuration:

```asm
  ldr   r5, [r4]                                 // load current PAD register value
  bic   r5, r5, #(1<<7)                          // clear OD (output disable) bit
  orr   r5, r5, #(1<<6)                          // set IE (input enable) bit
  bic   r5, r5, #(1<<8)                          // clear ISO (isolation) bit
  str   r5, [r4]                                 // store modified value back
```

This is the most important pattern in hardware programming.  It changes only the bits we need while preserving all others.

## Byte and Halfword Access

ARM provides size-specific load/store variants:

| Instruction | Size | Description |
|-------------|------|-------------|
| `ldr` / `str` | 32-bit (word) | Default |
| `ldrh` / `strh` | 16-bit (halfword) | |
| `ldrb` / `strb` | 8-bit (byte) | |
| `ldrsb` | 8-bit signed | Sign-extends to 32 bits |
| `ldrsh` | 16-bit signed | Sign-extends to 32 bits |

Our firmware uses only `ldr`/`str` because all RP2350 peripheral registers are 32 bits wide.

## Push and Pop

The `push` and `pop` instructions are specialized store and load operations that use the stack pointer:

```asm
  push  {r4-r12, lr}                             // store registers to stack, SP decreases
  pop   {r4-r12, lr}                             // load registers from stack, SP increases
```

`push` is equivalent to multiple `str` instructions with pre-decrement addressing.  `pop` is equivalent to multiple `ldr` instructions with post-increment addressing.

These are critical for the calling convention — every non-leaf function in our firmware begins with `push` and ends with `pop`.

## Memory Access in Our Firmware

Every hardware interaction in our blink driver follows the load-store pattern:

| Operation | Instructions | Purpose |
|-----------|-------------|---------|
| Read register | `ldr r1, [r0]` | Read current hardware state |
| Write register | `str r1, [r0]` | Configure hardware |
| Read-modify-write | `ldr` → `bic`/`orr` → `str` | Change specific bits |
| Load constant | `ldr r0, =VALUE` | Get address or immediate |
| Save/restore | `push`/`pop` | Preserve registers across calls |

## Summary

- ARM is a load-store architecture: only `ldr`/`str` access memory.
- All computation happens in registers.
- `ldr` reads from memory; `str` writes to memory.
- The load-modify-store pattern is the fundamental hardware programming technique.
- `push` and `pop` save and restore registers using the stack.
- Our firmware uses word-sized (32-bit) access exclusively for peripheral registers.
