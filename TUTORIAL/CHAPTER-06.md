# Chapter 6: The Fetch-Decode-Execute Cycle in Detail

## Introduction

Chapter 1 introduced the fetch-decode-execute cycle as a concept.  Now we examine it in detail on the ARM Cortex-M33, including the pipeline, how branches disrupt it, and what happens at each stage when our blink driver executes.  This understanding is essential for reasoning about instruction timing and the delay loop in our firmware.

## The Three Stages

### Fetch

The processor reads the instruction bytes from the address in the Program Counter (PC).  On Cortex-M33, instructions can be 16 bits (narrow Thumb) or 32 bits (wide Thumb-2).  The fetch unit reads from flash (via XIP) or SRAM depending on where the PC points.

### Decode

The binary pattern of the fetched instruction is decoded to determine:

- The operation type (load, store, add, branch, etc.)
- The source and destination registers
- Any immediate value embedded in the instruction
- The condition code (for conditional instructions)

### Execute

The operation is performed:

- Arithmetic/logic instructions compute a result and write it to the destination register.
- Load instructions read from the address bus and write to a register.
- Store instructions write a register value to the address bus.
- Branch instructions update the PC.

## The Pipeline

The Cortex-M33 overlaps these stages so that while one instruction executes, the next is being decoded, and the one after that is being fetched:

```
Time:     T1        T2        T3        T4        T5
         +-------+ +-------+ +-------+ +-------+ +-------+
Instr 1: | Fetch | |Decode | |Execute|
         +-------+ +-------+ +-------+
Instr 2:           | Fetch | |Decode | |Execute|
                   +-------+ +-------+ +-------+
Instr 3:                     | Fetch | |Decode | |Execute|
                             +-------+ +-------+ +-------+
```

In ideal conditions, one instruction completes every clock cycle despite each instruction taking three cycles to process.

## A Concrete Example

Consider these three lines from our blink loop:

```asm
  ldr   r0, =16                                  // load GPIO number
  bl    GPIO_Set                                 // call GPIO_Set
  ldr   r0, =500                                 // 500ms
```

At cycle T1 the processor fetches the `ldr`.  At T2 it decodes the `ldr` while fetching the `bl`.  At T3 it executes the `ldr` (placing 16 in r0), decodes the `bl`, and fetches the next `ldr`.

## How Branch Instructions Affect the Pipeline

A branch instruction changes the PC to a new address.  The instructions already in the pipeline (fetched from the addresses *after* the branch) are wrong — they are from the old sequential path.  The pipeline must be **flushed** and refilled from the branch target:

```
Time:     T1        T2        T3        T4        T5        T6
         +-------+ +-------+ +-------+
Branch:  | Fetch | |Decode | |Execute| (PC updated)
         +-------+ +-------+ +-------+
Old next:          | Fetch | | FLUSH |
                   +-------+ +-------+
Target:                                | Fetch | |Decode | |Execute|
                                       +-------+ +-------+ +-------+
```

This pipeline flush costs 1–2 extra cycles per branch.  In our delay loop, the `bne .Delay_MS_Loop` branch executes thousands of times, and each taken branch costs these extra cycles.

## The Cortex-M33 Execution Model

The Cortex-M33 has a 3-stage pipeline optimized for:

- **Branch prediction** — a simple predictor reduces the flush penalty for backward branches (loops).
- **Single-cycle multiply** — the `mul` instruction completes in one cycle.
- **Dual-issue** — certain pairs of 16-bit instructions can execute in parallel (implementation-dependent).

The pipeline depth means the PC reads as "current instruction address + 4" — something to be aware of when debugging.

## Clock Speed

Our firmware runs the processor from the XOSC at approximately 14.5 MHz (12 MHz crystal, 1–15 MHz range configured).  At 14.5 MHz each clock cycle is approximately 69 nanoseconds.

The delay loop in delay.s uses this clock speed to calibrate timing:

```asm
  ldr   r4, =3600                                // loops per MS based on 14.5MHz clock
  mul   r5, r0, r4                               // total loop iterations
.Delay_MS_Loop:
  subs  r5, r5, #1                               // 1 cycle: decrement
  bne   .Delay_MS_Loop                           // 1-2 cycles: branch if not zero
```

Each loop iteration takes approximately 2–3 cycles.  At 14.5 MHz, 3600 iterations ≈ 1 ms.

## Summary

- The Cortex-M33 has a 3-stage pipeline: fetch, decode, execute.
- The pipeline allows one instruction to complete per cycle in steady state.
- Branches flush the pipeline, costing 1–2 extra cycles.
- The PC reads as current instruction address + 4 due to the pipeline.
- Our XOSC runs at approximately 14.5 MHz — each cycle is about 69 ns.
- The delay loop calibration (3600 iterations per ms) accounts for the pipeline behavior at this clock speed.
