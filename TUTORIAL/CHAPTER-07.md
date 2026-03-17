# Chapter 7: ARM Cortex-M33 ISA Overview

## Introduction

The instruction set architecture (ISA) defines every operation the processor can perform.  The ARM Cortex-M33 implements the ARMv8-M Mainline architecture using the **Thumb-2** instruction set — a variable-length encoding that mixes 16-bit and 32-bit instructions for both code density and performance.  This chapter surveys the ISA at a high level and identifies every instruction our blink driver uses.

## The ARM Design Philosophy

ARM processors follow the RISC (Reduced Instruction Set Computer) philosophy:

- **Fixed-width instructions** (with the Thumb-2 exception of 16/32-bit mix)
- **Load-store architecture** — only `ldr`/`str` access memory
- **Large register file** — 16 general-purpose registers
- **Simple addressing modes** — base + offset
- **Conditional execution** — many instructions can be conditionally executed

The Thumb-2 extension adds 32-bit encodings to the original 16-bit Thumb set, giving access to the full ARM feature set without sacrificing code density.

## Thumb-2 Instruction Encoding

Every instruction is either 16 bits (2 bytes) or 32 bits (4 bytes):

| Type | Size | When Used |
|------|------|-----------|
| Narrow (Thumb) | 16-bit | Simple operations with low registers (r0–r7) |
| Wide (Thumb-2) | 32-bit | Complex operations, large immediates, high registers |

The assembler chooses the narrowest encoding that can represent the operation.  The `.syntax unified` directive tells the assembler to accept both widths transparently.

16-bit example:

```asm
  bx    lr                                       // 16-bit: return to caller
```

32-bit example:

```asm
  bic   r5, r5, #(1<<7)                          // 32-bit: clear bit 7
```

## Instruction Categories

All Cortex-M33 instructions fall into these categories:

| Category | Examples | Used in Our Firmware |
|----------|---------|---------------------|
| Data processing | `add`, `sub`, `orr`, `bic`, `tst`, `cmp`, `mul` | Yes |
| Load/store | `ldr`, `str`, `push`, `pop` | Yes |
| Branch | `b`, `bl`, `bx`, `beq`, `bne`, `ble` | Yes |
| Move | `mov`, `ldr Rd, =imm` | Yes |
| System | `msr`, `mrs`, `dsb`, `isb`, `mcrr` | Yes |
| Multiply | `mul` | Yes |

## Instruction Encoding Formats

Thumb-2 32-bit instructions use several encoding formats:

| Format | Description | Example |
|--------|-------------|---------|
| Data processing (register) | op Rd, Rn, Rm | `orr r1, r1, #(1<<11)` |
| Data processing (immediate) | op Rd, Rn, #imm | `bic r5, r5, #(1<<7)` |
| Load/store (immediate) | ldr/str Rt, [Rn, #imm] | `ldr r1, [r0]` |
| Branch | b/bl label | `bl GPIO_Set` |
| System | msr/mrs special, Rn | `msr MSP, r0` |
| Coprocessor | mcrr pN, opc, Rt, Rt2, CRm | `mcrr p0, #4, r2, r4, c4` |

## Instructions Used in Our Firmware

Here is the complete list of instructions appearing in our blink driver:

| Instruction | Category | Description |
|-------------|----------|-------------|
| `ldr Rd, =imm` | Pseudo-instruction | Load 32-bit constant into register |
| `ldr Rd, [Rn]` | Load | Load word from memory |
| `str Rd, [Rn]` | Store | Store word to memory |
| `push {regs}` | Store (multiple) | Push registers onto stack |
| `pop {regs}` | Load (multiple) | Pop registers from stack |
| `add Rd, Rn, Rm` | Arithmetic | Add two registers |
| `sub Rd, Rd, #imm` | Arithmetic | Subtract immediate |
| `subs Rd, Rd, #imm` | Arithmetic | Subtract and update flags |
| `mul Rd, Rn, Rm` | Multiply | Multiply two registers |
| `orr Rd, Rd, #imm` | Logic | Bitwise OR with immediate |
| `bic Rd, Rd, #imm` | Logic | Bit clear (AND with NOT imm) |
| `tst Rn, #imm` | Logic | Test bits (AND, discard, set flags) |
| `cmp Rn, #imm` | Comparison | Compare (subtract, discard, set flags) |
| `b label` | Branch | Unconditional branch |
| `bl label` | Branch | Branch with link (call) |
| `bx Rn` | Branch | Branch exchange (return) |
| `beq label` | Branch | Branch if equal (Z=1) |
| `bne label` | Branch | Branch if not equal (Z=0) |
| `ble label` | Branch | Branch if less or equal |
| `msr reg, Rn` | System | Write to special register |
| `dsb` | Barrier | Data synchronization barrier |
| `isb` | Barrier | Instruction synchronization barrier |
| `mcrr pN, opc, Rt, Rt2, CRm` | Coprocessor | Move to coprocessor from two registers |

## Summary

- The Cortex-M33 uses the Thumb-2 instruction set: a mix of 16-bit and 32-bit encodings.
- `.syntax unified` enables transparent use of both widths.
- Instructions are categorized into data processing, load/store, branch, system, and coprocessor.
- Our blink driver uses approximately 23 distinct instructions to achieve full GPIO control from bare metal.
