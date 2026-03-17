# Chapter 1: What Is a Computer?

## Introduction

This chapter presents the fundamental model of computation that every computer — from a warehouse-scale server to the RP2350 microcontroller on your desk — follows.  By the end you will understand the three core components of any computer, the cycle that drives all program execution, and why we chose the RP2350 and ARM assembly language for this project.

## The Fetch-Decode-Execute Cycle

Every processor, regardless of its complexity, performs the same three-step loop for as long as it is powered on:

1. **Fetch** — read the next instruction from memory at the address held in the Program Counter (PC).
2. **Decode** — interpret the binary pattern of that instruction to determine the operation and its operands.
3. **Execute** — carry out the operation (arithmetic, memory access, branch, etc.) and update any affected registers or memory.

After execution the PC advances to the next instruction (or to a branch target) and the cycle repeats.  This loop is the heartbeat of every program.

```
+-------+     +--------+     +---------+
| Fetch | --> | Decode | --> | Execute |
+-------+     +--------+     +---------+
     ^                             |
     +-----------------------------+
```

## The Three Core Components

A computer is built from three fundamental components:

| Component | Purpose |
|-----------|---------|
| **Processor (CPU)** | Executes instructions — the fetch-decode-execute engine |
| **Memory** | Stores instructions and data at addressable locations |
| **I/O (Input/Output)** | Connects the processor to the outside world — buttons, LEDs, serial ports, sensors |

Everything else — caches, buses, interrupt controllers — exists to make these three components work together efficiently.

## Microcontroller vs Desktop Computer

A desktop computer distributes these three components across separate chips on a motherboard.  A **microcontroller** integrates all three onto a single silicon die:

| Feature | Desktop | Microcontroller |
|---------|---------|-----------------|
| CPU | Separate chip (Intel, AMD) | On-chip core (ARM Cortex-M33) |
| Memory | External DRAM sticks | On-chip SRAM (520 KB) and external flash |
| I/O | Expansion cards, chipset | On-chip peripherals (GPIO, UART, SPI, I2C) |
| Clock speed | 3–5 GHz | 12–150 MHz |
| Operating system | Windows, Linux, macOS | Usually none (bare-metal) |

The RP2350 is a microcontroller.  We will program it without an operating system — our code is the only software running.

## What Is RP2350?

The RP2350 is a dual-architecture microcontroller designed by Raspberry Pi.  Key specifications:

- **Cores**: two ARM Cortex-M33 cores *and* two Hazard3 RISC-V cores (selectable at boot)
- **SRAM**: 520 KB on-chip
- **Flash**: external, up to 32 MB via QSPI (execute-in-place)
- **Clock**: 12 MHz crystal oscillator, configurable PLLs up to 150 MHz
- **GPIO**: 30 general-purpose I/O pins with programmable function select
- **Peripherals**: 2× UART, 2× SPI, 2× I2C, 24 PWM channels, USB 1.1, ADC
- **Debug**: SWD via the Raspberry Pi Debug Probe
- **Security**: ARM TrustZone, secure boot, OTP memory

In this project we use the ARM Cortex-M33 core in secure mode.

## What Is ARM Cortex-M33?

ARM Cortex-M33 is a 32-bit processor core designed for microcontrollers.  It implements the ARMv8-M architecture with the Mainline Extension, which includes:

- **Thumb-2 instruction set** — a mix of 16-bit and 32-bit instructions for code density and performance
- **Hardware single-cycle multiply, divide**
- **Nested Vectored Interrupt Controller (NVIC)** — prioritized, low-latency interrupt handling
- **TrustZone** — hardware-enforced separation between secure and non-secure code
- **Optional FPU and DSP extensions** (not used in this project)

The Cortex-M33 has 16 general-purpose registers (r0–r15), a Program Status Register (xPSR), and several special-purpose registers.  We will explore every one in Chapter 4.

## What Is Assembly Language?

Assembly language is the human-readable form of machine code.  Each assembly instruction corresponds directly to one (or sometimes two) machine instructions that the processor executes.  There is no compiler making optimization decisions on your behalf — you control exactly what the hardware does.

In ARM Cortex-M33 assembly a typical instruction looks like:

```asm
  ldr   r0, =0x40028000                          // load GPIO base address
```

The assembler translates this into the exact binary encoding the processor fetches and executes.

## Why Learn Assembly?

1. **Total hardware control** — you decide every register load, every memory access, every branch.
2. **No hidden abstractions** — there is no runtime, no standard library, no heap allocator between you and the silicon.
3. **Debugging mastery** — when something breaks at the hardware level, assembly is the language the debugger speaks.
4. **Security analysis** — reverse engineering, vulnerability research, and exploit development all require reading assembly.
5. **Performance insight** — understanding assembly makes you a better programmer in every language.

## What We Will Build

Over the next 29 chapters we will build, explain, and test a complete bare-metal LED blink driver for the RP2350:

1. **Boot from flash** — the RP2350 boot ROM reads our image metadata block, finds the vector table, and jumps to our Reset_Handler.
2. **Initialize hardware** — configure the crystal oscillator, release GPIO from reset, enable the coprocessor, and configure GPIO16 as an output.
3. **Blink loop** — toggle GPIO16 high and low with a 500 ms delay, driving an LED on and off in an infinite loop.

Every single line of assembly will be explained.  By Chapter 30 you will understand the entire firmware from power-on to blinking LED.

## Summary

- A computer is a processor, memory, and I/O executing the fetch-decode-execute cycle.
- A microcontroller integrates all three on one chip.
- The RP2350 contains ARM Cortex-M33 cores running the Thumb-2 instruction set.
- Assembly language gives direct, one-to-one control over the hardware.
- We will build a complete bare-metal LED blink driver and explain every line.
