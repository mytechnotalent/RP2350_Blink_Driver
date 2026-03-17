# Chapter 30: Full Integration — From Source to Blinking LED

## Introduction

Across 29 chapters we have studied every concept, instruction, hardware block, and source file that makes up our blink driver.  This final chapter brings everything together: we build the firmware, wire the hardware, flash the chip, and verify that the LED blinks.  This is the complete journey from source code to a working embedded system.

## Step 1: Verify the Toolchain

Open PowerShell and confirm the ARM toolchain is installed:

```powershell
arm-none-eabi-as --version
arm-none-eabi-ld --version
arm-none-eabi-objcopy --version
```

All three must report version information.  If any command is not found, revisit the Install ARM Toolchain section of the README.

## Step 2: Build the Firmware

From the project root directory, run:

```powershell
.\build.bat
```

The build script:

1. **Assembles** 10 source files → 10 object files (.o)
2. **Links** all object files using linker.ld → blink.elf
3. **Extracts** raw binary → blink.bin
4. **Converts** to UF2 format → blink.uf2

A successful build prints:

```
Building...

=================================
SUCCESS! Created blink.uf2
=================================
```

If any step fails, the script stops with an error message identifying the failing stage.

## Step 3: Verify Build Artifacts

After a successful build, these files exist:

| File | Size (approx) | Purpose |
|------|---------------|---------|
| `*.o` (10 files) | ~1 KB each | Intermediate object files |
| `blink.elf` | ~10 KB | Linked binary with debug info |
| `blink.bin` | ~1 KB | Raw binary (flash content) |
| `blink.uf2` | ~2 KB | UF2 image for USB flashing |

## Step 4: Wire the Hardware

### Components Needed

| Component | Quantity |
|-----------|----------|
| Raspberry Pi Pico 2 (RP2350) | 1 |
| LED (any color) | 1 |
| 330 Ω resistor | 1 |
| Breadboard | 1 |
| Jumper wires | 2 |
| USB cable (Micro-B or USB-C) | 1 |

### Wiring Diagram

```
Pico 2 Pin 21 (GPIO16) ---[330 Ω]---[LED anode (+)]
                                      [LED cathode (-)]--- Pico 2 Pin 23 (GND)
```

### Physical Layout

```
+---------------------------+
|       Raspberry Pi        |
|         Pico 2            |
|                           |
|  Pin 21 (GPIO16) o--------+---[330Ω]---[>|]---+
|                           |                     |
|  Pin 23 (GND)    o--------+--------------------+
|                           |
+---------------------------+
```

**LED orientation matters:** The anode (longer leg) connects to the resistor side.  The cathode (shorter leg, flat side of the lens) connects to ground.

**Resistor value:** 330 Ω limits current to approximately 10 mA at 3.3V, which is safe for both the LED and the GPIO pin.  Values from 220 Ω to 1 kΩ work — lower values produce a brighter LED.

## Step 5: Flash the Firmware

### Method A: UF2 (USB Mass Storage)

1. Hold the **BOOTSEL** button on the Pico 2
2. Connect the USB cable to your computer
3. Release BOOTSEL — a drive named **RP2350** appears
4. Copy `blink.uf2` to the RP2350 drive
5. The drive disappears and the firmware starts running

### Method B: OpenOCD (Debug Probe)

If you have a CMSIS-DAP debug probe connected:

```powershell
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program blink.elf verify reset exit"
```

This programs the ELF file, verifies the flash contents, resets the chip, and exits.

### Method C: picotool

```powershell
picotool load blink.uf2 -f
```

This loads the UF2 file and forces a reboot.

## Step 6: Verify Operation

After flashing, the LED should blink:

```
Time:  0s    0.5s   1.0s   1.5s   2.0s   2.5s
LED:   ON    OFF    ON     OFF    ON     OFF
       |-----|------|------|------|------|
       500ms  500ms  500ms  500ms  500ms
```

The LED turns on for 500 ms, off for 500 ms, repeating indefinitely.  The blink frequency is 1 Hz.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| LED does not light | Wrong GPIO pin | Verify wiring to Pin 21 (GPIO16) |
| LED stays on constantly | LED wired to 3.3V instead of GPIO | Check wiring |
| LED is very dim | Resistor too large | Try a lower value (330Ω) |
| Build fails at assembly | Syntax error | Check error message for file and line |
| Build fails at linking | Undefined symbol | Verify all `.global` declarations |
| UF2 drive does not appear | BOOTSEL not held | Hold BOOTSEL before plugging USB |
| Chip does not boot | Missing PICOBIN block | Verify image_def.s and linker script |

## The Complete Architecture

```
+----------------------------------------------------------+
|                    Source Files                           |
|                                                          |
|  image_def.s   constants.s   vector_table.s   stack.s    |
|  reset_handler.s   xosc.s   reset.s   coprocessor.s     |
|  gpio.s   delay.s   main.s                              |
+----------------------------+-----------------------------+
                             |
                      [build.bat]
                             |
                      blink.uf2
                             |
                    [Flash to Pico 2]
                             |
              +-----------------------------+
              |         RP2350              |
              |                             |
              |  Boot ROM → PICOBIN check   |
              |         ↓                   |
              |  Reset_Handler              |
              |    Init_Stack               |
              |    Init_XOSC                |
              |    Enable_XOSC_Peri_Clock   |
              |    Init_Subsystem           |
              |    Enable_Coprocessor       |
              |         ↓                   |
              |  main                       |
              |    GPIO_Config(GPIO16)      |
              |    Loop:                    |
              |      GPIO_Set → Delay_MS    |
              |      GPIO_Clear → Delay_MS  |
              |      ↻                      |
              +-----------------------------+
                             |
                      GPIO16 Pin 21
                             |
                      [330Ω + LED]
                             |
                           GND
```

## What You Have Learned

Across these 30 chapters, you have built understanding from the ground up:

- **Chapters 1–6:** Computer fundamentals — binary, memory, registers, execution model
- **Chapters 7–12:** ARM Cortex-M33 instruction set — moves, arithmetic, logic, memory, branches, calls
- **Chapters 13–17:** Assembly programming — directives, symbols, sections, system registers, bit manipulation
- **Chapter 18:** RP2350 hardware — XOSC, clocks, resets, GPIO, SIO coprocessor
- **Chapters 19–20:** Build system — linker script and build pipeline
- **Chapters 21–29:** Source code walkthrough — every file, every function, every instruction

You now understand every byte of a working bare-metal firmware, from the PICOBIN boot block to the final `b .Loop` that keeps the LED blinking.

## Summary

- Build with `.\build.bat` to produce blink.uf2.
- Wire an LED with a 330 Ω resistor between GPIO16 (Pin 21) and GND (Pin 23).
- Flash via UF2 drag-and-drop, OpenOCD, or picotool.
- The LED blinks at 1 Hz: 500 ms on, 500 ms off.
- Every instruction in the firmware has been explained across the preceding 29 chapters.
