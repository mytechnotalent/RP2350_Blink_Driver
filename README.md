## FREE Embedded Hacking Course [HERE](https://github.com/mytechnotalent/Embedded-Hacking)
### VIDEO PROMO [HERE](https://www.youtube.com/watch?v=aD7X9sXirF8)

<br>

# RP2350 Blink Driver
An RP2350 Blink driver written entirely in ARM Assembler.

<br>

# Install ARM Toolchain (Windows / RP2350 Cortex-M33)
Official Raspberry Pi guidance for RP2350 ARM recommends the Arm GNU Toolchain from developer.arm.com.

## Official References
- Raspberry Pi Pico SDK quick start: [HERE](https://github.com/raspberrypi/pico-sdk#quick-start-your-own-project)
- Tool downloads (official): [HERE](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

## Install (PowerShell)
```powershell
$url = "https://developer.arm.com/-/media/Files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-mingw-w64-x86_64-arm-none-eabi.zip"
$zipPath = "$env:TEMP\arm-toolchain-15-x64-win.zip"
$extractPath = "$env:TEMP\arm-extract"
$dest = "$HOME\arm-toolchain-15"

Invoke-WebRequest -Uri $url -OutFile $zipPath
Expand-Archive -LiteralPath $zipPath -DestinationPath $extractPath -Force
Move-Item "$extractPath\arm-gnu-toolchain-*" $dest -Force
Get-ChildItem -Path $dest | Select-Object Name
```

## Add Toolchain To User PATH (PowerShell)
```powershell
$toolBin = "$HOME\arm-toolchain-15\bin"
$currentUserPath = [Environment]::GetEnvironmentVariable("Path", "User")
if ($currentUserPath -notlike "*$toolBin*") {
  [Environment]::SetEnvironmentVariable("Path", "$currentUserPath;$toolBin", "User")
}
```

Close and reopen your terminal after updating PATH.

## Verify Toolchain
```powershell
arm-none-eabi-as --version
arm-none-eabi-ld --version
arm-none-eabi-objcopy --version
```

## Build This Project
```powershell
.\build.bat
```

## LED Wiring (Pico 2 Target)
- GP16 (Pin 21) → 330 Ω resistor → LED anode
- LED cathode → GND (Pin 23)

<br>

# Hardware
## Raspberry Pi Pico 2 w/ Header [BUY](https://www.pishop.us/product/raspberry-pi-pico-2-with-header)
## USB A-Male to USB Micro-B Cable [BUY](https://www.pishop.us/product/usb-a-male-to-usb-micro-b-cable-6-inches)
## Raspberry Pi Pico Debug Probe [BUY](https://www.pishop.us/product/raspberry-pi-debug-probe)
## Complete Component Kit for Raspberry Pi [BUY](https://www.pishop.us/product/complete-component-kit-for-raspberry-pi)
## 10pc 25v 1000uF Capacitor [BUY](https://www.amazon.com/Cionyce-Capacitor-Electrolytic-CapacitorsMicrowave/dp/B0B63CCQ2N?th=1)
### 10% PiShop DISCOUNT CODE - KVPE_HS320548_10PC

<br>

# Build
```
.\build.bat
```

<br>

# Clean
```
.\clean.bat
```

<br>

# Tutorial

## Part I — Foundations (Chapters 1–6)

### [Chapter 1: What Is a Computer?](TUTORIAL/CHAPTER-01.md)
- [Introduction](TUTORIAL/CHAPTER-01.md#introduction)
- [The Fetch-Decode-Execute Cycle](TUTORIAL/CHAPTER-01.md#the-fetch-decode-execute-cycle)
- [The Three Core Components](TUTORIAL/CHAPTER-01.md#the-three-core-components)
- [Microcontroller vs Desktop Computer](TUTORIAL/CHAPTER-01.md#microcontroller-vs-desktop-computer)
- [What Is RP2350?](TUTORIAL/CHAPTER-01.md#what-is-rp2350)
- [What Is ARM Cortex-M33?](TUTORIAL/CHAPTER-01.md#what-is-arm-cortex-m33)
- [What Is Assembly Language?](TUTORIAL/CHAPTER-01.md#what-is-assembly-language)
- [Why Learn Assembly?](TUTORIAL/CHAPTER-01.md#why-learn-assembly)
- [What We Will Build](TUTORIAL/CHAPTER-01.md#what-we-will-build)
- [Summary](TUTORIAL/CHAPTER-01.md#summary)

### [Chapter 2: Number Systems — Binary, Hexadecimal, and Decimal](TUTORIAL/CHAPTER-02.md)
- [Introduction](TUTORIAL/CHAPTER-02.md#introduction)
- [Decimal — Base 10](TUTORIAL/CHAPTER-02.md#decimal--base-10)
- [Binary — Base 2](TUTORIAL/CHAPTER-02.md#binary--base-2)
- [Hexadecimal — Base 16](TUTORIAL/CHAPTER-02.md#hexadecimal--base-16)
- [The 0x Prefix](TUTORIAL/CHAPTER-02.md#the-0x-prefix)
- [Bit Numbering](TUTORIAL/CHAPTER-02.md#bit-numbering)
- [Common Bit Patterns in Our Firmware](TUTORIAL/CHAPTER-02.md#common-bit-patterns-in-our-firmware)
- [Two's Complement — Signed Numbers](TUTORIAL/CHAPTER-02.md#twos-complement--signed-numbers)
- [Data Sizes on ARM Cortex-M33](TUTORIAL/CHAPTER-02.md#data-sizes-on-arm-cortex-m33)
- [Summary](TUTORIAL/CHAPTER-02.md#summary)

### [Chapter 3: Memory — Addresses, Bytes, Words, and Endianness](TUTORIAL/CHAPTER-03.md)
- [Introduction](TUTORIAL/CHAPTER-03.md#introduction)
- [The Address Space](TUTORIAL/CHAPTER-03.md#the-address-space)
- [Bytes, Halfwords, and Words](TUTORIAL/CHAPTER-03.md#bytes-halfwords-and-words)
- [Alignment](TUTORIAL/CHAPTER-03.md#alignment)
- [Little-Endian Byte Order](TUTORIAL/CHAPTER-03.md#little-endian-byte-order)
- [Memory-Mapped Registers](TUTORIAL/CHAPTER-03.md#memory-mapped-registers)
- [The Stack](TUTORIAL/CHAPTER-03.md#the-stack)
- [Flash Memory (XIP)](TUTORIAL/CHAPTER-03.md#flash-memory-xip)
- [SRAM](TUTORIAL/CHAPTER-03.md#sram)
- [Reading the Address Map](TUTORIAL/CHAPTER-03.md#reading-the-address-map)
- [Summary](TUTORIAL/CHAPTER-03.md#summary)

### [Chapter 4: What Is a Register?](TUTORIAL/CHAPTER-04.md)
- [Introduction](TUTORIAL/CHAPTER-04.md#introduction)
- [The ARM Cortex-M33 Register File](TUTORIAL/CHAPTER-04.md#the-arm-cortex-m33-register-file)
- [Registers r0-r3: Arguments and Scratch](TUTORIAL/CHAPTER-04.md#registers-r0-r3-arguments-and-scratch)
- [Registers r4-r11: Callee-Saved](TUTORIAL/CHAPTER-04.md#registers-r4-r11-callee-saved)
- [Register r12 (IP): Intra-Procedure Scratch](TUTORIAL/CHAPTER-04.md#register-r12-ip-intra-procedure-scratch)
- [Register r13 (SP): Stack Pointer](TUTORIAL/CHAPTER-04.md#register-r13-sp-stack-pointer)
- [Register r14 (LR): Link Register](TUTORIAL/CHAPTER-04.md#register-r14-lr-link-register)
- [Register r15 (PC): Program Counter](TUTORIAL/CHAPTER-04.md#register-r15-pc-program-counter)
- [Special Registers](TUTORIAL/CHAPTER-04.md#special-registers)
- [The Program Status Register (xPSR)](TUTORIAL/CHAPTER-04.md#the-program-status-register-xpsr)
- [Register Usage in Our Firmware](TUTORIAL/CHAPTER-04.md#register-usage-in-our-firmware)
- [Summary](TUTORIAL/CHAPTER-04.md#summary)

### [Chapter 5: Load-Store Architecture — How ARM Accesses Memory](TUTORIAL/CHAPTER-05.md)
- [Introduction](TUTORIAL/CHAPTER-05.md#introduction)
- [Why Load-Store?](TUTORIAL/CHAPTER-05.md#why-load-store)
- [The Load Instruction: ldr](TUTORIAL/CHAPTER-05.md#the-load-instruction-ldr)
- [The Store Instruction: str](TUTORIAL/CHAPTER-05.md#the-store-instruction-str)
- [The Load-Modify-Store Pattern](TUTORIAL/CHAPTER-05.md#the-load-modify-store-pattern)
- [Byte and Halfword Access](TUTORIAL/CHAPTER-05.md#byte-and-halfword-access)
- [Push and Pop](TUTORIAL/CHAPTER-05.md#push-and-pop)
- [Memory Access in Our Firmware](TUTORIAL/CHAPTER-05.md#memory-access-in-our-firmware)
- [Summary](TUTORIAL/CHAPTER-05.md#summary)

### [Chapter 6: The Fetch-Decode-Execute Cycle in Detail](TUTORIAL/CHAPTER-06.md)
- [Introduction](TUTORIAL/CHAPTER-06.md#introduction)
- [The Three Stages](TUTORIAL/CHAPTER-06.md#the-three-stages)
- [The Pipeline](TUTORIAL/CHAPTER-06.md#the-pipeline)
- [A Concrete Example](TUTORIAL/CHAPTER-06.md#a-concrete-example)
- [How Branch Instructions Affect the Pipeline](TUTORIAL/CHAPTER-06.md#how-branch-instructions-affect-the-pipeline)
- [The Cortex-M33 Execution Model](TUTORIAL/CHAPTER-06.md#the-cortex-m33-execution-model)
- [Clock Speed](TUTORIAL/CHAPTER-06.md#clock-speed)
- [Summary](TUTORIAL/CHAPTER-06.md#summary)

## Part II — The ARM Instruction Set (Chapters 7–12)

### [Chapter 7: ARM Cortex-M33 ISA Overview](TUTORIAL/CHAPTER-07.md)
- [Introduction](TUTORIAL/CHAPTER-07.md#introduction)
- [The ARM Design Philosophy](TUTORIAL/CHAPTER-07.md#the-arm-design-philosophy)
- [Thumb-2 Instruction Encoding](TUTORIAL/CHAPTER-07.md#thumb-2-instruction-encoding)
- [Instruction Categories](TUTORIAL/CHAPTER-07.md#instruction-categories)
- [Instruction Encoding Formats](TUTORIAL/CHAPTER-07.md#instruction-encoding-formats)
- [Instructions Used in Our Firmware](TUTORIAL/CHAPTER-07.md#instructions-used-in-our-firmware)
- [Summary](TUTORIAL/CHAPTER-07.md#summary)

### [Chapter 8: ARM Immediate and Move Instructions](TUTORIAL/CHAPTER-08.md)
- [Introduction](TUTORIAL/CHAPTER-08.md#introduction)
- [The mov Instruction](TUTORIAL/CHAPTER-08.md#the-mov-instruction)
- [The ldr Rd, =value Pseudo-Instruction](TUTORIAL/CHAPTER-08.md#the-ldr-rd-value-pseudo-instruction)
- [Literal Pool Placement](TUTORIAL/CHAPTER-08.md#literal-pool-placement)
- [Our Firmware's Use of Immediates](TUTORIAL/CHAPTER-08.md#our-firmwares-use-of-immediates)
- [Why Not Always Use mov?](TUTORIAL/CHAPTER-08.md#why-not-always-use-mov)
- [Summary](TUTORIAL/CHAPTER-08.md#summary)

### [Chapter 9: ARM Arithmetic and Logic Instructions](TUTORIAL/CHAPTER-09.md)
- [Introduction](TUTORIAL/CHAPTER-09.md#introduction)
- [Arithmetic Instructions](TUTORIAL/CHAPTER-09.md#arithmetic-instructions)
- [Logic Instructions](TUTORIAL/CHAPTER-09.md#logic-instructions)
- [The APSR Flags](TUTORIAL/CHAPTER-09.md#the-apsr-flags)
- [Read-Modify-Write Pattern](TUTORIAL/CHAPTER-09.md#read-modify-write-pattern)
- [Summary](TUTORIAL/CHAPTER-09.md#summary)

### [Chapter 10: ARM Memory Access Instructions](TUTORIAL/CHAPTER-10.md)
- [Introduction](TUTORIAL/CHAPTER-10.md#introduction)
- [ldr — Load Register](TUTORIAL/CHAPTER-10.md#ldr--load-register)
- [str — Store Register](TUTORIAL/CHAPTER-10.md#str--store-register)
- [push and pop — Stack Operations](TUTORIAL/CHAPTER-10.md#push-and-pop--stack-operations)
- [Memory Map and Peripheral Access](TUTORIAL/CHAPTER-10.md#memory-map-and-peripheral-access)
- [Alignment Requirements](TUTORIAL/CHAPTER-10.md#alignment-requirements)
- [msr and mrs — Special Register Access](TUTORIAL/CHAPTER-10.md#msr-and-mrs--special-register-access)
- [Summary](TUTORIAL/CHAPTER-10.md#summary)

### [Chapter 11: ARM Branch Instructions](TUTORIAL/CHAPTER-11.md)
- [Introduction](TUTORIAL/CHAPTER-11.md#introduction)
- [Unconditional Branches](TUTORIAL/CHAPTER-11.md#unconditional-branches)
- [Conditional Branches](TUTORIAL/CHAPTER-11.md#conditional-branches)
- [Branch Encoding and Range](TUTORIAL/CHAPTER-11.md#branch-encoding-and-range)
- [Condition Codes](TUTORIAL/CHAPTER-11.md#condition-codes)
- [Polling Loops](TUTORIAL/CHAPTER-11.md#polling-loops)
- [Summary](TUTORIAL/CHAPTER-11.md#summary)

### [Chapter 12: ARM Calls, Returns, and the Stack Frame](TUTORIAL/CHAPTER-12.md)
- [Introduction](TUTORIAL/CHAPTER-12.md#introduction)
- [The Link Register](TUTORIAL/CHAPTER-12.md#the-link-register)
- [Leaf Functions vs. Non-Leaf Functions](TUTORIAL/CHAPTER-12.md#leaf-functions-vs-non-leaf-functions)
- [The Call Chain](TUTORIAL/CHAPTER-12.md#the-call-chain)
- [The Stack Frame](TUTORIAL/CHAPTER-12.md#the-stack-frame)
- [Nested Calls](TUTORIAL/CHAPTER-12.md#nested-calls)
- [The ARM Calling Convention (AAPCS)](TUTORIAL/CHAPTER-12.md#the-arm-calling-convention-aapcs)
- [Parameter Passing Example](TUTORIAL/CHAPTER-12.md#parameter-passing-example)
- [Reset_Handler: A Special Case](TUTORIAL/CHAPTER-12.md#reset_handler-a-special-case)
- [Summary](TUTORIAL/CHAPTER-12.md#summary)

## Part III — Assembly Programming (Chapters 13–17)

### [Chapter 13: Assembler Directives](TUTORIAL/CHAPTER-13.md)
- [Introduction](TUTORIAL/CHAPTER-13.md#introduction)
- [Syntax and Instruction Set Directives](TUTORIAL/CHAPTER-13.md#syntax-and-instruction-set-directives)
- [Section Directives](TUTORIAL/CHAPTER-13.md#section-directives)
- [Symbol Directives](TUTORIAL/CHAPTER-13.md#symbol-directives)
- [Data Directives](TUTORIAL/CHAPTER-13.md#data-directives)
- [Function Directives](TUTORIAL/CHAPTER-13.md#function-directives)
- [Include Directive](TUTORIAL/CHAPTER-13.md#include-directive)
- [Summary](TUTORIAL/CHAPTER-13.md#summary)

### [Chapter 14: Labels, Symbols, and the Symbol Table](TUTORIAL/CHAPTER-14.md)
- [Introduction](TUTORIAL/CHAPTER-14.md#introduction)
- [Defining Labels](TUTORIAL/CHAPTER-14.md#defining-labels)
- [Global vs. Local Labels](TUTORIAL/CHAPTER-14.md#global-vs-local-labels)
- [Label Types in Our Firmware](TUTORIAL/CHAPTER-14.md#label-types-in-our-firmware)
- [The Symbol Table](TUTORIAL/CHAPTER-14.md#the-symbol-table)
- [.equ Constants in the Symbol Table](TUTORIAL/CHAPTER-14.md#equ-constants-in-the-symbol-table)
- [Cross-File Resolution](TUTORIAL/CHAPTER-14.md#cross-file-resolution)
- [The Linking Process](TUTORIAL/CHAPTER-14.md#the-linking-process)
- [The Thumb Bit](TUTORIAL/CHAPTER-14.md#the-thumb-bit)
- [Summary](TUTORIAL/CHAPTER-14.md#summary)

### [Chapter 15: Sections, Memory Layout, and the Linker Script](TUTORIAL/CHAPTER-15.md)
- [Introduction](TUTORIAL/CHAPTER-15.md#introduction)
- [What Are Sections?](TUTORIAL/CHAPTER-15.md#what-are-sections)
- [Our Linker Script](TUTORIAL/CHAPTER-15.md#our-linker-script)
- [Section-by-Section Walkthrough](TUTORIAL/CHAPTER-15.md#section-by-section-walkthrough)
- [The Final Memory Map](TUTORIAL/CHAPTER-15.md#the-final-memory-map)
- [ENTRY Directive](TUTORIAL/CHAPTER-15.md#entry-directive)
- [Symbol Exports from the Linker Script](TUTORIAL/CHAPTER-15.md#symbol-exports-from-the-linker-script)
- [Why Sections Matter](TUTORIAL/CHAPTER-15.md#why-sections-matter)
- [Summary](TUTORIAL/CHAPTER-15.md#summary)

### [Chapter 16: System Registers and Coprocessor Interface](TUTORIAL/CHAPTER-16.md)
- [Introduction](TUTORIAL/CHAPTER-16.md#introduction)
- [Special-Purpose Registers](TUTORIAL/CHAPTER-16.md#special-purpose-registers)
- [Memory-Mapped System Registers](TUTORIAL/CHAPTER-16.md#memory-mapped-system-registers)
- [Barrier Instructions](TUTORIAL/CHAPTER-16.md#barrier-instructions)
- [The Coprocessor Interface](TUTORIAL/CHAPTER-16.md#the-coprocessor-interface)
- [The Complete Coprocessor Flow](TUTORIAL/CHAPTER-16.md#the-complete-coprocessor-flow)
- [Summary](TUTORIAL/CHAPTER-16.md#summary)

### [Chapter 17: Bit Manipulation Patterns](TUTORIAL/CHAPTER-17.md)
- [Introduction](TUTORIAL/CHAPTER-17.md#introduction)
- [The Fundamental Operations](TUTORIAL/CHAPTER-17.md#the-fundamental-operations)
- [Set a Single Bit](TUTORIAL/CHAPTER-17.md#set-a-single-bit)
- [Clear a Single Bit](TUTORIAL/CHAPTER-17.md#clear-a-single-bit)
- [Clear a Multi-Bit Field](TUTORIAL/CHAPTER-17.md#clear-a-multi-bit-field)
- [Test a Bit](TUTORIAL/CHAPTER-17.md#test-a-bit)
- [Combined Patterns](TUTORIAL/CHAPTER-17.md#combined-patterns)
- [Bit Fields in Our Registers](TUTORIAL/CHAPTER-17.md#bit-fields-in-our-registers)
- [Why This Matters](TUTORIAL/CHAPTER-17.md#why-this-matters)
- [Summary](TUTORIAL/CHAPTER-17.md#summary)

## Part IV — RP2350 Hardware (Chapter 18)

### [Chapter 18: RP2350 Hardware Architecture](TUTORIAL/CHAPTER-18.md)
- [Introduction](TUTORIAL/CHAPTER-18.md#introduction)
- [RP2350 Block Diagram](TUTORIAL/CHAPTER-18.md#rp2350-block-diagram)
- [Memory Map](TUTORIAL/CHAPTER-18.md#memory-map)
- [Crystal Oscillator (XOSC)](TUTORIAL/CHAPTER-18.md#crystal-oscillator-xosc)
- [Clock System](TUTORIAL/CHAPTER-18.md#clock-system)
- [Reset Controller](TUTORIAL/CHAPTER-18.md#reset-controller)
- [GPIO Architecture](TUTORIAL/CHAPTER-18.md#gpio-architecture)
- [GPIO16 and the LED](TUTORIAL/CHAPTER-18.md#gpio16-and-the-led)
- [Boot Sequence](TUTORIAL/CHAPTER-18.md#boot-sequence)
- [Summary](TUTORIAL/CHAPTER-18.md#summary)

## Part V — Build System (Chapters 19–20)

### [Chapter 19: The Linker Script](TUTORIAL/CHAPTER-19.md)
- [Introduction](TUTORIAL/CHAPTER-19.md#introduction)
- [Entry Point](TUTORIAL/CHAPTER-19.md#entry-point)
- [Memory Constants](TUTORIAL/CHAPTER-19.md#memory-constants)
- [Memory Regions](TUTORIAL/CHAPTER-19.md#memory-regions)
- [Program Headers](TUTORIAL/CHAPTER-19.md#program-headers)
- [Section Placement](TUTORIAL/CHAPTER-19.md#section-placement)
- [The Resulting Memory Layout](TUTORIAL/CHAPTER-19.md#the-resulting-memory-layout)
- [Summary](TUTORIAL/CHAPTER-19.md#summary)

### [Chapter 20: The Build System](TUTORIAL/CHAPTER-20.md)
- [Introduction](TUTORIAL/CHAPTER-20.md#introduction)
- [The Build Pipeline](TUTORIAL/CHAPTER-20.md#the-build-pipeline)
- [Stage 1: Assembly](TUTORIAL/CHAPTER-20.md#stage-1-assembly)
- [Stage 2: Linking](TUTORIAL/CHAPTER-20.md#stage-2-linking)
- [Stage 3: Binary Extraction](TUTORIAL/CHAPTER-20.md#stage-3-binary-extraction)
- [Stage 4: UF2 Conversion](TUTORIAL/CHAPTER-20.md#stage-4-uf2-conversion)
- [Error Handling](TUTORIAL/CHAPTER-20.md#error-handling)
- [Flashing the Firmware](TUTORIAL/CHAPTER-20.md#flashing-the-firmware)
- [The Clean Script](TUTORIAL/CHAPTER-20.md#the-clean-script)
- [Summary](TUTORIAL/CHAPTER-20.md#summary)

## Part VI — Source Code Walkthroughs (Chapters 21–29)

### [Chapter 21: image_def.s — The PICOBIN Boot Block](TUTORIAL/CHAPTER-21.md)
- [Introduction](TUTORIAL/CHAPTER-21.md#introduction)
- [Complete Source Code](TUTORIAL/CHAPTER-21.md#complete-source-code)
- [Section Placement](TUTORIAL/CHAPTER-21.md#section-placement)
- [Block Structure](TUTORIAL/CHAPTER-21.md#block-structure)
- [Byte-by-Byte Analysis](TUTORIAL/CHAPTER-21.md#byte-by-byte-analysis)
- [Why This Matters](TUTORIAL/CHAPTER-21.md#why-this-matters)
- [Summary](TUTORIAL/CHAPTER-21.md#summary)

### [Chapter 22: constants.s — Memory Addresses and Constants](TUTORIAL/CHAPTER-22.md)
- [Introduction](TUTORIAL/CHAPTER-22.md#introduction)
- [Complete Source Code](TUTORIAL/CHAPTER-22.md#complete-source-code)
- [Preamble](TUTORIAL/CHAPTER-22.md#preamble)
- [Stack Constants](TUTORIAL/CHAPTER-22.md#stack-constants)
- [Crystal Oscillator Constants](TUTORIAL/CHAPTER-22.md#crystal-oscillator-constants)
- [System Registers](TUTORIAL/CHAPTER-22.md#system-registers)
- [Clock Constants](TUTORIAL/CHAPTER-22.md#clock-constants)
- [Reset Controller Constants](TUTORIAL/CHAPTER-22.md#reset-controller-constants)
- [GPIO Constants](TUTORIAL/CHAPTER-22.md#gpio-constants)
- [How .include Works](TUTORIAL/CHAPTER-22.md#how-include-works)
- [Design Principle](TUTORIAL/CHAPTER-22.md#design-principle)
- [Summary](TUTORIAL/CHAPTER-22.md#summary)

### [Chapter 23: vector_table.s and stack.s — Boot Foundation](TUTORIAL/CHAPTER-23.md)
- [Introduction](TUTORIAL/CHAPTER-23.md#introduction)
- [vector_table.s — Complete Source Code](TUTORIAL/CHAPTER-23.md#vector_tables--complete-source-code)
- [stack.s — Complete Source Code](TUTORIAL/CHAPTER-23.md#stacks--complete-source-code)
- [The Boot Sequence](TUTORIAL/CHAPTER-23.md#the-boot-sequence)
- [Summary](TUTORIAL/CHAPTER-23.md#summary)

### [Chapter 24: reset_handler.s — The Boot Sequence](TUTORIAL/CHAPTER-24.md)
- [Introduction](TUTORIAL/CHAPTER-24.md#introduction)
- [Complete Source Code](TUTORIAL/CHAPTER-24.md#complete-source-code)
- [Symbol Metadata](TUTORIAL/CHAPTER-24.md#symbol-metadata)
- [The Initialization Sequence](TUTORIAL/CHAPTER-24.md#the-initialization-sequence)
- [Dependency Chain](TUTORIAL/CHAPTER-24.md#dependency-chain)
- [Reset_Handler Is Not a Normal Function](TUTORIAL/CHAPTER-24.md#reset_handler-is-not-a-normal-function)
- [Summary](TUTORIAL/CHAPTER-24.md#summary)

### [Chapter 25: xosc.s — Crystal Oscillator and Clock Configuration](TUTORIAL/CHAPTER-25.md)
- [Introduction](TUTORIAL/CHAPTER-25.md#introduction)
- [Complete Source Code](TUTORIAL/CHAPTER-25.md#complete-source-code)
- [Init_XOSC — Line-by-Line](TUTORIAL/CHAPTER-25.md#init_xosc--line-by-line)
- [Enable_XOSC_Peri_Clock — Line-by-Line](TUTORIAL/CHAPTER-25.md#enable_xosc_peri_clock--line-by-line)
- [Both Functions Are Leaf Functions](TUTORIAL/CHAPTER-25.md#both-functions-are-leaf-functions)
- [Clock Domain After Configuration](TUTORIAL/CHAPTER-25.md#clock-domain-after-configuration)
- [Summary](TUTORIAL/CHAPTER-25.md#summary)

### [Chapter 26: reset.s — Releasing Peripherals from Reset](TUTORIAL/CHAPTER-26.md)
- [Introduction](TUTORIAL/CHAPTER-26.md#introduction)
- [Complete Source Code](TUTORIAL/CHAPTER-26.md#complete-source-code)
- [Line-by-Line Walkthrough](TUTORIAL/CHAPTER-26.md#line-by-line-walkthrough)
- [Reset Register Bit Map](TUTORIAL/CHAPTER-26.md#reset-register-bit-map)
- [Why the Wait Is Necessary](TUTORIAL/CHAPTER-26.md#why-the-wait-is-necessary)
- [Function Characteristics](TUTORIAL/CHAPTER-26.md#function-characteristics)
- [Local Labels](TUTORIAL/CHAPTER-26.md#local-labels)
- [Summary](TUTORIAL/CHAPTER-26.md#summary)

### [Chapter 27: gpio.s Part 1 — GPIO_Config](TUTORIAL/CHAPTER-27.md)
- [Introduction](TUTORIAL/CHAPTER-27.md#introduction)
- [Complete Source Code — GPIO_Config](TUTORIAL/CHAPTER-27.md#complete-source-code--gpio_config)
- [Function Signature](TUTORIAL/CHAPTER-27.md#function-signature)
- [Register Save and Restore](TUTORIAL/CHAPTER-27.md#register-save-and-restore)
- [Phase 1: Pad Configuration](TUTORIAL/CHAPTER-27.md#phase-1-pad-configuration)
- [Phase 2: Function Select](TUTORIAL/CHAPTER-27.md#phase-2-function-select)
- [Phase 3: Enable Output via Coprocessor](TUTORIAL/CHAPTER-27.md#phase-3-enable-output-via-coprocessor)
- [The Three Layers of GPIO Configuration](TUTORIAL/CHAPTER-27.md#the-three-layers-of-gpio-configuration)
- [How main.s Calls GPIO_Config](TUTORIAL/CHAPTER-27.md#how-mains-calls-gpio_config)
- [Summary](TUTORIAL/CHAPTER-27.md#summary)

### [Chapter 28: gpio.s Part 2, delay.s, and coprocessor.s — Output Control and Timing](TUTORIAL/CHAPTER-28.md)
- [Introduction](TUTORIAL/CHAPTER-28.md#introduction)
- [GPIO_Set — Drive Pin High](TUTORIAL/CHAPTER-28.md#gpio_set--drive-pin-high)
- [GPIO_Clear — Drive Pin Low](TUTORIAL/CHAPTER-28.md#gpio_clear--drive-pin-low)
- [Delay_MS — Millisecond Delay](TUTORIAL/CHAPTER-28.md#delay_ms--millisecond-delay)
- [Enable_Coprocessor — CP0 Access](TUTORIAL/CHAPTER-28.md#enable_coprocessor--cp0-access)
- [The Runtime Flow](TUTORIAL/CHAPTER-28.md#the-runtime-flow)
- [Summary](TUTORIAL/CHAPTER-28.md#summary)

### [Chapter 29: main.s — The Blink Loop](TUTORIAL/CHAPTER-29.md)
- [Introduction](TUTORIAL/CHAPTER-29.md#introduction)
- [Complete Source Code](TUTORIAL/CHAPTER-29.md#complete-source-code)
- [Function Metadata](TUTORIAL/CHAPTER-29.md#function-metadata)
- [Register Save](TUTORIAL/CHAPTER-29.md#register-save)
- [GPIO16 Configuration (One-Time Setup)](TUTORIAL/CHAPTER-29.md#gpio16-configuration-one-time-setup)
- [The Infinite Blink Loop](TUTORIAL/CHAPTER-29.md#the-infinite-blink-loop)
- [Unreachable Code](TUTORIAL/CHAPTER-29.md#unreachable-code)
- [Data Sections](TUTORIAL/CHAPTER-29.md#data-sections)
- [Complete Execution Flow](TUTORIAL/CHAPTER-29.md#complete-execution-flow)
- [Summary](TUTORIAL/CHAPTER-29.md#summary)

## Part VII — Full Integration (Chapter 30)

### [Chapter 30: Full Integration — From Source to Blinking LED](TUTORIAL/CHAPTER-30.md)
- [Introduction](TUTORIAL/CHAPTER-30.md#introduction)
- [Step 1: Verify the Toolchain](TUTORIAL/CHAPTER-30.md#step-1-verify-the-toolchain)
- [Step 2: Build the Firmware](TUTORIAL/CHAPTER-30.md#step-2-build-the-firmware)
- [Step 3: Verify Build Artifacts](TUTORIAL/CHAPTER-30.md#step-3-verify-build-artifacts)
- [Step 4: Wire the Hardware](TUTORIAL/CHAPTER-30.md#step-4-wire-the-hardware)
- [Step 5: Flash the Firmware](TUTORIAL/CHAPTER-30.md#step-5-flash-the-firmware)
- [Step 6: Verify Operation](TUTORIAL/CHAPTER-30.md#step-6-verify-operation)
- [Troubleshooting](TUTORIAL/CHAPTER-30.md#troubleshooting)
- [The Complete Architecture](TUTORIAL/CHAPTER-30.md#the-complete-architecture)
- [What You Have Learned](TUTORIAL/CHAPTER-30.md#what-you-have-learned)
- [Summary](TUTORIAL/CHAPTER-30.md#summary)

<br>

# main.s Code
```
/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * RP2350 Bare-Metal Blink Main Application.
 * 
 * BRIEF:
 * Main application entry point for RP2350 blink driver. Contains the
 * main loop that toggles GPIO16 to blink an LED.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 2, 2025
 * UPDATE DATE: November 27, 2025
 */

.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

/**
 * Initialize the .text section. 
 * The .text section contains executable code.
 */
.section .text                                   // code section
.align 2                                         // align to 4-byte boundary

/**
 * @brief   Main application entry point.
 *
 * @details Implements the infinite blink loop.
 *
 * @param   None
 * @retval  None
 */
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

/**
 * Test data and constants.
 * The .rodata section is used for constants and static data.
 */
.section .rodata                                 // read-only data section

/**
 * Initialized global data.
 * The .data section is used for initialized global or static variables.
 */
.section .data                                   // data section

/**
 * Uninitialized global data.
 * The .bss section is used for uninitialized global or static variables.
 */
.section .bss                                    // BSS section
```

<br>

# License
[Apache License 2.0](https://github.com/mytechnotalent/RP2350_Blink_Driver/blob/main/LICENSE)
