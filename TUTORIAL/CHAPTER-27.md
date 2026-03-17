# Chapter 27: gpio.s Part 1 — GPIO_Config

## Introduction

`GPIO_Config` is the most complex function in our firmware.  It configures a GPIO pin for output by modifying two separate register banks (pads and IO control), then enables the output driver through the SIO coprocessor.  This chapter walks through every instruction, explaining how the physical pin is connected to internal logic and prepared to drive an LED.

## Complete Source Code — GPIO_Config

```asm
.global GPIO_Config
.type GPIO_Config, %function
GPIO_Config:
.GPIO_Config_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Config_Modify_Pad:
  ldr   r4, =PADS_BANK0_BASE                     // load PADS_BANK0_BASE address
  add   r4, r4, r0                               // PADS_BANK0_BASE + PAD_OFFSET
  ldr   r5, [r4]                                 // read PAD_OFFSET value
  bic   r5, r5, #(1<<7)                          // clear OD bit
  orr   r5, r5, #(1<<6)                          // set IE bit
  bic   r5, r5, #(1<<8)                          // clear ISO bit
  str   r5, [r4]                                 // store value into PAD_OFFSET
.GPIO_Config_Modify_CTRL:
  ldr   r4, =IO_BANK0_BASE                       // load IO_BANK0 base
  add   r4, r4, r1                               // IO_BANK0_BASE + CTRL_OFFSET
  ldr   r5, [r4]                                 // read CTRL_OFFSET value
  bic   r5, r5, #0x1f                            // clear FUNCSEL
  orr   r5, r5, #0x05                            // set FUNCSEL 0x05->SIO_0
  str   r5, [r4]                                 // store value into CTRL_OFFSET
.GPIO_Config_Enable_OE:
  ldr   r4, =1                                   // enable output
  mcrr  p0, #4, r2, r4, c4                       // gpioc_bit_oe_put(GPIO, 1)
.GPIO_Config_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr to the stack
  bx    lr                                       // return
```

## Function Signature

```
GPIO_Config(r0 = PAD_OFFSET, r1 = CTRL_OFFSET, r2 = GPIO_NUMBER)
```

The caller provides:

| Register | Value | Example (GPIO16) |
|----------|-------|-----------------|
| r0 | Pad register offset | `0x44` (PADS_BANK0_GPIO16_OFFSET) |
| r1 | Control register offset | `0x84` (IO_BANK0_GPIO16_CTRL_OFFSET) |
| r2 | GPIO pin number | `16` |

By accepting offsets as parameters, this function can configure any GPIO pin — not just GPIO16.

## Register Save and Restore

```asm
  push  {r4-r12, lr}                             // save 10 registers (40 bytes)
  ...
  pop   {r4-r12, lr}                             // restore
  bx    lr                                       // return
```

GPIO_Config uses r4 and r5 as working registers.  Since these are callee-saved per the AAPCS, they must be preserved.  Pushing r4-r12 and lr saves all callee-saved registers plus the return address.

## Phase 1: Pad Configuration

The pad register controls the electrical characteristics of the physical pin.

### Calculate Pad Address

```asm
  ldr   r4, =PADS_BANK0_BASE                     // r4 = 0x40038000
  add   r4, r4, r0                               // r4 = 0x40038000 + 0x44 = 0x40038044
```

For GPIO16, the pad register is at `0x40038044`.

### Read-Modify-Write the Pad Register

```asm
  ldr   r5, [r4]                                 // read current pad value
  bic   r5, r5, #(1<<7)                          // clear OD (Output Disable) → allow output
  orr   r5, r5, #(1<<6)                          // set IE (Input Enable) → enable input
  bic   r5, r5, #(1<<8)                          // clear ISO (Isolation) → connect pad
  str   r5, [r4]                                 // write back
```

Three bit modifications in a single read-modify-write:

| Bit | Field | Before | After | Effect |
|-----|-------|--------|-------|--------|
| 8 | ISO | 1 | 0 | Remove pad isolation (connect to logic) |
| 7 | OD | 1 | 0 | Allow output driving |
| 6 | IE | 0 | 1 | Enable input (needed for readback) |

After reset, pads are isolated (ISO=1) with output disabled (OD=1) and input disabled (IE=0).  We must change all three for a functioning output pin.

The pad register bit layout:

```
Bit:  8    7    6    5    4    3    2    1    0
    +----+----+----+----+----+----+----+----+----+
    |ISO | OD | IE | DR1| DR0| PUE| PDE| SCH| SLW|
    +----+----+----+----+----+----+----+----+----+
      0    0    1    (unchanged)
```

## Phase 2: Function Select

The IO control register routes the pin to an internal peripheral.

### Calculate Control Address

```asm
  ldr   r4, =IO_BANK0_BASE                       // r4 = 0x40028000
  add   r4, r4, r1                               // r4 = 0x40028000 + 0x84 = 0x40028084
```

For GPIO16, the control register is at `0x40028084`.

### Set FUNCSEL to SIO

```asm
  ldr   r5, [r4]                                 // read current control value
  bic   r5, r5, #0x1f                            // clear FUNCSEL bits [4:0]
  orr   r5, r5, #0x05                            // set FUNCSEL = 5 (SIO)
  str   r5, [r4]                                 // write back
```

This is a clear-then-set operation on a 5-bit field:

1. `bic` with `0x1f` (`0b11111`) clears bits [4:0]
2. `orr` with `0x05` sets the field to 5

FUNCSEL = 5 connects the pin to the SIO (Single-cycle IO) block:

| FUNCSEL | Function |
|---------|----------|
| 0 | SPI |
| 1 | UART |
| 2 | I2C |
| 3 | PWM |
| 5 | SIO |
| 31 | NULL (disabled) |

## Phase 3: Enable Output via Coprocessor

```asm
  ldr   r4, =1                                   // r4 = 1 (enable)
  mcrr  p0, #4, r2, r4, c4                       // OE enable: set output enable for GPIO
```

The `mcrr` instruction talks to coprocessor 0 (SIO):

| Field | Value | Meaning |
|-------|-------|---------|
| `p0` | Coprocessor 0 | SIO block |
| `#4` | Opcode | GPIO bit operation |
| `r2` | GPIO number | 16 |
| `r4` | Value | 1 (enable) |
| `c4` | Register | Output Enable control |

This sets the output enable bit for GPIO16 in the SIO block.  After this instruction, the pin's output driver is active and can drive the LED.

## The Three Layers of GPIO Configuration

```
Layer 1: Pad (electrical)     → ISO=0, OD=0, IE=1
Layer 2: IO Bank (routing)    → FUNCSEL=5 (SIO)
Layer 3: SIO (output enable)  → OE=1 for GPIO16
```

All three layers must be configured correctly for the pin to drive output.  If any layer is misconfigured:

| Misconfiguration | Result |
|-----------------|--------|
| ISO=1 | Pin electrically disconnected |
| OD=1 | Output driver disabled |
| FUNCSEL≠5 | Pin routed to wrong peripheral |
| OE=0 | SIO output not driving |

## How main.s Calls GPIO_Config

```asm
  ldr   r0, =PADS_BANK0_GPIO16_OFFSET            // r0 = 0x44
  ldr   r1, =IO_BANK0_GPIO16_CTRL_OFFSET         // r1 = 0x84
  ldr   r2, =16                                  // r2 = GPIO number
  bl    GPIO_Config                              // configure GPIO16
```

## Summary

- GPIO_Config configures a GPIO pin for output in three phases: pad, function select, and output enable.
- The pad register (PADS_BANK0) controls electrical characteristics: clearing ISO and OD, setting IE.
- The control register (IO_BANK0) routes the pin to SIO via FUNCSEL = 5.
- The SIO coprocessor (`mcrr p0, #4, r2, r4, c4`) enables the output driver.
- The function accepts offsets as parameters, making it reusable for any GPIO pin.
- All three layers must be correctly configured for the output to function.
