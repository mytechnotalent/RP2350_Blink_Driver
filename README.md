<img src="https://github.com/mytechnotalent/RP2350_Blink_Driver/blob/main/RP2350_Blink_Driver.png?raw=true">

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)
### VIDEO PROMO [HERE](https://www.youtube.com/watch?v=aD7X9sXirF8)

<br>

# RP2350 Blink Driver
An RP2350 Blink driver written entirely in Assembler.

<br>

# Install ARM Toolchain
## NOTE: Be SURE to select `Add path to environment variable` on setup.
[HERE](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

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

# constants Code
```assembler
/**
 * FILE: constants.s
 *
 * DESCRIPTION:
 * RP2350 Memory Addresses and Constants.
 * 
 * BRIEF:
 * Defines all memory-mapped register addresses and constants used
 * throughout the RP2350 Blink driver.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
 * UPDATE DATE: November 27, 2025
 */

.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

/**
 * Memory addresses and constants.
 */
.equ STACK_TOP,                   0x20082000               
.equ STACK_LIMIT,                 0x2007a000             
.equ XOSC_BASE,                   0x40048000          
.equ XOSC_CTRL,                   XOSC_BASE + 0x00       
.equ XOSC_STATUS,                 XOSC_BASE + 0x04       
.equ XOSC_STARTUP,                XOSC_BASE + 0x0c        
.equ PPB_BASE,                    0xe0000000               
.equ CPACR,                       PPB_BASE + 0x0ed88       
.equ CLOCKS_BASE,                 0x40010000              
.equ CLK_PERI_CTRL,               CLOCKS_BASE + 0x48       
.equ RESETS_BASE,                 0x40020000               
.equ RESETS_RESET,                RESETS_BASE + 0x0        
.equ RESETS_RESET_CLEAR,          RESETS_BASE + 0x3000     
.equ RESETS_RESET_DONE,           RESETS_BASE + 0x8        
.equ IO_BANK0_BASE,               0x40028000               
.equ IO_BANK0_GPIO16_CTRL_OFFSET, 0x84                   
.equ PADS_BANK0_BASE,             0x40038000               
.equ PADS_BANK0_GPIO16_OFFSET,    0x44                    
```

<br>

# vector_table Code
```assembler
/**
 * FILE: vector_table.s
 *
 * DESCRIPTION:
 * RP2350 Vector Table.
 * 
 * BRIEF:
 * Defines the vector table for the RP2350 containing the initial
 * stack pointer and reset handler entry point.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
 * UPDATE DATE: November 27, 2025
 */

.syntax unified                                  // use unified assembly syntax
.cpu cortex-m33                                  // target Cortex-M33 core
.thumb                                           // use Thumb instruction set

.include "constants.s"

/**
 * Initialize the .vectors section. The .vectors section contains vector
 * table and Reset_Handler.
 */
.section .vectors, "ax"                          // vector table section
.align 2                                         // align to 4-byte boundary

/**
 * Vector table section.
 */
.global _vectors                                 // export _vectors symbol
_vectors:
  .word STACK_TOP                                // initial stack pointer
  .word Reset_Handler + 1                        // reset handler (Thumb bit set)
```

<br>

# stack Code
```assembler
/**
 * FILE: stack.s
 *
 * DESCRIPTION:
 * RP2350 Stack Initialization.
 * 
 * BRIEF:
 * Provides stack pointer initialization for Main and Process Stack
 * Pointers (MSP/PSP) and their limits.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Initialize stack pointers.
 *
 * @details Sets Main and Process Stack Pointers (MSP/PSP) and their limits.
 *
 * @param   None
 * @retval  None
 */
.global Init_Stack
.type Init_Stack, %function
Init_Stack:
  ldr   r0, =STACK_TOP                           // load stack top
  msr   PSP, r0                                  // set PSP
  ldr   r0, =STACK_LIMIT                         // load stack limit
  msr   MSPLIM, r0                               // set MSP limit
  msr   PSPLIM, r0                               // set PSP limit
  ldr   r0, =STACK_TOP                           // reload stack top
  msr   MSP, r0                                  // set MSP
  bx    lr                                       // return
```

<br>

# xosc Code
```assembler
/**
 * FILE: xosc.s
 *
 * DESCRIPTION:
 * RP2350 External Crystal Oscillator (XOSC) Functions.
 * 
 * BRIEF:
 * Provides functions to initialize the external crystal oscillator
 * and enable the XOSC peripheral clock.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Init XOSC and wait until it is ready.
 *
 * @details Configures and initializes the external crystal oscillator (XOSC).
 *          Waits for the XOSC to become stable before returning.
 *
 * @param   None
 * @retval  None
 */
.global Init_XOSC
.type Init_XOSC, %function
Init_XOSC:
  ldr   r0, =XOSC_STARTUP                        // load XOSC_STARTUP address
  ldr   r1, =0x00c4                              // set delay 50,000 cycles
  str   r1, [r0]                                 // store value into XOSC_STARTUP
  ldr   r0, =XOSC_CTRL                           // load XOSC_CTRL address
  ldr   r1, =0x00FABAA0                          // set 1_15MHz, freq range, actual 14.5MHz
  str   r1, [r0]                                 // store value into XOSC_CTRL
.Init_XOSC_Wait:
  ldr   r0, =XOSC_STATUS                         // load XOSC_STATUS address
  ldr   r1, [r0]                                 // read XOSC_STATUS value
  tst   r1, #(1<<31)                             // test STABLE bit
  beq   .Init_XOSC_Wait                          // wait until stable bit is set
  bx    lr                                       // return

/**
 * @brief   Enable XOSC peripheral clock.
 *
 * @details Sets the peripheral clock to use XOSC as its AUXSRC.
 *
 * @param   None
 * @retval  None
 */
.global Enable_XOSC_Peri_Clock
.type Enable_XOSC_Peri_Clock, %function
Enable_XOSC_Peri_Clock:
  ldr   r0, =CLK_PERI_CTRL                       // load CLK_PERI_CTRL address
  ldr   r1, [r0]                                 // read CLK_PERI_CTRL value
  orr   r1, r1, #(1<<11)                         // set ENABLE bit
  orr   r1, r1, #(4<<5)                          // set AUXSRC: XOSC_CLKSRC bit
  str   r1, [r0]                                 // store value into CLK_PERI_CTRL
  bx    lr                                       // return
```

<br>

# reset Code
```assembler
/**
 * FILE: reset.s
 *
 * DESCRIPTION:
 * RP2350 Reset Controller Functions.
 * 
 * BRIEF:
 * Provides functions to initialize subsystems by clearing their
 * reset bits in the Reset controller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Init subsystem.
 *
 * @details Initiates the various subsystems by clearing their reset bits.
 *
 * @param   None
 * @retval  None
 */
.global Init_Subsystem
.type Init_Subsystem, %function
Init_Subsystem:
.GPIO_Subsystem_Reset:
  ldr   r0, =RESETS_RESET                        // load RESETS->RESET address
  ldr   r1, [r0]                                 // read RESETS->RESET value
  bic   r1, r1, #(1<<6)                          // clear IO_BANK0 bit
  str   r1, [r0]                                 // store value into RESETS->RESET address
.GPIO_Subsystem_Reset_Wait:
  ldr   r0, =RESETS_RESET_DONE                   // load RESETS->RESET_DONE address
  ldr   r1, [r0]                                 // read RESETS->RESET_DONE value
  tst   r1, #(1<<6)                              // test IO_BANK0 reset done
  beq   .GPIO_Subsystem_Reset_Wait               // wait until done
  bx    lr                                       // return
```

<br>

# coprocessor Code
```assembler
/**
 * FILE: coprocessor.s
 *
 * DESCRIPTION:
 * RP2350 Coprocessor Access Functions.
 * 
 * BRIEF:
 * Provides functions to enable coprocessor access control.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Enable coprocessor access.
 *
 * @details Grants full access to coprocessor 0 via CPACR.
 *
 * @param   None
 * @retval  None
 */
.global Enable_Coprocessor
.type Enable_Coprocessor , %function
Enable_Coprocessor:
  ldr   r0, =CPACR                               // load CPACR address
  ldr   r1, [r0]                                 // read CPACR value
  orr   r1, r1, #(1<<1)                          // set CP0: Ctrl access priv coproc 0 bit
  orr   r1, r1, #(1<<0)                          // set CP0: Ctrl access priv coproc 0 bit
  str   r1, [r0]                                 // store value into CPACR
  dsb                                            // data sync barrier
  isb                                            // instruction sync barrier
  bx    lr                                       // return
```

<br>

# gpio Code
```assembler
/**
 * FILE: gpio.s
 *
 * DESCRIPTION:
 * RP2350 GPIO Functions.
 * 
 * BRIEF:
 * Provides GPIO configuration, set, and clear functions using
 * coprocessor instructions.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Configure GPIO.
 *
 * @details Configures a GPIO pin's pad control and function select.
 *
 * @param   r0 - PAD_OFFSET
 * @param   r1 - CTRL_OFFSET
 * @param   r2 - GPIO
 * @retval  None
 */
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

/**
 * @brief   GPIO set.
 *
 * @details Drives GPIO output high via coprocessor.
 *
 * @param   r0 - GPIO
 * @retval  None
 */
.global GPIO_Set
.type GPIO_Set, %function
GPIO_Set:
.GPIO_Set_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Set_Execute:
  ldr   r4, =1                                   // enable output
  mcrr  p0, #4, r0, r4, c0                       // gpioc_bit_out_put(GPIO, 1)
.GPIO_Set_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return

/**
 * @brief   GPIO clear.
 *
 * @details Drives GPIO output high via coprocessor.
 *
 * @param   r0 - GPIO
 * @retval  None
 */
.global GPIO_Clear
.type GPIO_Clear, %function
GPIO_Clear:
.GPIO_Clear_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.GPIO_Clear_Execute:
  ldr   r4, =0                                   // disable output
  mcrr  p0, #4, r0, r4, c0                       // gpioc_bit_out_put(GPIO, 1)
.GPIO_Clear_Pop_Registers:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return
```

<br>

# delay Code
```assembler
/**
 * FILE: delay.s
 *
 * DESCRIPTION:
 * RP2350 Delay Functions.
 * 
 * BRIEF:
 * Provides millisecond delay functions based on a 14.5MHz clock.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Delay_MS.
 *
 * @details Delays for r0 milliseconds. Conversion: loop_count = ms * 3600
 *          based on a 14.5MHz clock.
 *
 * @param   r0 - milliseconds
 * @retval  None
 */
.global Delay_MS
.type Delay_MS, %function
Delay_MS:
.Delay_MS_Push_Registers:
  push  {r4-r12, lr}                             // push registers r4-r12, lr to the stack
.Delay_MS_Check:
  cmp   r0, #0                                   // if MS is not valid, return
  ble   .Delay_MS_Done                           // branch if less or equal to 0 
.Delay_MS_Setup:
  ldr   r4, =3600                                // loops per MS based on 14.5MHz clock
  mul   r5, r0, r4                               // MS * 3600
.Delay_MS_Loop:
  subs  r5, r5, #1                               // decrement counter
  bne   .Delay_MS_Loop                           // branch until zero
.Delay_MS_Done:
  pop   {r4-r12, lr}                             // pop registers r4-r12, lr from the stack
  bx    lr                                       // return
```

<br>

# reset_handler Code
```assembler
/**
 * FILE: reset_handler.s
 *
 * DESCRIPTION:
 * RP2350 Reset Handler.
 * 
 * BRIEF:
 * Entry point after reset. Performs initialization sequence including
 * stack setup, oscillator configuration, and subsystem initialization 
 * before branching to main application.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: November 27, 2025
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
 * @brief   Reset handler for RP2350.
 *
 * @details Entry point after reset. Performs:
 *          - Stack initialization
 *          - Coprocessor enable
 *          - GPIO16 pad/function configuration
 *          - Branches to main() which contains the blink loop
 *
 * @param   None
 * @retval  None
 */
.global Reset_Handler                            // export Reset_Handler symbol
.type Reset_Handler, %function                        
Reset_Handler:
  bl    Init_Stack                               // initialize MSP/PSP and limits
  bl    Init_XOSC                                // initialize external crystal oscillator
  bl    Enable_XOSC_Peri_Clock                   // enable XOSC peripheral clock
  bl    Init_Subsystem                           // initialize subsystems
  bl    Enable_Coprocessor                       // enable CP0 coprocessor
  b     main                                     // branch to main loop
.size Reset_Handler, . - Reset_Handler
```

<br>

# main Code
```assembler

```

<br>

# License
[Apache License 2.0](https://github.com/mytechnotalent/RP2350_Blink_Driver/blob/main/LICENSE)
