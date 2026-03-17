# Chapter 21: image_def.s — The PICOBIN Boot Block

## Introduction

The very first thing the RP2350 boot ROM looks for in flash is a valid PICOBIN block — a small metadata structure that identifies the image as a legitimate program.  Without it, the boot ROM treats the flash as blank and refuses to boot.  image_def.s defines this block, and the linker script places it at the start of flash.  This chapter examines every byte.

## Complete Source Code

```asm
.section .picobin_block, "a"                     // place IMAGE_DEF block in flash

.word  0xffffded3                                // PICOBIN_BLOCK_MARKER_START
.byte  0x42                                      // PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE
.byte  0x1                                       // item is 1 word in size
.hword 0b0001000000100001                        // SECURE mode (0x1021)
.byte  0xff                                      // PICOBIN_BLOCK_ITEM_2BS_LAST
.hword 0x0001                                    // item is 1 word in size
.byte  0x0                                       // pad
.word  0x0                                       // relative pointer to next block (0 = loop to self)
.word  0xab123579                                // PICOBIN_BLOCK_MARKER_END
```

## Section Placement

```asm
.section .picobin_block, "a"                     // place IMAGE_DEF block in flash
```

The `"a"` flag means allocatable — this section occupies space in the final binary.  The linker script places it first:

```ld
.embedded_block :
{
  KEEP(*(.picobin_block))
} > FLASH :text
```

`KEEP` prevents the linker from discarding this section, even though no code references it.  The boot ROM reads it by its position at the start of flash.

Note: The section name in the assembly is `.picobin_block`, while the linker script's `KEEP` matches this via `*(.picobin_block)` within the `.embedded_block` output section.

## Block Structure

The PICOBIN block is a linked-list structure.  Each block starts and ends with magic markers and contains one or more items:

```
+----------------------------+ Offset 0x00
| MARKER_START (0xffffded3)  | 4 bytes
+----------------------------+ Offset 0x04
| Item: IMAGE_TYPE           | 4 bytes
+----------------------------+ Offset 0x08
| Item: LAST                 | 4 bytes
+----------------------------+ Offset 0x0C
| Next block pointer (0)     | 4 bytes
+----------------------------+ Offset 0x10
| MARKER_END (0xab123579)    | 4 bytes
+----------------------------+ Offset 0x14
```

Total size: 20 bytes (5 words).

## Byte-by-Byte Analysis

### Start Marker

```asm
.word  0xffffded3                                // PICOBIN_BLOCK_MARKER_START
```

The boot ROM scans flash for this 32-bit pattern.  It marks the beginning of a PICOBIN block.

### IMAGE_TYPE Item

```asm
.byte  0x42                                      // PICOBIN_BLOCK_ITEM_1BS_IMAGE_TYPE
.byte  0x1                                       // item is 1 word in size
.hword 0b0001000000100001                        // SECURE mode (0x1021)
```

This 4-byte item breaks down as:

| Field | Value | Meaning |
|-------|-------|---------|
| Item type | `0x42` | IMAGE_TYPE (1-byte size variant) |
| Size | `0x01` | 1 word (4 bytes including these header bytes) |
| Data | `0x1021` | Image type flags |

The data word `0x1021` (`0b0001000000100001`) encodes:

| Bits | Value | Meaning |
|------|-------|---------|
| 0 | 1 | Image is executable |
| 5 | 1 | ARM architecture (Cortex-M33) |
| 12 | 1 | Security mode: secure |

### LAST Item

```asm
.byte  0xff                                      // PICOBIN_BLOCK_ITEM_2BS_LAST
.hword 0x0001                                    // item is 1 word in size
.byte  0x0                                       // pad
```

| Field | Value | Meaning |
|-------|-------|---------|
| Item type | `0xFF` | LAST item (terminates the item list) |
| Size | `0x0001` | 1 word |
| Pad | `0x00` | Alignment padding |

The LAST item signals the end of the item list within this block.

### Next Block Pointer

```asm
.word  0x0                                       // relative pointer to next block (0 = loop to self)
```

A value of 0 means this is the only block — it loops back to itself.  In more complex images, multiple blocks can form a linked list.

### End Marker

```asm
.word  0xab123579                                // PICOBIN_BLOCK_MARKER_END
```

The end marker validates the block's integrity.  The boot ROM checks both markers before accepting the block.

## Why This Matters

Without a valid PICOBIN block:

1. The boot ROM scans flash and finds no recognized image
2. The chip falls back to USB boot mode (BOOTSEL)
3. Our code never executes

The PICOBIN block is the handshake between our firmware and the boot ROM — it says "this is a valid ARM executable image, please run it."

## Summary

- image_def.s defines a 20-byte PICOBIN block placed at the start of flash.
- The block contains start/end markers, an IMAGE_TYPE item (ARM, executable, secure), and a LAST item.
- The linker script uses `KEEP` to ensure this section is never discarded.
- The RP2350 boot ROM scans for this block to validate the firmware image before execution.
