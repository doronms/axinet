/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mcu_pcie.h - MCU PCIe Bridge shared definitions
 *
 * Used by mcu_pcie.c (bridge) and child drivers (mcu_axienet, p2puart, etc.)
 */

#ifndef _MCU_PCIE_H_
#define _MCU_PCIE_H_

#include <linux/pci.h>

/* PCI IDs */
#define MCU_VENDOR_ID       0x10EE  /* Xilinx */
#define MCU_DEVICE_ID       0x9021  /* Custom XDMA device */

/* BAR layout */
#define MCU_BAR0            0       /* AXI-Lite: registers, peripherals */
#define MCU_BAR1            1       /* XDMA: DMA engine, MSI config */

/* Child device offsets in BAR0 */
#define MCU_FPGA_REGS_OFFSET    0x020000
#define MCU_MAX10_REGS_OFFSET   0x030000
#define MCU_P2PUART_BASE        0x070000
#define MCU_P2PUART_STRIDE      0x010000
#define MCU_TIMER_OFFSET        0x020000
#define MCU_AXIENET_OFFSET      0x240000    /* AXI Ethernet Subsystem */

/* Child device sizes */
#define MCU_FPGA_REGS_SIZE      0x1000
#define MCU_MAX10_REGS_SIZE     0x1000
#define MCU_P2PUART_SIZE        0x10000
#define MCU_TIMER_SIZE          0x1000
#define MCU_AXIENET_SIZE        0x40000     /* 256KB for AXI Ethernet */

/* XDMA User Interrupt mapping (from block design xlconcat_0)
 * 
 * IRQ Bit | MSI Vec | Device
 * --------|---------|------------------
 *   0-11  |  0-11   | P2PUART (12 ports)
 *    12   |   12    | timer1pps
 *    13   |   13    | timer1ppm
 *    14   |   14    | AXI Ethernet (MAC IRQ)
 *    15   |   15    | (reserved/general)
 *
 * NOTE: With only 1 MSI vector allocated, all map to vector 0.
 * The child still gets pci_irq_vector(pdev, 0) which is the
 * single MSI IRQ number. The irq_idx in child_info is used only
 * when multiple MSI vectors are available.
 */
#define MCU_IRQ_P2PUART_BASE    0
#define MCU_IRQ_P2PUART_COUNT   12
#define MCU_IRQ_TIMER1PPS       12
#define MCU_IRQ_TIMER1PPM       13
#define MCU_IRQ_AXIENET         14  /* FIX: was incorrectly 0, should be 14 per block design */
#define MCU_IRQ_GENERAL         15
#define MCU_IRQ_COUNT           16

/* XDMA registers in BAR1 */
#define XDMA_IRQ_BLOCK_OFFSET       0x2000
#define XDMA_IRQ_ENABLE_MASK        0x2004  /* Enable bits for user IRQs */
#define XDMA_IRQ_PENDING            0x2048  /* Pending IRQ status */
#define XDMA_IRQ_USER_VEC_BASE      0x2080  /* MSI vector mapping */
#define XDMA_IRQ_USER_VEC_STRIDE    0x04

#define XDMA_GLOBAL_INT_ENABLE      0x0050
#define XDMA_GLOBAL_INT_ENABLE_BIT  0x01

/**
 * struct mcu_pcie_pdata - Platform data passed to child devices
 * @pci_dev:    Parent PCI device (for DMA operations)
 * @bar0:       Mapped BAR0 base address
 * @bar1:       Mapped BAR1 base address (XDMA)
 * @bar0_phys:  Physical address of BAR0
 * @bar1_phys:  Physical address of BAR1
 *
 * Child drivers use this to access their registers and perform DMA.
 */
struct mcu_pcie_pdata {
    struct pci_dev *pci_dev;
    void __iomem *bar0;
    void __iomem *bar1;
    resource_size_t bar0_phys;
    resource_size_t bar1_phys;
};

/* Functions exported by mcu_pcie.c for child drivers */

/**
 * xdma_get_and_clear_pending - Read and clear pending IRQ bits
 * @dev: Child device (parent must be mcu_pcie)
 *
 * Returns: Bitmask of pending interrupts (bits 0-15)
 */
u32 xdma_get_and_clear_pending(struct device *dev);

#endif /* _MCU_PCIE_H_ */
