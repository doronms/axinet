// SPDX-License-Identifier: GPL-2.0
/*
 * mcu_pcie.c - MCU PCIe Bridge Driver
 *
 * Creates child platform devices for FPGA peripherals:
 *   - fpga_regs: General FPGA registers
 *   - max10_regs: MAX10 registers  
 *   - p2puart: 12 UART ports
 *   - timer1pps/timer1ppm: Timer devices
 *   - mcu_axienet: AXI Ethernet Subsystem (NEW)
 *
 * All children share MSI vectors allocated by this driver.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>

#include "mcu_pcie.h"

#define DRV_NAME    "mcu_pcie"
#define DRV_VERSION "2.0"

/* Debug control */
static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0=off, 1=info, 2=verbose)");

#define DBG(lvl, fmt, ...) do { \
    if (debug >= (lvl)) \
        pr_info(DRV_NAME ": " fmt, ##__VA_ARGS__); \
} while (0)

/* Child device descriptor */
struct child_info {
    const char *name;           /* Platform driver name */
    resource_size_t offset;     /* Offset in BAR0 */
    resource_size_t size;       /* Size of memory region */
    bool needs_irq;             /* Needs interrupt? */
    int irq_idx;                /* MSI vector index (0-15) */
    bool needs_pdata;           /* Needs platform data with BAR pointers? */
};

/* Bridge private data */
struct mcu_bridge_data {
    struct pci_dev *pdev;
    void __iomem *bar0;         /* AXI-Lite registers */
    void __iomem *bar1;         /* XDMA registers */
    resource_size_t bar0_phys;
    resource_size_t bar1_phys;
    struct platform_device **children;
    int num_children;
    spinlock_t lock;
    
    /* Platform data for children that need it */
    struct mcu_pcie_pdata pdata;
};

/* IRQ statistics for debugfs */
static atomic_t xdma_irq_cnt[MCU_IRQ_COUNT];
static struct dentry *dbg_dir;

/*
 * Child device table
 * 
 * Note: mcu_axienet needs platform data because it accesses both BAR0 and BAR1
 */
static const struct child_info children[] = {
    /* FPGA registers */
#if 0
    {
        .name = "fpga_regs",
        .offset = MCU_FPGA_REGS_OFFSET,
        .size = MCU_FPGA_REGS_SIZE,
        .needs_irq = false,
        .irq_idx = 0,
        .needs_pdata = false,
    },
    /* MAX10 registers */
    {
        .name = "max10_regs",
        .offset = MCU_MAX10_REGS_OFFSET,
        .size = MCU_MAX10_REGS_SIZE,
        .needs_irq = false,
        .irq_idx = 0,
        .needs_pdata = false,
    },
    /* P2PUART ports 0-11 */
    { .name = "p2puart", .offset = 0x070000, .size = 0x10000, .needs_irq = true, .irq_idx = 0,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x080000, .size = 0x10000, .needs_irq = true, .irq_idx = 1,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x090000, .size = 0x10000, .needs_irq = true, .irq_idx = 2,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x0A0000, .size = 0x10000, .needs_irq = true, .irq_idx = 3,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x0B0000, .size = 0x10000, .needs_irq = true, .irq_idx = 4,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x0C0000, .size = 0x10000, .needs_irq = true, .irq_idx = 5,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x0D0000, .size = 0x10000, .needs_irq = true, .irq_idx = 6,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x0E0000, .size = 0x10000, .needs_irq = true, .irq_idx = 7,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x0F0000, .size = 0x10000, .needs_irq = true, .irq_idx = 8,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x100000, .size = 0x10000, .needs_irq = true, .irq_idx = 9,  .needs_pdata = false },
    { .name = "p2puart", .offset = 0x110000, .size = 0x10000, .needs_irq = true, .irq_idx = 10, .needs_pdata = false },
    { .name = "p2puart", .offset = 0x120000, .size = 0x10000, .needs_irq = true, .irq_idx = 11, .needs_pdata = false },
    /* Timers */
    {
        .name = "timer1pps",
        .offset = MCU_TIMER_OFFSET,
        .size = MCU_TIMER_SIZE,
        .needs_irq = true,
        .irq_idx = MCU_IRQ_TIMER1PPS,
        .needs_pdata = false,
    },
    {
        .name = "timer1ppm",
        .offset = MCU_TIMER_OFFSET,
        .size = MCU_TIMER_SIZE,
        .needs_irq = true,
        .irq_idx = MCU_IRQ_TIMER1PPM,
        .needs_pdata = false,
    },
#endif   
    /* AXI Ethernet - needs platform data for BAR0/BAR1 access */
    {
        .name = "mcu_axienet",
        .offset = MCU_AXIENET_OFFSET,
        .size = MCU_AXIENET_SIZE,
        .needs_irq = true,
        .irq_idx = MCU_IRQ_AXIENET,  /* 14 per block design; clamped to 0 with 1 MSI vec */
        .needs_pdata = true,
    },
    /* Terminator */
    { }
};

/* PCI device table */
static const struct pci_device_id mcu_pci_ids[] = {
    { PCI_DEVICE(MCU_VENDOR_ID, MCU_DEVICE_ID) },
    { 0, }
};
MODULE_DEVICE_TABLE(pci, mcu_pci_ids);

/* ============================================================================
 * DebugFS for IRQ statistics
 * ============================================================================
 */

static void mcu_debugfs_init(struct device *dev)
{
    int i;
    char name[16];

    dbg_dir = debugfs_create_dir(DRV_NAME, NULL);
    if (IS_ERR_OR_NULL(dbg_dir))
        return;

    for (i = 0; i < MCU_IRQ_COUNT; i++) {
        atomic_set(&xdma_irq_cnt[i], 0);
        snprintf(name, sizeof(name), "irq_vec%02d", i);
        debugfs_create_atomic_t(name, 0444, dbg_dir, &xdma_irq_cnt[i]);
    }

    dev_info(dev, "DebugFS created at /sys/kernel/debug/%s\n", DRV_NAME);
}

static void mcu_debugfs_exit(void)
{
    debugfs_remove_recursive(dbg_dir);
    dbg_dir = NULL;
}

/* ============================================================================
 * Exported function for child drivers
 * ============================================================================
 */

/**
 * xdma_get_and_clear_pending - Read and clear pending XDMA user interrupts
 * @dev: Child device whose parent is the mcu_pcie device
 *
 * Returns: Bitmask of pending interrupts (bits 0-15)
 */
u32 xdma_get_and_clear_pending(struct device *dev)
{
    struct pci_dev *pdev;
    struct mcu_bridge_data *bd;
    void __iomem *base;
    u32 pending;
    unsigned long flags;
    int i;

    if (!dev || !dev->parent)
        return 0;

    pdev = to_pci_dev(dev->parent);
    bd = pci_get_drvdata(pdev);
    if (!bd || !bd->bar1)
        return 0;

    base = bd->bar1;

    spin_lock_irqsave(&bd->lock, flags);

    /* Read pending interrupts */
    pending = readl(base + XDMA_IRQ_PENDING);

    /* Clear by writing back */
    if (pending)
        writel(pending, base + XDMA_IRQ_PENDING);

    spin_unlock_irqrestore(&bd->lock, flags);

    /* Update statistics */
    for (i = 0; i < MCU_IRQ_COUNT && pending; i++) {
        if (pending & BIT(i))
            atomic_inc(&xdma_irq_cnt[i]);
        pending &= ~BIT(i);
    }

    return pending;
}
EXPORT_SYMBOL_GPL(xdma_get_and_clear_pending);

/* ============================================================================
 * MSI Configuration
 * ============================================================================
 */

static int mcu_configure_msi(struct mcu_bridge_data *bd, int num_vectors)
{
    void __iomem *base = bd->bar1;
    u32 reg_val;
    int i;

    if (!base) {
        dev_err(&bd->pdev->dev, "BAR1 not mapped, cannot configure MSI\n");
        return -ENODEV;
    }

    /* Enable global interrupts */
    writel(XDMA_GLOBAL_INT_ENABLE_BIT, base + XDMA_GLOBAL_INT_ENABLE);

    /* Configure MSI vector mapping for each user IRQ line */
    for (i = 0; i < MCU_IRQ_COUNT; i++) {
        int group = i / 4;
        int slot = i % 4;
        u32 offset = XDMA_IRQ_USER_VEC_BASE + (group * XDMA_IRQ_USER_VEC_STRIDE);

        /* Read-modify-write: set vector number for this IRQ line */
        reg_val = readl(base + offset);
        reg_val &= ~(0xFF << (slot * 8));
        reg_val |= (i & 0x1F) << (slot * 8);
        writel(reg_val, base + offset);

        DBG(2, "IRQ line %d -> MSI vector %d (reg 0x%x = 0x%08x)\n",
            i, i, offset, reg_val);
    }

    /* Enable all user IRQ lines we need */
    writel(0xFFFF, base + XDMA_IRQ_ENABLE_MASK);

    dev_info(&bd->pdev->dev, "MSI configured: %d vectors, all user IRQs enabled\n",
             num_vectors);

    return 0;
}

/* ============================================================================
 * Child Device Creation
 * ============================================================================
 */

static int mcu_create_children(struct mcu_bridge_data *bd)
{
    struct pci_dev *pdev = bd->pdev;
    const struct child_info *ci;
    struct platform_device *child;
    struct resource res[2];
    int i, ret, num_res;
    int irq_num;

    /* Count children */
    for (bd->num_children = 0, ci = children; ci->name; ci++)
        bd->num_children++;

    bd->children = devm_kcalloc(&pdev->dev, bd->num_children,
                                sizeof(struct platform_device *), GFP_KERNEL);
    if (!bd->children)
        return -ENOMEM;

    /* Prepare platform data for children that need it */
    bd->pdata.pci_dev = pdev;
    bd->pdata.bar0 = bd->bar0;
    bd->pdata.bar1 = bd->bar1;
    bd->pdata.bar0_phys = bd->bar0_phys;
    bd->pdata.bar1_phys = bd->bar1_phys;

    for (i = 0, ci = children; ci->name; ci++, i++) {
        memset(res, 0, sizeof(res));
        num_res = 0;

        /* Memory resource */
        res[num_res].start = bd->bar0_phys + ci->offset;
        res[num_res].end = res[num_res].start + ci->size - 1;
        res[num_res].flags = IORESOURCE_MEM;
        res[num_res].name = ci->name;
        num_res++;

        /* IRQ resource
         * With fewer MSI vectors than IRQ lines, clamp to vector 0.
         * E.g., if only 1 MSI vector allocated, all children share it.
         */
        if (ci->needs_irq) {
            int vec_idx = ci->irq_idx;
            int nvec_max = pci_msix_vec_count(pdev);  /* May be -1 */

            /* Clamp: with 1 MSI vector, all IRQs go to vector 0 */
            irq_num = pci_irq_vector(pdev, vec_idx);
            if (irq_num < 0) {
                /* Requested vector doesn't exist, fall back to 0 */
                dev_info(&pdev->dev, "IRQ vector %d unavailable for %s, using vector 0\n",
                         vec_idx, ci->name);
                irq_num = pci_irq_vector(pdev, 0);
            }
            if (irq_num < 0) {
                dev_err(&pdev->dev, "Failed to get any IRQ for %s\n", ci->name);
                continue;
            }
            res[num_res].start = irq_num;
            res[num_res].end = irq_num;
            res[num_res].flags = IORESOURCE_IRQ;
            res[num_res].name = "irq";
            num_res++;
        }

        /* Allocate platform device */
        child = platform_device_alloc(ci->name, PLATFORM_DEVID_AUTO);
        if (!child) {
            dev_err(&pdev->dev, "Failed to allocate %s\n", ci->name);
            continue;
        }

        child->dev.parent = &pdev->dev;

        /* Add resources */
        ret = platform_device_add_resources(child, res, num_res);
        if (ret) {
            dev_err(&pdev->dev, "Failed to add resources for %s: %d\n",
                    ci->name, ret);
            platform_device_put(child);
            continue;
        }

        /* Add platform data if needed */
        if (ci->needs_pdata) {
            ret = platform_device_add_data(child, &bd->pdata, sizeof(bd->pdata));
            if (ret) {
                dev_err(&pdev->dev, "Failed to add pdata for %s: %d\n",
                        ci->name, ret);
                platform_device_put(child);
                continue;
            }
        }

        /* Register the device */
        ret = platform_device_add(child);
        if (ret) {
            dev_err(&pdev->dev, "Failed to add device %s: %d\n", ci->name, ret);
            platform_device_put(child);
            continue;
        }

        bd->children[i] = child;

        dev_info(&pdev->dev, "Created '%s' @ 0x%llx size 0x%llx%s%s\n",
                 ci->name,
                 (unsigned long long)(bd->bar0_phys + ci->offset),
                 (unsigned long long)ci->size,
                 ci->needs_irq ? " IRQ " : "",
                 ci->needs_irq ? 
                     kasprintf(GFP_KERNEL, "%d", pci_irq_vector(pdev, ci->irq_idx)) : "");
    }

    return 0;
}

static void mcu_destroy_children(struct mcu_bridge_data *bd)
{
    int i;

    for (i = 0; i < bd->num_children; i++) {
        if (bd->children[i])
            platform_device_unregister(bd->children[i]);
    }
}

/* ============================================================================
 * PCI Driver
 * ============================================================================
 */

static int mcu_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct mcu_bridge_data *bd;
    int ret, nvec;

    dev_info(&pdev->dev, "MCU PCIe Bridge v%s probing...\n", DRV_VERSION);

    /* Enable device */
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "pci_enable_device failed: %d\n", ret);
        return ret;
    }

    pci_set_master(pdev);

    /* Request regions */
    ret = pci_request_regions(pdev, DRV_NAME);
    if (ret) {
        dev_err(&pdev->dev, "pci_request_regions failed: %d\n", ret);
        goto err_disable;
    }

    /* Allocate bridge data */
    bd = devm_kzalloc(&pdev->dev, sizeof(*bd), GFP_KERNEL);
    if (!bd) {
        ret = -ENOMEM;
        goto err_release;
    }

    bd->pdev = pdev;
    spin_lock_init(&bd->lock);
    pci_set_drvdata(pdev, bd);

    /* Map BAR0 */
    bd->bar0_phys = pci_resource_start(pdev, MCU_BAR0);
    bd->bar0 = devm_ioremap(&pdev->dev, bd->bar0_phys,
                            pci_resource_len(pdev, MCU_BAR0));
    if (!bd->bar0) {
        dev_err(&pdev->dev, "Failed to map BAR0\n");
        ret = -ENOMEM;
        goto err_release;
    }
    dev_info(&pdev->dev, "BAR0: %pR -> %p\n",
             &pdev->resource[MCU_BAR0], bd->bar0);

    /* Map BAR1 */
    if (pci_resource_len(pdev, MCU_BAR1) > 0) {
        bd->bar1_phys = pci_resource_start(pdev, MCU_BAR1);
        bd->bar1 = devm_ioremap(&pdev->dev, bd->bar1_phys,
                                pci_resource_len(pdev, MCU_BAR1));
        if (!bd->bar1) {
            dev_err(&pdev->dev, "Failed to map BAR1\n");
            ret = -ENOMEM;
            goto err_release;
        }
        dev_info(&pdev->dev, "BAR1: %pR -> %p\n",
                 &pdev->resource[MCU_BAR1], bd->bar1);
    } else {
        dev_warn(&pdev->dev, "BAR1 not available\n");
    }

    /* Set DMA mask */
    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret) {
            dev_err(&pdev->dev, "Failed to set DMA mask\n");
            goto err_release;
        }
    }

    /* Allocate MSI vectors */
    //nvec = pci_alloc_irq_vectors(pdev, MCU_IRQ_COUNT, MCU_IRQ_COUNT, PCI_IRQ_MSI);
    nvec = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
    if (nvec < 0) {
        /* Try fewer vectors */
        nvec = pci_alloc_irq_vectors(pdev, 1, MCU_IRQ_COUNT, PCI_IRQ_MSI);
        if (nvec < 0) {
            dev_err(&pdev->dev, "Failed to allocate MSI vectors: %d\n", nvec);
            ret = nvec;
            goto err_release;
        }
    }
    dev_info(&pdev->dev, "Allocated %d MSI vectors\n", nvec);

    /* Configure XDMA MSI routing */
    ret = mcu_configure_msi(bd, nvec);
    if (ret)
        goto err_free_irq;

    /* Create child devices */
    ret = mcu_create_children(bd);
    if (ret)
        goto err_free_irq;

    /* Setup debugfs */
    mcu_debugfs_init(&pdev->dev);

    dev_info(&pdev->dev, "MCU PCIe Bridge initialized\n");
    return 0;

err_free_irq:
    pci_free_irq_vectors(pdev);
err_release:
    pci_release_regions(pdev);
err_disable:
    pci_disable_device(pdev);
    return ret;
}

static void mcu_pci_remove(struct pci_dev *pdev)
{
    struct mcu_bridge_data *bd = pci_get_drvdata(pdev);

    mcu_debugfs_exit();
    mcu_destroy_children(bd);
    pci_free_irq_vectors(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);

    dev_info(&pdev->dev, "MCU PCIe Bridge removed\n");
}

static struct pci_driver mcu_pci_driver = {
    .name = DRV_NAME,
    .id_table = mcu_pci_ids,
    .probe = mcu_pci_probe,
    .remove = mcu_pci_remove,
};

module_pci_driver(mcu_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("MCU BSP Team");
MODULE_DESCRIPTION("MCU PCIe Bridge Driver");
MODULE_VERSION(DRV_VERSION);
