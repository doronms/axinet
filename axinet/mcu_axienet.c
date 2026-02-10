// SPDX-License-Identifier: GPL-2.0
/*
 * mcu_axienet.c - AXI 1G/2.5G Ethernet Subsystem Driver (v4.0-poll)
 *
 * For MCU PCIe FPGA with:
 *   - XDMA in AXI-Stream mode (H2C_0/C2H_0 for TX/RX)
 *   - AXI Ethernet Subsystem (PG138) at BAR0 + 0x240000
 *   - ADIN1100 10BASE-T1L PHY via RGMII + MDIO
 *
 * v4.0 changes (from v3.0):
 *   - Full polling mode (no IRQ dependency) for TX completion + RX
 *   - C2H descriptor credits posted (CRITICAL FIX - was missing)
 *   - Force MAC speed to 10 Mbps for ADIN1100 (Generic PHY reports -1)
 *   - PHY interface mode set to MII (ADIN1100 MAC-side is MII/RGMII)
 *   - Extensive DMA state debug dumps
 *   - Optional IRQ mode via module param
 *
 * Architecture:
 *   Host <-PCIe-> XDMA <-AXI-Stream-> AXI Ethernet <-RGMII/MDIO-> ADIN1100 PHY
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include "mcu_pcie.h"

#define DRV_NAME    "mcu_axienet"
#define DRV_VERSION "4.0-poll"

/* ============================================================
 * Module parameters
 * ============================================================ */
static int debug = 2;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0=off, 1=info, 2=verbose, 3=trace)");

static int poll_mode = 1;
module_param(poll_mode, int, 0644);
MODULE_PARM_DESC(poll_mode, "1=polling (default), 0=IRQ-driven");

static int poll_interval_ms = 1;
module_param(poll_interval_ms, int, 0644);
MODULE_PARM_DESC(poll_interval_ms, "Polling interval in ms (default 1)");

#define DBG(lvl, priv, fmt, ...) do { \
    if (debug >= (lvl)) \
        dev_info((priv)->dev, fmt, ##__VA_ARGS__); \
} while (0)

/* ==========================================================================
 * AXI Ethernet Register Definitions (PG138 / PG051)
 * Base: BAR0 + 0x240000
 * ========================================================================== */

/* Soft registers (directly accessible) */
#define XAE_RAF         0x000   /* Reset and Address Filter */
#define XAE_TPF         0x004   /* TX Pause Frame */
#define XAE_IFGP        0x008   /* TX Inter-frame Gap */
#define XAE_IS          0x00C   /* Interrupt Status */
#define XAE_IP          0x010   /* Interrupt Pending */
#define XAE_IE          0x014   /* Interrupt Enable */
#define XAE_TTAG        0x018   /* TX VLAN Tag */
#define XAE_RTAG        0x01C   /* RX VLAN Tag */
#define XAE_UAWL        0x020   /* Unicast Address Lower */
#define XAE_UAWU        0x024   /* Unicast Address Upper */
#define XAE_TPID0       0x028   /* VLAN TPID Word 0 */
#define XAE_TPID1       0x02C   /* VLAN TPID Word 1 */
#define XAE_PPST        0x030   /* PCS/PMA Status */

/* TEMAC hard registers */
#define XAE_RCW0        0x400   /* RX Config Word 0 */
#define XAE_RCW1        0x404   /* RX Config Word 1 */
#define XAE_TC          0x408   /* TX Config */
#define XAE_FCC         0x40C   /* Flow Control Config */
#define XAE_SPEED       0x410   /* Speed Config */
#define XAE_RX_MAX      0x414   /* RX Max Frame Size */
#define XAE_TX_MAX      0x418   /* TX Max Frame Size */
#define XAE_TX_TS_ADJ   0x41C   /* TX Timestamp Adjust */
#define XAE_ID          0x4F8   /* Identification */
#define XAE_ABILITY     0x4FC   /* Ability */

/* MDIO registers (PG051 Table 2-33) */
#define XAE_MDIO_MC     0x500   /* MDIO Setup */
#define XAE_MDIO_MCR    0x504   /* MDIO Control */
#define XAE_MDIO_MWD    0x508   /* MDIO Write Data */
#define XAE_MDIO_MRD    0x50C   /* MDIO Read Data */

/* TEMAC Interrupt registers */
#define XAE_TEMAC_IS    0x600   /* TEMAC Interrupt Status */
#define XAE_TEMAC_IP    0x610   /* TEMAC Interrupt Pending */
#define XAE_TEMAC_IE    0x620   /* TEMAC Interrupt Enable */
#define XAE_TEMAC_IC    0x630   /* TEMAC Interrupt Clear */

/* Unicast Address (Frame Filter) */
#define XAE_UAW0        0x700   /* Unicast Address Word 0 */
#define XAE_UAW1        0x704   /* Unicast Address Word 1 */
#define XAE_FFC         0x708   /* Frame Filter Control */

/* ==========================================================================
 * Register Bit Definitions
 * ========================================================================== */

/* RAF bits */
#define XAE_RAF_MCSTREJ         BIT(1)
#define XAE_RAF_BCSTREJ         BIT(2)
#define XAE_RAF_TXVSTRP         BIT(3)
#define XAE_RAF_RXVSTRP         BIT(4)
#define XAE_RAF_NEWFNCENBL      BIT(11)

/* Interrupt Status/Pending/Enable bits (0x00C/0x010/0x014) */
#define XAE_INT_HARDACSCMPLT    BIT(0)
#define XAE_INT_AUTONEG         BIT(1)
#define XAE_INT_RXCMPLT         BIT(2)
#define XAE_INT_RXRJECT         BIT(3)
#define XAE_INT_RXMEMOVR        BIT(4)
#define XAE_INT_TXCMPLT         BIT(5)
#define XAE_INT_RXDCMLOCK       BIT(6)
#define XAE_INT_MGTRDY          BIT(7)
#define XAE_INT_PHYRSTCMPLT     BIT(8)
#define XAE_INT_ALL             0x1FF

/* RCW1 bits */
#define XAE_RCW1_RST            BIT(31)
#define XAE_RCW1_JUM            BIT(30)
#define XAE_RCW1_FCS            BIT(29)
#define XAE_RCW1_RX             BIT(28)
#define XAE_RCW1_VLAN           BIT(27)
#define XAE_RCW1_LT_DIS         BIT(25)

/* TC bits */
#define XAE_TC_RST              BIT(31)
#define XAE_TC_JUM              BIT(30)
#define XAE_TC_FCS              BIT(29)
#define XAE_TC_TX               BIT(28)
#define XAE_TC_VLAN             BIT(27)
#define XAE_TC_LB               BIT(1)  /* Loopback */

/* FCC bits */
#define XAE_FCC_FCTX            BIT(30)
#define XAE_FCC_FCRX            BIT(29)

/* Speed values */
#define XAE_SPEED_10            0
#define XAE_SPEED_100           1
#define XAE_SPEED_1000          2
#define XAE_SPEED_2500          3

/* MDIO Setup (0x500) */
#define XAE_MDIO_MC_CLOCK_DIV_MASK  0x3F
#define XAE_MDIO_MC_MDIOEN          BIT(6)

/* MDIO Control (0x504) */
#define XAE_MDIO_MCR_PHYAD_SHIFT    24
#define XAE_MDIO_MCR_PHYAD_MASK     (0x1F << 24)
#define XAE_MDIO_MCR_REGAD_SHIFT    16
#define XAE_MDIO_MCR_REGAD_MASK     (0x1F << 16)
#define XAE_MDIO_MCR_OP_SHIFT       14
#define XAE_MDIO_MCR_OP_WRITE       (1 << 14)
#define XAE_MDIO_MCR_OP_READ        (2 << 14)
#define XAE_MDIO_MCR_INITIATE       BIT(11)
#define XAE_MDIO_MCR_READY          BIT(7)

/* ==========================================================================
 * XDMA Register Definitions (BAR1)
 * ========================================================================== */

/* Channel base addresses */
#define XDMA_H2C_CHAN(n)        (0x0000 + ((n) * 0x100))
#define XDMA_C2H_CHAN(n)        (0x1000 + ((n) * 0x100))

/* Channel registers (offset from channel base) */
#define XDMA_CHAN_ID            0x00
#define XDMA_CHAN_CTRL          0x04
#define XDMA_CHAN_CTRL_W1S      0x08
#define XDMA_CHAN_CTRL_W1C      0x0C
#define XDMA_CHAN_STATUS        0x40
#define XDMA_CHAN_STATUS_RC     0x44
#define XDMA_CHAN_COMPLETED     0x48
#define XDMA_CHAN_ALIGNMENTS    0x4C
#define XDMA_CHAN_POLL_LO       0x88
#define XDMA_CHAN_POLL_HI       0x8C
#define XDMA_CHAN_INT_EN        0x90
#define XDMA_CHAN_INT_EN_W1S    0x94
#define XDMA_CHAN_INT_EN_W1C    0x98
#define XDMA_CHAN_PERF_CTRL     0xC0

/* SG DMA registers */
#define XDMA_SGDMA_DESC_LO      0x80
#define XDMA_SGDMA_DESC_HI      0x84
#define XDMA_SGDMA_DESC_ADJ     0x88
#define XDMA_SGDMA_DESC_CREDITS 0x8C  /* C2H descriptor credits - CRITICAL */

/* Control bits */
#define XDMA_CTRL_RUN           BIT(0)
#define XDMA_CTRL_IE_DESC_STOP  BIT(1)
#define XDMA_CTRL_IE_DESC_CMPLT BIT(2)
#define XDMA_CTRL_IE_ALIGN_ERR  BIT(3)
#define XDMA_CTRL_IE_MAGIC_STOP BIT(4)
#define XDMA_CTRL_IE_IDLE_STOP  BIT(6)
#define XDMA_CTRL_IE_RD_SLV_ERR BIT(9)
#define XDMA_CTRL_IE_WR_SLV_ERR BIT(10)
#define XDMA_CTRL_IE_DESC_ERR   BIT(19)
#define XDMA_CTRL_NON_INCR_ADDR BIT(25)
#define XDMA_CTRL_POLL_MODE     BIT(26)

/* Status bits */
#define XDMA_STAT_BUSY          BIT(0)
#define XDMA_STAT_DESC_STOP     BIT(1)
#define XDMA_STAT_DESC_CMPLT    BIT(2)
#define XDMA_STAT_ALIGN_ERR     BIT(3)
#define XDMA_STAT_MAGIC_STOP    BIT(4)
#define XDMA_STAT_IDLE_STOP     BIT(6)
#define XDMA_STAT_RD_SLV_ERR    BIT(9)
#define XDMA_STAT_WR_SLV_ERR    BIT(10)
#define XDMA_STAT_DESC_ERR      BIT(19)

/* ==========================================================================
 * XDMA Scatter-Gather Descriptor
 * ========================================================================== */

struct xdma_desc {
    __le32 control;
    __le32 bytes;
    __le64 src_addr;
    __le64 dst_addr;
    __le64 next_addr;
} __packed __aligned(32);

#define XDMA_DESC_MAGIC         0xAD4B0000
#define XDMA_DESC_STOPPED       BIT(0)
#define XDMA_DESC_COMPLETED     BIT(1)
#define XDMA_DESC_EOP           BIT(4)
#define XDMA_DESC_SOP           BIT(5)

/* ==========================================================================
 * Driver Constants
 * ========================================================================== */

#define TX_RING_SIZE        64
#define RX_RING_SIZE        64
#define RX_BUF_SIZE         2048
#define MAX_MTU             9000
#define MIN_MTU             60

#define MDIO_TIMEOUT_US     20000
#define RESET_TIMEOUT_US    50000

/* Known PHY IDs */
#define PHY_ID_ADIN1100         0x0283bc81
#define PHY_ID_XILINX_SGMII    0x01740c00
#define PHY_ID_DP83867          0x2000a231

/* ==========================================================================
 * Driver Private Data
 * ========================================================================== */

struct axienet_tx_buf {
    struct sk_buff *skb;
    dma_addr_t dma;
    u32 len;
};

struct axienet_rx_buf {
    void *data;
    dma_addr_t dma;
};

struct axienet_priv {
    struct net_device *ndev;
    struct device *dev;
    struct platform_device *pdev;
    struct pci_dev *pci_dev;

    /* Register bases */
    void __iomem *regs;         /* AXI Ethernet (BAR0 + 0x240000) */
    void __iomem *xdma;         /* XDMA (BAR1) */

    /* TX ring */
    struct xdma_desc *tx_ring;
    dma_addr_t tx_ring_dma;
    struct axienet_tx_buf tx_bufs[TX_RING_SIZE];
    u32 tx_head;
    u32 tx_tail;
    u32 tx_submitted;
    spinlock_t tx_lock;

    /* RX ring */
    struct xdma_desc *rx_ring;
    dma_addr_t rx_ring_dma;
    struct axienet_rx_buf rx_bufs[RX_RING_SIZE];
    u32 rx_head;
    u32 rx_tail;
    u32 rx_credits_posted;      /* Track how many C2H credits we've given */

    /* NAPI */
    struct napi_struct napi;

    /* PHY */
    struct mii_bus *mii_bus;
    struct phy_device *phydev;
    bool has_mdio;
    bool is_adin1100;           /* True if ADIN1100 detected */

    /* IRQ */
    int irq;

    /* Polling timer */
    struct timer_list poll_timer;
    bool poll_running;

    /* Statistics */
    u64 tx_packets;
    u64 tx_bytes;
    u64 rx_packets;
    u64 rx_bytes;
    u64 tx_errors;
    u64 rx_errors;
    u64 rx_dropped;
    u32 irq_count;
    u32 tx_irq_count;
    u32 rx_irq_count;
    u32 poll_count;
    u32 poll_rx_empty;
    u32 poll_tx_cleaned;
};

/* ==========================================================================
 * Register Access Helpers
 * ========================================================================== */

static inline u32 axienet_read(struct axienet_priv *priv, u32 reg)
{
    return readl(priv->regs + reg);
}

static inline void axienet_write(struct axienet_priv *priv, u32 reg, u32 val)
{
    writel(val, priv->regs + reg);
}

static inline u32 xdma_read(struct axienet_priv *priv, u32 reg)
{
    return readl(priv->xdma + reg);
}

static inline void xdma_write(struct axienet_priv *priv, u32 reg, u32 val)
{
    writel(val, priv->xdma + reg);
}

static inline u32 h2c_read(struct axienet_priv *priv, int ch, u32 reg)
{
    return xdma_read(priv, XDMA_H2C_CHAN(ch) + reg);
}

static inline void h2c_write(struct axienet_priv *priv, int ch, u32 reg, u32 val)
{
    xdma_write(priv, XDMA_H2C_CHAN(ch) + reg, val);
}

static inline u32 c2h_read(struct axienet_priv *priv, int ch, u32 reg)
{
    return xdma_read(priv, XDMA_C2H_CHAN(ch) + reg);
}

static inline void c2h_write(struct axienet_priv *priv, int ch, u32 reg, u32 val)
{
    xdma_write(priv, XDMA_C2H_CHAN(ch) + reg, val);
}

/* ==========================================================================
 * Debug: Dump DMA channel state
 * ========================================================================== */

static void axienet_dump_dma_state(struct axienet_priv *priv, const char *context)
{
    u32 h2c_id, h2c_ctrl, h2c_stat, h2c_cmplt;
    u32 c2h_id, c2h_ctrl, c2h_stat, c2h_cmplt;
    u32 c2h_desc_lo, c2h_desc_hi;

    if (debug < 2)
        return;

    h2c_id    = h2c_read(priv, 0, XDMA_CHAN_ID);
    h2c_ctrl  = h2c_read(priv, 0, XDMA_CHAN_CTRL);
    h2c_stat  = h2c_read(priv, 0, XDMA_CHAN_STATUS);
    h2c_cmplt = h2c_read(priv, 0, XDMA_CHAN_COMPLETED);

    c2h_id    = c2h_read(priv, 0, XDMA_CHAN_ID);
    c2h_ctrl  = c2h_read(priv, 0, XDMA_CHAN_CTRL);
    c2h_stat  = c2h_read(priv, 0, XDMA_CHAN_STATUS);
    c2h_cmplt = c2h_read(priv, 0, XDMA_CHAN_COMPLETED);

    c2h_desc_lo = c2h_read(priv, 0, XDMA_SGDMA_DESC_LO);
    c2h_desc_hi = c2h_read(priv, 0, XDMA_SGDMA_DESC_HI);

    dev_info(priv->dev, "=== DMA STATE [%s] ===\n", context);
    dev_info(priv->dev, "  H2C_0: ID=0x%08x CTRL=0x%08x STAT=0x%08x CMPLT=%u\n",
             h2c_id, h2c_ctrl, h2c_stat, h2c_cmplt);
    dev_info(priv->dev, "  C2H_0: ID=0x%08x CTRL=0x%08x STAT=0x%08x CMPLT=%u\n",
             c2h_id, c2h_ctrl, c2h_stat, c2h_cmplt);
    dev_info(priv->dev, "  C2H_0: DESC=0x%08x%08x credits_posted=%u\n",
             c2h_desc_hi, c2h_desc_lo, priv->rx_credits_posted);
    dev_info(priv->dev, "  TX: head=%u tail=%u submitted=%u\n",
             priv->tx_head, priv->tx_tail, priv->tx_submitted);
    dev_info(priv->dev, "  RX: head=%u tail=%u\n",
             priv->rx_head, priv->rx_tail);

    /* Also check if C2H has error bits */
    if (c2h_stat & (XDMA_STAT_ALIGN_ERR | XDMA_STAT_RD_SLV_ERR |
                    XDMA_STAT_WR_SLV_ERR | XDMA_STAT_DESC_ERR))
        dev_err(priv->dev, "  C2H_0 ERRORS: 0x%08x\n", c2h_stat);

    if (h2c_stat & (XDMA_STAT_ALIGN_ERR | XDMA_STAT_RD_SLV_ERR |
                    XDMA_STAT_WR_SLV_ERR | XDMA_STAT_DESC_ERR))
        dev_err(priv->dev, "  H2C_0 ERRORS: 0x%08x\n", h2c_stat);
}

/* Dump first N RX descriptors */
static void axienet_dump_rx_descs(struct axienet_priv *priv, int count)
{
    int i;

    if (debug < 2)
        return;

    dev_info(priv->dev, "=== RX Descriptors (first %d) ===\n", count);
    for (i = 0; i < count && i < RX_RING_SIZE; i++) {
        struct xdma_desc *d = &priv->rx_ring[i];
        dev_info(priv->dev, "  [%2d] ctrl=0x%08x bytes=%u dst=0x%llx next=0x%llx\n",
                 i, le32_to_cpu(d->control), le32_to_cpu(d->bytes),
                 le64_to_cpu(d->dst_addr), le64_to_cpu(d->next_addr));
    }
}

/* ==========================================================================
 * MDIO Bus Operations
 * ========================================================================== */

static int axienet_mdio_wait(struct axienet_priv *priv)
{
    int timeout = MDIO_TIMEOUT_US;
    u32 val;

    while (timeout > 0) {
        val = axienet_read(priv, XAE_MDIO_MCR);
        if (val & XAE_MDIO_MCR_READY)
            return 0;
        udelay(1);
        timeout--;
    }

    dev_err(priv->dev, "MDIO timeout! MCR=0x%08x MC=0x%08x\n",
            axienet_read(priv, XAE_MDIO_MCR),
            axienet_read(priv, XAE_MDIO_MC));
    return -ETIMEDOUT;
}

static int axienet_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
    struct axienet_priv *priv = bus->priv;
    u32 ctrl;
    int ret;

    ret = axienet_mdio_wait(priv);
    if (ret)
        return ret;

    ctrl = ((phy_id & 0x1F) << XAE_MDIO_MCR_PHYAD_SHIFT) |
           ((reg & 0x1F) << XAE_MDIO_MCR_REGAD_SHIFT) |
           XAE_MDIO_MCR_OP_READ |
           XAE_MDIO_MCR_INITIATE;

    DBG(3, priv, "MDIO read: phy=%d reg=%d ctrl=0x%08x\n", phy_id, reg, ctrl);
    axienet_write(priv, XAE_MDIO_MCR, ctrl);

    ret = axienet_mdio_wait(priv);
    if (ret)
        return ret;

    ret = axienet_read(priv, XAE_MDIO_MRD) & 0xFFFF;
    DBG(3, priv, "MDIO read result: 0x%04x\n", ret);
    return ret;
}

static int axienet_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 val)
{
    struct axienet_priv *priv = bus->priv;
    u32 ctrl;
    int ret;

    ret = axienet_mdio_wait(priv);
    if (ret)
        return ret;

    axienet_write(priv, XAE_MDIO_MWD, val);

    ctrl = ((phy_id & 0x1F) << XAE_MDIO_MCR_PHYAD_SHIFT) |
           ((reg & 0x1F) << XAE_MDIO_MCR_REGAD_SHIFT) |
           XAE_MDIO_MCR_OP_WRITE |
           XAE_MDIO_MCR_INITIATE;

    DBG(3, priv, "MDIO write: phy=%d reg=%d val=0x%04x ctrl=0x%08x\n",
        phy_id, reg, val, ctrl);
    axienet_write(priv, XAE_MDIO_MCR, ctrl);

    return axienet_mdio_wait(priv);
}

static int axienet_mdio_setup(struct axienet_priv *priv)
{
    struct mii_bus *bus;
    u32 mc_val, mcr_val;
    int ret, clk_div;

    mc_val = axienet_read(priv, XAE_MDIO_MC);
    mcr_val = axienet_read(priv, XAE_MDIO_MCR);
    dev_info(priv->dev, "MDIO initial state: MC=0x%08x MCR=0x%08x\n",
             mc_val, mcr_val);

    clk_div = 49;  /* 1 MHz MDC */

    mc_val = (clk_div & XAE_MDIO_MC_CLOCK_DIV_MASK) | XAE_MDIO_MC_MDIOEN;
    axienet_write(priv, XAE_MDIO_MC, mc_val);

    dev_info(priv->dev, "MDIO setup: MC=0x%08x (clk_div=%d, MDIOEN=1)\n",
             mc_val, clk_div);

    msleep(10);

    mcr_val = axienet_read(priv, XAE_MDIO_MCR);
    dev_info(priv->dev, "MDIO after setup: MCR=0x%08x (ready=%d)\n",
             mcr_val, !!(mcr_val & XAE_MDIO_MCR_READY));

    if (!(mcr_val & XAE_MDIO_MCR_READY))
        dev_warn(priv->dev, "MDIO not ready - PHY access may fail\n");

    bus = mdiobus_alloc();
    if (!bus)
        return -ENOMEM;

    bus->name = "mcu_axienet_mdio";
    snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(priv->dev));
    bus->priv = priv;
    bus->parent = priv->dev;
    bus->read = axienet_mdio_read;
    bus->write = axienet_mdio_write;

    ret = mdiobus_register(bus);
    if (ret) {
        dev_err(priv->dev, "MDIO bus registration failed: %d\n", ret);
        mdiobus_free(bus);
        return ret;
    }

    priv->mii_bus = bus;
    priv->has_mdio = true;
    dev_info(priv->dev, "MDIO bus registered\n");

    return 0;
}

static void axienet_mdio_teardown(struct axienet_priv *priv)
{
    if (priv->mii_bus) {
        mdiobus_unregister(priv->mii_bus);
        mdiobus_free(priv->mii_bus);
        priv->mii_bus = NULL;
    }
}

/* ==========================================================================
 * PHY Handling
 * ========================================================================== */

static void axienet_adjust_link(struct net_device *ndev)
{
    struct axienet_priv *priv = netdev_priv(ndev);
    struct phy_device *phydev = priv->phydev;
    u32 speed_reg;

    if (!phydev)
        return;

    DBG(1, priv, "adjust_link: link=%d speed=%d duplex=%d\n",
        phydev->link, phydev->speed, phydev->duplex);

    if (!phydev->link) {
        if (netif_carrier_ok(ndev)) {
            netif_carrier_off(ndev);
            netdev_info(ndev, "Link down\n");
        }
        return;
    }

    /*
     * ADIN1100 FIX: Generic PHY driver reports speed=-1 because it
     * doesn't understand ADIN1100's 10BASE-T1L registers.
     * Force 10 Mbps for ADIN1100.
     */
    if (priv->is_adin1100 && (phydev->speed <= 0 || phydev->speed == SPEED_UNKNOWN)) {
        DBG(1, priv, "ADIN1100: Generic PHY reports speed=%d, forcing 10 Mbps\n",
            phydev->speed);
        phydev->speed = SPEED_10;
        phydev->duplex = DUPLEX_FULL;  /* T1L is always full duplex */
    }

    switch (phydev->speed) {
    case SPEED_10:
        speed_reg = XAE_SPEED_10;
        break;
    case SPEED_100:
        speed_reg = XAE_SPEED_100;
        break;
    case SPEED_1000:
        speed_reg = XAE_SPEED_1000;
        break;
    case SPEED_2500:
        speed_reg = XAE_SPEED_2500;
        break;
    default:
        dev_warn(priv->dev, "Unknown speed %d, forcing 10 for ADIN1100\n",
                 phydev->speed);
        speed_reg = XAE_SPEED_10;
        phydev->speed = SPEED_10;
    }

    DBG(1, priv, "Writing SPEED reg = 0x%x (speed=%d)\n", speed_reg, phydev->speed);
    axienet_write(priv, XAE_SPEED, speed_reg);

    if (!netif_carrier_ok(ndev)) {
        netif_carrier_on(ndev);
        netdev_info(ndev, "Link up: %d Mbps, %s duplex (SPEED=0x%x PPST=0x%08x)\n",
                    phydev->speed,
                    phydev->duplex == DUPLEX_FULL ? "full" : "half",
                    speed_reg, axienet_read(priv, XAE_PPST));
    }
}

/*
 * Scan MDIO bus and report all PHYs found
 */
static void axienet_mdio_scan(struct axienet_priv *priv)
{
    struct mii_bus *bus = priv->mii_bus;
    int addr, id1, id2;
    u32 phy_id;

    dev_info(priv->dev, "=== MDIO Bus Scan ===\n");

    for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
        id1 = mdiobus_read(bus, addr, MII_PHYSID1);
        if (id1 < 0 || id1 == 0xFFFF || id1 == 0)
            continue;

        id2 = mdiobus_read(bus, addr, MII_PHYSID2);
        if (id2 < 0 || id2 == 0xFFFF)
            continue;

        phy_id = (id1 << 16) | id2;

        dev_info(priv->dev, "  Addr %2d: ID 0x%08x", addr, phy_id);

        if (phy_id == PHY_ID_ADIN1100)
            pr_cont(" [ADIN1100 - External PHY]\n");
        else if ((phy_id & 0xFFFF0000) == 0x01740000)
            pr_cont(" [Xilinx SGMII PCS/PMA - Internal]\n");
        else if ((phy_id & 0xFFFFFFF0) == 0x2000a230)
            pr_cont(" [DP83867 - TI Gigabit]\n");
        else
            pr_cont("\n");
    }

    dev_info(priv->dev, "SGMII PCS Status (PPST=0x%08x)\n",
             axienet_read(priv, XAE_PPST));
}

/*
 * Try Clause 45 MMD access to find ADIN1100
 */
static u32 axienet_read_c45_phy_id(struct axienet_priv *priv, int addr, int devad)
{
    struct mii_bus *bus = priv->mii_bus;
    int id1, id2;

    /* C45-over-C22: indirect access via registers 13/14 */
    mdiobus_write(bus, addr, MII_MMD_CTRL, devad);
    mdiobus_write(bus, addr, MII_MMD_DATA, 2);  /* DEVID1 */
    mdiobus_write(bus, addr, MII_MMD_CTRL, (1 << 14) | devad);
    id1 = mdiobus_read(bus, addr, MII_MMD_DATA);

    if (id1 < 0 || id1 == 0xFFFF || id1 == 0)
        return 0;

    mdiobus_write(bus, addr, MII_MMD_CTRL, devad);
    mdiobus_write(bus, addr, MII_MMD_DATA, 3);  /* DEVID2 */
    mdiobus_write(bus, addr, MII_MMD_CTRL, (1 << 14) | devad);
    id2 = mdiobus_read(bus, addr, MII_MMD_DATA);

    if (id2 < 0 || id2 == 0xFFFF)
        return 0;

    return (id1 << 16) | id2;
}

static void axienet_mdio_scan_c45(struct axienet_priv *priv)
{
    int addr, devad;
    u32 phy_id;

    dev_info(priv->dev, "=== MDIO Clause 45 Scan ===\n");

    for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
        for (devad = 1; devad <= 7; devad++) {
            phy_id = axienet_read_c45_phy_id(priv, addr, devad);
            if (phy_id && phy_id != 0xFFFFFFFF) {
                dev_info(priv->dev, "  Addr %d MMD %d: ID 0x%08x%s\n",
                         addr, devad, phy_id,
                         (phy_id == PHY_ID_ADIN1100) ? " [ADIN1100!]" : "");
            }
        }
    }
}

static int axienet_phy_connect(struct axienet_priv *priv)
{
    struct phy_device *phydev;
    int i;

    if (!priv->mii_bus) {
        dev_info(priv->dev, "No MDIO bus - using fixed link\n");
        return 0;
    }

    /* Scan and report */
    axienet_mdio_scan(priv);
    axienet_mdio_scan_c45(priv);

    /* Find PHY - prefer ADIN1100 */
    phydev = NULL;

    for (i = 0; i < PHY_MAX_ADDR; i++) {
        struct phy_device *p = mdiobus_get_phy(priv->mii_bus, i);
        if (p && p->phy_id == PHY_ID_ADIN1100) {
            phydev = p;
            dev_info(priv->dev, "Found ADIN1100 at address %d\n", i);
            break;
        }
    }

    /* Fallback: any non-internal PHY */
    if (!phydev) {
        for (i = 0; i < PHY_MAX_ADDR; i++) {
            struct phy_device *p = mdiobus_get_phy(priv->mii_bus, i);
            if (p && p->phy_id != 0 && p->phy_id != 0xFFFFFFFF &&
                (p->phy_id & 0xFFFF0000) != 0x01740000) {
                phydev = p;
                dev_info(priv->dev, "Using PHY at address %d (ID 0x%08x)\n",
                         i, p->phy_id);
                break;
            }
        }
    }

    /* Last resort: any PHY */
    if (!phydev)
        phydev = phy_find_first(priv->mii_bus);

    if (!phydev) {
        dev_warn(priv->dev, "No PHY found - using fixed link\n");
        return 0;
    }

    dev_info(priv->dev, "Connecting to PHY at address %d, ID 0x%08x\n",
             phydev->mdio.addr, phydev->phy_id);

    /*
     * ADIN1100 10BASE-T1L:
     *   - The MAC-side interface in firmware is RGMII
     *   - But the Generic PHY driver doesn't support RGMII for ADIN1100
     *   - Use MII as the interface mode - phylib handles the rest
     *   - The CRITICAL thing is MAC speed matches PHY speed (10 Mbps)
     *
     * The interface mode tells phylib what signaling to expect between
     * MAC and PHY. With ADIN1100, the AXI Ethernet subsystem handles
     * the RGMII signaling internally - we just need to tell phylib
     * something it accepts.
     */
    if (phydev->phy_id == PHY_ID_ADIN1100) {
        priv->is_adin1100 = true;

        /* Try RGMII first (matches firmware), fall back to MII */
        phydev = phy_connect(priv->ndev, phydev_name(phydev),
                             axienet_adjust_link, PHY_INTERFACE_MODE_MII);
        if (IS_ERR(phydev)) {
            dev_warn(priv->dev, "MII connect failed: %ld, trying RMII\n",
                     PTR_ERR(phydev));
            phydev = phy_connect(priv->ndev, phydev_name(phydev),
                                 axienet_adjust_link, PHY_INTERFACE_MODE_RMII);
        }

        if (!IS_ERR(phydev)) {
            /* Force 10 Mbps for 10BASE-T1L */
            phy_set_max_speed(phydev, SPEED_10);
            dev_info(priv->dev, "ADIN1100: 10BASE-T1L, 10 Mbps, full duplex\n");
        }
    } else {
        /* Other PHYs: try RGMII_ID */
        phydev = phy_connect(priv->ndev, phydev_name(phydev),
                             axienet_adjust_link, PHY_INTERFACE_MODE_RGMII_ID);
        if (IS_ERR(phydev)) {
            phydev = phy_connect(priv->ndev, phydev_name(phydev),
                                 axienet_adjust_link, PHY_INTERFACE_MODE_RGMII);
        }
    }

    if (IS_ERR(phydev)) {
        dev_err(priv->dev, "PHY connect failed: %ld\n", PTR_ERR(phydev));
        return PTR_ERR(phydev);
    }

    priv->phydev = phydev;
    dev_info(priv->dev, "Connected to PHY: %s (driver: %s, interface: %s)\n",
             phydev_name(phydev),
             phydev->drv ? phydev->drv->name : "generic",
             phy_modes(phydev->interface));

    return 0;
}

static void axienet_phy_disconnect(struct axienet_priv *priv)
{
    if (priv->phydev) {
        phy_stop(priv->phydev);
        phy_disconnect(priv->phydev);
        priv->phydev = NULL;
    }
}

/* ==========================================================================
 * MAC Configuration
 * ========================================================================== */

static void axienet_set_mac_address(struct net_device *ndev)
{
    struct axienet_priv *priv = netdev_priv(ndev);
    u32 lo, hi;

    lo = (ndev->dev_addr[2] << 24) | (ndev->dev_addr[3] << 16) |
         (ndev->dev_addr[4] << 8)  | ndev->dev_addr[5];
    hi = (ndev->dev_addr[0] << 8)  | ndev->dev_addr[1];

    axienet_write(priv, XAE_UAW0, lo);
    axienet_write(priv, XAE_UAW1, hi);

    DBG(1, priv, "MAC address set: %pM (UAW0=0x%08x UAW1=0x%08x)\n",
        ndev->dev_addr, lo, hi);
}

static int axienet_mac_init(struct axienet_priv *priv)
{
    u32 val;
    int timeout;

    DBG(1, priv, "Initializing MAC...\n");

    /* Reset RX */
    axienet_write(priv, XAE_RCW1, XAE_RCW1_RST);
    timeout = RESET_TIMEOUT_US;
    while (timeout > 0) {
        val = axienet_read(priv, XAE_RCW1);
        if (!(val & XAE_RCW1_RST))
            break;
        udelay(1);
        timeout--;
    }
    if (timeout <= 0)
        dev_warn(priv->dev, "RX reset timeout\n");

    /* Reset TX */
    axienet_write(priv, XAE_TC, XAE_TC_RST);
    timeout = RESET_TIMEOUT_US;
    while (timeout > 0) {
        val = axienet_read(priv, XAE_TC);
        if (!(val & XAE_TC_RST))
            break;
        udelay(1);
        timeout--;
    }
    if (timeout <= 0)
        dev_warn(priv->dev, "TX reset timeout\n");

    /* Configure RX: enable */
    axienet_write(priv, XAE_RCW1, XAE_RCW1_RX);

    /* Configure TX: enable + FCS generation */
    axienet_write(priv, XAE_TC, XAE_TC_TX | XAE_TC_FCS);

    /* Flow control */
    axienet_write(priv, XAE_FCC, XAE_FCC_FCTX | XAE_FCC_FCRX);

    /* Frame sizes */
    axienet_write(priv, XAE_RX_MAX, 1518);
    axienet_write(priv, XAE_TX_MAX, 1518);

    /*
     * Speed: Set to 10 Mbps for ADIN1100 (10BASE-T1L).
     * This was defaulting to 1000 Mbps which is WRONG for ADIN1100.
     * adjust_link will update it once PHY reports, but we need a sane default.
     */
    if (priv->is_adin1100) {
        axienet_write(priv, XAE_SPEED, XAE_SPEED_10);
        DBG(1, priv, "MAC speed defaulted to 10 Mbps (ADIN1100)\n");
    } else {
        axienet_write(priv, XAE_SPEED, XAE_SPEED_1000);
        DBG(1, priv, "MAC speed defaulted to 1000 Mbps\n");
    }

    /* Set MAC address */
    axienet_set_mac_address(priv->ndev);

    /* Enable new function mode + accept broadcast */
    axienet_write(priv, XAE_RAF, XAE_RAF_NEWFNCENBL);

    /* Clear all pending interrupts */
    axienet_write(priv, XAE_IS, XAE_INT_ALL);

    /* Enable relevant MAC interrupts */
    axienet_write(priv, XAE_IE, XAE_INT_RXCMPLT | XAE_INT_TXCMPLT |
                                XAE_INT_RXRJECT | XAE_INT_RXMEMOVR);

    /* Dump state */
    DBG(1, priv, "MAC init complete:\n");
    DBG(1, priv, "  ID=0x%08x RCW1=0x%08x TC=0x%08x\n",
        axienet_read(priv, XAE_ID),
        axienet_read(priv, XAE_RCW1),
        axienet_read(priv, XAE_TC));
    DBG(1, priv, "  SPEED=0x%08x FCC=0x%08x RAF=0x%08x\n",
        axienet_read(priv, XAE_SPEED),
        axienet_read(priv, XAE_FCC),
        axienet_read(priv, XAE_RAF));
    DBG(1, priv, "  IS=0x%08x IE=0x%08x PPST=0x%08x\n",
        axienet_read(priv, XAE_IS),
        axienet_read(priv, XAE_IE),
        axienet_read(priv, XAE_PPST));

    return 0;
}

/* ==========================================================================
 * DMA Ring Management
 * ========================================================================== */

static int axienet_tx_ring_init(struct axienet_priv *priv)
{
    int i;

    priv->tx_ring = dma_alloc_coherent(&priv->pci_dev->dev,
                                       TX_RING_SIZE * sizeof(struct xdma_desc),
                                       &priv->tx_ring_dma, GFP_KERNEL);
    if (!priv->tx_ring)
        return -ENOMEM;

    memset(priv->tx_ring, 0, TX_RING_SIZE * sizeof(struct xdma_desc));

    for (i = 0; i < TX_RING_SIZE; i++) {
        priv->tx_bufs[i].skb = NULL;
        priv->tx_bufs[i].dma = 0;
    }

    priv->tx_head = 0;
    priv->tx_tail = 0;
    priv->tx_submitted = 0;

    DBG(1, priv, "TX ring: %d descriptors at %pad\n",
        TX_RING_SIZE, &priv->tx_ring_dma);

    return 0;
}

static void axienet_tx_ring_free(struct axienet_priv *priv)
{
    int i;

    for (i = 0; i < TX_RING_SIZE; i++) {
        if (priv->tx_bufs[i].skb) {
            dma_unmap_single(&priv->pci_dev->dev,
                            priv->tx_bufs[i].dma,
                            priv->tx_bufs[i].len,
                            DMA_TO_DEVICE);
            dev_kfree_skb_any(priv->tx_bufs[i].skb);
            priv->tx_bufs[i].skb = NULL;
        }
    }

    if (priv->tx_ring) {
        dma_free_coherent(&priv->pci_dev->dev,
                         TX_RING_SIZE * sizeof(struct xdma_desc),
                         priv->tx_ring, priv->tx_ring_dma);
        priv->tx_ring = NULL;
    }
}

static int axienet_rx_ring_init(struct axienet_priv *priv)
{
    int i;

    priv->rx_ring = dma_alloc_coherent(&priv->pci_dev->dev,
                                       RX_RING_SIZE * sizeof(struct xdma_desc),
                                       &priv->rx_ring_dma, GFP_KERNEL);
    if (!priv->rx_ring)
        return -ENOMEM;

    memset(priv->rx_ring, 0, RX_RING_SIZE * sizeof(struct xdma_desc));

    /* Pre-allocate RX buffers and setup descriptors */
    for (i = 0; i < RX_RING_SIZE; i++) {
        priv->rx_bufs[i].data = kmalloc(RX_BUF_SIZE, GFP_KERNEL);
        if (!priv->rx_bufs[i].data)
            goto err_free;

        priv->rx_bufs[i].dma = dma_map_single(&priv->pci_dev->dev,
                                              priv->rx_bufs[i].data,
                                              RX_BUF_SIZE,
                                              DMA_FROM_DEVICE);
        if (dma_mapping_error(&priv->pci_dev->dev, priv->rx_bufs[i].dma)) {
            kfree(priv->rx_bufs[i].data);
            priv->rx_bufs[i].data = NULL;
            goto err_free;
        }

        /* Setup descriptor */
        priv->rx_ring[i].control = cpu_to_le32(XDMA_DESC_MAGIC);
        priv->rx_ring[i].bytes = cpu_to_le32(RX_BUF_SIZE);
        priv->rx_ring[i].dst_addr = cpu_to_le64(priv->rx_bufs[i].dma);
        priv->rx_ring[i].src_addr = 0;

        /* Chain to next descriptor */
        if (i < RX_RING_SIZE - 1)
            priv->rx_ring[i].next_addr = cpu_to_le64(priv->rx_ring_dma +
                                                     (i + 1) * sizeof(struct xdma_desc));
        else
            priv->rx_ring[i].next_addr = cpu_to_le64(priv->rx_ring_dma); /* Wrap */
    }

    priv->rx_head = 0;
    priv->rx_tail = 0;
    priv->rx_credits_posted = 0;

    DBG(1, priv, "RX ring: %d descriptors at %pad (chained, wrap-around)\n",
        RX_RING_SIZE, &priv->rx_ring_dma);

    return 0;

err_free:
    for (i--; i >= 0; i--) {
        dma_unmap_single(&priv->pci_dev->dev,
                        priv->rx_bufs[i].dma,
                        RX_BUF_SIZE, DMA_FROM_DEVICE);
        kfree(priv->rx_bufs[i].data);
    }
    dma_free_coherent(&priv->pci_dev->dev,
                     RX_RING_SIZE * sizeof(struct xdma_desc),
                     priv->rx_ring, priv->rx_ring_dma);
    priv->rx_ring = NULL;
    return -ENOMEM;
}

static void axienet_rx_ring_free(struct axienet_priv *priv)
{
    int i;

    for (i = 0; i < RX_RING_SIZE; i++) {
        if (priv->rx_bufs[i].data) {
            dma_unmap_single(&priv->pci_dev->dev,
                            priv->rx_bufs[i].dma,
                            RX_BUF_SIZE, DMA_FROM_DEVICE);
            kfree(priv->rx_bufs[i].data);
            priv->rx_bufs[i].data = NULL;
        }
    }

    if (priv->rx_ring) {
        dma_free_coherent(&priv->pci_dev->dev,
                         RX_RING_SIZE * sizeof(struct xdma_desc),
                         priv->rx_ring, priv->rx_ring_dma);
        priv->rx_ring = NULL;
    }
}

/* ==========================================================================
 * XDMA Channel Control
 * ========================================================================== */

static void axienet_dma_start(struct axienet_priv *priv)
{
    u32 c2h_id, h2c_id;

    /* Read channel IDs to verify they exist */
    h2c_id = h2c_read(priv, 0, XDMA_CHAN_ID);
    c2h_id = c2h_read(priv, 0, XDMA_CHAN_ID);
    dev_info(priv->dev, "XDMA H2C_0 ID=0x%08x  C2H_0 ID=0x%08x\n", h2c_id, c2h_id);

    /* ---- Stop both channels first ---- */
    c2h_write(priv, 0, XDMA_CHAN_CTRL_W1C, XDMA_CTRL_RUN);
    h2c_write(priv, 0, XDMA_CHAN_CTRL_W1C, XDMA_CTRL_RUN);
    wmb();
    udelay(10);

    /* ---- Clear any pending status ---- */
    c2h_read(priv, 0, XDMA_CHAN_STATUS_RC);
    h2c_read(priv, 0, XDMA_CHAN_STATUS_RC);

    /* ================================================================
     * Setup C2H (RX) channel - THIS WAS THE CRITICAL MISSING PIECE
     * ================================================================ */
    DBG(1, priv, "Starting RX DMA (C2H_0)...\n");

    /* Set descriptor ring address */
    c2h_write(priv, 0, XDMA_SGDMA_DESC_LO, lower_32_bits(priv->rx_ring_dma));
    c2h_write(priv, 0, XDMA_SGDMA_DESC_HI, upper_32_bits(priv->rx_ring_dma));

    /* Set adjacent descriptor count (for prefetch optimization) */
    c2h_write(priv, 0, XDMA_SGDMA_DESC_ADJ, RX_RING_SIZE - 1);

    /*
     * *** CRITICAL FIX: Post descriptor credits for C2H ***
     *
     * In XDMA C2H Scatter-Gather mode, the engine needs to know how many
     * descriptors are available for it to write into. Without posting
     * credits, the C2H engine has ZERO buffers and silently drops all
     * incoming packets.
     *
     * This was the PRIMARY reason RX wasn't working!
     */
    wmb();  /* Ensure descriptors are visible before posting credits */
    c2h_write(priv, 0, XDMA_SGDMA_DESC_CREDITS, RX_RING_SIZE);
    priv->rx_credits_posted = RX_RING_SIZE;

    dev_info(priv->dev, "C2H_0: Posted %d descriptor credits (CRITICAL)\n",
             RX_RING_SIZE);

    /* Enable interrupts (even in poll mode, for status tracking) */
    if (!poll_mode)
        c2h_write(priv, 0, XDMA_CHAN_INT_EN_W1S, XDMA_CTRL_IE_DESC_CMPLT);

    /* Start C2H channel */
    wmb();
    c2h_write(priv, 0, XDMA_CHAN_CTRL_W1S, XDMA_CTRL_RUN);

    DBG(1, priv, "C2H_0: CTRL=0x%08x STATUS=0x%08x\n",
        c2h_read(priv, 0, XDMA_CHAN_CTRL),
        c2h_read(priv, 0, XDMA_CHAN_STATUS));

    /* H2C setup (TX - started per-packet, just verify it's there) */
    DBG(1, priv, "H2C_0: CTRL=0x%08x STATUS=0x%08x\n",
        h2c_read(priv, 0, XDMA_CHAN_CTRL),
        h2c_read(priv, 0, XDMA_CHAN_STATUS));

    /* Dump initial state */
    axienet_dump_dma_state(priv, "after DMA start");
    axienet_dump_rx_descs(priv, 4);

    DBG(1, priv, "DMA channels ready (mode=%s)\n",
        poll_mode ? "POLLING" : "IRQ");
}

static void axienet_dma_stop(struct axienet_priv *priv)
{
    /* Stop C2H (RX) */
    c2h_write(priv, 0, XDMA_CHAN_CTRL_W1C, XDMA_CTRL_RUN);
    c2h_write(priv, 0, XDMA_CHAN_INT_EN_W1C, 0xFFFFFFFF);

    /* Stop H2C (TX) */
    h2c_write(priv, 0, XDMA_CHAN_CTRL_W1C, XDMA_CTRL_RUN);
    h2c_write(priv, 0, XDMA_CHAN_INT_EN_W1C, 0xFFFFFFFF);

    DBG(1, priv, "DMA channels stopped\n");
    axienet_dump_dma_state(priv, "after DMA stop");
}

/* ==========================================================================
 * TX Path
 * ========================================================================== */

static netdev_tx_t axienet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct axienet_priv *priv = netdev_priv(ndev);
    struct xdma_desc *desc;
    dma_addr_t dma;
    unsigned long flags;
    u32 head;

    spin_lock_irqsave(&priv->tx_lock, flags);

    /* Check for space */
    if (((priv->tx_head + 1) % TX_RING_SIZE) == priv->tx_tail) {
        netif_stop_queue(ndev);
        spin_unlock_irqrestore(&priv->tx_lock, flags);
        return NETDEV_TX_BUSY;
    }

    head = priv->tx_head;

    /* Map SKB data */
    dma = dma_map_single(&priv->pci_dev->dev, skb->data, skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&priv->pci_dev->dev, dma)) {
        spin_unlock_irqrestore(&priv->tx_lock, flags);
        dev_kfree_skb_any(skb);
        priv->tx_errors++;
        return NETDEV_TX_OK;
    }

    /* Store buffer info */
    priv->tx_bufs[head].skb = skb;
    priv->tx_bufs[head].dma = dma;
    priv->tx_bufs[head].len = skb->len;

    /* Build descriptor */
    desc = &priv->tx_ring[head];
    desc->control = cpu_to_le32(XDMA_DESC_MAGIC | XDMA_DESC_SOP | XDMA_DESC_EOP);
    desc->bytes = cpu_to_le32(skb->len);
    desc->src_addr = cpu_to_le64(dma);
    desc->dst_addr = 0;
    desc->next_addr = 0;  /* Single descriptor */

    wmb();

    /* Submit to H2C channel */
    h2c_write(priv, 0, XDMA_SGDMA_DESC_LO,
              lower_32_bits(priv->tx_ring_dma + head * sizeof(struct xdma_desc)));
    h2c_write(priv, 0, XDMA_SGDMA_DESC_HI,
              upper_32_bits(priv->tx_ring_dma + head * sizeof(struct xdma_desc)));
    h2c_write(priv, 0, XDMA_SGDMA_DESC_ADJ, 0);

    /* Start transfer */
    wmb();
    h2c_write(priv, 0, XDMA_CHAN_CTRL_W1S, XDMA_CTRL_RUN);

    priv->tx_head = (head + 1) % TX_RING_SIZE;
    priv->tx_submitted++;

    spin_unlock_irqrestore(&priv->tx_lock, flags);

    /* Packet tracing */
    if (debug >= 2) {
        struct ethhdr *eth = (struct ethhdr *)skb->data;
        dev_info(priv->dev, "TX[%u]: len=%d dst=%pM src=%pM proto=0x%04x\n",
                 head, skb->len, eth->h_dest, eth->h_source, ntohs(eth->h_proto));
    }

    return NETDEV_TX_OK;
}

static int axienet_tx_complete(struct axienet_priv *priv)
{
    unsigned long flags;
    u32 tail;
    int cleaned = 0;

    spin_lock_irqsave(&priv->tx_lock, flags);

    while (priv->tx_tail != priv->tx_head) {
        tail = priv->tx_tail;

        /* Check if this descriptor is complete */
        if (!(le32_to_cpu(priv->tx_ring[tail].control) & XDMA_DESC_COMPLETED))
            break;

        if (priv->tx_bufs[tail].skb) {
            dma_unmap_single(&priv->pci_dev->dev,
                            priv->tx_bufs[tail].dma,
                            priv->tx_bufs[tail].len,
                            DMA_TO_DEVICE);

            priv->tx_packets++;
            priv->tx_bytes += priv->tx_bufs[tail].skb->len;
            dev_kfree_skb_any(priv->tx_bufs[tail].skb);
            priv->tx_bufs[tail].skb = NULL;
        }

        priv->tx_tail = (tail + 1) % TX_RING_SIZE;
        cleaned++;
    }

    /* Wake queue if we have space */
    if (netif_queue_stopped(priv->ndev)) {
        int space = (priv->tx_tail - priv->tx_head - 1 + TX_RING_SIZE) % TX_RING_SIZE;
        if (space >= TX_RING_SIZE / 4)
            netif_wake_queue(priv->ndev);
    }

    spin_unlock_irqrestore(&priv->tx_lock, flags);

    return cleaned;
}

/* ==========================================================================
 * RX Path (polling-compatible)
 * ========================================================================== */

static int axienet_rx_process(struct axienet_priv *priv, int budget)
{
    int work_done = 0;
    u32 tail, len;
    struct xdma_desc *desc;
    struct sk_buff *skb;
    void *data;
    int credits_to_post = 0;

    while (work_done < budget) {
        tail = priv->rx_tail;
        desc = &priv->rx_ring[tail];

        /* Read descriptor control - need rmb to see device writes */
        rmb();

        /* Check if descriptor is complete */
        if (!(le32_to_cpu(desc->control) & XDMA_DESC_COMPLETED))
            break;

        len = le32_to_cpu(desc->bytes);
        data = priv->rx_bufs[tail].data;

        /* Sync DMA buffer */
        dma_sync_single_for_cpu(&priv->pci_dev->dev,
                               priv->rx_bufs[tail].dma,
                               len, DMA_FROM_DEVICE);

        /* Packet tracing */
        if (debug >= 2 && len >= sizeof(struct ethhdr)) {
            struct ethhdr *eth = (struct ethhdr *)data;
            dev_info(priv->dev, "RX[%u]: len=%d dst=%pM src=%pM proto=0x%04x\n",
                     tail, len, eth->h_dest, eth->h_source, ntohs(eth->h_proto));
            if (debug >= 3) {
                print_hex_dump(KERN_INFO, "RX DATA: ", DUMP_PREFIX_OFFSET,
                              16, 1, data, min((int)len, 64), true);
            }
        }

        /* Allocate SKB and copy data */
        skb = netdev_alloc_skb_ip_align(priv->ndev, len);
        if (skb) {
            memcpy(skb_put(skb, len), data, len);
            skb->protocol = eth_type_trans(skb, priv->ndev);

            if (poll_mode)
                netif_rx(skb);  /* Non-NAPI path for polling */
            else
                napi_gro_receive(&priv->napi, skb);

            priv->rx_packets++;
            priv->rx_bytes += len;
        } else {
            priv->rx_dropped++;
        }

        /* Reset descriptor for reuse */
        desc->control = cpu_to_le32(XDMA_DESC_MAGIC);
        desc->bytes = cpu_to_le32(RX_BUF_SIZE);
        wmb();

        priv->rx_tail = (tail + 1) % RX_RING_SIZE;
        work_done++;
        credits_to_post++;
    }

    /*
     * Re-post credits for consumed descriptors.
     * The XDMA engine decrements its credit counter as it fills descriptors.
     * We must replenish credits so it can continue receiving.
     */
    if (credits_to_post > 0) {
        c2h_write(priv, 0, XDMA_SGDMA_DESC_CREDITS, credits_to_post);
        priv->rx_credits_posted += credits_to_post;
        DBG(3, priv, "Re-posted %d C2H credits (total=%u)\n",
            credits_to_post, priv->rx_credits_posted);
    }

    return work_done;
}

/* NAPI poll handler (used in IRQ mode) */
static int axienet_rx_poll(struct napi_struct *napi, int budget)
{
    struct axienet_priv *priv = container_of(napi, struct axienet_priv, napi);
    int work_done;

    work_done = axienet_rx_process(priv, budget);

    if (work_done < budget) {
        napi_complete_done(napi, work_done);
        /* Re-enable RX interrupts */
        c2h_write(priv, 0, XDMA_CHAN_INT_EN_W1S, XDMA_CTRL_IE_DESC_CMPLT);
    }

    return work_done;
}

/* ==========================================================================
 * Polling Timer (replaces IRQ when poll_mode=1)
 * ========================================================================== */

static void axienet_poll_timer_fn(struct timer_list *t)
{
    struct axienet_priv *priv = from_timer(priv, t, poll_timer);
    int rx_done, tx_cleaned;

    if (!priv->poll_running)
        return;

    priv->poll_count++;

    /* Process TX completions */
    tx_cleaned = axienet_tx_complete(priv);
    if (tx_cleaned)
        priv->poll_tx_cleaned += tx_cleaned;

    /* Process RX packets */
    rx_done = axienet_rx_process(priv, RX_RING_SIZE);
    if (rx_done == 0)
        priv->poll_rx_empty++;

    /* Periodic state dump (every ~5 seconds at 1ms interval) */
    if (debug >= 2 && (priv->poll_count % 5000) == 0) {
        dev_info(priv->dev, "POLL stats: count=%u rx_pkts=%llu tx_pkts=%llu "
                 "rx_empty=%u tx_cleaned=%u\n",
                 priv->poll_count, priv->rx_packets, priv->tx_packets,
                 priv->poll_rx_empty, priv->poll_tx_cleaned);

        /* Check C2H channel health */
        axienet_dump_dma_state(priv, "periodic");

        /* Check MAC status */
        dev_info(priv->dev, "  MAC: IS=0x%08x PPST=0x%08x SPEED=0x%08x\n",
                 axienet_read(priv, XAE_IS),
                 axienet_read(priv, XAE_PPST),
                 axienet_read(priv, XAE_SPEED));
    }

    /* Re-arm timer */
    mod_timer(&priv->poll_timer,
              jiffies + msecs_to_jiffies(poll_interval_ms));
}

/* ==========================================================================
 * Interrupt Handler (used when poll_mode=0)
 * ========================================================================== */

static irqreturn_t axienet_irq(int irq, void *dev_id)
{
    struct axienet_priv *priv = dev_id;
    u32 eth_status, h2c_status, c2h_status;
    bool handled = false;

    priv->irq_count++;

    /* Check AXI Ethernet interrupts */
    eth_status = axienet_read(priv, XAE_IS);
    if (eth_status) {
        axienet_write(priv, XAE_IS, eth_status);
        DBG(2, priv, "IRQ: ETH IS=0x%08x\n", eth_status);
        handled = true;
    }

    /* Check XDMA H2C (TX) status */
    h2c_status = h2c_read(priv, 0, XDMA_CHAN_STATUS_RC);
    if (h2c_status & XDMA_STAT_DESC_CMPLT) {
        priv->tx_irq_count++;
        axienet_tx_complete(priv);
        handled = true;
    }

    /* Check XDMA C2H (RX) status */
    c2h_status = c2h_read(priv, 0, XDMA_CHAN_STATUS_RC);
    if (c2h_status & XDMA_STAT_DESC_CMPLT) {
        priv->rx_irq_count++;
        c2h_write(priv, 0, XDMA_CHAN_INT_EN_W1C, XDMA_CTRL_IE_DESC_CMPLT);
        napi_schedule(&priv->napi);
        handled = true;
    }

    if (h2c_status & ~XDMA_STAT_DESC_CMPLT)
        DBG(2, priv, "IRQ: H2C extra status=0x%08x\n", h2c_status);
    if (c2h_status & ~XDMA_STAT_DESC_CMPLT)
        DBG(2, priv, "IRQ: C2H extra status=0x%08x\n", c2h_status);

    return handled ? IRQ_HANDLED : IRQ_NONE;
}

/* ==========================================================================
 * Net Device Operations
 * ========================================================================== */

static int axienet_open(struct net_device *ndev)
{
    struct axienet_priv *priv = netdev_priv(ndev);
    int ret;

    dev_info(priv->dev, "Opening %s (mode=%s, poll_interval=%dms)\n",
             ndev->name, poll_mode ? "POLLING" : "IRQ", poll_interval_ms);

    /* Allocate rings */
    ret = axienet_tx_ring_init(priv);
    if (ret)
        return ret;

    ret = axienet_rx_ring_init(priv);
    if (ret) {
        axienet_tx_ring_free(priv);
        return ret;
    }

    /* Initialize MAC */
    axienet_mac_init(priv);

    /* Start DMA (includes C2H credit posting) */
    axienet_dma_start(priv);

    /* Start PHY */
    if (priv->phydev)
        phy_start(priv->phydev);
    else
        netif_carrier_on(ndev);

    if (poll_mode) {
        /* Start polling timer */
        priv->poll_running = true;
        timer_setup(&priv->poll_timer, axienet_poll_timer_fn, 0);
        mod_timer(&priv->poll_timer,
                  jiffies + msecs_to_jiffies(poll_interval_ms));
        dev_info(priv->dev, "Polling timer started (%d ms interval)\n",
                 poll_interval_ms);
    } else {
        napi_enable(&priv->napi);
    }

    netif_start_queue(ndev);

    netdev_info(ndev, "Interface opened\n");
    return 0;
}

static int axienet_close(struct net_device *ndev)
{
    struct axienet_priv *priv = netdev_priv(ndev);

    DBG(1, priv, "Closing %s\n", ndev->name);

    netif_stop_queue(ndev);

    if (poll_mode) {
        priv->poll_running = false;
        del_timer_sync(&priv->poll_timer);
        dev_info(priv->dev, "Polling timer stopped\n");
    } else {
        napi_disable(&priv->napi);
    }

    if (priv->phydev)
        phy_stop(priv->phydev);

    axienet_dma_stop(priv);

    netif_carrier_off(ndev);

    axienet_tx_ring_free(priv);
    axienet_rx_ring_free(priv);

    netdev_info(ndev, "Interface closed (stats: TX=%llu RX=%llu IRQ=%u poll=%u)\n",
                priv->tx_packets, priv->rx_packets,
                priv->irq_count, priv->poll_count);

    return 0;
}

static void axienet_get_stats64(struct net_device *ndev,
                                struct rtnl_link_stats64 *stats)
{
    struct axienet_priv *priv = netdev_priv(ndev);

    stats->tx_packets = priv->tx_packets;
    stats->tx_bytes = priv->tx_bytes;
    stats->tx_errors = priv->tx_errors;
    stats->rx_packets = priv->rx_packets;
    stats->rx_bytes = priv->rx_bytes;
    stats->rx_errors = priv->rx_errors;
    stats->rx_dropped = priv->rx_dropped;
}

static int axienet_set_mac_address_op(struct net_device *ndev, void *p)
{
    struct sockaddr *addr = p;

    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;

    eth_hw_addr_set(ndev, addr->sa_data);
    axienet_set_mac_address(ndev);

    return 0;
}

static const struct net_device_ops axienet_netdev_ops = {
    .ndo_open               = axienet_open,
    .ndo_stop               = axienet_close,
    .ndo_start_xmit         = axienet_start_xmit,
    .ndo_get_stats64        = axienet_get_stats64,
    .ndo_set_mac_address    = axienet_set_mac_address_op,
    .ndo_validate_addr      = eth_validate_addr,
};

/* ==========================================================================
 * Ethtool Operations
 * ========================================================================== */

static void axienet_get_drvinfo(struct net_device *ndev,
                                struct ethtool_drvinfo *info)
{
    strscpy(info->driver, DRV_NAME, sizeof(info->driver));
    strscpy(info->version, DRV_VERSION, sizeof(info->version));
}

static u32 axienet_get_link(struct net_device *ndev)
{
    return netif_carrier_ok(ndev);
}

static const struct ethtool_ops axienet_ethtool_ops = {
    .get_drvinfo    = axienet_get_drvinfo,
    .get_link       = axienet_get_link,
    .get_link_ksettings = phy_ethtool_get_link_ksettings,
    .set_link_ksettings = phy_ethtool_set_link_ksettings,
};

/* ==========================================================================
 * Sysfs: Manual DMA state dump
 * ========================================================================== */

static ssize_t dma_state_show(struct device *d, struct device_attribute *attr,
                              char *buf)
{
    struct net_device *ndev = to_net_dev(d);
    struct axienet_priv *priv = netdev_priv(ndev);

    axienet_dump_dma_state(priv, "sysfs");
    axienet_dump_rx_descs(priv, 8);

    return scnprintf(buf, PAGE_SIZE,
        "TX: head=%u tail=%u pkts=%llu\n"
        "RX: head=%u tail=%u pkts=%llu credits=%u\n"
        "Poll: count=%u rx_empty=%u tx_cleaned=%u\n"
        "IRQ: total=%u tx=%u rx=%u\n",
        priv->tx_head, priv->tx_tail, priv->tx_packets,
        priv->rx_head, priv->rx_tail, priv->rx_packets, priv->rx_credits_posted,
        priv->poll_count, priv->poll_rx_empty, priv->poll_tx_cleaned,
        priv->irq_count, priv->tx_irq_count, priv->rx_irq_count);
}
static DEVICE_ATTR_RO(dma_state);

/* ==========================================================================
 * Platform Driver
 * ========================================================================== */

static int axienet_probe(struct platform_device *pdev)
{
    struct mcu_pcie_pdata *pdata = dev_get_platdata(&pdev->dev);
    struct net_device *ndev;
    struct axienet_priv *priv;
    int ret;

    dev_info(&pdev->dev, "Probing AXI Ethernet v%s (debug=%d, poll_mode=%d)\n",
             DRV_VERSION, debug, poll_mode);

    if (!pdata || !pdata->bar0 || !pdata->bar1 || !pdata->pci_dev) {
        dev_err(&pdev->dev, "Invalid platform data\n");
        return -EINVAL;
    }

    ndev = alloc_etherdev(sizeof(*priv));
    if (!ndev)
        return -ENOMEM;

    SET_NETDEV_DEV(ndev, &pdev->dev);
    platform_set_drvdata(pdev, ndev);

    priv = netdev_priv(ndev);
    priv->ndev = ndev;
    priv->pdev = pdev;
    priv->dev = &pdev->dev;
    priv->pci_dev = pdata->pci_dev;
    spin_lock_init(&priv->tx_lock);

    /* Map registers */
    priv->regs = pdata->bar0 + MCU_AXIENET_OFFSET;
    priv->xdma = pdata->bar1;

    dev_info(&pdev->dev, "AXI Ethernet regs at %p (BAR0+0x%x)\n",
             priv->regs, MCU_AXIENET_OFFSET);
    dev_info(&pdev->dev, "XDMA regs at %p (BAR1)\n", priv->xdma);

    /* Get IRQ (used only in IRQ mode) */
    priv->irq = platform_get_irq(pdev, 0);
    if (priv->irq >= 0 && !poll_mode) {
        ret = devm_request_irq(&pdev->dev, priv->irq, axienet_irq,
                               IRQF_SHARED, ndev->name, priv);
        if (ret) {
            dev_warn(&pdev->dev, "Failed to request IRQ %d: %d (using poll mode)\n",
                     priv->irq, ret);
            poll_mode = 1;
        }
    } else if (poll_mode) {
        dev_info(&pdev->dev, "Poll mode enabled, IRQ %d not requested\n", priv->irq);
    }

    /* Read hardware ID */
    dev_info(&pdev->dev, "AXI Ethernet ID: 0x%08x\n", axienet_read(priv, XAE_ID));
    dev_info(&pdev->dev, "AXI Ethernet ABILITY: 0x%08x\n", axienet_read(priv, XAE_ABILITY));

    /* Setup MDIO */
    ret = axienet_mdio_setup(priv);
    if (ret)
        dev_warn(&pdev->dev, "MDIO setup failed: %d\n", ret);

    /* Connect PHY */
    ret = axienet_phy_connect(priv);
    if (ret)
        dev_info(&pdev->dev, "PHY connect failed: %d - using fixed link\n", ret);

    /* Configure netdev */
    ndev->netdev_ops = &axienet_netdev_ops;
    ndev->ethtool_ops = &axienet_ethtool_ops;
    ndev->min_mtu = MIN_MTU;
    ndev->max_mtu = MAX_MTU;

    /* Random MAC address */
    eth_hw_addr_random(ndev);

    /* Setup NAPI (used in IRQ mode) */
    netif_napi_add(ndev, &priv->napi, axienet_rx_poll);

    /* Register netdev */
    ret = register_netdev(ndev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register netdev: %d\n", ret);
        goto err_cleanup;
    }

    netif_carrier_off(ndev);

    /* Create sysfs for debug */
    device_create_file(&ndev->dev, &dev_attr_dma_state);

    dev_info(&pdev->dev, "Registered %s (MAC: %pM, IRQ: %d, mode: %s)\n",
             ndev->name, ndev->dev_addr, priv->irq,
             poll_mode ? "POLLING" : "IRQ");

    return 0;

err_cleanup:
    netif_napi_del(&priv->napi);
    axienet_phy_disconnect(priv);
    axienet_mdio_teardown(priv);
    free_netdev(ndev);
    return ret;
}

static int axienet_remove(struct platform_device *pdev)
{
    struct net_device *ndev = platform_get_drvdata(pdev);
    struct axienet_priv *priv = netdev_priv(ndev);

    device_remove_file(&ndev->dev, &dev_attr_dma_state);
    unregister_netdev(ndev);
    netif_napi_del(&priv->napi);
    axienet_phy_disconnect(priv);
    axienet_mdio_teardown(priv);
    free_netdev(ndev);

    dev_info(&pdev->dev, "Removed\n");
    return 0;
}

static struct platform_driver axienet_driver = {
    .probe  = axienet_probe,
    .remove = axienet_remove,
    .driver = {
        .name = DRV_NAME,
    },
};

module_platform_driver(axienet_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("MCU BSP Team");
MODULE_DESCRIPTION("AXI 1G/2.5G Ethernet Driver for MCU PCIe (v4 polling)");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:" DRV_NAME);
