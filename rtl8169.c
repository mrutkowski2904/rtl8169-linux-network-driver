#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

/* it will give one page per descriptor buffer */
#define RTL8169_RX_DESC_COUNT 256
#define RTL8169_TX_DESC_COUNT 256

#define RTL8169_RX_DESC_BUFF_SIZE (RTL8169_RX_DESC_COUNT * sizeof(struct rx_desc))
#define RTL8169_TX_DESC_BUFF_SIZE (RTL8169_TX_DESC_COUNT * sizeof(struct tx_desc))

#define RTL8169_RX_BUFF_SIZE (SZ_16K - 1)

#define RTL8169_MMIO_MAC0 0x00
#define RTL8169_MMIO_MAR0 0x08
#define RTL8169_MMIO_CMD 0x37
#define RTL8169_MMIO_IMASK 0x3c
#define RTL8169_MMIO_ISTAT 0x3e
#define RTL8169_MMIO_TXCFG 0x40
#define RTL8169_MMIO_RXCFG 0x44
#define RTL8169_MMIO_93C46CFG 0x50
#define RTL8169_MMIO_PHY 0x60
#define RTL8169_MMIO_CPCMD 0xe0

/* offsets in registers */
/* CMD */
#define RTL8169_MMIO_CMD_RST_SHIFT 4
#define RTL8169_MMIO_CMD_RE_SHIFT 3
#define RTL8169_MMIO_CMD_TE_SHIFT 2

/* IMASK/ISTAT */
#define RTL8169_INT_FLAG_RX_OK_SHIFT 0
#define RTL8169_INT_FLAG_RX_ERR_SHIFT 1
#define RTL8169_INT_FLAG_TX_OK_SHIFT 2
#define RTL8169_INT_FLAG_TX_ERR_SHIFT 3
#define RTL8169_INT_FLAG_RX_DESC_UNAVAL_SHIFT 4
#define RTL8169_INT_FLAG_LINK_CHANGE_SHIFT 5
#define RTL8169_INT_FLAG_RX_FIFO_OVF_SHIFT 6
#define RTL8169_INT_FLAG_TX_DESC_UNAVAL_SHIFT 7
#define RTL8169_INT_FLAG_SOFTWARE_SHIFT 8
#define RTL8169_INT_FLAG_TIME_OUT_SHIFT 14
#define RTL8169_INT_FLAG_SYS_ERR_SHIFT 15

/* RXCFG */
#define RTL8169_MMIO_RXCFG_DMA_SHIFT 8
#define RTL8169_MMIO_RXCFG_RXFIFO_TRH_SHIFT 13

/* PHY */
#define RTL8169_MMIO_PHY_DATA_SHIFT 0
#define RTL8169_MMIO_PHY_ADDR_SHIFT 16
#define RTL8169_MMIO_PHY_FLAG_SHIFT 31

/* CPCMD */
#define RTL8169_MMIO_CPCMD_RXCSUM_SHIFT 5

/* rx descriptor */
#define RTL8169_RX_DESC_OPTS1_OWN_SHIFT 31
#define RTL8169_RX_DESC_OPTS1_EOR_SHIFT 30
#define RTL8169_RX_DESC_OPTS1_BUFF_SIZE_SHIFT 0

/* references skb to be transmitted and its data size */
struct tx_skb
{
	struct sk_buff *tx_sk;
	u32 tx_sk_size;
};

struct rtl8169_context
{
	struct pci_dev *pci_dev;
	void __iomem *mmio_base;
	struct net_device *ndev;
	struct phy_device *phydev;
	struct clk *clk;
	u8 mac[ETH_ALEN];
	u16 interrupt_mask;
	int irq;

	u32 tx_descriptors_index;
	struct tx_desc *tx_descriptors;
	dma_addr_t tx_descriptors_dma_addr;
	struct tx_skb skbs_to_transmit[RTL8169_TX_DESC_COUNT];

	u32 rx_descriptors_index;
	struct rx_desc *rx_descriptors;
	dma_addr_t rx_descriptors_dma_addr;
	/* pages for storing incoming data */
	struct page *rx_data[RTL8169_RX_DESC_COUNT];
};

struct tx_desc
{
	u32 opts1;
	u32 opts2;
	u64 addr;
} __attribute__((packed));

struct rx_desc
{
	u32 opts1;
	u32 opts2;
	u64 addr;
} __attribute__((packed));

struct rtl8169_net_context
{
	struct rtl8169_context *ctx;
};

/* PHY access callbacks */
static int rtl8169_mdio_read(struct mii_bus *mii, int addr, int reg);
static int rtl8169_mdio_write(struct mii_bus *mii, int addr, int reg, u16 val);
static void rtl8169_phylink_handler(struct net_device *dev);

/* device operations */
static void rtl8169_commit_pci_write(struct rtl8169_context *ctx);
static void rtl8169_enable_config_write(struct rtl8169_context *ctx);
static void rtl8169_disable_config_write(struct rtl8169_context *ctx);
static void rtl8169_irq_enable(struct rtl8169_context *ctx);
static void rtl8169_irq_disable(struct rtl8169_context *ctx);
static void rtl8169_reset_chip(struct rtl8169_context *ctx);

/* interrupt handler */
static irqreturn_t rtl8169_interrupt(int irq, void *ctx);

static int rtl8169_ndo_open(struct net_device *dev);
static int rtl8169_ndo_close(struct net_device *dev);
static netdev_tx_t rtl8169_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev);

static int rtl8169_rx_ring_init(struct rtl8169_context *ctx);
static void rtl8169_rx_ring_deinit(struct rtl8169_context *ctx);

static int rtl8169_netdev_init(struct rtl8169_context *ctx);
static int rtl8169_chip_init(struct rtl8169_context *ctx);

static int rtl8169_probe(struct pci_dev *pci_dev, const struct pci_device_id *id);
static void rtl8169_remove(struct pci_dev *pci_dev);

static const struct pci_device_id rtl8169_ids[] = {
	{PCI_VDEVICE(REALTEK, 0x8169)},
	{},
};

MODULE_DEVICE_TABLE(pci, rtl8169_ids);

static struct pci_driver rtl8169_driver = {
	.name = "rtl8169",
	.id_table = rtl8169_ids,
	.probe = rtl8169_probe,
	.remove = rtl8169_remove,
};

static struct net_device_ops rtl8169_ndev_ops = {
	.ndo_open = rtl8169_ndo_open,
	.ndo_stop = rtl8169_ndo_close,
	.ndo_start_xmit = rtl8169_ndo_start_xmit,
};

static int rtl8169_mdio_read(struct mii_bus *mii, int addr, int reg)
{
	u32 read_reg;
	u32 read_cmd = 0;
	int retries = 200;
	struct rtl8169_context *ctx = mii->priv;

	if (addr > 0)
		return -ENODEV;

	read_cmd |= (reg & 0x1f) << RTL8169_MMIO_PHY_ADDR_SHIFT;
	iowrite32(read_cmd, ctx->mmio_base + RTL8169_MMIO_PHY);

	/* wait until data is ready to be read */
	while (retries)
	{
		read_reg = ioread32(ctx->mmio_base + RTL8169_MMIO_PHY);
		/* flag high - data from MII has been read */
		if (read_reg & (1 << RTL8169_MMIO_PHY_FLAG_SHIFT))
			break;
		fsleep(25);
		retries--;
	}

	if (!retries)
		return -ETIMEDOUT;

	read_reg = ioread32(ctx->mmio_base + RTL8169_MMIO_PHY) & 0xffff;
	udelay(20);
	return read_reg;
}

static int rtl8169_mdio_write(struct mii_bus *mii, int addr, int reg, u16 val)
{
	u32 read_reg;
	u32 write_cmd = 0;
	int retries = 20;
	struct rtl8169_context *ctx = mii->priv;

	if (addr > 0)
		return -ENODEV;

	write_cmd |= ((1 << RTL8169_MMIO_PHY_FLAG_SHIFT) |
				  ((reg & 0x1f) << RTL8169_MMIO_PHY_ADDR_SHIFT) |
				  (val << RTL8169_MMIO_PHY_DATA_SHIFT));

	iowrite32(write_cmd, ctx->mmio_base + RTL8169_MMIO_PHY);

	/* wait until data has been written */
	while (retries)
	{
		read_reg = ioread32(ctx->mmio_base + RTL8169_MMIO_PHY);
		/* flag low - data has been written to MII */
		if (!(read_reg & (1 << RTL8169_MMIO_PHY_FLAG_SHIFT)))
			break;
		fsleep(25);
		retries--;
	}

	if (!retries)
		return -ETIMEDOUT;

	udelay(20);
	return 0;
}

static void rtl8169_phylink_handler(struct net_device *dev)
{
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	/* TODO: remove */
	/*
	if(netif_carrier_ok(dev))
		pm_request_resume(&ctx->pci_dev->dev);
	else
		pm_runtime_idle(&ctx->pci_dev->dev);
	*/

	phy_print_status(ctx->phydev);
}

/* TODO: remove function if not needed */
static void rtl8169_commit_pci_write(struct rtl8169_context *ctx)
{
	/* TODO: check - may not be needed */
	ioread8(ctx->mmio_base + RTL8169_MMIO_CMD);
}

static void rtl8169_enable_config_write(struct rtl8169_context *ctx)
{
	iowrite8(0xc0, ctx->mmio_base + RTL8169_MMIO_93C46CFG);
}

static void rtl8169_disable_config_write(struct rtl8169_context *ctx)
{
	iowrite8(0x00, ctx->mmio_base + RTL8169_MMIO_93C46CFG);
}

static void rtl8169_irq_enable(struct rtl8169_context *ctx)
{
	iowrite16(ctx->interrupt_mask, ctx->mmio_base + RTL8169_MMIO_IMASK);
}

static void rtl8169_irq_disable(struct rtl8169_context *ctx)
{
	iowrite16(0, ctx->mmio_base + RTL8169_MMIO_IMASK);
}

static void rtl8169_reset_chip(struct rtl8169_context *ctx)
{
	u8 reg;
	reg = ioread8(ctx->mmio_base + RTL8169_MMIO_CMD);
	reg |= (1 << RTL8169_MMIO_CMD_RST_SHIFT);
	iowrite8(reg, ctx->mmio_base + RTL8169_MMIO_CMD);
	mdelay(10);
}

static irqreturn_t rtl8169_interrupt(int irq, void *ctx)
{
	return IRQ_HANDLED;
}

static int rtl8169_ndo_open(struct net_device *dev)
{
	int status = 0;
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	ctx->rx_descriptors = dma_alloc_coherent(&ctx->pci_dev->dev,
											 RTL8169_RX_DESC_BUFF_SIZE,
											 &ctx->rx_descriptors_dma_addr,
											 GFP_KERNEL);
	if (ctx->rx_descriptors == NULL)
	{
		status = -ENOMEM;
		goto dma_rx_alloc_fail;
	}

	ctx->tx_descriptors = dma_alloc_coherent(&ctx->pci_dev->dev,
											 RTL8169_TX_DESC_BUFF_SIZE,
											 &ctx->tx_descriptors_dma_addr,
											 GFP_KERNEL);
	if (ctx->tx_descriptors == NULL)
	{
		status = -ENOMEM;
		goto dma_tx_alloc_fail;
	}

	ctx->tx_descriptors_index = 0;
	ctx->rx_descriptors_index = 0;

	/* prepare tx and rx buffers */
	/* array of skbs to be transmitted */
	memset(ctx->skbs_to_transmit, 0, sizeof(ctx->skbs_to_transmit));

	status = rtl8169_rx_ring_init(ctx);
	if (status)
		goto rx_ring_init_fail;

	/* add IRQ handler */
	status = request_irq(ctx->irq, rtl8169_interrupt, IRQF_SHARED, "rtl8169", ctx);
	if (status < 0)
		goto request_irq_fail;

	/* prepare PHY */
	status = phy_connect_direct(ctx->ndev, ctx->phydev, rtl8169_phylink_handler, PHY_INTERFACE_MODE_MII);
	if (status)
		goto phy_connect_fail;

	phy_set_max_speed(ctx->phydev, SPEED_100);
	phy_attached_info(ctx->phydev);
	phy_init_hw(ctx->phydev);
	phy_resume(ctx->phydev);

	netif_start_queue(ctx->ndev);

	return status;

phy_connect_fail:
	free_irq(ctx->irq, ctx);

request_irq_fail:
rx_ring_init_fail:
	rtl8169_rx_ring_deinit(ctx);

	dma_free_coherent(&ctx->pci_dev->dev,
					  RTL8169_TX_DESC_BUFF_SIZE,
					  ctx->tx_descriptors,
					  ctx->tx_descriptors_dma_addr);
	ctx->tx_descriptors = NULL;

dma_tx_alloc_fail:
	dma_free_coherent(&ctx->pci_dev->dev,
					  RTL8169_RX_DESC_BUFF_SIZE,
					  ctx->rx_descriptors,
					  ctx->rx_descriptors_dma_addr);
dma_rx_alloc_fail:
	ctx->rx_descriptors = NULL;
	return status;
}

static int rtl8169_ndo_close(struct net_device *dev)
{
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	netif_stop_queue(ctx->ndev);
	phy_stop(ctx->phydev);
	synchronize_net();

	netdev_reset_queue(ctx->ndev);
	free_irq(ctx->irq, ctx);
	phy_disconnect(ctx->phydev);

	rtl8169_rx_ring_deinit(ctx);
	dma_free_coherent(&ctx->pci_dev->dev,
					  RTL8169_TX_DESC_BUFF_SIZE,
					  ctx->tx_descriptors,
					  ctx->tx_descriptors_dma_addr);
	ctx->tx_descriptors = NULL;

	dma_free_coherent(&ctx->pci_dev->dev,
					  RTL8169_RX_DESC_BUFF_SIZE,
					  ctx->rx_descriptors,
					  ctx->rx_descriptors_dma_addr);
	ctx->rx_descriptors = NULL;

	/* TODO: unamp skbs_to_transmit */

	memset(ctx->skbs_to_transmit, 0, sizeof(ctx->skbs_to_transmit));
	ctx->tx_descriptors_index = 0;
	ctx->rx_descriptors_index = 0;

	return 0;
}

static netdev_tx_t rtl8169_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	dev_info_ratelimited(&ctx->pci_dev->dev, "in start xmit callback\n");

	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int rtl8169_rx_ring_init(struct rtl8169_context *ctx)
{
	struct rx_desc *desc;
	dma_addr_t desc_buff_mapping;

	memset(ctx->rx_data, 0, sizeof(ctx->rx_data));
	for (int i = 0; i < RTL8169_RX_DESC_COUNT; i++)
	{
		desc = ctx->rx_descriptors + i;

		/* allocate pages */
		ctx->rx_data[i] = alloc_pages_node(NUMA_NO_NODE, GFP_KERNEL, get_order(RTL8169_RX_BUFF_SIZE));
		if (ctx->rx_data[i] == NULL)
			return -ENOMEM;

		/* create DMA mapping for pages */
		desc_buff_mapping = dma_map_page(&ctx->pci_dev->dev, ctx->rx_data[i], 0, RTL8169_RX_BUFF_SIZE, DMA_FROM_DEVICE);
		if (dma_mapping_error(&ctx->pci_dev->dev, desc_buff_mapping))
		{
			__free_pages(ctx->rx_data[i], get_order(RTL8169_RX_BUFF_SIZE));
			return -ENOMEM;
		}

		/* mark as owned by the NIC, set buffer size */
		desc->opts1 = ((1 << RTL8169_RX_DESC_OPTS1_OWN_SHIFT) |
					   (RTL8169_RX_BUFF_SIZE << RTL8169_RX_DESC_OPTS1_BUFF_SIZE_SHIFT));
		desc->opts2 = 0;
		desc->addr = desc_buff_mapping;
	}

	/* mark last as end of buffer */
	ctx->rx_descriptors[RTL8169_RX_DESC_COUNT - 1].opts1 |= (1 << RTL8169_RX_DESC_OPTS1_EOR_SHIFT);

	return 0;
}

static void rtl8169_rx_ring_deinit(struct rtl8169_context *ctx)
{
	for (int i = 0; i < RTL8169_RX_DESC_COUNT; i++)
	{
		if (ctx->rx_data[i] == NULL)
			break;

		/* undo dma mapping */
		dma_unmap_page(&ctx->pci_dev->dev,
					   ctx->rx_descriptors[i].addr,
					   RTL8169_RX_BUFF_SIZE,
					   DMA_FROM_DEVICE);

		/* free pages */
		__free_pages(ctx->rx_data[i], get_order(RTL8169_RX_BUFF_SIZE));

		ctx->rx_data[i] = NULL;
	}
	memset(ctx->rx_data, 0, sizeof(ctx->rx_data));
}

static int rtl8169_netdev_init(struct rtl8169_context *ctx)
{
	int status;
	struct rtl8169_net_context *net_ctx;

	ctx->ndev = devm_alloc_etherdev(&ctx->pci_dev->dev, sizeof(struct rtl8169_net_context));
	if (ctx->ndev == NULL)
		return -ENOMEM;

	ctx->ndev->dev.parent = &ctx->pci_dev->dev;
	ctx->ndev->netdev_ops = &rtl8169_ndev_ops;
	ctx->ndev->features = NETIF_F_RXCSUM;
	eth_hw_addr_set(ctx->ndev, ctx->mac);

	net_ctx = netdev_priv(ctx->ndev);
	net_ctx->ctx = ctx;

	status = devm_register_netdev(&ctx->pci_dev->dev, ctx->ndev);
	return status;
}

static int rtl8169_chip_init(struct rtl8169_context *ctx)
{
	u16 reg16;
	u32 reg32;
	struct mii_bus *mii;
	int status;

	/* get MAC address */
	for (int i = 0; i < ETH_ALEN; i++)
		ctx->mac[i] = ioread8(ctx->mmio_base + RTL8169_MMIO_MAC0 + i);

	/* rx config */
	reg32 = ioread32(ctx->mmio_base + RTL8169_MMIO_RXCFG);
	/* no rx fifo threshold, ulimited DMA burst size  */
	reg32 |= (0b111 << RTL8169_MMIO_RXCFG_RXFIFO_TRH_SHIFT);
	reg32 |= (0b111 << RTL8169_MMIO_RXCFG_DMA_SHIFT);
	iowrite32(reg32, ctx->mmio_base + RTL8169_MMIO_RXCFG);

	rtl8169_irq_disable(ctx);
	rtl8169_reset_chip(ctx);

	/* enable checksum offloading */
	reg16 = ioread16(ctx->mmio_base + RTL8169_MMIO_CPCMD);
	reg16 |= (1 << RTL8169_MMIO_CPCMD_RXCSUM_SHIFT);
	iowrite16(reg16, ctx->mmio_base + RTL8169_MMIO_CPCMD);
	rtl8169_commit_pci_write(ctx);

	/* set default interrupt mask */
	ctx->interrupt_mask |= ((1 << RTL8169_INT_FLAG_RX_OK_SHIFT) |
							(1 << RTL8169_INT_FLAG_RX_ERR_SHIFT) |
							(1 << RTL8169_INT_FLAG_TX_OK_SHIFT) |
							(1 << RTL8169_INT_FLAG_TX_ERR_SHIFT) |
							(1 << RTL8169_INT_FLAG_LINK_CHANGE_SHIFT));

	/* setup PHY */
	mii = devm_mdiobus_alloc(&ctx->pci_dev->dev);
	if (mii == NULL)
		return -ENOMEM;

	mii->name = "r8169";
	mii->priv = ctx;
	mii->parent = &ctx->pci_dev->dev;
	mii->irq[0] = PHY_MAC_INTERRUPT;
	snprintf(mii->id, MII_BUS_ID_SIZE, "r8169-%x-%x",
			 pci_domain_nr(ctx->pci_dev->bus), pci_dev_id(ctx->pci_dev));

	mii->read = rtl8169_mdio_read;
	mii->write = rtl8169_mdio_write;

	status = devm_mdiobus_register(&ctx->pci_dev->dev, mii);
	if (status)
		return status;

	ctx->phydev = mdiobus_get_phy(mii, 0);
	if (!ctx->phydev)
	{
		return -ENODEV;
	}
	else if (!ctx->phydev->drv)
	{
		dev_err(&ctx->pci_dev->dev, "no dedicated PHY driver found\n");
		return -EUNATCH;
	}

	phy_support_asym_pause(ctx->phydev);
	phy_suspend(ctx->phydev);

	return 0;
}

static int rtl8169_probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
	int status, mmio_region;
	struct rtl8169_context *ctx;

	ctx = devm_kzalloc(&pci_dev->dev, sizeof(struct rtl8169_context), GFP_KERNEL);
	if (ctx == NULL)
		return -ENOMEM;

	pci_set_drvdata(pci_dev, ctx);
	ctx->pci_dev = pci_dev;

	ctx->clk = devm_clk_get_optional_enabled(&pci_dev->dev, "ether_clk");
	if (IS_ERR(ctx->clk))
		return dev_err_probe(&pci_dev->dev, PTR_ERR(ctx->clk), "could not get access to required clock\n");

	status = pcim_enable_device(pci_dev);
	if (status)
		return status;

	pci_set_master(pci_dev);

	/* TODO: check - may not be needed */
	if (pcim_set_mwi(pci_dev) < 0)
		dev_info(&pci_dev->dev, "mem wr inval unavailable\n");

	mmio_region = ffs(pci_select_bars(pci_dev, IORESOURCE_MEM)) - 1;
	if (mmio_region < 0)
	{
		dev_err(&pci_dev->dev, "could not find MMIO region\n");
		return -ENODEV;
	}

	status = pcim_iomap_regions(pci_dev, BIT(mmio_region), "rtl8169");
	if (status < 0)
	{
		dev_err(&pci_dev->dev, "could not remap MMIO region\n");
		return status;
	}

	ctx->mmio_base = pcim_iomap_table(pci_dev)[mmio_region];
	if (ctx->mmio_base == NULL)
		return -ENOMEM;

	status = pci_alloc_irq_vectors(pci_dev, 1, 1, PCI_IRQ_LEGACY);
	if (status < 0)
		return status;

	ctx->irq = pci_irq_vector(pci_dev, 0);

	status = rtl8169_chip_init(ctx);
	if (status < 0)
	{
		dev_err(&pci_dev->dev, "error while configuring network device\n");
		return status;
	}

	status = rtl8169_netdev_init(ctx);
	if (status < 0)
		return status;

	dev_info(&pci_dev->dev, "rtl8169 probe successful\n");
	return 0;
}

static void rtl8169_remove(struct pci_dev *pci_dev)
{
	struct rtl8169_context *ctx;

	ctx = pci_get_drvdata(pci_dev);
	pci_free_irq_vectors(pci_dev);
	dev_info(&pci_dev->dev, "rtl8169 removed\n");
}

module_pci_driver(rtl8169_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maciej Rutkowski");
MODULE_DESCRIPTION("Driver for RTL8169 PCI network card");