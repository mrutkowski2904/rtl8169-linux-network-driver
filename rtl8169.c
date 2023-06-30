#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>

#define RTL8169_DATA_BUFF_SIZE 2048

#define RTL8169_MMIO_MAC0 0x00
#define RTL8169_MMIO_MAR0 0x08
#define RTL8169_MMIO_TNPDS 0x20
#define RTL8169_MMIO_CMD 0x37
#define RTL8169_MMIO_TPPOLL 0x38
#define RTL8169_MMIO_IMASK 0x3c
#define RTL8169_MMIO_ISTAT 0x3e
#define RTL8169_MMIO_TXCFG 0x40
#define RTL8169_MMIO_RXCFG 0x44
#define RTL8169_MMIO_93C46CFG 0x50
#define RTL8169_MMIO_MIS 0x5c
#define RTL8169_MMIO_PHY 0x60
#define RTL8169_MMIO_TBICSR 0x64
#define RTL8169_MMIO_RMS 0xda
#define RTL8169_MMIO_ETTHR 0xec
#define RTL8169_MMIO_CPCMD 0xe0
#define RTL8169_MMIO_RDSAR 0xe4

/* offsets and masks for registers */
/* CMD */
#define RTL8169_MMIO_CMD_RST_SHIFT 4
#define RTL8169_MMIO_CMD_RE_SHIFT 3
#define RTL8169_MMIO_CMD_TE_SHIFT 2
#define RTL8169_MMIO_CMD_INIT_MASK ((1 << RTL8169_MMIO_CMD_TE_SHIFT) | (1 << RTL8169_MMIO_CMD_RE_SHIFT))

/* TPPOLL */
#define RTL8169_MMIO_TPPOLL_TX_PENDING_MASK (1 << 6)

/* IMASK/ISTAT */
#define RTL8169_MMIO_INT_FLAG_RX_OK_SHIFT 0
#define RTL8169_MMIO_INT_FLAG_RX_ERR_SHIFT 1
#define RTL8169_MMIO_INT_FLAG_TX_OK_SHIFT 2
#define RTL8169_MMIO_INT_FLAG_TX_ERR_SHIFT 3
#define RTL8169_MMIO_INT_FLAG_RX_DESC_UNAVAL_SHIFT 4
#define RTL8169_MMIO_INT_FLAG_LINK_CHANGE_SHIFT 5
#define RTL8169_MMIO_INT_FLAG_RX_FIFO_OVF_SHIFT 6
#define RTL8169_MMIO_INT_FLAG_TX_DESC_UNAVAL_SHIFT 7
#define RTL8169_MMIO_INT_FLAG_SOFTWARE_SHIFT 8
#define RTL8169_MMIO_INT_FLAG_TIME_OUT_SHIFT 14
#define RTL8169_MMIO_INT_FLAG_SYS_ERR_SHIFT 15
#define RTL8169_MMIO_INT_INIT_MASK ((1 << RTL8169_MMIO_INT_FLAG_RX_OK_SHIFT) | (1 << RTL8169_MMIO_INT_FLAG_RX_ERR_SHIFT) | \
									(1 << RTL8169_MMIO_INT_FLAG_TX_OK_SHIFT) | (1 << RTL8169_MMIO_INT_FLAG_TX_ERR_SHIFT) | \
									(1 << RTL8169_MMIO_INT_FLAG_LINK_CHANGE_SHIFT))

/* TXCFG */
#define RTL8169_MMIO_TXCFG_DMA_BRST_UNLIMITED_MASK (0b111 << 8)
#define RTL8169_MMIO_TXCFG_IFG_NORMAL_MASK (0b011 << 24)
#define RTL8169_MMIO_TXCFG_LOOPBACK_MASK (0b01 << 17)
#define RTL8169_MMIO_TXCFG_INIT_MASK (RTL8169_MMIO_TXCFG_DMA_BRST_UNLIMITED_MASK | RTL8169_MMIO_TXCFG_IFG_NORMAL_MASK)

/* RXCFG */
#define RTL8169_MMIO_RXCFG_ACCEPT_ALL_MASK (0b111111 << 0)
#define RTL8169_MMIO_RXCFG_DMA_BRST_UNLIMITED_MASK (0b111 << 8)
#define RTL8169_MMIO_RXCFG_RXFIFO_NO_THS_MASK (0b111 << 13)
#define RTL8169_MMIO_RXCFG_RXFIFO_64_THS_MASK (0b010 << 13)
#define RTL8169_MMIO_RXCFG_MERINT_MASK (1 << 24)
#define RTL8169_MMIO_RXCFG_INIT_MASK (RTL8169_MMIO_RXCFG_ACCEPT_ALL_MASK | RTL8169_MMIO_RXCFG_DMA_BRST_UNLIMITED_MASK | \
									  RTL8169_MMIO_RXCFG_RXFIFO_64_THS_MASK)

/* PHY */
#define RTL8169_MMIO_PHY_DATA_SHIFT 0
#define RTL8169_MMIO_PHY_ADDR_SHIFT 16
#define RTL8169_MMIO_PHY_FLAG_SHIFT 31

/* CPCMD */
#define RTL8169_MMIO_CPCMD_RXCSUM_SHIFT 5

/* rx descriptor */
#define RTL8169_RX_DESC_OPTS1_OWN_SHIFT 31
#define RTL8169_RX_DESC_OPTS1_EOR_SHIFT 30

/* tx descriptor */
#define RTL8169_TX_DESC_OPTS1_OWN_SHIFT 31
#define RTL8169_TX_DESC_OPTS1_EOR_SHIFT 30
#define RTL8169_TX_DESC_OPTS1_FS_SHIFT 29
#define RTL8169_TX_DESC_OPTS1_LS_SHIFT 28
#define RTL8169_TX_DESC_OPTS1_IPCS_SHIFT 18
#define RTL8169_TX_DESC_OPTS1_UDPCS_SHIFT 17
#define RTL8169_TX_DESC_OPTS1_TCPCS_SHIFT 16

struct rtl8169_desc
{
	u32 opts1;
	u32 opts2;
	u32 bus_addr_low;
	u32 bus_addr_high;
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

	/* 256 byte aligned desciptors */
	struct rtl8169_desc *rx_descriptor;
	dma_addr_t rx_descriptor_bus_addr;

	struct rtl8169_desc *tx_descriptor;
	dma_addr_t tx_descriptor_bus_addr;

	u8 *rx_data_buffer;
	dma_addr_t rx_data_bus_addr;

	u8 *tx_data_buffer;
	dma_addr_t tx_data_bus_addr;
};

struct rtl8169_net_context
{
	struct rtl8169_context *ctx;
};

/* PHY access callbacks */
static int rtl8169_mdio_read(struct mii_bus *mii, int addr, int reg);
static int rtl8169_mdio_write(struct mii_bus *mii, int addr, int reg, u16 val);
static void rtl8169_phylink_handler(struct net_device *dev);

/* device operations */
static void rtl8169_enable_config_write(struct rtl8169_context *ctx);
static void rtl8169_disable_config_write(struct rtl8169_context *ctx);
static void rtl8169_reset_chip(struct rtl8169_context *ctx);
static void rtl8169_handle_rx(struct rtl8169_context *ctx);

/* interrupt handler */
static irqreturn_t rtl8169_interrupt(int irq, void *data);

static int rtl8169_ndo_open(struct net_device *dev);
static int rtl8169_ndo_close(struct net_device *dev);
static netdev_tx_t rtl8169_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev);

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

	phy_print_status(ctx->phydev);
}

static void rtl8169_enable_config_write(struct rtl8169_context *ctx)
{
	iowrite8(0xc0, ctx->mmio_base + RTL8169_MMIO_93C46CFG);
}

static void rtl8169_disable_config_write(struct rtl8169_context *ctx)
{
	iowrite8(0x00, ctx->mmio_base + RTL8169_MMIO_93C46CFG);
}

static void rtl8169_reset_chip(struct rtl8169_context *ctx)
{
	u8 reg;
	reg = ioread8(ctx->mmio_base + RTL8169_MMIO_CMD);
	reg |= (1 << RTL8169_MMIO_CMD_RST_SHIFT);
	iowrite8(reg, ctx->mmio_base + RTL8169_MMIO_CMD);
	mdelay(10);
}

static void rtl8169_handle_rx(struct rtl8169_context *ctx)
{
	u16 rx_size;
	struct sk_buff *skb;

	/* get len from descriptor */
	rx_size = ctx->rx_descriptor->opts1 & 0x3fff;

	skb = netdev_alloc_skb(ctx->ndev, rx_size + 2);
	if (!skb)
	{
		dev_err_ratelimited(&ctx->pci_dev->dev, "could not allocate skb!\n");
		return;
	}

	skb_copy_to_linear_data(skb, ctx->rx_data_buffer, rx_size);
	skb_put(skb, rx_size);
	skb->protocol = eth_type_trans(skb, ctx->ndev);
	if (netif_rx(skb) == NET_RX_DROP)
		dev_err_ratelimited(&ctx->pci_dev->dev, "rx skb dropped!\n");

	/* prepare for rx */
	ctx->rx_descriptor->opts1 = ((1 << RTL8169_RX_DESC_OPTS1_OWN_SHIFT) |
								 (1 << RTL8169_RX_DESC_OPTS1_EOR_SHIFT) |
								 (RTL8169_DATA_BUFF_SIZE & 0x3fff));
}

static irqreturn_t rtl8169_interrupt(int irq, void *data)
{
	u16 interrupt_status;
	struct rtl8169_context *ctx;

	ctx = data;
	interrupt_status = ioread16(ctx->mmio_base + RTL8169_MMIO_ISTAT);
	if (interrupt_status == 0)
		return IRQ_NONE;

	if (interrupt_status & (1 << RTL8169_MMIO_INT_FLAG_LINK_CHANGE_SHIFT))
		phy_mac_interrupt(ctx->phydev);

	if (interrupt_status & (1 << RTL8169_MMIO_INT_FLAG_RX_OK_SHIFT))
		rtl8169_handle_rx(ctx);

	/* ack interrupts */
	iowrite16(interrupt_status, ctx->mmio_base + RTL8169_MMIO_ISTAT);

	return IRQ_HANDLED;
}

static int rtl8169_ndo_open(struct net_device *dev)
{
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	/* start tx queue */
	netif_start_queue(ctx->ndev);

	return 0;
}

static int rtl8169_ndo_close(struct net_device *dev)
{
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	netif_stop_queue(ctx->ndev);
	synchronize_net();
	netdev_reset_queue(ctx->ndev);

	return 0;
}

static netdev_tx_t rtl8169_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rtl8169_context *ctx;
	struct rtl8169_net_context *net_ctx;
	int data_size;

	net_ctx = netdev_priv(dev);
	ctx = net_ctx->ctx;

	if (skb_shinfo(skb)->nr_frags)
	{
		dev_err_ratelimited(&ctx->pci_dev->dev, "fragmented tx not supported\n");
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	data_size = skb_headlen(skb);
	if (data_size >= RTL8169_DATA_BUFF_SIZE)
	{
		dev_err_ratelimited(&ctx->pci_dev->dev, "data is too big to transmit\n");
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (ctx->tx_descriptor->opts1 & (1 << RTL8169_TX_DESC_OPTS1_OWN_SHIFT))
		return NETDEV_TX_BUSY;

	memcpy(ctx->tx_data_buffer, skb->data, data_size);

	ctx->tx_descriptor->opts1 = (data_size & 0x3fff);
	ctx->tx_descriptor->opts1 |= ((1 << RTL8169_TX_DESC_OPTS1_OWN_SHIFT) |
								  (1 << RTL8169_TX_DESC_OPTS1_EOR_SHIFT) |
								  (1 << RTL8169_TX_DESC_OPTS1_FS_SHIFT) |
								  (1 << RTL8169_TX_DESC_OPTS1_LS_SHIFT));

	ctx->tx_descriptor->opts2 = 0;

	/* notify the card that tx is pending */
	iowrite8(RTL8169_MMIO_TPPOLL_TX_PENDING_MASK, ctx->mmio_base + RTL8169_MMIO_TPPOLL);

	kfree_skb(skb);
	return NETDEV_TX_OK;
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
	ctx->ndev->features = 0;
	eth_hw_addr_set(ctx->ndev, ctx->mac);

	net_ctx = netdev_priv(ctx->ndev);
	net_ctx->ctx = ctx;

	status = devm_register_netdev(&ctx->pci_dev->dev, ctx->ndev);
	return status;
}

static int rtl8169_chip_init(struct rtl8169_context *ctx)
{
	struct mii_bus *mii;
	int status;

	/* reset the chip */
	rtl8169_reset_chip(ctx);

	/* get MAC address */
	for (int i = 0; i < ETH_ALEN; i++)
		ctx->mac[i] = ioread8(ctx->mmio_base + RTL8169_MMIO_MAC0 + i);

	ctx->rx_descriptor = dmam_alloc_coherent(&ctx->pci_dev->dev, sizeof(struct rtl8169_desc), &ctx->rx_descriptor_bus_addr, GFP_KERNEL);
	if (ctx->rx_descriptor == NULL)
		return -ENOMEM;

	ctx->rx_data_buffer = dmam_alloc_coherent(&ctx->pci_dev->dev, RTL8169_DATA_BUFF_SIZE, &ctx->rx_data_bus_addr, GFP_KERNEL);
	if (ctx->rx_data_buffer == NULL)
		return -ENOMEM;

	/* write bus address of the buffer to the rx descriptor */
	ctx->rx_descriptor->bus_addr_low = ctx->rx_data_bus_addr & DMA_BIT_MASK(32);
	ctx->rx_descriptor->bus_addr_high = (ctx->rx_data_bus_addr >> 32);

	ctx->rx_descriptor->opts1 = ((1 << RTL8169_RX_DESC_OPTS1_OWN_SHIFT) |
								 (1 << RTL8169_RX_DESC_OPTS1_EOR_SHIFT) |
								 (RTL8169_DATA_BUFF_SIZE & 0x3fff));

	/* vlan settings - unused */
	ctx->rx_descriptor->opts2 = 0;

	/* prepare tx descriptor and its buffer */
	ctx->tx_descriptor = dmam_alloc_coherent(&ctx->pci_dev->dev, sizeof(struct rtl8169_desc), &ctx->tx_descriptor_bus_addr, GFP_KERNEL);
	if (ctx->tx_descriptor == NULL)
		return -ENOMEM;

	ctx->tx_data_buffer = dmam_alloc_coherent(&ctx->pci_dev->dev, RTL8169_DATA_BUFF_SIZE, &ctx->tx_data_bus_addr, GFP_KERNEL);
	if (ctx->tx_data_buffer == NULL)
		return -ENOMEM;

	/* write bus address of the buffer to the rx descriptor */
	ctx->tx_descriptor->bus_addr_low = ctx->tx_data_bus_addr & DMA_BIT_MASK(32);
	ctx->tx_descriptor->bus_addr_high = (ctx->tx_data_bus_addr >> 32);
	ctx->tx_descriptor->opts1 |= (1 << RTL8169_TX_DESC_OPTS1_EOR_SHIFT);
	ctx->tx_descriptor->opts2 = 0;

	/* setup PHY */
	mii = devm_mdiobus_alloc(&ctx->pci_dev->dev);
	if (mii == NULL)
		return -ENOMEM;

	mii->name = "rtl8169";
	mii->priv = ctx;
	mii->parent = &ctx->pci_dev->dev;
	mii->irq[0] = PHY_MAC_INTERRUPT;
	snprintf(mii->id, MII_BUS_ID_SIZE, "rtl8169-%x-%x",
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

	/* apply configuration */
	/* unlock configuration registers */
	rtl8169_enable_config_write(ctx);

	/* rx config */
	iowrite32(RTL8169_MMIO_RXCFG_INIT_MASK, ctx->mmio_base + RTL8169_MMIO_RXCFG);

	/* enable tx */
	iowrite8(1 << RTL8169_MMIO_CMD_TE_SHIFT, ctx->mmio_base + RTL8169_MMIO_CMD);

	/* tx config */
	iowrite32(RTL8169_MMIO_TXCFG_INIT_MASK, ctx->mmio_base + RTL8169_MMIO_TXCFG);

	/* max rx packet size */
	iowrite16(RTL8169_DATA_BUFF_SIZE, ctx->mmio_base + RTL8169_MMIO_RMS);

	/* minimum tx threshold, will also send packet if at least one whole package is pending */
	iowrite8(0x3B, ctx->mmio_base + RTL8169_MMIO_ETTHR);

	/* write bus address of rx descriptors array to the hardware */
	iowrite32(ctx->rx_descriptor_bus_addr & DMA_BIT_MASK(32), ctx->mmio_base + RTL8169_MMIO_RDSAR);
	iowrite32(ctx->rx_descriptor_bus_addr >> 32, ctx->mmio_base + RTL8169_MMIO_RDSAR + 4);

	/* write bus address of tx descriptors array to the hardware */
	iowrite32(ctx->tx_descriptor_bus_addr & DMA_BIT_MASK(32), ctx->mmio_base + RTL8169_MMIO_TNPDS);
	iowrite32(ctx->tx_descriptor_bus_addr >> 32, ctx->mmio_base + RTL8169_MMIO_TNPDS + 4);

	/* configure interrupts */
	iowrite16(RTL8169_MMIO_INT_INIT_MASK, ctx->mmio_base + RTL8169_MMIO_IMASK);

	/* rx/tx enabled */
	iowrite8(RTL8169_MMIO_CMD_INIT_MASK, ctx->mmio_base + RTL8169_MMIO_CMD);

	/* accept all */
	iowrite32(0xffffffff, ctx->mmio_base + RTL8169_MMIO_MAR0);
	iowrite32(0xffffffff, ctx->mmio_base + RTL8169_MMIO_MAR0 + 4);

	/* lock configuration registers */
	rtl8169_disable_config_write(ctx);

	return 0;
}

static int rtl8169_probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
	int status;
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

	if (pcim_set_mwi(pci_dev) < 0)
		dev_info(&pci_dev->dev, "mem wr inval unavailable\n");

	status = pci_request_regions(pci_dev, "rtl8169");
	if (status)
	{
		dev_err(&pci_dev->dev, "could not request memory regions\n");
		return -ENODEV;
	}

	ctx->mmio_base = pcim_iomap(pci_dev, 0, 0);
	if (ctx->mmio_base == NULL)
	{
		dev_err(&pci_dev->dev, "could not remap MMIO region\n");
		return status;
	}

	status = pci_alloc_irq_vectors(pci_dev, 1, 1, PCI_IRQ_LEGACY);
	if (status < 0)
		return status;

	ctx->irq = pci_irq_vector(pci_dev, 0);
	status = devm_request_irq(&pci_dev->dev, ctx->irq, rtl8169_interrupt, IRQF_SHARED, "rtl8169", ctx);
	if (status < 0)
	{
		dev_err(&pci_dev->dev, "error while requesting irq\n");
		goto undo_irq_alloc;
	}

	status = rtl8169_chip_init(ctx);
	if (status < 0)
	{
		dev_err(&pci_dev->dev, "error while configuring network device\n");
		goto undo_irq_setup;
	}

	/* only irq setup uses non managed APIs, everything else will be freed automatically */
	status = rtl8169_netdev_init(ctx);
	if (status < 0)
		goto undo_irq_setup;

	/* prepare PHY */
	status = phy_connect_direct(ctx->ndev, ctx->phydev, rtl8169_phylink_handler, PHY_INTERFACE_MODE_GMII);
	if (status)
		goto undo_irq_setup;

	phy_attached_info(ctx->phydev);
	phy_init_hw(ctx->phydev);
	phy_start(ctx->phydev);

	dev_info(&pci_dev->dev, "rtl8169 probe successful\n");
	return 0;

undo_irq_setup:
	devm_free_irq(&pci_dev->dev, ctx->irq, ctx);
undo_irq_alloc:
	pci_free_irq_vectors(pci_dev);
	return status;
}

static void rtl8169_remove(struct pci_dev *pci_dev)
{
	struct rtl8169_context *ctx;

	ctx = pci_get_drvdata(pci_dev);
	pci_free_irq_vectors(pci_dev);
	phy_stop(ctx->phydev);
	phy_disconnect(ctx->phydev);
	dev_info(&pci_dev->dev, "rtl8169 removed\n");
}

module_pci_driver(rtl8169_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maciej Rutkowski");
MODULE_DESCRIPTION("Driver for RTL8169 PCI network card");