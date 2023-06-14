#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

struct rtl8169_context
{
	struct pci_dev *pci_dev;
	void __iomem *mmio_base;

	struct net_device *ndev;
};

struct rtl8169_net_context
{
	struct rtl8169_context *ctx;
};

static netdev_tx_t rtl8169_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev);

static int rtl8169_netdev_init(struct rtl8169_context *ctx);

static int rtl8169_probe(struct pci_dev *pci_dev, const struct pci_device_id *id);
static void rtl8169_remove(struct pci_dev *pci_dev);

static const struct pci_device_id rtl9168_ids[] = {
	{PCI_VDEVICE(REALTEK, 0x8169)},
	{},
};

MODULE_DEVICE_TABLE(pci, rtl9168_ids);

static struct pci_driver rtl9168_driver = {
	.name = "rtl9168",
	.id_table = rtl9168_ids,
	.probe = rtl8169_probe,
	.remove = rtl8169_remove,
};

static struct net_device_ops rtl8169_ndev_ops = {
	.ndo_start_xmit = rtl8169_ndo_start_xmit,
};

static netdev_tx_t rtl8169_ndo_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rtl8169_context *ctx;
	ctx = netdev_priv(dev);

	dev_info_ratelimited(&ctx->pci_dev->dev, "in start xmit callback\n");

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
	ctx->ndev->features = NETIF_F_IP_CSUM | NETIF_F_RXCSUM;

	net_ctx = netdev_priv(ctx->ndev);
	net_ctx->ctx = ctx;

	status = devm_register_netdev(&ctx->pci_dev->dev, ctx->ndev);
	return status;
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

	status = pcim_enable_device(pci_dev);
	if (status)
		return status;

	pci_set_master(pci_dev);

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

	status = rtl8169_netdev_init(ctx);
	if (status < 0)
	{
		dev_err(&pci_dev->dev, "error while configuring network device\n");
		return status;
	}

	dev_info(&pci_dev->dev, "rtl8169 probe successful\n");
	return 0;
}

static void rtl8169_remove(struct pci_dev *pci_dev)
{
	struct rtl8169_context *ctx;

	ctx = pci_get_drvdata(pci_dev);
	dev_info(&pci_dev->dev, "rtl8169 removed\n");
}

module_pci_driver(rtl9168_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maciej Rutkowski");
MODULE_DESCRIPTION("Driver for RTL8169 PCI network card");