#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>

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

static int rtl8169_probe(struct pci_dev *pci_dev, const struct pci_device_id *id)
{
	dev_info(&pci_dev->dev, "in probe\n");
	return 0;
}

static void rtl8169_remove(struct pci_dev *pci_dev)
{
	dev_info(&pci_dev->dev, "in remove\n");
}

module_pci_driver(rtl9168_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maciej Rutkowski");
MODULE_DESCRIPTION("Driver for RTL8169 PCI network card");