#include <linux/cma.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/mm.h>
#include <linux/module.h> /* Needed by all modules */
#include <linux/pci.h>
#include <stddef.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include "thing.h"

MODULE_LICENSE("GPL");      // The license type -- this affects available functionality
MODULE_AUTHOR("Tim Allen"); // The author -- visible when you use modinfo
MODULE_DESCRIPTION("video thing");
MODULE_VERSION("0.1"); // A version number to inform users

static const size_t dbsize = 16 * 1024;
static struct class *class;
static struct device *device_file;
static struct context *ctxs[8]; //up to 8 cards

// define which file operations are supported
struct file_operations fops = {
    .owner  = THIS_MODULE,
    .llseek = NULL,
    .read   = fops_read,
    .write  = fops_write,
    .poll   = NULL,
    .unlocked_ioctl = NULL,
    .mmap   = NULL,					//mmap
    .open   = fops_open,			//open
    .flush  = NULL,
    .release = fops_release,		//close
    .fsync  = NULL,
    .fasync = NULL,
    .lock   = NULL,
};


static const struct pci_device_id pci_device_ids[] = {
  {PCI_DEVICE(0x1002, 0x6610)}, // some GPU card I'm using for dev
  {PCI_DEVICE(0x10de, 0x0f00)}, // Nvidia something or other
  {PCI_DEVICE(0x102b, 0x0532)}, // some Matrox VGA card
  {0, /* end of list */},
};

static struct pci_driver thing_driver = {
  .name = DRIVER_NAME,
  .id_table = pci_device_ids,
  .probe = thing_probe,
  .remove = thing_remove,
};

/** init is the entry point for the module
 *
 */
static int __init thing_init(void) {
	class = class_create(THIS_MODULE, "video_thing");
	if(!class) {
		printk (KERN_ERR "[%s:init] unable to create class device", DRIVER_NAME);
		return -EIO;
	}

	if (pci_register_driver(&thing_driver) < 0) {
		printk(KERN_ERR "[%s:init] unable to register", DRIVER_NAME);
		return -EIO;
	}
	printk(KERN_INFO "[%s:init] driver registered", DRIVER_NAME);

	return 0; // A non 0 return means init_module failed; module can't be loaded.
}
module_init(thing_init); // this macro registers the module entry point

/** a card has been inserted, time to set it up
 *
 */
static int thing_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
	int err;
	uint8_t byte = 0;
	uint16_t word = 0;
	uint32_t dword = 0;
	char devfilename[255];
	struct context *ctx;
	static int cards = 0;

	//begin filling out the context struct
	ctx = kmalloc(sizeof (struct context), GFP_KERNEL);
	memset(ctx, 0, sizeof (struct context));
	ctxs[cards] = ctx; // store *ctx indexed by card probe order
	pci_set_drvdata(pdev, (void*)ctx); // store *ctx indexed by pdev
	ctx->card = cards++; // card number in arbitrary order
	ctx->pdev = pdev;

	err = pci_enable_device(pdev);
	if (err) {
		printk(KERN_ERR "[%s:probe] pci_enable_device returned %d", DRIVER_NAME, err);
        return -ENODEV;
	}

	err = pci_request_region(pdev, 0, DRIVER_NAME);
	if (err) {
		printk(KERN_ERR "[%s:probe] request_region returned %d", DRIVER_NAME, err);
		return -ENODEV;
	}

	// say some useless stuff about the card
	pci_read_config_word(pdev, PCI_VENDOR_ID, &word);
	printk(KERN_INFO "[%s:probe] PCI_VENDOR_ID: %4.4x", DRIVER_NAME, word);
	pci_read_config_word(pdev, PCI_DEVICE_ID, &word);
	printk(KERN_INFO "[%s:probe] PCI_DEVICE_ID: %4.4x", DRIVER_NAME, word);
	pci_read_config_word(pdev, PCI_COMMAND, &word);
	printk(KERN_INFO "[%s:probe] PCI_COMMAND: %4.4x", DRIVER_NAME, word);
	pci_read_config_word(pdev, PCI_STATUS, &word);
	printk(KERN_INFO "[%s:probe] PCI_STATUS: %4.4x", DRIVER_NAME, word);
	pci_read_config_byte(pdev, PCI_REVISION_ID, &byte);
	printk(KERN_INFO "[%s:probe] PCI_REVISION_ID: %2.2x", DRIVER_NAME, byte);
	pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &byte);
	printk(KERN_INFO "[%s:probe] PCI_INTERRUPT_LINE: %2.2x", DRIVER_NAME, byte);
	pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &byte);
	printk(KERN_INFO "[%s:probe] PCI_INTERRUPT_PIN: %2.2x", DRIVER_NAME, byte);

	ctx->pci_mem_start = pci_resource_start(pdev, 0);
    ctx->pci_mem_length = pci_resource_len(pdev, 0);
	pci_read_config_dword(pdev, ctx->pci_mem_start, &dword);
	printk(KERN_INFO "[%s:probe] device_info: %8.8x", DRIVER_NAME, dword);
	pci_read_config_dword(pdev, ctx->pci_mem_start + 0x3c, &dword);
	printk(KERN_INFO "[%s:probe] revision_number: %8.8x", DRIVER_NAME, dword);

    ctx->bar0_base_addr = pci_iomap(pdev, 0, ctx->pci_mem_length); // map the BARs
	printk(KERN_INFO "[%s:probe] bar0_base_addr: %p", DRIVER_NAME, ctx->bar0_base_addr);

	// create the device file entry (/dev/thing)
	snprintf(devfilename, sizeof devfilename, "thing%d", ctx->card);
	device_file = device_create(class, NULL, MKDEV(810, ctx->card), 0, devfilename);
	if(!device_file) {
		printk (KERN_ERR "[%s:init] unable to create device class device", DRIVER_NAME);
		return -EIO;
	} else {
		printk(KERN_INFO "[%s:probe] device %s created.", DRIVER_NAME, devfilename);
	}

	// set-up file ops
	cdev_init(&ctx->cdev, &fops);
    ctx->cdev.owner = THIS_MODULE;
    ctx->cdev.ops = &fops;
    err = cdev_add(&ctx->cdev, MKDEV(810, ctx->card), 1);
	if (err) {
		printk(KERN_INFO "[%s:probe] cdev_add failed with %d", DRIVER_NAME, err);
		return -EIO;
	}

	// set-up the DMA buffers
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err) {
		printk(KERN_INFO "[%s:probe] dma_set_mask returned: %d", DRIVER_NAME, err);
		return -EIO;
	}
	ctx->buf1 = dma_alloc_coherent(&pdev->dev, dbsize, &ctx->buf1_dma_handle, GFP_KERNEL);
	if (!ctx->buf1) {
		printk(KERN_ALERT "[%s:probe] failed to allocate coherent buffer", DRIVER_NAME);
		return -EIO;
	}
	printk(KERN_INFO "[%s:probe] buf1 = %p buf1_dma_handle = %16.16llx ", DRIVER_NAME, ctx->buf1, ctx->buf1_dma_handle);
	iowrite32(ctx->buf1_dma_handle, ctx->bar0_base_addr + 0x140); // tell card where to DMA from

	// set-up IRQs
	err = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	printk(KERN_INFO "[%s:probe] pci_alloc_irq_vectors returns %d", DRIVER_NAME, err);

	ctx->irq = pci_irq_vector(pdev, 0);
	printk(KERN_INFO "[%s:probe] pci_irq_vector returns %d", DRIVER_NAME, ctx->irq);

	err = request_irq(ctx->irq, &thing_isr, IRQF_SHARED, DRIVER_NAME, ctx);
	printk(KERN_INFO "[%s:probe] request_irq returns %d", DRIVER_NAME, err);

	// config done
	dword = ioread32(ctx->bar0_base_addr + 0x00);
	dword |= 0x10000000; // config_done
	iowrite32(dword, ctx->bar0_base_addr + 0x00);

	return 0; // A non 0 return means init_module failed; module can't be loaded.
}

/** open function - called when the device file is opened
 *
 */
static int fops_open (struct inode *inode, struct file *filp) {
	int major, minor;
	int card;
	struct context *ctx;

	major = imajor(inode);
	card = minor = iminor(inode);
	ctx = ctxs[card];

	printk(KERN_INFO "[%s:open] major = %d", DRIVER_NAME, major);
	printk(KERN_INFO "[%s:open] minor = %d", DRIVER_NAME, minor);

	ctx->opens++;

	return 0;
}

/** read function - called when the device file is written to
 *
 */
static ssize_t fops_read(struct file *filp, char __user *ubuf, size_t count, loff_t *offp) {
	char kbuf[512];
	uint c = 0; //position in kbuf, start at 0
	int card; // the minor tells us which card
	uint8_t byte = 0;
	uint16_t word = 0;
	uint32_t dword =0;
	struct context *ctx;

	card = iminor(filp->f_inode);
	ctx = ctxs[card];

	printk(KERN_INFO "[%s:read] minor = %d count = %ld *offp = %lld", DRIVER_NAME, card, count, *offp);
	if (*offp)
		return 0;

	c += snprintf(&kbuf[c], sizeof kbuf, "read from card\t%d\n", card);
	pci_read_config_word(ctx->pdev, PCI_VENDOR_ID, &word);
	c += snprintf(&kbuf[c], sizeof kbuf, "PCI_VENDOR_ID:\t%4.4x\n", word);
	pci_read_config_word(ctx->pdev, PCI_DEVICE_ID, &word);
	c += snprintf(&kbuf[c], sizeof kbuf, "PCI_DEVICE_ID:\t%4.4x\n", word);
	pci_read_config_word(ctx->pdev, PCI_COMMAND, &word);
	c += snprintf(&kbuf[c], sizeof kbuf, "PCI_COMMAND:\t%4.4x\n", word);
	pci_read_config_word(ctx->pdev, PCI_STATUS, &word);
	c += snprintf(&kbuf[c], sizeof kbuf, "PCI_STATUS:\t%4.4x\n", word);
	pci_read_config_byte(ctx->pdev, PCI_REVISION_ID, &byte);
	c += snprintf(&kbuf[c], sizeof kbuf, "PCI_REV_ID:\t%2.2x\n", byte);

	dword = ioread32(ctx->bar0_base_addr + 0x00);
	c += snprintf(&kbuf[c], sizeof kbuf, "DEV NFO:\t%8.8x\n", dword);
	dword = ioread32(ctx->bar0_base_addr + 0x04);
	c += snprintf(&kbuf[c], sizeof kbuf, "IRQ PND:\t%8.8x\n", dword);
	dword = ioread32(ctx->bar0_base_addr + 0x08);
	c += snprintf(&kbuf[c], sizeof kbuf, "IRQ ACK:\t%8.8x\n", dword);
	dword = ioread32(ctx->bar0_base_addr + 0x200);
	c += snprintf(&kbuf[c], sizeof kbuf, "FPGA Status:\t%8.8x\n", dword);
	copy_to_user(ubuf, kbuf, c);
	*offp += c;
	return c;
}

/** write function - called when the device file is written to
 *
 */
static ssize_t fops_write(struct file *filp, const char __user *buffer, size_t length, loff_t * offset) {
	int card; // the minor tells us which card
	struct context *ctx;

	card = iminor(filp->f_inode);
	ctx = ctxs[card];

	if (length%32)
		printk(KERN_INFO "[%s:write] WARNING: not a mutiple of 32, sending anyway", DRIVER_NAME);

	if (length > 16*32) {
		printk(KERN_INFO "[%s:write] ERROR: maximum of 16", DRIVER_NAME);
		return -EIO;
	}

	printk(KERN_INFO "[%s:write] sent %ld of 'em", DRIVER_NAME, length / 32);

	memset(ctx->buf1, 0, dbsize);
	copy_from_user(ctx->buf1, buffer, length);
	iowrite32(length / 32, ctx->bar0_base_addr + 0x24); //number of 32B (256b) words to be downloaded to the card
	iowrite32(0x80000000, ctx->bar0_base_addr + 0x20); //Data0 Buffer and trigger
	ndelay(250);
	iowrite32(0x00000000, ctx->bar0_base_addr + 0x20); //Data0 Buffer and trigger
	return length;
}

/** Interrupt Service Routine
 *
 */
static irqreturn_t thing_isr(int irq, void *lp) {
	int irq_pending;

	//XXX ctxs, uh no, need em sorted by irq
	irq_pending = ioread32(ctxs[0]->bar0_base_addr + 0x04);
	printk(KERN_INFO "[%s:isr] RXd IRQ %d, irq_pending = %2.2x\n", DRIVER_NAME, irq, irq_pending);
	iowrite32(irq_pending, ctxs[0]->bar0_base_addr + 0x08);

	return IRQ_HANDLED;
}

/** close function - called when the device file is closed
 *
 */
static int fops_release (struct inode *inode, struct file *filp) {
	int major, minor;
	int card;
	struct context *ctx;

	major = imajor(inode);
	card = minor = iminor(inode);
	ctx = ctxs[card];

	printk(KERN_INFO "[%s:release] major = %d", DRIVER_NAME, major);
	printk(KERN_INFO "[%s:release] minor = %d", DRIVER_NAME, minor);

	ctx->opens--;
	return 0;
}

/** remove a card
 *
 */
static void thing_remove(struct pci_dev *pdev) {
	struct context *ctx;

	ctx = (struct context*) pci_get_drvdata(pdev);
	printk(KERN_INFO "[%s:remove] free coherent buffer", DRIVER_NAME);
	if (ctx->buf1)
		dma_free_coherent(&pdev->dev, dbsize, ctx->buf1, ctx->buf1_dma_handle);

	free_irq(ctx->irq, ctx);
	pci_free_irq_vectors(pdev);

	device_destroy(class, MKDEV(810, ctx->card));
	cdev_del(&ctx->cdev);
	pci_release_region(pdev, 0);
	pci_disable_device(pdev);
	kfree(ctx);
}

/** prepare to be gone
 *
 */
static void __exit thing_exit(void) {
	pci_unregister_driver(&thing_driver);
	printk(KERN_INFO "[%s:exit] driver unregistered", DRIVER_NAME);
	class_unregister(class);
	printk(KERN_INFO "[%s:exit] Goodbye world", DRIVER_NAME);
}
module_exit(thing_exit); // this macro registers the module exit point
