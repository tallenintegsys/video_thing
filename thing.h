#ifndef thing_H
#define thing_H

#define DEVICE_NAME				"video_thing"
#define DRIVER_NAME				"video_thing"

struct context {
	uint card;						// cards probed
	struct pci_dev *pdev;			// pci dev
	void *buf1; 					// DMA download data
	dma_addr_t buf1_dma_handle;		// physical address of the DMA download region
	struct cdev cdev; 				// char dev
	uint opens; 					// XXX do I really need to know this?
	uint32_t pci_mem_start; 		// pci config space
	uint32_t pci_mem_length; 		// pci config space
    void __iomem *bar0_base_addr; 	// the BARs were mapped here
	int irq; 						// our IRQ
};

static int thing_probe(struct pci_dev *, const struct pci_device_id *);
static void thing_remove(struct pci_dev *);
static int fops_open (struct inode *, struct file *);
static ssize_t fops_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t fops_write(struct file *, const char __user *, size_t, loff_t *);
static irqreturn_t thing_isr(int, void *);
static int fops_release (struct inode *, struct file *);

#endif //thing_H
