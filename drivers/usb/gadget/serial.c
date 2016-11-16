/*
 * serial.c -- USB gadget serial driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/wait.h>

#include "u_serial.h"
#include "gadget_chips.h"


/* Defines */

#define GS_VERSION_STR			"v2.4"
#define GS_VERSION_NUM			0x2400

#define GS_LONG_NAME			"Gadget Serial"
#define GS_VERSION_NAME			GS_LONG_NAME " " GS_VERSION_STR


#define AOA_IOCTL_SWITCH_TO_AOA _IO('g', 1)
#define AOA_IOCTL_SWITCH_TO_ACM _IO('g', 2)
#define AOA_IOCTL_RESET _IO('g', 3)

#define AOA_MAX_STR_SIZE 256

#define AOA_PROTOCOL_VERSION 2

#define AOA_REQ_GET_PROTOCOL 51
#define AOA_REQ_SEND_STRING 52
#define AOA_REQ_START 53

enum aoa_string_type {
	AOA_STRING_MANUFACTURER = 0,
	AOA_STRING_MODEL = 1,
	AOA_STRING_DESCRIPTION = 2,
	AOA_STRING_VERSION = 3,
	AOA_STRING_URI = 4,
	AOA_STRING_SERIAL = 5
};

enum aoa_event_type {
	AOA_EVENT_CONNECTED_ACM = 0,
	AOA_EVENT_DISCONNECTED_ACM = 1,
	AOA_EVENT_STRING_RECEIVED = 2,
	AOA_EVENT_START_REQUESTED = 3
};

struct aoa_string {
	enum aoa_string_type type;
	char str[AOA_MAX_STR_SIZE];
};

struct aoa_event {
	enum aoa_event_type type;
	struct aoa_string string;
};

static void aoa_event_add(enum aoa_event_type type);
static void aoa_str_received_event_add(enum aoa_string_type type, void *strbuf,
				       size_t len);

static atomic_t aoa_connected_acm = ATOMIC_INIT(false);

static void aoa_set_connected_acm(bool new_connected_acm)
{
	if (atomic_xchg(&aoa_connected_acm, new_connected_acm) !=
			new_connected_acm) {
		if (new_connected_acm)
			aoa_event_add(AOA_EVENT_CONNECTED_ACM);
		else
			aoa_event_add(AOA_EVENT_DISCONNECTED_ACM);
	}
}

enum gadget_mode {
	MODE_NONE,
	MODE_ACM,
	MODE_AOA
};

static DEFINE_MUTEX(mode_switch_mutex);
static enum gadget_mode current_mode = MODE_NONE;
static int switch_mode(enum gadget_mode new_mode);
static int reset_mode(void);

static atomic_t aoa_ctrl_open = ATOMIC_INIT(false);

/*-------------------------------------------------------------------------*/

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

#include "f_acm.c"
#include "f_obex.c"
#include "f_serial.c"
#include "u_serial.c"

/*-------------------------------------------------------------------------*/

/* Thanks to NetChip Technologies for donating this product ID.
*
* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
* Instead:  allocate your own, using normal USB-IF procedures.
*/
#define GS_ACM_VENDOR_ID	0x0525	/* NetChip */
#define GS_ACM_PRODUCT_ID	0xa4a7	/* ... as CDC-ACM */

#define GS_AOA_VENDOR_ID	0x18d1	/* Google */
#define GS_AOA_PRODUCT_ID	0x2d00 /* Accessory mode */

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_DESCRIPTION_IDX		2

static char manufacturer[50];

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer,
	[STRING_PRODUCT_IDX].s = GS_VERSION_NAME,
	[STRING_DESCRIPTION_IDX].s = NULL /* updated; f(use_acm) */,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength =		USB_DT_DEVICE_SIZE,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(0x0200),
	/* .bDeviceClass = f(use_acm) */
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */
	/* .idVendor = DYNAMIC */
	/* .idProduct =	f(use_acm) */
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

/*-------------------------------------------------------------------------*/

/* Module */
MODULE_DESCRIPTION(GS_VERSION_NAME);
MODULE_AUTHOR("Al Borchers");
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");

/*-------------------------------------------------------------------------*/

static struct usb_configuration serial_config_driver = {
	/* .label = f(use_acm) */
	/* .bConfigurationValue = f(use_acm) */
	/* .iConfiguration = DYNAMIC */
	.bmAttributes	= USB_CONFIG_ATT_SELFPOWER,
};

static int gs_bind(struct usb_composite_dev *cdev, bool in_aoa_mode)
{
	int			gcnum;
	struct usb_gadget	*gadget = cdev->gadget;
	int			status;

	if (in_aoa_mode) {
		status = gserial_setup_ex(cdev->gadget, 1, "ttyAOA");
	} else {
		status = gserial_setup(cdev->gadget, 1);
	}
	if (status < 0)
		return status;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	/* device description: manufacturer, product */
	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name);
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_MANUFACTURER_IDX].id = status;

	device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_PRODUCT_IDX].id = status;

	device_desc.iProduct = status;

	/* config description */
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_DESCRIPTION_IDX].id = status;

	serial_config_driver.iConfiguration = status;

	/* set up other descriptors */
	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(GS_VERSION_NUM | gcnum);
	else {
		/* this is so simple (for now, no altsettings) that it
		 * SHOULD NOT have problems with bulk-capable hardware.
		 * so warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("gs_bind: controller '%s' not recognized\n",
			gadget->name);
		device_desc.bcdDevice =
			cpu_to_le16(GS_VERSION_NUM | 0x0099);
	}

	if (gadget_is_otg(cdev->gadget)) {
		serial_config_driver.descriptors = otg_desc;
		serial_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	/* register our configuration */
	status = usb_add_config(cdev, &serial_config_driver,
				in_aoa_mode ? gser_bind_config : acm_bind_config);
	if (status < 0)
		goto fail;

	INFO(cdev, "%s\n", GS_VERSION_NAME);

	return 0;

fail:

	gserial_cleanup();
	return status;
}

static int gs_bind_acm(struct usb_composite_dev *cdev)
{
	return gs_bind(cdev, false);
}

static int gs_bind_aoa(struct usb_composite_dev *cdev)
{
	return gs_bind(cdev, true);
}

static struct usb_composite_driver gserial_driver = {
	.name		= "g_serial",
	.dev		= &device_desc,
	.strings	= dev_strings,
};

struct aoa_event_list_node {
	enum aoa_event_type type;
	enum aoa_string_type str_type;
	char *str;
	struct list_head list;
};

DECLARE_WAIT_QUEUE_HEAD(aoa_waitqueue);
struct aoa_event_list_node *first_aoa_event_node = NULL;
struct aoa_event_list_node *last_aoa_event_node = NULL;
struct aoa_event_list_node *next_aoa_event_node = NULL;

static bool aoa_event_node_add(enum aoa_event_type type,
			       enum aoa_string_type *str_type,
			       char *str)
{
	struct aoa_event_list_node* new_node;
	unsigned long flags;

	new_node = kmalloc(sizeof(struct aoa_event_list_node), GFP_ATOMIC);
	if (unlikely(new_node == NULL)) {
		printk(KERN_ERR
		       "Could not aquire memory for AOA event list node!\n");
		return false;
	}

	new_node->type = type;
	if (str_type != NULL)
		new_node->str_type = *str_type;
	new_node->str = str;

	spin_lock_irqsave(&aoa_waitqueue.lock, flags);

	if ((first_aoa_event_node != NULL)
			&& (type == AOA_EVENT_CONNECTED_ACM)) {
		struct aoa_event_list_node *node, *temp;
		list_for_each_entry_safe(node, temp,
					 &first_aoa_event_node->list,
					 list) {
			list_del(&node->list);
			kfree(node->str);
			kfree(node);
		}
		first_aoa_event_node = NULL;
		next_aoa_event_node = NULL;
	}

	INIT_LIST_HEAD(&new_node->list);

	if (first_aoa_event_node == NULL) {
		first_aoa_event_node = new_node;
		next_aoa_event_node = new_node;
	} else
		list_add(&new_node->list, &last_aoa_event_node->list);

	if (next_aoa_event_node == NULL)
		next_aoa_event_node = new_node;
	last_aoa_event_node = new_node;

	wake_up_locked(&aoa_waitqueue);

	spin_unlock_irqrestore(&aoa_waitqueue.lock, flags);

	return true;
}

static void aoa_event_add(enum aoa_event_type type)
{
	aoa_event_node_add(type, NULL, NULL);
}

static void aoa_str_received_event_add(enum aoa_string_type type, void *strbuf,
				       size_t len)
{
	size_t str_mem_len = min(len + 1, (size_t) AOA_MAX_STR_SIZE);
	char *str = kmalloc(str_mem_len, GFP_ATOMIC);

	if (unlikely(str == NULL)) {
		printk(KERN_ERR
		       "Could not aquire memory for AOA string!\n");
		kfree(str);
		return;
	}

	memcpy(str, strbuf, str_mem_len - 1);
	str[str_mem_len - 1] = 0;

	if (unlikely(!aoa_event_node_add(AOA_EVENT_STRING_RECEIVED, &type,
					 str)))
		kfree(str);
}

static int aoa_open(struct inode *ip, struct file *fp)
{
	int ret;

	if (atomic_xchg(&aoa_ctrl_open, true))
		return -EBUSY;

	ret = switch_mode(MODE_ACM);
	if (ret == 0) {
		spin_lock_irq(&aoa_waitqueue.lock);
		next_aoa_event_node = first_aoa_event_node;
		spin_unlock_irq(&aoa_waitqueue.lock);
	} else {
		atomic_set(&aoa_ctrl_open, false);
	}

	return ret;
}

static int aoa_release(struct inode *ip, struct file *fp)
{
	switch_mode(MODE_NONE);
	atomic_set(&aoa_ctrl_open, false);
	return 0;
}

static ssize_t aoa_read(struct file *file, char __user *buf, size_t len,
			loff_t *ptr)
{
	ssize_t ret;

	spin_lock_irq(&aoa_waitqueue.lock);

	/* reading anything else than one single event is not supported */
	if (unlikely((buf == NULL) || (len != sizeof(struct aoa_event)))) {
		ret = -EINVAL;
		goto done;
	}

	if (unlikely(wait_event_interruptible_exclusive_locked_irq(
				aoa_waitqueue, next_aoa_event_node != NULL))) {
		ret = -EINTR;
		goto done;
	}

	if (next_aoa_event_node->str == NULL) {
		ret = sizeof(enum aoa_event_type);
		if (unlikely(copy_to_user(buf, &next_aoa_event_node->type,
					  ret))) {
			ret = -EFAULT;
			goto done;
		}
	} else {
		size_t str_mem_len = strlen(next_aoa_event_node->str) + 1;
		struct aoa_event ev;
		ret = sizeof(enum aoa_event_type)
				+ sizeof(enum aoa_string_type)
				+ str_mem_len;
		ev.type = next_aoa_event_node->type;
		ev.string.type = next_aoa_event_node->str_type;
		memcpy(ev.string.str, next_aoa_event_node->str, str_mem_len);

		if (unlikely(copy_to_user(buf, &ev, ret))) {
			ret = -EFAULT;
			goto done;
		}
	}

	if (last_aoa_event_node == next_aoa_event_node)
		next_aoa_event_node = NULL;
	else
		next_aoa_event_node = list_entry(next_aoa_event_node->list.next,
						 struct aoa_event_list_node,
						 list);

done:
	spin_unlock_irq(&aoa_waitqueue.lock);
	return ret;
}

static long aoa_ioctl(struct file *fp, unsigned code, unsigned long value)
{
	switch (code) {
	case AOA_IOCTL_SWITCH_TO_AOA:
		return switch_mode(MODE_AOA);

	case AOA_IOCTL_SWITCH_TO_ACM:
		return switch_mode(MODE_ACM);

	case AOA_IOCTL_RESET:
		return reset_mode();

	default:
		return -ENOTTY;
	}
}

static unsigned int aoa_poll(struct file *file, poll_table *wait)
{
	int ret;

	poll_wait(file, &aoa_waitqueue, wait);
	spin_lock_irq(&aoa_waitqueue.lock);
	if (next_aoa_event_node == NULL)
		ret = 0;
	else
		ret = POLLIN | POLLRDNORM;
	spin_unlock_irq(&aoa_waitqueue.lock);

	return ret;
}

static const struct file_operations aoa_ctrl_device_fops = {
	.owner = THIS_MODULE,
	.open = aoa_open,
	.release = aoa_release,
	.read = aoa_read,
	.unlocked_ioctl = aoa_ioctl,
	.poll = aoa_poll,
};

static struct miscdevice aoa_ctrl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aoa_ctrl",
	.fops = &aoa_ctrl_device_fops,
};

static int switch_mode(enum gadget_mode new_mode)
{
	int ret;

	mutex_lock(&mode_switch_mutex);

	if (current_mode == new_mode) {
		ret = 0;
		goto done;
	}

	if (current_mode != MODE_NONE) {
		usb_composite_unregister(&gserial_driver);
		gserial_cleanup();
	}

	switch (new_mode) {
	case MODE_NONE:
		current_mode = MODE_NONE;
		ret = 0;
		goto done;

	case MODE_ACM:
		serial_config_driver.label = "CDC ACM config";
		serial_config_driver.bConfigurationValue = 2;
		device_desc.bDeviceClass = USB_CLASS_COMM;
		device_desc.idVendor = cpu_to_le16(GS_ACM_VENDOR_ID);
		device_desc.idProduct = cpu_to_le16(GS_ACM_PRODUCT_ID);
		strings_dev[STRING_DESCRIPTION_IDX].s =
				serial_config_driver.label;

		ret = usb_composite_probe(&gserial_driver, gs_bind_acm);
		if (ret != 0)
			goto done;
		break;

	case MODE_AOA:
		serial_config_driver.label = "Android Open Accessory config";
		serial_config_driver.bConfigurationValue = 1;
		device_desc.bDeviceClass = USB_CLASS_VENDOR_SPEC;
		device_desc.idVendor = cpu_to_le16(GS_AOA_VENDOR_ID);
		device_desc.idProduct = cpu_to_le16(GS_AOA_PRODUCT_ID);
		strings_dev[STRING_DESCRIPTION_IDX].s =
				serial_config_driver.label;

		ret = usb_composite_probe(&gserial_driver, gs_bind_aoa);
		if (ret != 0)
			goto done;
		break;
	};

	current_mode = new_mode;

done:
	mutex_unlock(&mode_switch_mutex);
	return ret;
}

static int reset_mode(void)
{
	int ret;

	mutex_lock(&mode_switch_mutex);

	if (current_mode == MODE_NONE) {
		ret = 0;
		goto done;
	}

	usb_composite_unregister(&gserial_driver);
	gserial_cleanup();

	ret = usb_composite_probe(&gserial_driver,
			(current_mode == MODE_ACM) ? gs_bind_acm : gs_bind_aoa);

done:
	mutex_unlock(&mode_switch_mutex);
	return ret;
}

static int __init init(void)
{
	return misc_register(&aoa_ctrl_device);
}
module_init(init);

static void __exit cleanup(void)
{
	misc_deregister(&aoa_ctrl_device);
}
module_exit(cleanup);
