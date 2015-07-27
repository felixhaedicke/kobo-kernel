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

#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include "u_serial.h"
#include "gadget_chips.h"


/* Defines */

#define GS_VERSION_STR			"v1.0"
#define GS_VERSION_NUM			0x1000

#define GS_LONG_NAME			"Gadget Serial / Open Accessory"
#define GS_VERSION_NAME			GS_LONG_NAME " " GS_VERSION_STR

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
#define GS_ACM_VENDOR_ID			0x0525	/* NetChip */
#define GS_ACM_PRODUCT_ID			0xa4a7	/* ... as CDC-ACM */
#define GS_ACCESSORY_VENDOR_ID			0x18d1	/* Google */
#define GS_ACCESSORY_PRODUCT_ID			0x2d00	/* Accessory mode */

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_DESCRIPTION_IDX		2

#define ACM_PORT_NO		0
#define ACCESSORY_PORT_NO	1

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
	/* .idVendor =		DYNAMIC */
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
MODULE_AUTHOR("Felix Hädicke");
MODULE_LICENSE("GPL");

/*-------------------------------------------------------------------------*/

static int gs_bind(struct usb_composite_dev *cdev);

static struct usb_composite_driver gserial_driver = {
	.name		= "g_accessory_serial",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= gs_bind,
};

static int acm_bind_config_port(struct usb_configuration *c)
{
	return acm_bind_config(c, ACM_PORT_NO);
}

static struct usb_configuration config_acm = {
	.label = "CDC ACM",
	.bind = acm_bind_config_port,
	.bConfigurationValue = 2,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes	= USB_CONFIG_ATT_SELFPOWER,
};

static int gser_bind_config_port(struct usb_configuration *c)
{
	return gser_bind_config(c, ACCESSORY_PORT_NO);
}

static struct usb_configuration config_accessory = {
	.label = "Open Accessory",
	.bind = gser_bind_config_port,
	.bConfigurationValue = 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes	= USB_CONFIG_ATT_SELFPOWER,
};

static struct class *usb_composite_device_class;
static struct device *accessory_multi_class_device;

static atomic_t in_accessory_mode	= ATOMIC_INIT(0);
static atomic_t gserial_initialized	= ATOMIC_INIT(0);
static atomic_t gadget_registered	= ATOMIC_INIT(0);
static atomic_t can_change_mode		= ATOMIC_INIT(0);

static ssize_t accessory_port_no_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", (int) ACCESSORY_PORT_NO);
}

static ssize_t accessory_mode_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&in_accessory_mode));
}

static ssize_t accessory_mode_store(
		struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	bool new_value;
	int status = 0;

	if (size == 0)
		return -EINVAL;

	switch (*buf) {
	case '1':
		new_value = true;
		break;
	case '0':
		new_value = false;
		break;
	default:
		return -EINVAL;
	}

	if (new_value != ((atomic_read(&in_accessory_mode)) ? true : false)) {
		if (atomic_xchg(&can_change_mode, 0) == 0)
			return -1;

		usb_composite_unregister(&gserial_driver);
		
		if (new_value) {
			device_desc.bDeviceClass = USB_CLASS_VENDOR_SPEC;
			device_desc.idProduct =
				cpu_to_le16(GS_ACCESSORY_PRODUCT_ID);
			device_desc.idVendor =
				cpu_to_le16(GS_ACCESSORY_VENDOR_ID);
		} else {
			device_desc.bDeviceClass = USB_CLASS_COMM;
			device_desc.idProduct =
				cpu_to_le16(GS_ACM_PRODUCT_ID);
			device_desc.idVendor =
				cpu_to_le16(GS_ACM_VENDOR_ID);
		}

		atomic_set(&in_accessory_mode, new_value ? 1 : 0);
		status = usb_composite_register(&gserial_driver);
		if (status != 0)
			atomic_set(&gadget_registered, 0);
	}

	if (status == 0)
		return size;
	else
		return (status < 0) ? status : -status;
}

static DEVICE_ATTR(accessory_port_no, S_IRUGO,
		accessory_port_no_show,
		NULL);
static DEVICE_ATTR(accessory_mode, S_IRUGO | S_IWUSR,
		accessory_mode_show,
		accessory_mode_store);

static struct device_attribute *serial_attributes[] = {
	&dev_attr_accessory_port_no,
	&dev_attr_accessory_mode,
	NULL
};

static int gs_bind(struct usb_composite_dev *cdev)
{
	int			gcnum;
	struct usb_gadget	*gadget = cdev->gadget;
	int			status;

	atomic_set(&can_change_mode, 0);

	if (!atomic_read(&gserial_initialized)) {
		status = gserial_setup(cdev->gadget, 2);
		if (status < 0)
			return status;
		else
			atomic_set(&gserial_initialized, 1);
	}

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

	config_acm.iConfiguration = status;
	config_accessory.iConfiguration = status;

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
		config_acm.descriptors = otg_desc;
		config_acm.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
		config_accessory.descriptors = otg_desc;
		config_accessory.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	/* register our configuration */
	if (atomic_read(&in_accessory_mode))
		status = usb_add_config(cdev, &config_accessory);
	else
		status = usb_add_config(cdev, &config_acm);

	if (status < 0)
		goto fail;

	atomic_set(&can_change_mode, 1);

	INFO(cdev, "%s\n", GS_VERSION_NAME);

	return 0;

fail:
	return status;
}

static int __init init(void)
{
	int status;
	struct device_attribute **attrs = serial_attributes;
	struct device_attribute *attr;

	device_desc.bDeviceClass = USB_CLASS_COMM;
	device_desc.idProduct = cpu_to_le16(GS_ACM_PRODUCT_ID);
	device_desc.idVendor = cpu_to_le16(GS_ACM_VENDOR_ID);

	usb_composite_device_class = class_create(THIS_MODULE,
		"usb_composite_device");
	if (IS_ERR(usb_composite_device_class)) {
		status = PTR_ERR(usb_composite_device_class);
		goto fail_create_device_class;
	}

	accessory_multi_class_device = device_create(
			usb_composite_device_class, NULL,
			MKDEV(0, 0), NULL, "accessory_multi");
	if (IS_ERR(accessory_multi_class_device)) {
		status = PTR_ERR(accessory_multi_class_device);
		goto fail_device_create;
	}

	while ((attr = *attrs++)) {
		status = device_create_file(accessory_multi_class_device, attr);
		if (status)
			goto fail_device_create_file;
	}

	status = usb_composite_register(&gserial_driver);
	if (status == 0) {
		atomic_set(&gadget_registered, 1);
		return 0;
	}

fail_device_create_file:
	device_destroy(usb_composite_device_class,
		accessory_multi_class_device->devt);
fail_device_create:
	class_destroy(usb_composite_device_class);
	usb_composite_device_class = NULL;
fail_create_device_class:
	if (atomic_read(&gserial_initialized))
		gserial_cleanup();

	return status;
}
module_init(init);

static void __exit cleanup(void)
{
	device_destroy(usb_composite_device_class,
		accessory_multi_class_device->devt);
	class_destroy(usb_composite_device_class);

	if (atomic_read(&gadget_registered))
		usb_composite_unregister(&gserial_driver);

	if (atomic_read(&gserial_initialized))
		gserial_cleanup();
}
module_exit(cleanup);
