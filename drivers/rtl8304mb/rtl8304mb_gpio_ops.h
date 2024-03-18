#ifndef _RTL8304MB_GPIO_OPS_H_
#define _RTL8304MB_GPIO_OPS_H_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include<linux/err.h>
#include<linux/uaccess.h>
#include <linux/mod_devicetable.h>
#include <linux/mod_devicetable.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <dt-bindings/gpio/meson-t3-gpio.h>

int rtl8304mb_gpio_init(struct platform_device *pdev);
int rtl8304mb_gpio_deinit(struct platform_device *pdev);

void rtl8304mb_gpio_read_bit(uint * pdata);
void rtl8304mb_gpio_write_bit(uint data);
void _smiZbit(void);

#endif
