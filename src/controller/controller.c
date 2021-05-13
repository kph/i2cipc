// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C master mode stream driver
 *
 * Copyright (C) 2020 by Kevin Paul Herbert, Platina Systems, Inc.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/sched/signal.h>
#include <linux/crc32.h>

#define  DEVICE_NAME "i2c-master-stream-0" /* fixme per unit */
#define  CLASS_NAME  "i2c-master-stream"

#define STREAM_DATA_REG (0x40)
#define STREAM_CNT_REG (0x41)
#define STREAM_READ_CRC_REG0 (0x42)
#define STREAM_READ_CRC_REG1 (0x43)
#define STREAM_READ_CRC_REG2 (0x44)
#define STREAM_READ_CRC_REG3 (0x45)
#define STREAM_WRITE_CRC_REG0 (0x46)
#define STREAM_WRITE_CRC_REG1 (0x47)
#define STREAM_WRITE_CRC_REG2 (0x48)
#define STREAM_WRITE_CRC_REG3 (0x49)
#define STREAM_CTL_REG (0x4a)

struct stream_data {
	struct device dev;
	struct cdev cdev;
	u32 crc;
	struct i2c_client *client;
};

static int i2c_controller_stream_major;
static struct class *i2c_controller_stream_class;

static int i2c_controller_stream_open(struct inode *inode, struct file *filep)
{
	struct stream_data *stream = container_of(inode->i_cdev,
						  struct stream_data, cdev);

	get_device(&stream->dev);
	filep->private_data = stream;
	
	return 0;
}

static ssize_t i2c_controller_stream_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct stream_data *stream = filep->private_data;
	struct i2c_client *client = stream->client;
	int cnt;
	size_t done = 0;
	int error_cnt = 0;

	while (done < len) {
		cnt = i2c_smbus_read_byte_data(client, STREAM_CNT_REG);
		if (cnt < 0) {
			if (error_cnt++ > 10) {
				return cnt;
			}
			cnt = 0;
		} else {
			error_cnt = 0;
		}
		if (cnt == 0) {
			if (done != 0) {
				return done;
			}
			if (filep->f_flags & O_NONBLOCK)
				return -EAGAIN;
			msleep_interruptible(100);
			if (signal_pending(current))
				return -ERESTARTSYS;
		} else {
			size_t todo = min(len - done, (size_t)cnt);
			size_t i;
			int val;
			u8 buf[32];
			u32 crc32_calc, crc32_recv;
		
			todo = min(todo, (size_t)32);
//			printk("%s: cnt=%d todo=%ld\n", __func__, cnt, todo);

			for (i = 0; i < todo; i++) {
				val = i2c_smbus_read_byte_data(client,
								  STREAM_DATA_REG);
				if (val < 0) {
					if (error_cnt++ > 10) {
						return val;
					}
					msleep_interruptible(100);
					i--;
				} else {
					buf[i] = val;
					error_cnt = 0;
				}
			}

			crc32_calc = ~crc32_le(~0, buf, todo);
			crc32_recv = (i2c_smbus_read_byte_data(client, STREAM_READ_CRC_REG0) |
				      (i2c_smbus_read_byte_data(client, STREAM_READ_CRC_REG1) << 8) |
				      (i2c_smbus_read_byte_data(client, STREAM_READ_CRC_REG2) << 16) |
				      (i2c_smbus_read_byte_data(client, STREAM_READ_CRC_REG3) << 24));
			if (crc32_calc != crc32_recv) {
				i2c_smbus_write_byte_data(client, STREAM_CTL_REG,
							  0x20);
				continue;
			}
			
			for (i = 0; i < 10; i++) {
				i2c_smbus_write_byte_data(client,
							  STREAM_CTL_REG,
							  0x00);
				val = i2c_smbus_read_byte_data(client,
							       STREAM_CTL_REG);
				if (val != 0x00)
					continue;
				i2c_smbus_write_byte_data(client,
							  STREAM_CTL_REG,
							  0x80);
				val = i2c_smbus_read_byte_data(client,
							       STREAM_CTL_REG);
				if (val != 0x80)
					continue;
				break;
			}
			if (i == 10)
				return -EIO;
			if (copy_to_user(&buffer[done], buf, todo)) {
				return -EFAULT;
			}
			done += todo;
		}
	}
	return done;
}

static ssize_t i2c_controller_stream_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct stream_data *stream = filep->private_data;
	struct i2c_client *client = stream->client;
	size_t done = 0;
	int ret;
	int error_cnt;
	
	while (done < len) {
		size_t todo = min(len - done, (size_t)32);
		size_t i;
		u8 buf[32];
		u32 crc32_calc, crc32_recv;;

		if (signal_pending(current))
			return done ? done : -ERESTARTSYS;

		if (copy_from_user(buf, &buffer[done], todo)) {
			return -EFAULT;
		}

		error_cnt = 0;
		for (i = 0; i < todo; i++) {
			ret = i2c_smbus_write_byte_data(client,
							STREAM_DATA_REG,
							buf[i]);
			if (ret < 0) {
				if (error_cnt++ > 10) {
					return done != 0 ? done : ret;
				}
				msleep_interruptible(100);
				i--;
			} else {
				error_cnt = 0;
			}
		}

		crc32_calc = ~crc32_le(~0, buf, todo);
		crc32_recv = (i2c_smbus_read_byte_data(client, STREAM_WRITE_CRC_REG0) |
			      (i2c_smbus_read_byte_data(client, STREAM_WRITE_CRC_REG1) << 8) |
			      (i2c_smbus_read_byte_data(client, STREAM_WRITE_CRC_REG2) << 16) |
			      (i2c_smbus_read_byte_data(client, STREAM_WRITE_CRC_REG3) << 24));
		if (crc32_calc != crc32_recv) {
			i2c_smbus_write_byte_data(client, STREAM_CTL_REG,
						  0x10);
			continue;
		}

		for (i = 0; i < 10; i++) {
			i2c_smbus_write_byte_data(client,
						  STREAM_CTL_REG,
						  0x00);
			ret = i2c_smbus_read_byte_data(client,
						       STREAM_CTL_REG);
			if (ret != 0x00)
				continue;
			i2c_smbus_write_byte_data(client,
						  STREAM_CTL_REG,
						  0x40);
			ret = i2c_smbus_read_byte_data(client,
						       STREAM_CTL_REG);
			if (ret != 0x40)
				continue;
			break;
		}
		if (i == 10)
			return -EIO;

		done += todo;
	}

	return done;
}

static int i2c_controller_stream_release(struct inode *inodep, struct file *filep)
{
	struct stream_data *stream = filep->private_data;

	put_device(&stream->dev);
	return 0;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = i2c_controller_stream_open,
	.read = i2c_controller_stream_read,
	.write = i2c_controller_stream_write,
	.release = i2c_controller_stream_release,
};

static void i2c_controller_mux_data_release(struct device *dev) {
	struct stream_data *stream;

	stream = container_of(dev, struct stream_data, dev);

	kfree(stream);
}

static int i2c_controller_mux_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stream_data *stream;
	int ret;

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	device_initialize(&stream->dev);
	stream->dev.devt = MKDEV(i2c_controller_stream_major, 0);
	stream->dev.class = i2c_controller_stream_class;
	stream->dev.release = i2c_controller_mux_data_release;
	dev_set_name(&stream->dev, DEVICE_NAME);

	cdev_init(&stream->cdev, &fops);
	ret = cdev_device_add(&stream->cdev, &stream->dev);
	if (ret) {
		kfree(stream);
		return ret;
	}
	stream->cdev.owner = fops.owner;
	
	stream->client = client;

	i2c_set_clientdata(client, stream);
	get_device(&stream->dev);

	return 0;
};

static int i2c_controller_mux_remove(struct i2c_client *client)
{
	struct stream_data *stream = i2c_get_clientdata(client);

	cdev_device_del(&stream->cdev, &stream->dev);
	put_device(&stream->dev);

	return 0;
}

static const struct i2c_device_id i2c_controller_mux_id[] = {
	{ "i2c-master-stream", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_controller_mux_id);

static struct i2c_driver i2c_controller_mux_driver = {
	.driver = {
		.name = "i2c-master-stream",
	},
	.probe = i2c_controller_mux_probe,
	.remove = i2c_controller_mux_remove,
	.id_table = i2c_controller_mux_id,
};

static int __init i2c_controller_mux_init(void)
{
	int err;

	err = i2c_add_driver(&i2c_controller_mux_driver);
	if (err < 0)
		return err;

	i2c_controller_stream_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (i2c_controller_stream_major < 0) {
		i2c_del_driver(&i2c_controller_mux_driver);
		return i2c_controller_stream_major;
	}

	i2c_controller_stream_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(i2c_controller_stream_class)) {
		i2c_del_driver(&i2c_controller_mux_driver);
		unregister_chrdev(i2c_controller_stream_major, DEVICE_NAME);
		return PTR_ERR(i2c_controller_stream_class);
	}

	return 0;
}

static void __exit i2c_controller_mux_exit(void)
{
	class_unregister(i2c_controller_stream_class);
	unregister_chrdev(i2c_controller_stream_major, DEVICE_NAME);
	i2c_del_driver(&i2c_controller_mux_driver);
}

module_init(i2c_controller_mux_init);
module_exit(i2c_controller_mux_exit);

MODULE_AUTHOR("Kevin Paul Herbert <kph@platinaystems.com>");
MODULE_DESCRIPTION("I2C controller mode mux and stream transport");
MODULE_LICENSE("GPL v2");
