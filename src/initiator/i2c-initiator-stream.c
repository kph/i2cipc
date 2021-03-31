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

#define  DEVICE_NAME "i2c-master-stream-0" /* fixme per unit */
#define  CLASS_NAME  "i2c-master-stream"

#define STREAM_DATA_REG (0x40)
#define STREAM_CNT_REG (0x41)

struct stream_data {
	struct device dev;
	struct cdev cdev;
	struct i2c_client *client;
};

static int i2c_master_stream_major;
static struct class *i2c_master_stream_class;

static int i2c_master_stream_open(struct inode *inode, struct file *filep)
{
	struct stream_data *stream = container_of(inode->i_cdev,
						  struct stream_data, cdev);
	filep->private_data = stream;
	
	return 0;
}

static ssize_t i2c_master_stream_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
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
			u8 buf[32];
		
			todo = min(todo, (size_t)32);
//			printk("%s: cnt=%d todo=%ld\n", __func__, cnt, todo);

			for (i = 0; i < todo; i++) {
				buf[i] = i2c_smbus_read_byte_data(client,
								  STREAM_DATA_REG);
				if (buf[i] < 0) {
					if (error_cnt++ > 10) {
						return buf[i];
					}
					msleep_interruptible(100);
					i--;
				} else {
					error_cnt = 0;
				}
			}

			if (copy_to_user(&buffer[done], buf, todo)) {
				return -EFAULT;
			}
			done += todo;
		}
	}
	return done;
}

static ssize_t i2c_master_stream_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
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

		done += todo;
	}

	return done;
}

static int i2c_master_stream_release(struct inode *inodep, struct file *filep)
{
	return 0;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = i2c_master_stream_open,
	.read = i2c_master_stream_read,
	.write = i2c_master_stream_write,
	.release = i2c_master_stream_release,
};

static void i2c_master_stream_data_release(struct device *dev) {
	struct stream_data *stream;

	stream = container_of(dev, struct stream_data, dev);

	kfree(stream);
}

static int i2c_master_stream_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stream_data *stream;
	int ret;

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	device_initialize(&stream->dev);
	stream->dev.devt = MKDEV(i2c_master_stream_major, 0);
	stream->dev.class = i2c_master_stream_class;
	stream->dev.release = i2c_master_stream_data_release;
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

	return 0;
};

static int i2c_master_stream_remove(struct i2c_client *client)
{
	struct stream_data *stream = i2c_get_clientdata(client);

	cdev_device_del(&stream->cdev, &stream->dev);
	
	return 0;
}

static const struct i2c_device_id i2c_master_stream_id[] = {
	{ "i2c-master-stream", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_master_stream_id);

static struct i2c_driver i2c_master_stream_driver = {
	.driver = {
		.name = "i2c-master-stream",
	},
	.probe = i2c_master_stream_probe,
	.remove = i2c_master_stream_remove,
	.id_table = i2c_master_stream_id,
};

static int __init i2c_master_stream_init(void)
{
	int err;

	err = i2c_add_driver(&i2c_master_stream_driver);
	if (err < 0)
		return err;

	i2c_master_stream_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (i2c_master_stream_major < 0) {
		i2c_del_driver(&i2c_master_stream_driver);
		return i2c_master_stream_major;
	}

	i2c_master_stream_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(i2c_master_stream_class)) {
		i2c_del_driver(&i2c_master_stream_driver);
		unregister_chrdev(i2c_master_stream_major, DEVICE_NAME);
		return PTR_ERR(i2c_master_stream_class);
	}

	return 0;
}

static void __exit i2c_master_stream_exit(void)
{
	class_unregister(i2c_master_stream_class);
	class_destroy(i2c_master_stream_class);
	unregister_chrdev(i2c_master_stream_major, DEVICE_NAME);
}

module_init(i2c_master_stream_init);
module_exit(i2c_master_stream_exit);

MODULE_AUTHOR("Kevin Paul Herbert <kph@platinaystems.com>");
MODULE_DESCRIPTION("I2C master mode stream transport");
MODULE_LICENSE("GPL v2");