// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C controller mode mux and stream driver
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

#define REGS_PER_RESPONDER (16)
#define MAX_RESPONDER_REGS (16)
#define RESPONDER_REG(t) ((t) & (MAX_RESPONDER_REGS-1))

#define  DEVICE_NAME "i2c-master-stream-0" /* fixme per unit */
#define  CLASS_NAME  "i2c-master-stream"

#define STREAM_DATA_REG (0x0)
#define STREAM_CNT_REG (0x1)
#define STREAM_READ_CRC_REG0 (0x2)
#define STREAM_READ_CRC_REG1 (0x3)
#define STREAM_READ_CRC_REG2 (0x4)
#define STREAM_READ_CRC_REG3 (0x5)
#define STREAM_WRITE_CRC_REG0 (0x6)
#define STREAM_WRITE_CRC_REG1 (0x7)
#define STREAM_WRITE_CRC_REG2 (0x8)
#define STREAM_WRITE_CRC_REG3 (0x9)
#define STREAM_CTL_REG (0xa)

struct stream_data {
	struct device dev;
	struct cdev cdev;
	u32 crc;
	struct i2c_client *client;
	u8 base_port;
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
		cnt = i2c_smbus_read_byte_data(client,
					       stream->base_port + STREAM_CNT_REG);
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
							       stream->base_port +
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
			crc32_recv = (i2c_smbus_read_byte_data(client,
							       stream->base_port +
							       STREAM_READ_CRC_REG0) |
				      (i2c_smbus_read_byte_data(client,
								stream->base_port +
								STREAM_READ_CRC_REG1)
				       << 8) |
				      (i2c_smbus_read_byte_data(client,
								stream->base_port +
								STREAM_READ_CRC_REG2)
				       << 16) |
				      (i2c_smbus_read_byte_data(client,
								stream->base_port +
								STREAM_READ_CRC_REG3)
				       << 24));
			if (crc32_calc != crc32_recv) {
				i2c_smbus_write_byte_data(client, stream->base_port +
							  STREAM_CTL_REG, 0x20);
				continue;
			}
			
			for (i = 0; i < 10; i++) {
				i2c_smbus_write_byte_data(client,
							  stream->base_port +
							  STREAM_CTL_REG, 0x00);
				val = i2c_smbus_read_byte_data(client,
							       stream->base_port +
							       STREAM_CTL_REG);
				if (val != 0x00)
					continue;
				i2c_smbus_write_byte_data(client,
							  stream->base_port +
							  STREAM_CTL_REG, 0x80);
				val = i2c_smbus_read_byte_data(client,
							       stream->base_port +
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
							stream->base_port +
							STREAM_DATA_REG, buf[i]);
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
		crc32_recv = (i2c_smbus_read_byte_data(client, stream->base_port +
						       STREAM_WRITE_CRC_REG0) |
			      (i2c_smbus_read_byte_data(client, stream->base_port +
							STREAM_WRITE_CRC_REG1) << 8) |
			      (i2c_smbus_read_byte_data(client, stream->base_port +
							STREAM_WRITE_CRC_REG2) << 16) |
			      (i2c_smbus_read_byte_data(client, stream->base_port +
							STREAM_WRITE_CRC_REG3) << 24));
		if (crc32_calc != crc32_recv) {
			i2c_smbus_write_byte_data(client, stream->base_port +
						  STREAM_CTL_REG, 0x10);
			continue;
		}

		for (i = 0; i < 10; i++) {
			i2c_smbus_write_byte_data(client, stream->base_port +
						  STREAM_CTL_REG, 0x00);
			ret = i2c_smbus_read_byte_data(client, stream->base_port +
						       STREAM_CTL_REG);
			if (ret != 0x00)
				continue;
			i2c_smbus_write_byte_data(client, stream->base_port +
						  STREAM_CTL_REG, 0x40);
			ret = i2c_smbus_read_byte_data(client, stream->base_port +
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
	struct device *dev = &client->dev;
	struct stream_data *stream;
	int ret, nprops_port, nprops_pname;
	u32 stream_base_ports[REGS_PER_RESPONDER];
	const char *stream_base_port_names[REGS_PER_RESPONDER];

	nprops_port = device_property_read_u32_array(dev, "stream-ports",
						    NULL,
						    REGS_PER_RESPONDER);
	if (nprops_port < 0)
		return nprops_port;

	if (nprops_port == 0 || nprops_port > REGS_PER_RESPONDER)
		return -EINVAL;

	nprops_pname = device_property_read_string_array(dev, "stream-port-names",
							 NULL, REGS_PER_RESPONDER);
	if (nprops_pname < 0)
		return nprops_pname;

	if (nprops_port != nprops_pname)
		return -EINVAL;

	ret = device_property_read_u32_array(dev, "stream-ports",
					     stream_base_ports,
					     nprops_port);
	if (ret == 0) {
		ret = device_property_read_string_array(dev,
							"stream-port-names",
							stream_base_port_names,
							REGS_PER_RESPONDER);
		if (ret < 0)
			return ret;
	} else {
		return ret;
	}
	
	printk("nprops_port=%d nprops_pname=%d\n", nprops_port, nprops_pname);

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	stream->base_port = 0x40;
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
	{ "i2c-ipc-controller", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_controller_mux_id);

static const struct of_device_id controller_of_match[] = {
	{ .compatible = "linux,i2c-ipc-controller" },
	{ /* END OF LIST */ },

};
MODULE_DEVICE_TABLE(of, controller_of_match);

static struct i2c_driver i2c_controller_mux_driver = {
	.driver = {
		.name = "i2c-ipc-controller",
		.of_match_table = controller_of_match,
	},
	.probe = i2c_controller_mux_probe,
	.remove = i2c_controller_mux_remove,
	.id_table = i2c_controller_mux_id,
};

static int __init i2c_controller_mux_init(void)
{
	int err;

	i2c_controller_stream_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(i2c_controller_stream_class))
		return PTR_ERR(i2c_controller_stream_class);
	
	i2c_controller_stream_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (i2c_controller_stream_major < 0) {
		class_destroy(i2c_controller_stream_class);
		return i2c_controller_stream_major;
	}

	err = i2c_add_driver(&i2c_controller_mux_driver);
	if (err < 0) {
		unregister_chrdev(i2c_controller_stream_major, DEVICE_NAME);
		class_destroy(i2c_controller_stream_class);
		return err;
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
