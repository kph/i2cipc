// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C slave mode stream driver
 *
 * Copyright (C) 2020 by Kevin Paul Herbert, Platina Systems, Inc.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/circ_buf.h>
#include <linux/semaphore.h>

#define  DEVICE_NAME "i2c-slave-stream-0" /* fixme per unit */
#define  CLASS_NAME  "i2c-slave-stream"

#define I2C_SLAVE_STREAM_BUFSIZE 0x100 /* Must be a power of 2 */

struct stream_data {
	struct device *device;
	u8 offset;
	u8 reg;
	wait_queue_head_t wait_from_host, wait_to_host;
	struct semaphore sem_read, sem_write;
	spinlock_t write_lock;
	struct circ_buf write_buffer;
	u32 overrun;
	spinlock_t read_lock;
	struct circ_buf read_buffer;
	char readbuf[I2C_SLAVE_STREAM_BUFSIZE];
	char writebuf[I2C_SLAVE_STREAM_BUFSIZE];
};

static int i2c_slave_stream_major;
static struct class *i2c_slave_stream_class;

static int i2c_slave_stream_slave_cb(struct i2c_client *client,
				     enum i2c_slave_event event, u8 *val)
{
	struct stream_data *stream = i2c_get_clientdata(client);
	unsigned long head, tail;
	u8 ch;
	
	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		spin_lock(&stream->write_lock);
		stream->offset = 0;
		spin_unlock(&stream->write_lock);
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		spin_lock(&stream->write_lock);
		if (stream->offset == 0) {	/* Register is a single byte */
			stream->reg = *val;
		} else {
			head = stream->write_buffer.head;
			tail = READ_ONCE(stream->write_buffer.tail);
			if (CIRC_SPACE(head, tail, I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
				stream->write_buffer.buf[head] = *val;

				smp_store_release(&stream->write_buffer.head,
						  (head + 1) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
			} else {
				stream->overrun++;
			}
		}
		stream->offset++;
		spin_unlock(&stream->write_lock);
		break;

	case I2C_SLAVE_READ_PROCESSED:
		/* The previous byte made it to the bus, get next one */
		spin_lock(&stream->read_lock);
		head = smp_load_acquire(&stream->read_buffer.head);
		tail = stream->read_buffer.tail;

		if (CIRC_CNT(head, tail, I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
			smp_store_release(&stream->read_buffer.tail,
					  (tail + 1) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
		}
		*val = stream->read_buffer.buf[tail];	
		spin_unlock(&stream->read_lock);
		break;

	case I2C_SLAVE_READ_REQUESTED:

		spin_lock(&stream->read_lock);
		head = smp_load_acquire(&stream->read_buffer.head);
		tail = stream->read_buffer.tail;
		ch = 0;
		
		if (CIRC_CNT(stream->read_buffer.head, tail,
			     I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
			ch = stream->read_buffer.buf[tail];
		}			
		*val = ch;
		spin_unlock(&stream->read_lock);

		/*
		 * Do not increment buffer_idx here, because we don't know if
		 * this byte will be actually used. Read Linux I2C slave docs
		 * for details.
		 */
		break;

	case I2C_SLAVE_STOP:
		spin_lock(&stream->write_lock);
		if (CIRC_CNT(stream->write_buffer.head,
			     stream->write_buffer.tail,
			     I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
			wake_up(&stream->wait_from_host);
		}
		spin_unlock(&stream->write_lock);
		break;
	default:
		break;
	}

	return 0;
}

static int i2c_slave_stream_open(struct inode *inodep, struct file *filep)
{
	return 0;
}

static ssize_t i2c_slave_stream_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	return 0;
}

static ssize_t i2c_slave_stream_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	return len;
}

static int i2c_slave_stream_release(struct inode *inodep, struct file *filep)
{
	return 0;
}

static int i2c_slave_stream_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stream_data *stream;
	int ret;

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	stream->device = device_create(i2c_slave_stream_class, NULL,
				       MKDEV(i2c_slave_stream_major, 0),
				       NULL, DEVICE_NAME);
	if (IS_ERR(stream->device)) {
		ret = PTR_ERR(stream->device);
		goto err_mem;
	}

	spin_lock_init(&stream->write_lock);
	spin_lock_init(&stream->read_lock);

	stream->write_buffer.buf = stream->writebuf;
	stream->read_buffer.buf = stream->readbuf;

	i2c_set_clientdata(client, stream);

	ret = i2c_slave_register(client, i2c_slave_stream_slave_cb);
	if (ret) {
	err_mem:
		kfree(stream);
		return ret;
	}

	return 0;
};

static int i2c_slave_stream_remove(struct i2c_client *client)
{
	struct stream_data *stream = i2c_get_clientdata(client);

	i2c_slave_unregister(client);

	if (stream->device) {
		device_destroy(i2c_slave_stream_class,
			       MKDEV(i2c_slave_stream_major, 0));
	}

	return 0;
}

static const struct i2c_device_id i2c_slave_stream_id[] = {
	{ "slave-stream", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_slave_stream_id);

static struct i2c_driver i2c_slave_stream_driver = {
	.driver = {
		.name = "i2c-slave-stream",
	},
	.probe = i2c_slave_stream_probe,
	.remove = i2c_slave_stream_remove,
	.id_table = i2c_slave_stream_id,
};
module_i2c_driver(i2c_slave_stream_driver);

static struct file_operations fops =
{
	.open = i2c_slave_stream_open,
	.read = i2c_slave_stream_read,
	.write = i2c_slave_stream_write,
	.release = i2c_slave_stream_release,
};

static int __init i2c_slave_stream_init(void)
{
	i2c_slave_stream_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (i2c_slave_stream_major < 0) {
		return i2c_slave_stream_major;
	}

	i2c_slave_stream_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(i2c_slave_stream_class)) {
		unregister_chrdev(i2c_slave_stream_major, DEVICE_NAME);
		return PTR_ERR(i2c_slave_stream_class);
	}

	return 0;
}

static void __exit i2c_slave_stream_exit(void)
{
	class_unregister(i2c_slave_stream_class);
	class_destroy(i2c_slave_stream_class);
	unregister_chrdev(i2c_slave_stream_major, DEVICE_NAME);
}

module_init(i2c_slave_stream_init);
module_exit(i2c_slave_stream_exit);

MODULE_AUTHOR("Kevin Paul Herbert <kph@platinaystems.com>");
MODULE_DESCRIPTION("I2C slave mode stream transport");
MODULE_LICENSE("GPL v2");
