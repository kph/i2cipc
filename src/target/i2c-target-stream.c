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
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#define  DEVICE_NAME "i2c-slave-stream-0" /* fixme per unit */
#define  CLASS_NAME  "i2c-slave-stream"

#define I2C_SLAVE_STREAM_BUFSIZE 0x100 /* Must be a power of 2 */

#define STREAM_DATA_REG (0x40)
#define STREAM_CNT_REG (0x41)

struct stream_buffer {
	wait_queue_head_t wait;
	struct semaphore sem;
	spinlock_t lock;
	struct circ_buf buffer;
	char buf[I2C_SLAVE_STREAM_BUFSIZE];
};

struct stream_data {
	struct device dev;
	struct cdev cdev;
	u8 offset;
	u8 reg;
	u32 overrun;
	struct stream_buffer from_host, to_host;
};

static int i2c_slave_stream_major;
static struct class *i2c_slave_stream_class;

static int i2c_slave_stream_slave_cb(struct i2c_client *client,
				     enum i2c_slave_event event, u8 *val)
{
	struct stream_data *stream = i2c_get_clientdata(client);
	unsigned long head, tail, cnt;
	u8 ch;
	
	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		spin_lock(&stream->from_host.lock);
		stream->offset = 0;
		spin_unlock(&stream->from_host.lock);
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		spin_lock(&stream->from_host.lock);
		if (stream->offset == 0) {	/* Register is a single byte */
			stream->reg = *val;
			if (stream->reg != STREAM_DATA_REG &&
			    stream->reg != STREAM_CNT_REG) {
				spin_unlock(&stream->from_host.lock);
				return ENOENT;
			}
		} else {
			if (stream->reg != STREAM_DATA_REG) {
				spin_unlock(&stream->from_host.lock);
				return ENOENT;
			}
			head = stream->from_host.buffer.head;
			tail = READ_ONCE(stream->from_host.buffer.tail);
			if (CIRC_SPACE(head, tail, I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
				stream->from_host.buffer.buf[head] = *val;
				smp_store_release(&stream->from_host.buffer.head,
						  (head + 1) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
			} else {
				stream->overrun++;
			}
		}
		stream->offset++;
		spin_unlock(&stream->from_host.lock);
		break;

	case I2C_SLAVE_READ_PROCESSED:
		stream->offset++;
		if (stream->reg != STREAM_DATA_REG) {
			*val = 0;
			break;
		}
		/* The previous byte made it to the bus, get next one */
		spin_lock(&stream->to_host.lock);
		head = smp_load_acquire(&stream->to_host.buffer.head);
		tail = stream->to_host.buffer.tail;

		if (CIRC_CNT(head, tail, I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
			smp_store_release(&stream->to_host.buffer.tail,
					  (tail + 1) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
		}
		*val = stream->to_host.buffer.buf[tail];	
		spin_unlock(&stream->to_host.lock);
		break;

	case I2C_SLAVE_READ_REQUESTED:
		spin_lock(&stream->to_host.lock);
		head = smp_load_acquire(&stream->to_host.buffer.head);
		tail = stream->to_host.buffer.tail;
		cnt = CIRC_CNT(head, tail, I2C_SLAVE_STREAM_BUFSIZE);
		if (stream->reg == STREAM_CNT_REG) {
			if (cnt > 255) {
				cnt = 255;
			}
			if (stream->offset > 0) {
				cnt = 0;
			}
			*val = cnt;
		} else {
			ch = 0;
			if (cnt >= 1) {
				ch = stream->to_host.buffer.buf[tail];
			}			
			*val = ch;
		}
		spin_unlock(&stream->to_host.lock);

		/*
		 * Do not increment buffer_idx here, because we don't know if
		 * this byte will be actually used. Read Linux I2C slave docs
		 * for details.
		 */
		break;

	case I2C_SLAVE_STOP:
		stream->offset = 0;
		spin_lock(&stream->from_host.lock);
		if (CIRC_CNT(stream->from_host.buffer.head,
			     stream->from_host.buffer.tail,
			     I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
			wake_up(&stream->from_host.wait);
		}
		spin_unlock(&stream->from_host.lock);

		spin_lock(&stream->to_host.lock);
		if (CIRC_CNT(stream->to_host.buffer.head,
			     stream->to_host.buffer.tail,
			     I2C_SLAVE_STREAM_BUFSIZE) == 0) {
			wake_up(&stream->to_host.wait);
		}
		spin_unlock(&stream->to_host.lock);
		break;

	default:
		break;
	}

	return 0;
}

static int i2c_slave_stream_open(struct inode *inode, struct file *filep)
{
	struct stream_data *stream = container_of(inode->i_cdev,
						  struct stream_data, cdev);
	filep->private_data = stream;
	
	return 0;
}

static ssize_t i2c_slave_stream_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct stream_data *stream = filep->private_data;
	unsigned long head;
	unsigned long tail;
	int cnt;
	size_t done = 0;
	
	while (done < len) {
		if (down_interruptible(&stream->from_host.sem))
			return -ERESTARTSYS;
		    
		spin_lock(&stream->from_host.lock);

		head = smp_load_acquire(&stream->from_host.buffer.head);
		tail = stream->from_host.buffer.tail;

		cnt = CIRC_CNT_TO_END(head, tail, I2C_SLAVE_STREAM_BUFSIZE);
		spin_unlock(&stream->from_host.lock);
		
		if (cnt == 0) {
			up(&stream->from_host.sem);
			if (done != 0) {
				return done;
			}
			if (filep->f_flags & O_NONBLOCK)
				return -EAGAIN;
			if (wait_event_interruptible(stream->from_host.wait,
						     (smp_load_acquire(&stream->from_host.buffer.head) != 
						      stream->from_host.buffer.tail)))
				return -ERESTARTSYS;
		} else {
			size_t todo = min(len - done, (size_t)cnt);

			if (copy_to_user(&buffer[done], &stream->from_host.buffer.buf[tail],
					 todo)) {
				up(&stream->from_host.sem);
				return -EFAULT;
			}
			done += todo;
			smp_store_release(&stream->from_host.buffer.tail,
					  (tail + todo) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
			up(&stream->from_host.sem);
		}
	}
	return done;
}

static ssize_t i2c_slave_stream_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct stream_data *stream = filep->private_data;
	unsigned long head, tail;
	int cnt;
	size_t done = 0;
	
	while (done < len) {
		if (down_interruptible(&stream->to_host.sem))
			return -ERESTARTSYS;

		spin_lock(&stream->to_host.lock);
		
		head = stream->to_host.buffer.head;
		tail = READ_ONCE(stream->to_host.buffer.tail);

		cnt = CIRC_SPACE_TO_END(head, tail, I2C_SLAVE_STREAM_BUFSIZE);
		spin_unlock(&stream->to_host.lock);

		if (cnt == 0) {
			up(&stream->to_host.sem);
			if (filep->f_flags & O_NONBLOCK) {
				if (done != 0) {
					return done;
				}
				return -EAGAIN;
			}
			if (wait_event_interruptible(stream->to_host.wait,
						     (smp_load_acquire(&stream->to_host.buffer.head) != 
						      stream->to_host.buffer.tail)))
				return -ERESTARTSYS;
		} else {
			size_t todo = min(len - done, (size_t)cnt);

			if (copy_from_user(&stream->to_host.buffer.buf[head],
					   &buffer[done],
					   todo)) {
				up(&stream->to_host.sem);
				return -EFAULT;
			}
			done += todo;
			smp_store_release(&stream->to_host.buffer.head,
					  (head + todo) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
			up(&stream->to_host.sem);
		}
			
	}

	return len;
}

static int i2c_slave_stream_release(struct inode *inodep, struct file *filep)
{
	return 0;
}

static struct file_operations fops =
{
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = i2c_slave_stream_open,
	.read = i2c_slave_stream_read,
	.write = i2c_slave_stream_write,
	.release = i2c_slave_stream_release,
};

static void i2c_slave_stream_data_release(struct device *dev) {
	struct stream_data *stream;

	stream = container_of(dev, struct stream_data, dev);

	kfree(stream);
}

static int i2c_slave_stream_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stream_data *stream;
	int ret;

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	device_initialize(&stream->dev);
	stream->dev.devt = MKDEV(i2c_slave_stream_major, 0);
	stream->dev.class = i2c_slave_stream_class;
	stream->dev.release = i2c_slave_stream_data_release;
	dev_set_name(&stream->dev, DEVICE_NAME);

	cdev_init(&stream->cdev, &fops);
	ret = cdev_device_add(&stream->cdev, &stream->dev);
	if (ret) {
		goto err_mem;
	}
	stream->cdev.owner = fops.owner;
	
	init_waitqueue_head(&stream->from_host.wait);
	init_waitqueue_head(&stream->to_host.wait);
	
	sema_init(&stream->from_host.sem, 1);
	sema_init(&stream->to_host.sem, 1);

	spin_lock_init(&stream->from_host.lock);
	spin_lock_init(&stream->to_host.lock);

	stream->from_host.buffer.buf = stream->from_host.buf;
	stream->to_host.buffer.buf = stream->to_host.buf;

	i2c_set_clientdata(client, stream);

	ret = i2c_slave_register(client, i2c_slave_stream_slave_cb);
	if (ret) {
	err_mem:
		put_device(&stream->dev);
		return ret;
	}

	return 0;
};

static int i2c_slave_stream_remove(struct i2c_client *client)
{
	struct stream_data *stream = i2c_get_clientdata(client);

	cdev_device_del(&stream->cdev, &stream->dev);
	put_device(&stream->dev);
	i2c_slave_unregister(client);
	
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
