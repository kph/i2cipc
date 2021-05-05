// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C slave mode stream driver
 *
 * Copyright (C) 2020-2021 by Kevin Paul Herbert, Platina Systems, Inc.
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
#include <linux/crc32.h>

#define  DEVICE_NAME "i2c-slave-stream-0" /* fixme per unit */
#define  CLASS_NAME  "i2c-slave-stream"

#define I2C_SLAVE_STREAM_BUFSIZE 0x100 /* Must be a power of 2 */

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

struct stream_buffer {
	wait_queue_head_t wait;
	struct semaphore sem;
	spinlock_t lock;
	struct circ_buf buffer;
	int frame;
	u32 crc32;
	char buf[I2C_SLAVE_STREAM_BUFSIZE];
};

struct stream_end {	
	u8 ctl_write;
	u32 overrun;
	struct stream_buffer from_host, to_host;
};

struct stream_data {
	struct i2c_client *client;
	struct device dev;
	struct cdev cdev;
	u8 offset;
	u8 reg;
	struct stream_end sx;
};

#ifdef NOTYET
struct target_stream_ops tops =
{
	u8 (*read_cnt_reg)(struct stream_data *, u8); /* Read count register */
	u8 (*read_data_reg)(struct stream_data *, u8); /* Read data register */
	u8 (*read_read_crc0)(struct stream_data *, u8); /* Read 'read CRC' byte 0 */
	u8 (*read_read_crc1)(struct stream_data *, u8); /* Read 'read CRC' byte 1 */
	u8 (*read_read_crc2)(struct stream_data *, u8); /* Read 'read CRC' byte 2 */
	u8 (*read_read_crc3)(struct stream_data *, u8); /* Read 'read CRC' byte 3 */
	u8 (*read_write_crc0)(struct stream_data *, u8); /* Read 'write CRC' byte 0 */
	u8 (*read_write_crc1)(struct stream_data *, u8); /* Read 'write CRC' byte 1 */
	u8 (*read_write_crc2)(struct stream_data *, u8); /* Read 'write CRC' byte 2 */
	u8 (*read_write_crc3)(struct stream_data *, u8); /* Read 'write CRC' byte 3 */
	u8 (*read_write_ctl)(struct stream_data *, u8); /* Read write control */
	
	
};
#endif

static int i2c_slave_stream_major;
static struct class *i2c_slave_stream_class;

static void set_data_reg(struct stream_end *sx, u8 *val) {
	unsigned long head, tail;

	spin_lock(&sx->from_host.lock);
	head = sx->from_host.buffer.head;
	tail = READ_ONCE(sx->from_host.buffer.tail);
	if (CIRC_SPACE(head, tail, I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
		sx->from_host.buffer.buf[head] = *val;
		sx->from_host.crc32 = crc32_le(
			sx->from_host.crc32, val, 1);
		sx->from_host.buffer.head =
			(head + 1) & (I2C_SLAVE_STREAM_BUFSIZE - 1);
	} else {
		sx->overrun++;
	}
	spin_unlock(&sx->from_host.lock);
}

static void set_ctl_reg(struct stream_end *sx, u8 *val) {
	sx->ctl_write = *val;
	switch (*val) {
	case 0x80:
		spin_lock(&sx->to_host.lock);
		sx->to_host.crc32 = ~0;
		smp_store_release(&sx->to_host.frame,
				  sx->to_host.buffer.tail);
		if (CIRC_CNT(sx->to_host.buffer.head,
			     sx->to_host.frame,
			     I2C_SLAVE_STREAM_BUFSIZE) == 0) {
			wake_up(&sx->to_host.wait);
		}
		spin_unlock(&sx->to_host.lock);
		break;

	case 0x20:
		spin_lock(&sx->to_host.lock);
		sx->to_host.crc32 = ~0;
		sx->to_host.buffer.tail =
			smp_load_acquire(&sx->to_host.frame);
		spin_unlock(&sx->to_host.lock);
		break;

	case 0x40:
		spin_lock(&sx->from_host.lock);
		sx->from_host.crc32 = ~0;
		smp_store_release(&sx->from_host.frame,
				  sx->from_host.buffer.head);
		if (CIRC_CNT(sx->from_host.frame,
			     sx->from_host.buffer.tail,
			     I2C_SLAVE_STREAM_BUFSIZE) > 0) {
			wake_up(&sx->from_host.wait);
		}
		spin_unlock(&sx->from_host.lock);
		break;
	
	case 0x10:
		spin_lock(&sx->from_host.lock);
		sx->from_host.crc32 = ~0;
		sx->from_host.buffer.head =
			smp_load_acquire(&sx->from_host.frame);
		spin_unlock(&sx->from_host.lock);
		break;
		
	}
}

/*
 * get_reg32 - Read 8 bits of a 32 bit register
 *
 * This is a helper routine to return a byte of a 32 bit
 * register, little endian.
 */

static u8 get_reg32(u32 reg, int offset)
{
	return (reg >> (offset << 3)) & 0xff;
}

static u8 get_cnt_reg(struct stream_data *stream, struct stream_end *sx) {
	unsigned long head, tail, cnt;
	
	spin_lock(&sx->to_host.lock);
	head = smp_load_acquire(&sx->to_host.buffer.head);
	tail = sx->to_host.buffer.tail;
	cnt = CIRC_CNT(head, tail, I2C_SLAVE_STREAM_BUFSIZE);
	if (cnt > 255) {
		cnt = 255;
	}
	if (stream->offset > 0) {
		cnt = 0;
	}
	spin_unlock(&sx->to_host.lock);
	return cnt;
}

static u8 get_data_reg(struct stream_end *sx) {
	unsigned long head, tail, cnt;
	u8 ch;

	spin_lock(&sx->to_host.lock);
	head = smp_load_acquire(&sx->to_host.buffer.head);
	tail = sx->to_host.buffer.tail;
	cnt = CIRC_CNT(head, tail, I2C_SLAVE_STREAM_BUFSIZE);

	ch = 0;
	if (cnt >= 1) {
		ch = sx->to_host.buffer.buf[tail];
	}
	spin_unlock(&sx->to_host.lock);
	return ch;
}

static u8 get_crc_reg(struct stream_data *stream, struct stream_end *sx, u8 base_reg) {
	return get_reg32(~sx->from_host.crc32, stream->reg - base_reg);
}

static u8 get_ctl_reg(struct stream_end *sx) {
	return sx->ctl_write;
}

static int handle_write(struct stream_data *stream, struct stream_end *sx, u8 *val) {
	switch (stream->reg) {
	case STREAM_DATA_REG:
		set_data_reg(sx, val);
		break;
		
	case STREAM_CTL_REG:
		set_ctl_reg(sx, val);
		break;
			
	default:
		return -ENOENT;
	}
	return 0;
}

static void read_processed(struct stream_end *sx, u8 *val)
{
	unsigned long head, tail;
	u8 ch;

	/* The previous byte made it to the bus, get next one */
	spin_lock(&sx->to_host.lock);
	head = smp_load_acquire(&sx->to_host.buffer.head);
	tail = sx->to_host.buffer.tail;

	if (CIRC_CNT(head, tail, I2C_SLAVE_STREAM_BUFSIZE) >= 1) {
		ch = sx->to_host.buffer.buf[tail];
		sx->to_host.crc32 = crc32_le(sx->to_host.crc32,
						    &ch, 1);
		sx->to_host.buffer.tail =
			(tail + 1) & (I2C_SLAVE_STREAM_BUFSIZE - 1);
	}
	*val = sx->to_host.buffer.buf[tail];	
	spin_unlock(&sx->to_host.lock);
}

static void read_requested(struct stream_data *stream, struct stream_end *sx,
			   u8 *val)
{
	switch (stream->reg) {
	case STREAM_CNT_REG:
		*val = get_cnt_reg(stream, sx);
		break;

	case STREAM_DATA_REG:
		*val = get_data_reg(sx);
		break;

	case STREAM_READ_CRC_REG0:
	case STREAM_READ_CRC_REG1:
	case STREAM_READ_CRC_REG2:
	case STREAM_READ_CRC_REG3:
		*val = get_crc_reg(stream, sx, STREAM_READ_CRC_REG0);
		break;

	case STREAM_WRITE_CRC_REG0:
	case STREAM_WRITE_CRC_REG1:
	case STREAM_WRITE_CRC_REG2:
	case STREAM_WRITE_CRC_REG3:
		*val = get_crc_reg(stream, sx, STREAM_WRITE_CRC_REG0);
		break;

	case STREAM_CTL_REG:
		*val = get_ctl_reg(sx);
		break;
			
	default:
		*val = 0;
		break;
	}
}

static int i2c_slave_stream_cb(struct i2c_client *client,
			       enum i2c_slave_event event, u8 *val)
{
	struct stream_data *stream = i2c_get_clientdata(client);
	struct stream_end *sx = &stream->sx;
	int ret = 0;
	
	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		stream->offset = 0;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (stream->offset == 0) {	/* Register is a single byte */
			stream->reg = *val;
		} else {
			ret =  handle_write(stream, sx, val);
		}
		stream->offset++;
		break;

	case I2C_SLAVE_READ_PROCESSED:
		stream->offset++;
		
		if (stream->reg != STREAM_DATA_REG) {
			*val = 0;
			break;
		}
		read_processed(sx, val);
		break;

	case I2C_SLAVE_READ_REQUESTED:
		read_requested(stream, sx, val);

		/*
		 * Do not increment offset here, because we don't know if
		 * this byte will be actually used. Read Linux I2C slave docs
		 * for details.
		 */
		break;

	case I2C_SLAVE_STOP:
		stream->offset = 0;
		break;

	default:
		break;
	}

	return ret;
}

static int i2c_slave_stream_open(struct inode *inode, struct file *filep)
{
	struct stream_data *stream = container_of(inode->i_cdev,
						  struct stream_data, cdev);
	get_device(&stream->dev);
	filep->private_data = stream;

	return 0;
}

static ssize_t i2c_slave_stream_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct stream_data *stream = filep->private_data;
	struct stream_end *sx = &stream->sx;
	unsigned long head, tail;
	int cnt;
	size_t done = 0;
	
	while (done < len) {
		if (down_interruptible(&sx->from_host.sem))
			return -ERESTARTSYS;
		    
		spin_lock_irq(&sx->from_host.lock);

		head = smp_load_acquire(&sx->from_host.frame);
		tail = sx->from_host.buffer.tail;

		cnt = CIRC_CNT_TO_END(head, tail, I2C_SLAVE_STREAM_BUFSIZE);
		spin_unlock_irq(&sx->from_host.lock);
		
		if (cnt == 0) {
			up(&sx->from_host.sem);
			if (done != 0) {
				return done;
			}
			if (filep->f_flags & O_NONBLOCK)
				return -EAGAIN;
			if (wait_event_interruptible(sx->from_host.wait,
						     (CIRC_CNT(
							     smp_load_acquire(&sx->from_host.frame),
							     sx->from_host.buffer.tail,
							     I2C_SLAVE_STREAM_BUFSIZE) > 0)))
				return -ERESTARTSYS;
		} else {
			size_t todo = min(len - done, (size_t)cnt);

			if (copy_to_user(&buffer[done], &sx->from_host.buffer.buf[tail],
					 todo)) {
				up(&sx->from_host.sem);
				return -EFAULT;
			}
			done += todo;
			smp_store_release(&sx->from_host.buffer.tail,
					  (tail + todo) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
			up(&sx->from_host.sem);
		}
	}
	return done;
}

static ssize_t i2c_slave_stream_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct stream_data *stream = filep->private_data;
	struct stream_end *sx = &stream->sx;
	unsigned long head, tail;
	int cnt;
	size_t done = 0;
	
	while (done < len) {
		if (down_interruptible(&sx->to_host.sem))
			return -ERESTARTSYS;

		spin_lock_irq(&sx->to_host.lock);
		
		head = sx->to_host.buffer.head;
		tail = READ_ONCE(sx->to_host.frame);

		cnt = CIRC_SPACE_TO_END(head, tail, I2C_SLAVE_STREAM_BUFSIZE);
		spin_unlock_irq(&sx->to_host.lock);

		if (cnt == 0) {
			up(&sx->to_host.sem);
			if (filep->f_flags & O_NONBLOCK) {
				if (done != 0) {
					return done;
				}
				return -EAGAIN;
			}
			if (wait_event_interruptible(sx->to_host.wait,
						     (CIRC_SPACE(
							     smp_load_acquire(&sx->to_host.buffer.head),
							     sx->to_host.frame,
							     I2C_SLAVE_STREAM_BUFSIZE) > 0)))
				return -ERESTARTSYS;
		} else {
			size_t todo = min(len - done, (size_t)cnt);

			if (copy_from_user(&sx->to_host.buffer.buf[head],
					   &buffer[done],
					   todo)) {
				up(&sx->to_host.sem);
				return -EFAULT;
			}
			done += todo;
			smp_store_release(&sx->to_host.buffer.head,
					  (head + todo) & (I2C_SLAVE_STREAM_BUFSIZE - 1));
			up(&sx->to_host.sem);
		}
			
	}

	return len;
}

static int i2c_slave_stream_release(struct inode *inodep, struct file *filep)
{
	struct stream_data *stream = filep->private_data;

	put_device(&stream->dev);
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

static void i2c_slave_stream_data_release(struct device *dev)
{
	struct stream_data *stream = container_of(dev, struct stream_data, dev);

	printk(KERN_EMERG "in %s\n", __func__);

	kfree(stream);
}

static int i2c_slave_stream_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stream_data *stream;
	struct stream_end *sx;
	int ret;

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	stream->dev.devt = MKDEV(i2c_slave_stream_major, 0);
	stream->dev.class = i2c_slave_stream_class;
	stream->dev.release = i2c_slave_stream_data_release;
	stream->client = client;
	stream->dev.parent = &client->dev;
	dev_set_name(&stream->dev, DEVICE_NAME);
	ret = device_register(&stream->dev);
	if (ret) {
		return ret;
	}
	cdev_init(&stream->cdev, &fops);
	//ret = cdev_device_add(&stream->cdev, &stream->dev);
	ret = cdev_add(&stream->cdev, stream->dev.devt, 1);
	if (ret) {
		put_device(&stream->dev);
		//kfree(stream);
		return ret;
	}
	stream->cdev.owner = fops.owner;
	
	init_waitqueue_head(&sx->from_host.wait);
	init_waitqueue_head(&sx->to_host.wait);
	
	sema_init(&sx->from_host.sem, 1);
	sema_init(&sx->to_host.sem, 1);

	spin_lock_init(&sx->from_host.lock);
	spin_lock_init(&sx->to_host.lock);

	sx = &stream->sx;
	sx->from_host.buffer.buf = sx->from_host.buf;
	sx->from_host.crc32 = ~0;
	
	sx->to_host.buffer.buf = sx->to_host.buf;
	sx->to_host.crc32 = ~0;

	i2c_set_clientdata(client, stream);

	ret = i2c_slave_register(client, i2c_slave_stream_cb);
	if (ret) {
		cdev_device_del(&stream->cdev, &stream->dev);
		return ret;
	}

	//get_device(&stream->dev);
	
	return 0;
};

static int i2c_slave_stream_remove(struct i2c_client *client)
{
	struct stream_data *stream = i2c_get_clientdata(client);

	printk(KERN_EMERG "%s: stream=%px\n", __func__, stream);
	i2c_slave_unregister(stream->client);
	//cdev_device_del(&stream->cdev, &stream->dev);
	cdev_del(&stream->cdev);
	device_unregister(&stream->dev);
	
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

static int __init i2c_slave_stream_init(void)
{
	int err;

	err = i2c_register_driver(THIS_MODULE, &i2c_slave_stream_driver);
	if (err < 0)
		return err;

	i2c_slave_stream_major = register_chrdev(0, DEVICE_NAME, &fops);
	if (i2c_slave_stream_major < 0) {
		i2c_del_driver(&i2c_slave_stream_driver);
		return i2c_slave_stream_major;
	}

	i2c_slave_stream_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(i2c_slave_stream_class)) {
		i2c_del_driver(&i2c_slave_stream_driver);
		unregister_chrdev(i2c_slave_stream_major, DEVICE_NAME);
		return PTR_ERR(i2c_slave_stream_class);
	}

	return 0;
}

static void __exit i2c_slave_stream_exit(void)
{
	class_destroy(i2c_slave_stream_class);
	unregister_chrdev(i2c_slave_stream_major, DEVICE_NAME);
	i2c_del_driver(&i2c_slave_stream_driver);
}

module_init(i2c_slave_stream_init);
module_exit(i2c_slave_stream_exit);

MODULE_AUTHOR("Kevin Paul Herbert <kph@platinaystems.com>");
MODULE_DESCRIPTION("I2C slave mode stream transport");
MODULE_LICENSE("GPL v2");
