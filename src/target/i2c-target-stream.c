// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2C slave mode stream driver
 *
 * Copyright (C) 2020 by Kevin Paul Herbert, Platina Systems, Inc.
 *
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/circ_buf.h>
#include <linux/semaphore.h>

#define I2C_SLAVE_STREAM_BUFSIZE 0x100 /* Must be a power of 2 */

struct stream_data {
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

static int i2c_slave_stream_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stream_data *stream;
	int ret;

	stream = devm_kzalloc(&client->dev, sizeof(struct stream_data), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	spin_lock_init(&stream->write_lock);
	spin_lock_init(&stream->read_lock);

	stream->write_buffer.buf = stream->writebuf;
	stream->read_buffer.buf = stream->readbuf;

	i2c_set_clientdata(client, stream);

	ret = i2c_slave_register(client, i2c_slave_stream_slave_cb);
	if (ret) {
		return ret;
	}

	return 0;
};

static int i2c_slave_stream_remove(struct i2c_client *client)
{
	//struct stream_data *stream = i2c_get_clientdata(client);

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

MODULE_AUTHOR("Kevin Paul Herbert <kph@platinaystems.com>");
MODULE_DESCRIPTION("I2C slave mode stream transport");
MODULE_LICENSE("GPL v2");
