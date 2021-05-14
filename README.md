# I2C Interprocess Communications Drivers

This repository contains a pair of Linux device drivers that communicate across
an I2C bus.

## Background on I2C

The I2C bus is asymetric by its nature. I2C devices typically provide an
address space of 256 bytes. A controller addresses a device on the I2C bus
by its bus address, and then reads and writes to device addresses. One side
of the communications is active and the other passive, so the active side
must always poll for data.

## Driver design

The driver is designed as two layers. The upper layer acts as a multiplexor,
allowing for the 256 bytes of address space to be shared by 16 independent
lower layer drivers.

There is presently only one lower level driver, which implements a character
stream. It is possible to have multiple independent character streams.

## Device instantiation

Character devices are instantiated by system firmware (Open Firmware or ACPI).
The firmware must assign an unused I2C address, and the same value must be
used in the active and passive sides. Additionally, the firmware must assign
the base I/O addresses (distinct from the I2C bus address) for each communications
path to maintain.

## Protocol high-level design

Each communications path is represented by a series of registers.

The I2C bus is subject to data corruption. The design of the protocol allows
the active side to decide whether to implement error correction. This is done
via a novel protocol in which the passive side maintains two CRC-32 calculation
registers. One CRC-32 register contains the CRC-32 calculated on the bytes
written to the passive side. The second register contains the CRC-32 calculated
on the bytes read from the passive side. A control register is used by the active
side to either acknowledge the bytes written or received since the last
acknowledgement, or request retransmission (or indicate that it is going to
retransmit) from the point of the lask acknoweldgement.

If the active link partner does not want to implement a reliable transport, it
can simply send periodic transmit and receive acknowledgement commands, without
checking the CRC-32. Note that the current driver always implements a reliable
transport.

## Registers

STREAM_DATA_REG 0x0	Read/Write register 8 bit register

A write operation to STREAM_DATA_REG presents a byte of data for the stream.

A read operation from STREAM_DATA_REG returns a byte of data for the stream.
The client must check to make sure there is data by checking STREAM_CNT_REG
(described below).

STREAM_CNT_REG 0x1	Read-only register 8 bit register

A read operation from STREAM_CNT_REG returns a count of characters that may be
read from the STREAM_DATA_REG register. Since this is a byte register, the
largest value returned will be 255. There may be more data available after
reading all 255 bytes.

STREAM_READ_CRC 0x2-0x5 Read-only register  32 bits little endian format

This register contains an accumulation of the CRC-32 of the data bytes
which have been read via the STREAM_DATA_REG register since the last time
the active link partner has acknowledged receive data. The passive side
keeps the data in a buffer so that the active side can request retransmission
if there are data errors on the I2C bus.

STREAM_WRITE_CRC 0x6-0x9 Read-only register 32 bits little endian format

This register contains an accumulation of the CRC-32 of the data bytes
which have been written to the passive link partner since the last time
the active link partner has acknowledged that its transmitted data was
received properly.

Note that in both the transmit and receive cases, the active side does not
transmit a CRC. Instead, it calculates an expected CRC, and receives the
values calculated by the passive side. This keeps the active side in control
of the protocol and allows error correction to be optional.

STREAM_CTL_REG 0xa Read/write 8 bits

The STREAM_CTL_REG register is used by the active side to run the error
correction protocol. Writes to this register are either positive or
negative acknowledgements based on CRC checking. Reads to this register
are used by the active side to make sure that commands are not lost.

In order to reduce the risk of erronenous commands, each command is a single
bit in the register. This means that in the event of a data error, only 7
actual values (zero is a NOP command) out of the 256 possible values are
valid commands, so the chance of an actual command being issued is reduced
to 256/7.

Note: This might not be good enough. It might be better to write a command
byte to one register, and write its twos complement to another register.

Commands are:

0x80: Acknowledge bytes received from passive partner since last ACK.

Receipt of this command signals the passive partner that bytes it
transmitted over a stream have been received, and that these bytes should
be removed from the passive retransmission buffer. If the retransmission
buffer was full, there may have been a Linux process blocked waiting to
send more bytes, and it will be woken up.

0x40: Acknoweldge bytes written to the passive partner since last ACK

Receipt of this command signals the passive partner that the last series
of bytes which it has been buffering in temporary memory should be
committed to as received correctly, and passed to a waiting Linux
process.

0x20: Negative Acknowledge bytes received from passive partner since last ACK

Receipt of this command signals the passive partner that a data error has
occured. This will cause the passive partner to reset its internal buffer
pointer to the last acknowledgement point, and subsequent read operations
from the STREAM_DATA_REG will contain those bytes again. Additionally,
the STREAM_CNT_REG will reflect the number of available bytes.

0x10: Negative Acknoweldge bytes transmitted to passive partner since last ACK

Receipt of this command signals the passive partner that data the active
partner was not received properly. The passive partner will discard all of
the data received since the last acknoweldgement, and the active partner
will then retransmit it.
