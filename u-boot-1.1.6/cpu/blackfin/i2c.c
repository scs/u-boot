/*
 * i2c.c - driver for Blackfin on-chip TWI/I2C
 *
 * Copyright (c) 2006-2008 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>

#ifdef CONFIG_HARD_I2C

#include <i2c.h>

#include <asm/blackfin.h>
#include <asm/mach-common/bits/twi.h>

#define debugi(fmt, args...) \
	debug( \
		"MSTAT:0x%03x FSTAT:0x%x ISTAT:0x%02x\t" \
		"%-20s:%-3i: " fmt "\n", \
		bfin_read_TWI_MASTER_STAT(), bfin_read_TWI_FIFO_STAT(), bfin_read_TWI_INT_STAT(), \
		__func__, __LINE__, ## args)

#ifdef TWI0_CLKDIV
#define bfin_write_TWI_CLKDIV(val)           bfin_write_TWI0_CLKDIV(val)
#define bfin_write_TWI_CONTROL(val)          bfin_write_TWI0_CONTROL(val)
#define bfin_read_TWI_CONTROL(val)           bfin_read_TWI0_CONTROL(val)
#define bfin_write_TWI_MASTER_ADDR(val)      bfin_write_TWI0_MASTER_ADDR(val)
#define bfin_write_TWI_XMT_DATA8(val)        bfin_write_TWI0_XMT_DATA8(val)
#define bfin_read_TWI_RCV_DATA8()            bfin_read_TWI0_RCV_DATA8()
#define bfin_read_TWI_INT_STAT()             bfin_read_TWI0_INT_STAT()
#define bfin_write_TWI_INT_STAT(val)         bfin_write_TWI0_INT_STAT(val)
#define bfin_read_TWI_MASTER_STAT()          bfin_read_TWI0_MASTER_STAT()
#define bfin_write_TWI_MASTER_STAT(val)      bfin_write_TWI0_MASTER_STAT(val)
#define bfin_read_TWI_MASTER_CTL()           bfin_read_TWI0_MASTER_CTL()
#define bfin_write_TWI_MASTER_CTL(val)       bfin_write_TWI0_MASTER_CTL(val)
#define bfin_write_TWI_INT_MASK(val)         bfin_write_TWI0_INT_MASK(val)
#define bfin_write_TWI_FIFO_CTL(val)         bfin_write_TWI0_FIFO_CTL(val)
#endif

#ifdef CONFIG_TWICLK_KHZ
# error do not define CONFIG_TWICLK_KHZ ... use CFG_I2C_SPEED
#endif
#if CFG_I2C_SPEED > 400000
# error The Blackfin I2C hardware can only operate at 400KHz max
#endif

/* All transfers are described by this data structure */
struct i2c_msg {
	u8 flags;
#define I2C_M_COMBO		0x4
#define I2C_M_STOP		0x2
#define I2C_M_READ		0x1
	int len;		/* msg length */
	u8 *buf;		/* pointer to msg data */
	int alen;		/* addr length */
	u8 *abuf;		/* addr buffer */
};

/**
 *	wait_for_completion: manage the transfer
 */
static int wait_for_completion(struct i2c_msg *msg)
{
	uint16_t int_stat;

	while (!ctrlc()) {
		int_stat = bfin_read_TWI_INT_STAT();

		if (int_stat & XMTSERV) {
			debugi("processing XMTSERV");
			bfin_write_TWI_INT_STAT(XMTSERV);
			SSYNC();
			if (msg->alen) {
				bfin_write_TWI_XMT_DATA8(*(msg->abuf++));
				--msg->alen;
			} else if (!(msg->flags & I2C_M_COMBO) && msg->len) {
				bfin_write_TWI_XMT_DATA8(*(msg->buf++));
				--msg->len;
			} else {
				bfin_write_TWI_MASTER_CTL(bfin_read_TWI_MASTER_CTL() |
					(msg->flags & I2C_M_COMBO ? RSTART | MDIR : STOP));
				SSYNC();
			}
		}
		if (int_stat & RCVSERV) {
			debugi("processing RCVSERV");
			bfin_write_TWI_INT_STAT(RCVSERV);
			SSYNC();
			if (msg->len) {
				*(msg->buf++) = bfin_read_TWI_RCV_DATA8();
				--msg->len;
			} else if (msg->flags & I2C_M_STOP) {
				bfin_write_TWI_MASTER_CTL(bfin_read_TWI_MASTER_CTL() | STOP);
				SSYNC();
			}
		}
		if (int_stat & MERR) {
			debugi("processing MERR");
			bfin_write_TWI_INT_STAT(MERR);
			SSYNC();
			break;
		}
		if (int_stat & MCOMP) {
			debugi("processing MCOMP");
			bfin_write_TWI_INT_STAT(MCOMP);
			SSYNC();
			if (msg->flags & I2C_M_COMBO && msg->len) {
				bfin_write_TWI_MASTER_CTL((bfin_read_TWI_MASTER_CTL() & ~RSTART) |
					(min(msg->len, 0xff) << 6) | MEN | MDIR);
				SSYNC();
			} else
				break;
		}
	}

	return msg->len;
}

/**
 * i2c_transfer - setup an i2c transfer
 *
 *	Here we just get the i2c stuff all prepped and ready, and then tail off
 *	into wait_for_completion() for all the bits to go.
 *
 *	@return:	0 if things worked, non-0 if things failed
 */
static int i2c_transfer(uchar chip, uint addr, int alen, uchar *buffer, int len, u8 flags)
{
	uchar addr_buffer[] = {
		(addr >>  0),
		(addr >>  8),
		(addr >> 16),
	};
	struct i2c_msg msg = {
		.flags = flags | (len >= 0xff ? I2C_M_STOP : 0),
		.buf   = buffer,
		.len   = len,
		.abuf  = addr_buffer,
		.alen  = alen,
	};
	int ret;

	memset(buffer, 0xff, len);
	debugi("chip=0x%x addr=0x%02x alen=%i buf[0]=0x%02x len=%i flags=0x%02x[%s] ",
		chip, addr, alen, buffer[0], len, flags, (flags & I2C_M_READ ? "rd" : "wr"));

	/* wait for things to settle */
	while (bfin_read_TWI_MASTER_STAT() & BUSBUSY)
		if (ctrlc())
			return 1;

	/* Set Transmit device address */
	bfin_write_TWI_MASTER_ADDR(chip);

	/* Clear the FIFO before starting things */
	bfin_write_TWI_FIFO_CTL(XMTFLUSH | RCVFLUSH);
	SSYNC();
	bfin_write_TWI_FIFO_CTL(0);
	SSYNC();

	/* prime the pump */
	if (msg.alen) {
		len = msg.alen;
		debugi("first byte=0x%02x", *msg.abuf);
		bfin_write_TWI_XMT_DATA8(*(msg.abuf++));
		--msg.alen;
	} else if (!(msg.flags & I2C_M_READ) && msg.len) {
		debugi("first byte=0x%02x", *msg.buf);
		bfin_write_TWI_XMT_DATA8(*(msg.buf++));
		--msg.len;
	}

	/* clear int stat */
	bfin_write_TWI_MASTER_STAT(-1);
	bfin_write_TWI_INT_STAT(-1);
	bfin_write_TWI_INT_MASK(0);
	SSYNC();

	/* Master enable */
	bfin_write_TWI_MASTER_CTL(
			(bfin_read_TWI_MASTER_CTL() & FAST) |
			(min(len, 0xff) << 6) | MEN |
			((msg.flags & I2C_M_READ) ? MDIR : 0)
	);
	SSYNC();
	debugi("CTL=0x%04x", bfin_read_TWI_MASTER_CTL());

	/* process the rest */
	ret = wait_for_completion(&msg);
	debugi("ret=%d", ret);

	if (ret) {
		bfin_write_TWI_MASTER_CTL(bfin_read_TWI_MASTER_CTL() & ~MEN);
		bfin_write_TWI_CONTROL(bfin_read_TWI_CONTROL() & ~TWI_ENA);
		SSYNC();
		bfin_write_TWI_CONTROL(bfin_read_TWI_CONTROL() | TWI_ENA);
		SSYNC();
	}

	return ret;
}

/*
 * Initialization, must be called once on start up, may be called
 * repeatedly to change the speed and slave addresses.
 */
void i2c_init(int speed, int slaveaddr)
{
	uint8_t prescale = ((get_sclk() / 1024 / 1024 + 5) / 10) & 0x7F;

	debugi("CONTROL:0x%04x CLKDIV:0x%04x", prescale,
        ((5 * 1024 / (speed / 1000)) << 8) |
        ((5 * 1024 / (speed / 1000)) & 0xFF));

	/* Set TWI internal clock as 10MHz */
	bfin_write_TWI_CONTROL(prescale);

	/* Set TWI interface clock as specified */
	bfin_write_TWI_CLKDIV(
		((5 * 1024 / (speed / 1000)) << 8) |
		((5 * 1024 / (speed / 1000)) & 0xFF)
	);

	/* Don't turn it on */
	bfin_write_TWI_MASTER_CTL(speed > 100000 ? FAST : 0);

	/* But enable it */
	bfin_write_TWI_CONTROL(TWI_ENA | prescale);
	SSYNC();

#if CFG_I2C_SLAVE
# error I2C slave support not tested/supported
	/* If they want us as a slave, do it */
	if (slaveaddr) {
		bfin_write_TWI_SLAVE_ADDR(slaveaddr);
		bfin_write_TWI_SLAVE_CTL(SEN);
	}
#endif
}

/**
 * i2c_probe: - Test if a chip answers for a given i2c address
 *
 * @chip:	address of the chip which is searched for
 * @return: 	0 if a chip was found, -1 otherwhise
 */
int i2c_probe(uchar chip)
{
	u8 byte;
	return i2c_read(chip, 0, 0, &byte, 1);
}

/**
 *   i2c_read: - Read multiple bytes from an i2c device
 *
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   buffer:  Where to read/write the data
 *   len:     How many bytes to read/write
 *
 *   Returns: 0 on success, not 0 on failure
 */
int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	return i2c_transfer(chip, addr, alen, buffer, len, (alen ? I2C_M_COMBO : I2C_M_READ));
}

/**
 *   i2c_write: -  Write multiple bytes to an i2c device
 *
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   buffer:  Where to read/write the data
 *   len:     How many bytes to read/write
 *
 *   Returns: 0 on success, not 0 on failure
 */
int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	return i2c_transfer(chip, addr, alen, buffer, len, 0);
}

/*
 * Utility routines to read/write registers.
 */
uchar i2c_reg_read(uchar chip, uchar reg)
{
	uchar buf;
	i2c_read(chip, reg, 1, &buf, 1);
	return buf;
}
void i2c_reg_write(uchar chip, uchar reg, uchar val)
{
	i2c_write(chip, reg, 1, &val, 1);
}

#endif /* CONFIG_HARD_I2C */
