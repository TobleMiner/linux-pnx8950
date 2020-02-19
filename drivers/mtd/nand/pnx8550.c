/*
 * Copyright (C) 2005 Koninklijke Philips Electronics N.V.
 * Copyright (C) 2020 Tobias Schramm
 * All Rights Reserved.
 *
 * Based on: drivers/mtd/nand/pnx8550.c by Torbjorn Lundberg
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Overview:
 *   This is a device driver for the NAND flash XIO peripheral of
 *   the PNX8x50
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/interrupt.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/mach-pnx8550/nand.h>

#define XIO_ACK_TIMEOUT_US	2000000		// 2s wait for xio ack (in us)
#define XIO_DMA_TIMEOUT_US	30000000	// 30s wait for dma (in us)

#define NAND_ADDR(_col, _page) ((_col) & (pnx_nand.mtd.writesize - 1)) + ((_page) << this->page_shift)

#define NAND_ADDR_SEND(_addr) pnx_nand.nand_mem[(_addr)/sizeof(u16)] = 0

#define NAND_TRANSFER_TO(_addr, _buffer, _bytes) pnx8550_nand_transfer((_buffer), ((u8*)pnx_nand.nand_mem) + (_addr), (_bytes), 1)

#define NAND_TRANSFER_FROM(_addr, _buffer, _bytes) pnx8550_nand_transfer(((u8*)pnx_nand.nand_mem) + (_addr), (_buffer), (_bytes), 0)

static void pnx8550_nand_register_setup(u_char cmd_no, u_char addr_no,
					u_char include_data, u_char monitor_ACK,
					u_char enable64M, int cmd_a, int cmd_b);

static inline void pnx8550_nand_wait_for_dev_ready(void);

static void pnx8550_nand_transfer(void *from, void *to, size_t bytes, bool toxio);

#define MTD_TO_PNX(mtd) ((struct pnx8550_nand*)(container_of(mtd, struct pnx8550_nand, mtd)))

static const char *part_probes[] = { "cmdlinepart", NULL };

typedef enum {
	NAND_ADDRESS_COLUMN = 0,
	NAND_ADDRESS_PAGE = 1,
	NAND_ADDRESS_PAGE_EXTENDED = 2,
	NAND_ADDRESS_MAX
} pnx_address_cycle_t;

struct pnx8550_nand {
	struct mtd_info mtd;
	struct nand_chip nand;

	unsigned int current_cmd;

	// First address is column, second is low page, third is high page
	pnx_address_cycle_t address_cycle;
	u32 address_column;
	u32 address_page;

	u_char *xfer_buf;

	volatile u16 *nand_mem;

	bool is64m;
};

struct pnx8550_nand pnx_nand;

/* Bad block descriptor for 16Bit nand flash */
static uint8_t scan_ff_pattern[] = { 0xff, 0xff };
static struct nand_bbt_descr nand16bit_memorybased = {
	.options = 0,
	.offs = 0,
	.len = 2,
	.pattern = scan_ff_pattern
};

/* bad block descriptor located in the "middle" of the flash
 *  this is pretty evil, but since the end is used by the microBTM we don't
 *  have a real choice here
 */
#ifdef CONFIG_MTD_NAND_PNX8550_BADBLOCK

static u8 bbt_pattern[] = {'B', 'b', 't', '0' };
static u8 mirror_pattern[] = {'1', 't', 'b', 'B' };

static struct nand_bbt_descr nand_main_bbt_decr = {
	.options = NAND_BBT_ABSPAGE | NAND_BBT_CREATE | NAND_BBT_WRITE |
			NAND_BBT_2BIT | NAND_BBT_VERSION,
	.pages[0] = 0x460,
	.offs = 1,
	.len = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr nand_mirror_bbt_decr = {
	.options = NAND_BBT_ABSPAGE | NAND_BBT_CREATE | NAND_BBT_WRITE |
			NAND_BBT_2BIT | NAND_BBT_VERSION,
	.pages[0] = 0x480,
	.offs = 1,
	.len = 4,
	.pattern = mirror_pattern
};

#endif

#ifdef CONFIG_MTD_NAND_WINCE_ECC
// wince oob placement
static struct nand_ecclayout nand8bit_oob_wince = {
    .eccbytes = 6,
    .eccpos = {8, 9, 10, 11, 12, 13},
    .oobfree = { {0, 8} },
};

static u_char eccArray[] = {
		0x00, 0x01, 0x01, 0x02, 0x01, 0x02, 0x02, 0x03, 0x01, 0x02, 0x02, 0x03, 0x02, 0x03, 0x03, 0x04,
		0x01, 0x02, 0x02, 0x03, 0x02, 0x03, 0x03, 0x04, 0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05,
		0x01, 0x02, 0x02, 0x03, 0x02, 0x03, 0x03, 0x04, 0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05,
		0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05, 0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06,
		0x01, 0x02, 0x02, 0x03, 0x02, 0x03, 0x03, 0x04, 0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05,
		0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05, 0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06,
		0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05, 0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06,
		0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06, 0x04, 0x05, 0x05, 0x06, 0x05, 0x06, 0x06, 0x07,
		0x01, 0x02, 0x02, 0x03, 0x02, 0x03, 0x03, 0x04, 0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05,
		0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05, 0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06,
		0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05, 0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06,
		0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06, 0x04, 0x05, 0x05, 0x06, 0x05, 0x06, 0x06, 0x07,
		0x02, 0x03, 0x03, 0x04, 0x03, 0x04, 0x04, 0x05, 0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06,
		0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06, 0x04, 0x05, 0x05, 0x06, 0x05, 0x06, 0x06, 0x07,
		0x03, 0x04, 0x04, 0x05, 0x04, 0x05, 0x05, 0x06, 0x04, 0x05, 0x05, 0x06, 0x05, 0x06, 0x06, 0x07,
		0x04, 0x05, 0x05, 0x06, 0x05, 0x06, 0x06, 0x07, 0x05, 0x06, 0x06, 0x07, 0x06, 0x07, 0x07, 0x08,
		0x00, 0x01, 0x04, 0x05, 0x10, 0x11, 0x14, 0x15, 0x40, 0x41, 0x44, 0x45, 0x50, 0x51, 0x54, 0x55,
		0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
		0x00, 0x00, 0x01, 0x01,	0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
		0x04, 0x04, 0x05, 0x05,	0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07,	0x06, 0x06, 0x07, 0x07,
		0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
		0x00, 0x00, 0x01, 0x01,	0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x02, 0x02, 0x03, 0x03,
		0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03,	0x02, 0x02, 0x03, 0x03,
		0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
		0x04, 0x04, 0x05, 0x05,	0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07, 0x06, 0x06, 0x07, 0x07,
		0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B, 0x0A, 0x0A, 0x0B, 0x0B,
		0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B, 0x0A, 0x0A, 0x0B, 0x0B,
		0x0C, 0x0C, 0x0D, 0x0D, 0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0E, 0x0F, 0x0F, 0x0E, 0x0E, 0x0F, 0x0F,
		0x0C, 0x0C, 0x0D, 0x0D, 0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0E, 0x0F, 0x0F, 0x0E, 0x0E, 0x0F, 0x0F,
		0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09,	0x0A, 0x0A, 0x0B, 0x0B, 0x0A, 0x0A, 0x0B, 0x0B,
		0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B, 0x0A, 0x0A, 0x0B, 0x0B,
		0x0C, 0x0C, 0x0D, 0x0D, 0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0E, 0x0F, 0x0F, 0x0E, 0x0E, 0x0F, 0x0F,
		0x0C, 0x0C, 0x0D, 0x0D, 0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0E, 0x0F, 0x0F, 0x0E, 0x0E, 0x0F, 0x0F,
		0x52, 0x53, 0x44, 0x53, 0x67, 0x98, 0xE5, 0x1F, 0x71, 0x94, 0x75, 0x45, 0x9F, 0x77, 0x16, 0x55,
		0xA8, 0xCD, 0x45, 0x06, 0x11, 0x00, 0x00, 0x00,
};

#endif

/* OOB Placement information that lines up with the boot loader code */
static struct nand_ecclayout nand16bit_oob_16 = {
    .eccbytes = 6,
    .eccpos = {2, 3, 4, 5, 6, 7},
    .oobfree = { {8, 8} }
};

/**
 * Transfer data to/from the NAND chip using DMA
 *
 * @from:  Address to transfer data from
 * @to:    Address to transfer the data to
 * @bytes: Number of bytes to transfer
 * @toxio: Whether the transfer is going to XIO or not.
 */
static void pnx8550_nand_transferDMA(void *from, void *to, size_t bytes, bool toxio)
{
	int cmd = 0, timeout = 0;
	u32 internal;
	u32 external;

	if (toxio) {
		cmd = PNX8550_DMA_CTRL_PCI_CMD_WRITE;
		dma_cache_wback((unsigned long)from, bytes);
		internal = (u32) virt_to_phys(from);
		external = (u32) to - KSEG1;
	} else {
		cmd = PNX8550_DMA_CTRL_PCI_CMD_READ;
		internal = (u32) virt_to_phys(to);
		external = (u32) from - KSEG1;
	}

	local_irq_disable();
	PNX8550_DMA_TRANS_SIZE = bytes >> 2;	/* Length in words */
	PNX8550_DMA_EXT_ADDR = external;
	PNX8550_DMA_INT_ADDR = internal;
	PNX8550_DMA_INT_CLEAR = 0xffff;
	PNX8550_DMA_CTRL = PNX8550_DMA_CTRL_BURST_512 |
	    PNX8550_DMA_CTRL_SND2XIO | PNX8550_DMA_CTRL_INIT_DMA | cmd;

	while ((PNX8550_DMA_INT_STATUS & PNX8550_DMA_INT_COMPL) == 0) {
	    	udelay(1);
    		timeout++;
    		if(timeout > XIO_DMA_TIMEOUT_US) {
			WARN(true, "Timeout on NAND DMA wait!");
			BUG();
		}
	}

	if (!toxio) {
		dma_cache_inv((unsigned long)to, bytes);
	}
	local_irq_enable();
}

/**
 * Transfer data to/from the NAND chip.
 * This function decides whether to use DMA or not depending on
 * the amount of data to transfer and the alignment of the buffers.
 *
 * @from:  Address to transfer data from
 * @to:    Address to transfer the data to
 * @bytes: Number of bytes to transfer
 * @toxio: Whether the transfer is going to XIO or not.
 */
static void pnx8550_nand_transfer(void *from, void *to, size_t bytes, bool toxio)
{
	u16 *from16 = (u16 *) from;
	u16 *to16 = (u16 *) to;

	u32 i;

	if (((bytes & 3) || (bytes < 16)) || ((u32) to & 3) || ((u32) from & 3)) {
		if (((bytes & 1) == 0) &&
		    (((u32) to & 1) == 0) && (((u32) from & 1) == 0)) {
			size_t words = bytes / 2;

			local_irq_disable();
			for (i = 0; i < words; i++) {
				to16[i] = from16[i];
			}
			local_irq_enable();
		} else {
			WARN
			    (true, "%s: Transfer failed, byte-aligned transfers no allowed!\n",
			     __FUNCTION__);
		}
	} else {
		pnx8550_nand_transferDMA(from, to, bytes, toxio);
	}
}

/**
 * pnx8550_nand_read_byte16 - read one byte endianess aware from the chip (16 bit bus width)
 * @mtd:	MTD device structure
 *
 */
static u_char pnx8550_nand_read_byte16(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);

	u16 data = 0;
	int addr = NAND_ADDR(pnx->address_column, pnx->address_page);
	/*
	   Read ID is a special case as we have to read BOTH bytes at the same
	   time otherwise it doesn't work, once we have both bytes we work out
	   which one we want.
	 */
	if (pnx->current_cmd == NAND_CMD_READID) {
		u32 *pNandMem32 = (u32 *) pnx->nand_mem;
		u32 data32;
		data32 = cpu_to_le32(pNandMem32[0]);
		pr_info("Reading chip id @%p, offset: %u, data: %04x\n", pnx->nand_mem, pnx->address_column, data32);
		if (pnx->address_column) {
			data = (u16) (data32 << 16);
		} else {
			data = (u16) data32;
		}
	} else {
		data = cpu_to_le16(pnx->nand_mem[(addr / sizeof(u16))]);
		if ((addr & 0x1) == 1) {
			data = (data & 0xff00) >> 16;
		}
	}
	/*
	   Status is a special case, we don't need to increment the address
	   because the address isn't used by the chip
	 */
	if (pnx->current_cmd != NAND_CMD_STATUS) {
		pnx->address_column++;
		if (pnx->address_column >= mtd->writesize + mtd->oobsize) {
			pnx->address_column -= mtd->writesize + mtd->oobsize;
			pnx->address_page++;
		}
	}
	return data & 0xff;
}

/**
 * pnx8550_nand_read_byte8 - read one byte endianess aware from the chip (8 bit bus width)
 * @mtd:	MTD device structure
 *
 */
static u_char pnx8550_nand_read_byte8(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	u8 data = 0;
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);

	int addr = NAND_ADDR(pnx->address_column, pnx->address_page);
	/*
	   Read ID is a special case as we have to read BOTH bytes at the same
	   time otherwise it doesn't work, once we have both bytes we work out
	   which one we want.
	 */
	if (pnx->current_cmd == NAND_CMD_READID) {
		u16 *nand_mem16 = (u16 *) pnx->nand_mem;
		u16 data16;
		data16 = cpu_to_le16(nand_mem16[0]);
		pr_info("Reading chip id @%p, offset: %u, data: %02x\n", pnx->nand_mem, pnx->address_column, data16);
		if (pnx->address_column) {
			data = (u8) (data16 >> 8);
		} else {
			data = (u8) data16;
		}
	} else {
		data = cpu_to_le16(pnx->nand_mem[addr]);
	}
	/*
	   Status is a special case, we don't need to increment the address
	   because the address isn't used by the chip
	 */
	if (pnx->current_cmd != NAND_CMD_STATUS) {
		pnx->address_column++;
		if (pnx->address_column >= mtd->writesize + mtd->oobsize) {
			pnx->address_column -= mtd->writesize + mtd->oobsize;
			pnx->address_page++;
		}
	}
	return data & 0xff;
}

/**
 * pnx8550_nand_read_word - read one word from the chip
 * @mtd:	MTD device structure
 *
 * Read function for 16bit buswidth without
 * endianess conversion
 */
static u16 pnx8550_nand_read_word(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);
	int addr = NAND_ADDR(pnx->address_column, pnx->address_page);
	u16 data = pnx->nand_mem[(addr / sizeof(u16))];

	pnx->address_column += 2;
	if (pnx->address_column >= mtd->writesize + mtd->oobsize) {
		pnx->address_column -= mtd->writesize + mtd->oobsize;
		pnx->address_page++;
	}
	return data;
}

static int ensure_transfer_buffer(struct mtd_info *mtd) {
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);
	if(pnx->xfer_buf) {
		return 0;
	}
	pnx->xfer_buf =
	    kzalloc(mtd->writesize + mtd->oobsize,
		    GFP_DMA | GFP_KERNEL);
	if (!pnx->xfer_buf) {
		printk(KERN_ERR
		       "Unable to allocate NAND data buffer for PNX8550.\n");
		BUG();
		return -ENOMEM;
	}
	pr_info("Allocated transfer buffer @%px, size %zu\n", pnx->xfer_buf, mtd->writesize + mtd->oobsize);
	return 0;
}

/**
 * pnx8550_nand_write_buf - write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 */
#undef DONT_WRITE
static void pnx8550_nand_write_buf(struct mtd_info *mtd, const u_char * buf,
				   int len)
{
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);
#ifndef DONT_WRITE
	struct nand_chip *this = mtd->priv;
	int addr = NAND_ADDR(pnx->address_column, pnx->address_page);
	int pageLen;
	int oobLen = 0;
	u_char *transBuf = (u_char *) buf;

	/* some sanity checking, word access only please */
	if (len & 1) {
		pr_warning("%s: non-word aligned length requested!\n",
		       __FUNCTION__);
	}

	// TODO: Find alternate solution that can handle errors
	ensure_transfer_buffer(mtd);

	memcpy(pnx->xfer_buf, buf, len);
	transBuf = pnx->xfer_buf;

	/*
	   Work out whether we are going to write to the OOB area
	   after a standard page write.
	   This is not the case when the command function is called
	   with a column address page size. Then we write as though
	   it is to the page rather than the OOB as the command function
	   has already selected the OOB area.
	 */
	if ((pnx->address_column + len) > mtd->writesize)
		oobLen = (pnx->address_column + len) - mtd->writesize;
	pageLen = len - oobLen;

	/* Clear the done flag */
	PNX8550_GPXIO_CTRL |= PNX8550_GPXIO_CLR_DONE;
	if (pageLen > 0) {
		NAND_TRANSFER_TO(addr, transBuf, pageLen);
	}
	if (oobLen > 0) {
		pnx8550_nand_wait_for_dev_ready();

		pnx8550_nand_register_setup(1, 0, 0, 1, pnx->is64m, NAND_CMD_READOOB, 0);
		/* Work out where in the OOB we are going to start to write */
		addr = NAND_ADDR(pnx->address_column - mtd->writesize, pnx->address_page);
		NAND_ADDR_SEND(addr);
		pnx8550_nand_register_setup(2, 3, 1, 1, pnx->is64m, NAND_CMD_SEQIN,
					    NAND_CMD_PAGEPROG);

		/* Clear the done flag */
		PNX8550_GPXIO_CTRL |= PNX8550_GPXIO_CLR_DONE;
		NAND_TRANSFER_TO(addr, transBuf + pageLen, oobLen);
	}
#endif
	/*
	   Increment the address so on the next write we write in the
	   correct place.
	 */
	pnx->address_column += len;
	if (pnx->address_column >= mtd->writesize + mtd->oobsize) {
		pnx->address_column -= mtd->writesize + mtd->oobsize;
		pnx->address_page++;
	}
}

/**
 * pnx8550_nand_read_buf - read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 */
static void pnx8550_nand_read_buf(struct mtd_info *mtd, u_char * buf, int len)
{
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);
	struct nand_chip *this = mtd->priv;
	int addr = NAND_ADDR(pnx->address_column, pnx->address_page);
	int pageLen;
	int oobLen = 0;
	u_char *transBuf = buf;

	/* some sanity checking, word access only please */
	if (len & 1) {
		pr_warning("%s: non-word aligned length\n", __FUNCTION__);
	}

	// TODO: Find alternate solution that can handle errors
	ensure_transfer_buffer(mtd);

	transBuf = pnx->xfer_buf;

	/*
	   Work out whether we are going to read the OOB area
	   after a standard page read.
	   This is not the case when the command function is called
	   with a column address page size. Then we read as though
	   it is from the page rather than the OOB as the command
	   function has already selected the OOB area.
	 */
	if ((pnx->address_column + len) > mtd->writesize)
		oobLen = (pnx->address_column + len) - mtd->writesize;
	pageLen = len - oobLen;

	if (pageLen) {
		NAND_TRANSFER_FROM(addr, transBuf, pageLen);
	}
	if (oobLen > 0) {
		pnx8550_nand_register_setup(1, 3, 1, 1, pnx->is64m, NAND_CMD_READOOB, 0);
		addr = NAND_ADDR(pnx->address_column - mtd->writesize, pnx->address_page);
		NAND_TRANSFER_FROM(addr, transBuf + pageLen, oobLen);
	}
	if (transBuf != buf) {
		memcpy(buf, transBuf, len);
	}

	/*
	   Increment the address so on the next read we read from the
	   correct place.
	 */
	pnx->address_column += len;
	if (pnx->address_column > mtd->writesize + mtd->oobsize) {
		pnx->address_column -= mtd->writesize + mtd->oobsize;
		pnx->address_page++;
	}

//	pr_info("Read %d bytes from %px\n", len, &(pnx->nand_mem[addr / sizeof(u16)]));
//	print_hex_dump(KERN_INFO, "pnx8550 data read: ", DUMP_PREFIX_ADDRESS, 16, 1, buf, len, false);
	return;
}

/**
 * pnx8550_nand_verify_buf -  Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 */
static int pnx8550_nand_verify_buf(struct mtd_info *mtd, const u_char * buf,
				   int len)
{
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);
	int result = 0;

	/* some sanity checking, word access only please */
	if (len & 1) {
		pr_warning("%s: non-word aligned length\n", __FUNCTION__);
	}

	if((result = ensure_transfer_buffer(mtd))) {
		return result;
	}

	pnx8550_nand_read_buf(mtd, pnx->xfer_buf, len);
	if (memcmp(buf, pnx->xfer_buf, len)) {
		result = -EFAULT;
	}

	return result;

}

// Empty control command handler, might replace pnx8550_nand_command through this one later
static void pnx8550_nand_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
/*
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);

	struct nand_chip *chip = mtd->priv;

	BUG_ON((ctrl & NAND_ALE) && (ctrl & NAND_CLE));
	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_ALE) {
			// Change flag enabled, new address cycle
			pr_info("cmd_ctrl: New address cycle, column: %04x\n", dat);
			pnx->address_cycle = NAND_ADDRESS_COLUMN;
			pnx->address_column = dat;
			pnx->address_cycle++;
//			chip->IO_ADDR_W = nand->bank_base + JZ_NAND_MEM_ADDR_OFFSET;
		} else if (ctrl & NAND_CLE)
			// Problem: we need to know the number of address bytes to
			// Send the command to the nand controller
			// So store dat in pnx->pending_cmd and set pnx->required_address_len
			// to 
			pr_info("cmd_ctrl: CLE, %08x\n", dat);
//			chip->IO_ADDR_W = nand->bank_base + JZ_NAND_MEM_CMD_OFFSET;
		else
			pr_info("cmd_ctrl: CMD, %08x\n", dat);
//			chip->IO_ADDR_W = nand->bank_base;

//		reg = readl(nand->base + JZ_REG_NAND_CTRL);
//		if (ctrl & NAND_NCE)
//			reg |= JZ_NAND_CTRL_ASSERT_CHIP(0);
//		else
//			reg &= ~JZ_NAND_CTRL_ASSERT_CHIP(0);
//		writel(reg, nand->base + JZ_REG_NAND_CTRL);

	} else {
		if (ctrl & NAND_ALE) {
			// Continuation of previous address cycle
			// Accept maximum of 3 address cycles
			BUG_ON(pnx->address_cycle >= NAND_ADDRESS_MAX);
			if(pnx->address_cycle == NAND_ADDRESS_PAGE) {
				// Normal page address, 2nd cycle
				pnx->adress_page = dat;
			} else {
				// Extended page address, 3rd cycle
				pnx->adress_page |= dat >> 16;
			}
			pnx->address_cycle++;
		}
		pr_info("cmd_ctrl: %02x\n", dat);
	}

//	if (dat != NAND_CMD_NONE)
//		writeb(dat, chip->IO_ADDR_W);
*/
}

/**
 * pnx8550_nand_command - Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device.
 */
static void pnx8550_nand_command(struct mtd_info *mtd, unsigned command,
				 int column, int page_addr)
{
	struct pnx8550_nand *pnx = MTD_TO_PNX(mtd);
	register struct nand_chip *this = mtd->priv;
	u_char addr_no = 0;
	int addr;
	/*
	   If we are starting a write work out whether it is to the
	   OOB or the main page and position the pointer correctly.
	 */
	if (command == NAND_CMD_SEQIN) {
		int readcmd;
		int col = column;
		if (column >= mtd->writesize) {
			/* OOB area */
			col -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else {
			readcmd = NAND_CMD_READ0;
		}
		pnx8550_nand_register_setup(1, 0, 0, 1, pnx->is64m, readcmd, 0);
		addr = NAND_ADDR(col, page_addr);
		NAND_ADDR_SEND(addr);
	}

	/* Check the number of address bytes */
	if ((column == -1) && (page_addr == -1)) {
		addr_no = 0;
		column = 0;
		page_addr = 0;
	} else if ((column == -1) && (page_addr != -1)) {
		addr_no = 2;
		column = 0;
	} else if ((column != -1) && (page_addr == -1)) {
		addr_no = 1;
		page_addr = 0;
	} else {
		addr_no = 3;
	}

	pnx->current_cmd = command;
	pnx->address_column = column;
	pnx->address_page = page_addr;

	switch (command) {

	case NAND_CMD_PAGEPROG:
		// Nothing to do, we've already done it!
		return;

	case NAND_CMD_SEQIN:
		if (addr_no != 3)
			pr_warning
			    ("NAND: Error. Command %02x needs 3 byte address, but addr_no = %d\n",
			     command, addr_no);
		pnx8550_nand_register_setup(2, 3, 1, 1, pnx->is64m, NAND_CMD_SEQIN,
					    NAND_CMD_PAGEPROG);
		return;

	case NAND_CMD_ERASE1:
		if (addr_no != 2)
			pr_warning
			    ("NAND: Error. Command %02x needs 2 byte address, but addr_no = %d\n",
			     command, addr_no);

		PNX8550_GPXIO_CTRL |= PNX8550_GPXIO_CLR_DONE;

		pnx8550_nand_register_setup(2, 2, 0, 1, pnx->is64m, NAND_CMD_ERASE1,
					    NAND_CMD_ERASE2);
		addr = NAND_ADDR(column, page_addr);
		NAND_ADDR_SEND(addr);
		return;

	case NAND_CMD_ERASE2:
		// Nothing to do, we've already done it!
		return;

	case NAND_CMD_STATUS:
		if (addr_no != 0)
			pr_warning
			    ("NAND: Error. Command %02x needs 0 byte address, but addr_no = %d\n",
			     command, addr_no);
		pnx8550_nand_register_setup(1, 0, 1, 0, pnx->is64m, NAND_CMD_STATUS, 0);
		return;

	case NAND_CMD_RESET:
		if (addr_no != 0)
			pr_warning
			    ("NAND: Error. Command %02x needs 0 byte address, but addr_no = %d\n",
			     command, addr_no);
		pnx8550_nand_register_setup(1, 0, 0, 0, 0, NAND_CMD_RESET, 0);
		addr = NAND_ADDR(column, page_addr);
		NAND_ADDR_SEND(addr);
		return;

	case NAND_CMD_READ0:
	case NAND_CMD_READ1:
		if (addr_no != 3)
			pr_warning
			    ("NAND: Error. Command %02x needs 3 byte address, but addr_no = %d\n",
			     command, addr_no);

		pnx8550_nand_register_setup(1, 3, 1, 1, pnx->is64m, NAND_CMD_READ0, 0);
		return;

	case NAND_CMD_READOOB:
		if (addr_no != 3)
			pr_warning
			    ("NAND: Error. Command %02x needs 3 byte address, but addr_no = %d\n",
			     command, addr_no);
		pnx8550_nand_register_setup(1, 3, 1, 1, pnx->is64m, NAND_CMD_READOOB, 0);
		return;

	case NAND_CMD_READID:
		if (addr_no != 1)
			pr_warning
			    ("NAND: Error. Command %02x needs 1 byte address, but addr_no = %d\n",
			     command, addr_no);
		pnx8550_nand_register_setup(1, 1, 1, 0, 0, NAND_CMD_READID, 0);
		return;
	}
}

/*
 * Setup the registers in PCIXIO
 */
static void pnx8550_nand_register_setup(u_char cmd_no,
					u_char addr_no,
					u_char include_data,
					u_char monitor_ACK,
					u_char enable64M, int cmd_a, int cmd_b)
{
	unsigned int reg_nand = 0;
	reg_nand |= enable64M ? PNX8550_XIO_FLASH_64MB : 0;
	reg_nand |= include_data ? PNX8550_XIO_FLASH_INC_DATA : 0;
	reg_nand |= PNX8550_XIO_FLASH_CMD_PH(cmd_no);
	reg_nand |= PNX8550_XIO_FLASH_ADR_PH(addr_no);
	reg_nand |= PNX8550_XIO_FLASH_CMD_A(cmd_a);
	reg_nand |= PNX8550_XIO_FLASH_CMD_B(cmd_b);
	PNX8550_XIO_FLASH_CTRL = reg_nand;
	barrier();
}

/*
 * Wait for the device to be ready for the next command
 */
static inline void pnx8550_nand_wait_for_dev_ready(void)
{
	int timeout = 0;
	while ((PNX8550_XIO_CTRL & PNX8550_XIO_CTRL_XIO_ACK) == 0) {
	    	udelay(1);
    		timeout++;
    		if(timeout > XIO_ACK_TIMEOUT_US) {
			WARN(true, "Timeout on NAND ACK wait!");
			BUG();
		}
	}
}

/*
 * Return true if the device is ready, false otherwise
 */
static int pnx8550_nand_dev_ready(struct mtd_info *mtd)
{
	return ((PNX8550_XIO_CTRL & PNX8550_XIO_CTRL_XIO_ACK) != 0);
}

#ifdef CONFIG_MTD_NAND_WINCE_ECC

static int pnx8550_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
		       u_char *ecc_code)
{
#warning This code is converted from assembler!
	u32 a0, a1, a2, a3;
	u32 t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;
	u32 v0, v1;

	a0 = (u32) dat;
	a1 = (u32) ecc_code;

	a3 = 0;
	a2 = 0;
	v1 = 0;
	t0 = 0;
	t1 = a0;
	v0 = (u32) eccArray;

	for (t0 = 0; t0 < 0x40; t0++) {
		a0 = *((u32*) t1);
		v1 = a0 ^ v1;
		a0 = (a0 >> 16) ^ a0;
		t4 = ((a0 >> 8) ^ a0);
		t7 = *(eccArray + (t4 & 0xFF));
		if (t7 & 1) {
			a3 ^= t0;
			a2 ^= ~t0;
		}
		t1 = t1 + 4;
	}
	t1 = (a3 >> 4) & 3;
	t7 = (a2 >> 4) & 3;
	t4 = *(eccArray + 0x100 + t1);
	a0 = *(eccArray + 0x100 + t7);

	t1 = ((t4 << 1) | a0) << 7;
	t5 = *(eccArray + 0x100 + (a3 & 0xF));
	t6 = t1 | t5;

	a2 = *(eccArray + 0x100 + (a2 & 0xF));
	a3 = (t6 << 1) | a2;

	a0 = 0x55555555;
	a2 = v1 & a0;
	a2 = (a2 >> 16) ^ a2;
	t2 = (a2 >> 8) ^ a2;
	t6 = 0xAAAAAAAA;
	a2 = v1 & t6;
	a2 = (a2 >> 16) ^ a2;

	t5 = *(eccArray + (t2 & 0xFF));
	t9 = (a2 >> 8) ^ a2;

	t4 = 0x33333333;
	a2 = v1 & t4;
	a0 = t5 & 1;

	a2 = (a2 >> 16) ^ a2;
	t7 = (a2 >> 8) ^ a2;
	t1 = *(eccArray + (t9 & 0xFF));
	t3 = (t1 & 1) << 1;

	a2 = *(eccArray + (t7 & 0xFF));
	t2 = 0xCCCCCCCC;
	t0 = a2 & 1;
	a2 = v1 & t2;
	a0 = t3 | a0;
	a2 = (a2 >> 16) ^ a2;
	t5 = (a2 >> 8) ^ a2;
	t8 = *(eccArray + (t5 & 0xFF));
	t1 = t0 << 2;

	a0 = t1 | a0;
	a2 = (t8 & 1) << 3;
	t0 = 0xF0F0F0F;
	a0 = a2 | a0;
	a2 = v1 & t0;
	a2 = (a2 >> 16) ^ a2;
	t3 = (a2 >> 8) ^ a2;
	t9 = 0xF0F0F0F0;
	t6 = *(eccArray + (t3 & 0xFF));
	t7 = t6 & 1;
	a2 = v1 & t9;
	a2 = (a2 >> 16) ^ a2;
	t2 = (a2 >> 8) ^ a2;
	a0 = (t7 << 4) | a0;

	t5 = *(eccArray + (t2 & 0xFF));
	t8 = 0xFF00FF;
	a2 = v1 & t8;
	t6 = t5 & 1;
	a2 = (a2 >> 16) ^ a2;
	t5 = 0xFF00FF00;
	t0 = a2 & 0xFF;
	a2 = v1 & t5;
	a0 = (t6 << 5) | a0;
	t7 = (a2 >> 16) ^ a2;
	t2 = *(eccArray + t0);
	t9 = (t7 >> 8) & 0xFF;
	t0 = *(eccArray + t9);
	t3 = t2 & 1;
	a2 = v1 & 0xFFFF;
	a0 = (t3 << 6) | a0;
	t4 = (a2 >> 8) ^ a2;
	t7 = *(eccArray + (t4 & 0xFF));
	t2 = (t0 & 1) << 7;
	a0 = t2 | a0;
	t9 = (t7 & 1) << 8;
	v1 = v1 >> 16;
	a2 = t9 | a0;
	t0 = (v1 >> 8) ^ v1;
	t3 = *(eccArray + (t0 & 0xFF));
	*((u8*) a1) = a3;
	t5 = (t3 & 1) << 9;
	v0 = t5 | a2;
	t8 = (a3 >> 8) | (v0 << 4);
	v0 = (v0 >> 4) | 0xC0;
	*((u8*) a1 + 1) = t8;
	*((u8*) a1 + 2) = v0;

//	DEBUG("ECC: %02x %02x %02x", ecc_code[0], ecc_code[1], ecc_code[2]);

	return 0;
}

static void pnx8550_nand_ecc_hwctrl(struct mtd_info *mtdinfo, int mode)
{
	//do nothing
}

static inline int is_buf_blank(uint8_t *buf, size_t len)
{
	for (; len > 0; len--)
		if (*buf++ != 0xff)
			return 0;
	return 1;
}

/**
 * pnx8550_nand_correct_data - [NAND Interface] Detect and correct bit error(s)
 * @mtd:	MTD block structure
 * @dat:	raw data read from the chip
 * @read_ecc:	ECC from the chip
 * @calc_ecc:	the ECC calculated from raw data
 *
 * Detect and correct a 1 bit error for 256 byte block
 */
int pnx8550_nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc,
		u_char *calc_ecc) {

	uint8_t s0, s1, s2;

#ifdef CONFIG_MTD_NAND_ECC_SMC
	s0 = calc_ecc[0] ^ read_ecc[0];
	s1 = calc_ecc[1] ^ read_ecc[1];
	s2 = calc_ecc[2] ^ read_ecc[2];
#else
	s1 = calc_ecc[0] ^ read_ecc[0];
	s0 = calc_ecc[1] ^ read_ecc[1];
	s2 = calc_ecc[2] ^ read_ecc[2];
#endif
	if ((s0 | s1 | s2) == 0)
		return 0;

	// ignore ECC code of empty pages since they don't match the oob data
	// oob: FF FF FF	ecc: 00 00 C0
	if(is_buf_blank(dat, 256))
		return 0;

	WARN_ONCE(true, "exp: %02x %02x %02x got: %02x %02x %02x",
			read_ecc[0], read_ecc[1], read_ecc[2],
			calc_ecc[0], calc_ecc[1], calc_ecc[2]);

	WARN_ONCE(true, "Error correction not implemented!");
	return -1;
}

/**
 * nand_read_page_swecc - [REPLACABLE] software ecc based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 * @page:	page number to read
 */
static int pnx8550_nand_read_page_swecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	chip->ecc.read_page_raw(mtd, chip, buf, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);

	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	eccsteps = chip->ecc.steps;
	p = buf;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}
	return 0;
}

/**
 * nand_write_page_swecc - [REPLACABLE] software ecc based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 */
static void pnx8550_nand_write_page_swecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

	/* Software ecc calculation */
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);

	for (i = 0; i < chip->ecc.total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

	chip->ecc.write_page_raw(mtd, chip, buf);
}

#endif

/*
 * Main initialization routine
 */
int __init pnx8550_nand_init(void)
{
	int err = 0;
	struct nand_chip *this;
	u16 *nand_base;
	__iomem u16 *nand_base_iomem;

	this = &pnx_nand.nand;

	/* Initialize structures */
	memset((char *)&pnx_nand, 0, sizeof(struct pnx8550_nand));

	/* Work out address of Nand Flash */
	nand_base = (u16 *) (KSEG1 | (PNX8550_BASE18_ADDR & (~0x7)));

	nand_base = (u16 *) (((u32) nand_base) +
			     ((PNX8550_XIO_SEL0 & PNX8550_XIO_SEL0_OFFSET_MASK)
			      >> PNX8550_XIO_SEL0_OFFSET_SHIFT) * 8 * 1024 *
			     1024);

	pnx_nand.nand_mem = nand_base;
//	nand_base_iomem = ioremap(nand_base, )

	/* Link the private data with the MTD structure */
	pnx_nand.mtd.priv = this;
	pnx_nand.mtd.name = "pnx8550-nand";
	this->chip_delay = 15;
#ifdef CONFIG_MTD_NAND_PNX8550_16BIT
	this->options = NAND_BUSWIDTH_16;
	this->read_byte = pnx8550_nand_read_byte16;
	this->badblock_pattern = &nand16bit_memorybased;
        this->ecc.layout  = &nand16bit_oob_16;
#else
	this->read_byte = pnx8550_nand_read_byte8;
#endif
	this->cmd_ctrl = pnx8550_nand_cmd_ctrl;
	this->cmdfunc = pnx8550_nand_command;
	this->read_word = pnx8550_nand_read_word;
	this->read_buf = pnx8550_nand_read_buf;
	this->write_buf = pnx8550_nand_write_buf;
	this->verify_buf = pnx8550_nand_verify_buf;
	this->dev_ready = pnx8550_nand_dev_ready;
#ifdef CONFIG_MTD_NAND_WINCE_ECC
	this->ecc.layout  = &nand8bit_oob_wince;

	// only way to override the software ecc
	this->ecc.mode      = NAND_ECC_HW;
	this->ecc.calculate = pnx8550_nand_calculate_ecc;
	this->ecc.hwctl     = pnx8550_nand_ecc_hwctrl;
	this->ecc.correct   = pnx8550_nand_correct_data;
	this->ecc.size      = 256;
	this->ecc.bytes     = 3;
	this->ecc.read_page  = pnx8550_nand_read_page_swecc;
	this->ecc.write_page = pnx8550_nand_write_page_swecc;
#else
	this->ecc.mode     = NAND_ECC_SOFT;
#endif
#ifdef CONFIG_MTD_NAND_PNX8550_BADBLOCK
	this->bbt_options = NAND_BBT_USE_FLASH;
	this->bbt_td  = &nand_main_bbt_decr;
	this->bbt_md  = &nand_mirror_bbt_decr;
#endif
	pnx_nand.mtd.owner = THIS_MODULE;
	if ((err = nand_scan_ident(&pnx_nand.mtd, 1, NULL))) {
		pr_warning("No NAND flash devices found\n");
		return err;
	}

	pnx_nand.is64m = this->chipsize >= (64ull << 20);
	if (pnx_nand.is64m) {
		pr_info("Detected flash >= 64MiB, using extended addressing\n");
	}

	/* Scan to find existence of the device */
	if ((err = nand_scan_tail(&pnx_nand.mtd))) {
		pr_warning("NAND flash init failed\n");
		return err;
	}

	mtd_device_parse_register(&pnx_nand.mtd, part_probes, NULL, NULL, 0);

	return err;
}

module_init(pnx8550_nand_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit pnx8550_nand_cleanup(void)
{
	/* Unregister the device */
	del_mtd_device(&pnx8550_mtd);
	kfree(pnx_nand.xfer_buf);
}

module_exit(pnx8550_nand_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Adam Charrett");
MODULE_AUTHOR("Tobias Schramm");
MODULE_DESCRIPTION("Driver for 8/16Bit NAND Flash on the XIO bus for PNX8x50");

