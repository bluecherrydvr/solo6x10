/*
 * Copyright (C) 2010 Ben Collins <bcollins@bluecherry.net>
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
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __SOLO6010_REGISTERS_H
#define __SOLO6010_REGISTERS_H

#include "solo6010-offsets.h"

/* Global 6010 system configuration */
#define SOLO_SYS_CFG			0x0000
#define   SOLO_SYS_CFG_FOUT_EN		0x00000001
#define   SOLO_SYS_CFG_PLL_BYPASS	0x00000002
#define   SOLO_SYS_CFG_PLL_PWDN		0x00000004
#define   SOLO_SYS_CFG_OUTDIV(__n)	(((__n) & 0x003) << 3)
#define   SOLO_SYS_CFG_FEEDBACKDIV(__n)	(((__n) & 0x1ff) << 5)
#define   SOLO_SYS_CFG_INPUTDIV(__n)	(((__n) & 0x01f) << 14)
#define   SOLO_SYS_CFG_CLOCK_DIV	0x00080000
#define   SOLO_SYS_CFG_NCLK_DELAY(__n)	(((__n) & 0x003) << 24)
#define   SOLO_SYS_CFG_PCLK_DELAY(__n)	(((__n) & 0x00f) << 26)
#define   SOLO_SYS_CFG_SDRAM64BIT	0x40000000
#define   SOLO_SYS_CFG_RESET		0x80000000

/* DMA engine control and values */
#define SOLO_DMA_CTRL				0x0004
#define   SOLO_DMA_CTRL_LATENCY(__n)    	(((__n) & 0x0003) << 0)
#define   SOLO_DMA_CTRL_READ_CLK_SELECT		0x00000004
#define   SOLO_DMA_CTRL_READ_DATA_SELECT	0x00000008
#define   SOLO_DMA_CTRL_STROBE_SELECT		0x00000010
#define   SOLO_DMA_CTRL_SDRAM_CLK_INVERT	0x00000020
/* 0=16/32MB, 1=32/64MB, 2=64/128MB, 3=128/256MB */
#define   SOLO_DMA_CTRL_SDRAM_SIZE(__n)		(((__n) & 0x0003) << 6)
#define   SOLO_DMA_CTRL_REFRESH_CYCLE(__n)	(((__n) & 0xffff) << 8)

/* Interrupt related registers and values */
#define SOLO_IRQ_STAT			0x0010
#define SOLO_IRQ_ENABLE			0x0014
#define   SOLO_IRQ_ENC			0x00000001
#define   SOLO_IRQ_DEC			0x00000002
#define   SOLO_IRQ_G723			0x00000004
#define   SOLO_IRQ_UART_0		0x00000008
#define   SOLO_IRQ_UART_1		0x00000010
#define   SOLO_IRQ_IIC			0x00000020
#define   SOLO_IRQ_SPI			0x00000040
#define   SOLO_IRQ_PS2_0		0x00000080
#define   SOLO_IRQ_PS2_1		0x00000100
#define   SOLO_IRQ_PCI_ERR		0x00000200
#define   SOLO_IRQ_ATA_DIR		0x00000400
#define   SOLO_IRQ_ATA_CMD		0x00000800
#define   SOLO_IRQ_MOTION		0x00001000
#define   SOLO_IRQ_VID_IN		0x00002000
#define   SOLO_IRQ_VID_LOSS		0x00004000
#define   SOLO_IRQ_GPIO			0x00008000
#define   SOLO_IRQ_P2M(__n)		(1 << (16 + ((__n) & 0xf)))
#define   SOLO_IRQ_P2M_0		0x00010000
#define   SOLO_IRQ_P2M_1		0x00020000
#define   SOLO_IRQ_P2M_2		0x00040000
#define   SOLO_IRQ_P2M_3		0x00080000

#define SOLO_CHIP_OPTION		0x001c
#define   SOLO_CHIP_ID_MASK		0x00000007

#define SOLO_PCI_ERR			0x0070
#define   SOLO_PCI_ERR_FATAL		0x00000001
#define   SOLO_PCI_ERR_PARITY		0x00000002
#define   SOLO_PCI_ERR_TARGET		0x00000004
#define   SOLO_PCI_ERR_TIMEOUT		0x00000008
#define   SOLO_PCI_ERR_P2M		0x00000010
#define   SOLO_PCI_ERR_ATA		0x00000020
#define   SOLO_PCI_ERR_P2M_DESC		0x00000040
#define   SOLO_PCI_ERR_FSM0(__s)	(((__s) >> 16) & 0x0f)
#define   SOLO_PCI_ERR_FSM1(__s)	(((__s) >> 20) & 0x0f)
#define   SOLO_PCI_ERR_FSM2(__s)	(((__s) >> 24) & 0x1f)

/* P2M config register and values */
#define SOLO_P2M_CONFIG(__n)		(((__n) * 0x20) + 0x0080)
#define   SOLO_P2M_DESC_MODE		0x00000001
#define   SOLO_P2M_DESC_INTR_OPT	0x00000002
#define   SOLO_P2M_PCI_MASTER_MODE	0x00000004
#define   SOLO_P2M_UV_SWAP		0x00000008
/* 0:RGB->RGB, 1:BGR->RGB */
#define   SOLO_P2M_CSC_16BIT_565	0x00000010
#define   SOLO_P2M_CSC_BYTE_REORDER	0x00000020
#define   SOLO_P2M_DMA_INTERVAL(__n)	(((__n) & 0x1f) << 6)

/* P2M descriptor address */
#define SOLO_P2M_DES_ADR(__n)		(((__n) * 0x20) + 0x0084)

/* P2M control register */
#define SOLO_P2M_CONTROL(__n)		(((__n) * 0x20) + 0x0090)
#define   SOLO_P2M_TRANS_ON		0x00000001
#define   SOLO_P2M_WRITE		0x00000002
#define   SOLO_P2M_INTERRUPT_REQ	0x00000004
#define   SOLO_P2M_CSC_ON		0x00000008
/* 0:Y[0]<-0(OFF), 1:Y[0]<-1(ON), 2:Y[0]<-G[0], 3:Y[0]<-Bit[15] */
#define   SOLO_P2M_ALPHA_MODE(__n)	(((__n) & 0x003) << 4)
/* 0:24-bit, 1:16-bit */
#define   SOLO_P2M_CSC_16BIT		0x00000040
/* 0:512, 1:256, 2:128, 3:64, 4:32, 5:128(2page) */
#define   SOLO_P2M_BURST_SIZE(__n)	(((__n) & 0x007) << 7)
#define     SOLO_P2M_BURST_512		0
#define     SOLO_P2M_BURST_256		1
#define     SOLO_P2M_BURST_128		2
#define     SOLO_P2M_BURST_64		3
#define     SOLO_P2M_BURST_32		4
#define     SOLO_P2M_BURST_128_2PAGE	5
#define   SOLO_P2M_REPEAT(__n)		(((__n) & 0x3ff) << 10)
#define   SOLO_P2M_PCI_INC(__n)		(((__n) & 0xfff) << 20)

#define SOLO_P2M_EXT_CFG(__n)		(((__n) * 0x20) + 0x0094)
#define   SOLO_P2M_COPY_SIZE(__n)	(((__n) & 0xfffff) << 0)
#define   SOLO_P2M_EXT_INC(__n)		(((__n) & 0x00fff) << 20)

/* P2M address and buffer */
#define SOLO_P2M_TAR_ADR(__n)		(((__n) * 0x20) + 0x0098)
#define SOLO_P2M_EXT_ADR(__n)		(((__n) * 0x20) + 0x009c)
#define SOLO_P2M_BUFFER(__i)		(((__i) * 4) + 0x2000)

/* P2M address and buffer */
#define SOLO_P2M_TAR_ADR(__n)		(((__n) * 0x20) + 0x0098)
#define SOLO_P2M_EXT_ADR(__n)		(((__n) * 0x20) + 0x009c)
#define SOLO_P2M_BUFFER(__i)		(((__i) * 4) + 0x2000)

/* Video in registers and values */
#define SOLO_VI_CH_ENA			0x010C
#define SOLO_VI_STATUS0			0x0128
#define   SOLO_VI_STATUS0_PAGE(__n)	((__n) & 0x07)
#define SOLO_VI_STATUS1			0x012C

#define SOLO_VI_WIN_CTRL0(__ch)		(((__ch) * 4) + 0x0180)
#define   SOLO_VI_WIN_CHANNEL(__n)	(((__n) & 0x000f) << 28)
#define   SOLO_VI_WIN_PIP		0x08000000
#define   SOLO_VI_WIN_SCALE(__n)        (((__n) & 0x0007) << 24)
#define   SOLO_VI_WIN_SX(__n)		((((__n) >> 2) & 0x0fff) << 12)
#define   SOLO_VI_WIN_EX(__n)		((((__n) >> 2) & 0x0fff) << 0)
#define SOLO_VI_WIN_CTRL1(__ch)		(((__ch) * 4) + 0x01c0)
#define   SOLO_VI_WIN_SY(__n)		(((__n) & 0x0fff) << 12)
#define   SOLO_VI_WIN_EY(__n)		(((__n) & 0x0fff) << 0)

#define SOLO_VI_WIN_ON(__ch)		(((__ch) * 4) + 0x0200)
#define   SOLO_VI_WIN_LIVE_ON		0x00000001
#define   SOLO_VI_WIN_PB_ON		0x00000002
#define SOLO_VI_WIN_SW			0x0240

/* Video Out Encoder */
#define SOLO_VO_FMT_ENC			0x0300
#define   SOLO_VO_SCAN_MODE_PROGRESSIVE	0x80000000
#define   SOLO_VO_FMT_TYPE_PAL		0x40000000
#define   SOLO_VO_USER_SET		0x20000000
#define   SOLO_VO_FI_CHANGE		0x00100000
#define   SOLO_VO_USER_COLOR_SET_VSYNC	0x00080000
#define   SOLO_VO_USER_COLOR_SET_HSYNC	0x00040000
#define   SOLO_VO_USER_COLOR_SET_NAV	0x00020000
#define   SOLO_VO_USER_COLOR_SET_NAH	0x00010000
#define   SOLO_VO_NA_COLOR_Y(__n)       (((__n) & 0xff) << 8)
#define   SOLO_VO_NA_COLOR_CB(__n)	((((__n) / 16) & 0x0f) << 4) 
#define   SOLO_VO_NA_COLOR_CR(__n)	((((__n) / 16) & 0x0f) << 0) 

#define SOLO_VO_ACT_H			0x0304
#define   SOLO_VO_H_BLANK(__n)		(((__n) & 0x3ff) << 22) 
#define   SOLO_VO_H_START(__n)		(((__n) & 0x7ff) << 11)
#define   SOLO_VO_H_STOP(__n)		(((__n) & 0x7ff) << 0)

#define SOLO_VO_ACT_V			0x0308
#define   SOLO_VO_V_BLANK(__n)		(((__n) & 0x3ff) << 22)
#define   SOLO_VO_V_START(__n)		(((__n) & 0x7ff) << 11)
#define   SOLO_VO_V_STOP(__n)		(((__n) & 0x7ff) << 0)

#define SOLO_VO_RANGE_HV		0x030C
#define   SOLO_VO_SYNC_INVERT		0x01000000
#define   SOLO_VO_HSYNC_INVERT		0x00800000
#define   SOLO_VO_VSYNC_INVERT		0x00400000
#define   SOLO_VO_H_LEN(__n)		(((__n) & 0x7ff) << 11)
#define   SOLO_VO_V_LEN(__n)		(((__n) & 0x7ff) << 0)

#define SOLO_VO_DISP_CTRL		0x0310
#define   SOLO_VO_DISP_ON		0x80000000
#define   SOLO_VO_DISP_ERASE_COUNT(__n)	(((__n) & 0x000f) << 24)
#define   SOLO_VO_DISP_DOUBLE_SCAN	0x00400000
#define   SOLO_VO_DISP_SINGLE_PAGE	0x00200000
#define   SOLO_VO_DISP_BASE(__n)	(((__n) >> 16) & 0xffff)

#define SOLO_VO_DISP_ERASE		0x0314
#define   SOLO_VO_DISP_ERASE_ON		0x00000001

#define SOLO_VO_BKG_COLOR		0x0320
#define SOLO_VO_BG_YUV(__y, __u, __v) \
	((((__y) & 0xff) << 16) | (((__u) & 0xff) << 8) | (((__v) & 0xff)))

#define SOLO_VO_BORDER_LINE_COLOR	0x0330
#define SOLO_VO_BORDER_FILL_COLOR	0x0334

/* I2C config and control */
#define SOLO_IIC_CFG			0x0b20
#define   SOLO_IIC_PRESCALE(__n)	(((__n) & 0xff) << 0)
#define   SOLO_IIC_ENABLE		0x00000100

#define SOLO_IIC_CTRL			0x0b24
#define   SOLO_IIC_WRITE		0x00000001
#define   SOLO_IIC_READ			0x00000002
#define   SOLO_IIC_STOP			0x00000004
#define   SOLO_IIC_START		0x00000008
#define   SOLO_IIC_ACK_EN		0x00000010
#define   SOLO_IIC_CH_SET(__n)		(((__n) & 0x7) << 5)
#define   SOLO_IIC_CH_GET(__s)		(((__s) >> 5) & 0x7)
#define   SOLO_IIC_STATE_TRNS		0x00010000
#define   SOLO_IIC_STATE_SIG_ERR	0x00020000
#define   SOLO_IIC_STATE_BUSY		0x00040000
#define   SOLO_IIC_STATE_RX_ACK		0x00080000
#define   SOLO_IIC_AUTO_CLEAR		0x00100000

#define SOLO_IIC_TXD			0x0b28
#define SOLO_IIC_RXD			0x0b2c

/* Timers */
#define SOLO_TIMER_CLOCK_NUM		0x0be0
#define SOLO_TIMER_WATCHDOG		0x0be4
#define SOLO_TIMER_USEC			0x0be8
#define SOLO_TIMER_SEC			0x0bec

/* Watchdog register and status */
#define SOLO_WATCHDOG			0x0BE4
#define SOLO_WATCHDOG_ENABLE		0x00000100
#define SOLO_WATCHDOG_TIME(__n)		(((__n) & 0xff) << 0)

#endif /* __SOLO6010_REGISTERS_H */
