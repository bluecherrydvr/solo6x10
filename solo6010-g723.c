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

#include <linux/kernel.h>
#include <linux/mempool.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <sound/core.h>
#include <sound/initval.h>

#include "solo6010.h"

#define G723_INTR_ORDER		0
#define G723_FDMA_PAGES		32
#define G723_MAX_ITEM		120
#define G723_DEC_CHANNEL	17

/* XXX: Why the descrepancy of outmode mask? */
//#define OUTMODE_MASK		0x300
#define OUTMODE_MASK		0x000
//#define OUTMODE_MASK		0x3FF

#define SAMPLERATE		8000
#define BITRATE			25

static void solo_g723_config(struct solo6010_dev *solo_dev)
{
	int clk_div;

	clk_div = SOLO_CLOCK_MHZ / (SAMPLERATE * (BITRATE * 2) * 2);

	solo_reg_write(solo_dev, SOLO_AUDIO_SAMPLE,
		       SOLO_AUDIO_BITRATE(BITRATE) |
		       SOLO_AUDIO_CLK_DIV(clk_div));

	solo_reg_write(solo_dev, SOLO_AUDIO_FDMA_INTR,
		       SOLO_AUDIO_FDMA_INTERVAL(1) |
		       SOLO_AUDIO_INTR_ORDER(G723_INTR_ORDER) |
		       SOLO_AUDIO_FDMA_BASE((SOLO_G723_EXT_ADDR(solo_dev) >> 16)
		       & 0xffff));

	solo_reg_write(solo_dev, SOLO_AUDIO_CONTROL,
		       SOLO_AUDIO_ENABLE | SOLO_AUDIO_I2S_MODE |
		       SOLO_AUDIO_I2S_MULTI(3) | SOLO_AUDIO_MODE(OUTMODE_MASK));
}

void solo_g723_interrupt(struct solo6010_dev *solo_dev)
{
	solo_reg_write(solo_dev, SOLO_IRQ_STAT, SOLO_IRQ_G723);
}

int solo_g723_init(struct solo6010_dev *solo_dev)
{
	static struct snd_device_ops ops = { NULL };
	struct snd_card *card;
	int ret;

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, "Softlogic",
			      THIS_MODULE, 0, &solo_dev->snd_card);
	if (ret < 0)
		return ret;

	card = solo_dev->snd_card;

	strcpy(card->driver, SOLO6010_NAME);
	strcpy(card->shortname, "SOLO-6010 Audio");
	sprintf(card->longname, "%s on %s IRQ %d", card->shortname,
		pci_name(solo_dev->pdev), solo_dev->pdev->irq);
	snd_card_set_dev(card, &solo_dev->pdev->dev);

	ret = snd_device_new(card, SNDRV_DEV_PCM, solo_dev, &ops);
	if (ret < 0) {
		snd_card_free(card);
		return ret;
	}

	ret = snd_card_register(card);
	if (ret < 0) {
		snd_card_free(card);
		return ret;
	}

	solo_g723_config(solo_dev);
	solo6010_irq_on(solo_dev, SOLO_IRQ_G723);

	return 0;
}

void solo_g723_exit(struct solo6010_dev *solo_dev)
{
	if (solo_dev->snd_card) {
		snd_card_free(solo_dev->snd_card);
		solo_dev->snd_card = NULL;
	}
	solo_reg_write(solo_dev, SOLO_AUDIO_CONTROL, 0);
	solo6010_irq_off(solo_dev, SOLO_IRQ_G723);
}
