// SPDX-License-Identifier: GPL-2.0+
/*
 *  drivers/net/phy/bcm54210pe_ptp.c
 *
 * PTP module for BCM54210PE
 *
 * Authors: Carlos Fernandez
 * License: GPL
 * Copyright (C) 2021 Technica-Electronics GmbH
 */

#include <linux/gpio/consumer.h>
#include <linux/ip.h>                                                                                
#include <linux/net_tstamp.h>
#include <linux/mii.h>
#include <linux/phy.h>                                                                               
#include <linux/ptp_classify.h>
#include <linux/ptp_clock_kernel.h>                                                                  
#include <linux/udp.h>
#include <asm/unaligned.h> 
#include <linux/brcmphy.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/if_ether.h>

#include "bcm54210pe_ptp.h"
#include <linux/delay.h>
#include "bcm-phy-lib.h"

/* PTP header data offsets	*/
#define PTP_CONTROL_OFFSET	32
#define PTP_TSMT_OFFSET 	0
#define PTP_SEQUENCE_ID_OFFSET	30
#define PTP_CLOCK_ID_OFFSET	20
#define PTP_CLOCK_ID_SIZE	8
#define PTP_SEQUENCE_PORT_NUMER_OFFSET  (PTP_CLOCK_ID_OFFSET + PTP_CLOCK_ID_SIZE)


#define EXT_SELECT_REG		0x17
#define EXT_DATA_REG		0x15

#define EXT_ENABLE_REG1		0x17
#define EXT_ENABLE_DATA1	0x0F7E
#define EXT_ENABLE_REG2		0x15
#define EXT_ENABLE_DATA2	0x0000

#define EXT_1588_SLICE_REG	0x0810
#define EXT_1588_SLICE_DATA	0x0101

#define ORIGINAL_TIME_CODE_0 0x0854
#define ORIGINAL_TIME_CODE_1 0x0855
#define ORIGINAL_TIME_CODE_2 0x0856
#define ORIGINAL_TIME_CODE_3 0x0857
#define ORIGINAL_TIME_CODE_4 0x0858

#define TIME_STAMP_REG_0	0x0889
#define TIME_STAMP_REG_1	0x088A
#define TIME_STAMP_REG_2	0x088B
#define TIME_STAMP_REG_3	0x08C4
#define TIME_STAMP_INFO_1	0x088C
#define TIME_STAMP_INFO_2	0x088D
#define INTERRUPT_STATUS_REG	0x085F
#define INTERRUPT_MASK_REG	0x085E
#define EXT_SOFTWARE_RESET	0x0F70
#define EXT_RESET1		0x0001 //RESET
#define EXT_RESET2		0x0000 //NORMAL OPERATION
#define GLOBAL_TIMESYNC_REG	0x0FF5

#define TX_EVENT_MODE_REG	0x0811
#define RX_EVENT_MODE_REG	0x0819
#define TX_TSCAPTURE_ENABLE_REG	0x0821
#define RX_TSCAPTURE_ENABLE_REG	0x0822
#define TXRX_1588_OPTION_REG	0x0823

#define TX_TS_OFFSET_LSB	0x0834
#define TX_TS_OFFSET_MSB	0x0835
#define RX_TS_OFFSET_LSB	0x0844
#define RX_TS_OFFSET_MSB	0x0845
#define NSE_DPPL_NCO_6_REG	0x087F
#define NSE_DPPL_NCO_1_LSB_REG	0x0873
#define NSE_DPPL_NCO_1_MSB_REG	0x0874
#define DPLL_SELECT_REG		0x085b
#define TIMECODE_SEL_REG	0x08C3
#define SHADOW_REG_CONTROL	0x085C
#define SHADOW_REG_LOAD		0x085D

#define PTP_INTERRUPT_REG	0x0D0C

#define CTR_DBG_REG		0x088E
#define HEART_BEAT_REG4		0x08ED
#define HEART_BEAT_REG3		0x08EC
#define HEART_BEAT_REG2		0x0888
#define	HEART_BEAT_REG1		0x0887
#define	HEART_BEAT_REG0		0x0886

#define READ_END_REG		0x0885

#define FIFO_READ_DELAY		100*HZ/1000 /* delay milliseconds in jiffies */

struct bcm54210_skb_info {
	int ptp_type;
	unsigned long tmo;
};

static u64 ts_to_ns(u16 *ts)
{
	u64 ns1;
	u64 ns2;
	
	
	u32 Seconds1 = 0;
	u32 Seconds2 = 0;
	u32 Seconds3 = 0;
	u32 Seconds4 = 0;
	
	/*
	ns = ts[3]; 
	ns = (ns << 48) | ts[2];  
	ns = (ns << 32) | ts[1];
	ns = (ns << 16) | ts[0];
	*/
	
	ns1 = ( ((u64)ts[3]) << 48) + ( ((u64)ts[2]) << 32) + ( ((u64)ts[1]) << 16) + ((u64)ts[0]);
	
	u16 *ptr = NULL;
	
	ptr = (u16 *)&Seconds1;
	*ptr = ts[0]; ptr++; *ptr = ts[1];
		
	ptr = (u16 *)&Seconds2;
	*ptr = ts[1]; ptr++; *ptr = ts[0];
	
	ptr = (u16 *)&Seconds3;
	*ptr = ts[2]; ptr++; *ptr = ts[3];
	
	ptr = (u16 *)&Seconds4;
	*ptr = ts[3]; ptr++; *ptr = ts[2];
	
	ns2 = ((u64)Seconds3 * (u64)1000000000) + Seconds1;
	
	//printk("ts_to_ns - ns1 = %llu, ns2 = %llu, S1 = %d, S2 = %d, S3 = %d, S4 = %d\n", ns1, ns2, Seconds1, Seconds2, Seconds3, Seconds4);
	
	return ns2; 
}



void pkt_hex_dump(struct sk_buff *skb)
{
    size_t len;
    int rowsize = 16;
    int i, l, linelen, remaining;
    int li = 0;
    uint8_t *data, ch; 

    printk("Packet hex dump:\n");
    data = (uint8_t *) skb_mac_header(skb);

    if (skb_is_nonlinear(skb)) {
        len = skb->data_len;
    } else {
        len = skb->len;
    }

    remaining = len;
    for (i = 0; i < len; i += rowsize) {
        printk("%06d\t", li);

        linelen = min(remaining, rowsize);
        remaining -= rowsize;

        for (l = 0; l < linelen; l++) {
            ch = data[l];
            printk(KERN_CONT "%02X ", (uint32_t) ch);
        }

        data += linelen;
        li += 10; 

        printk(KERN_CONT "\n");
    }
}

/* get the start of the ptp header in this skb. Adapted from mv88e6xx/hwtstamp.c*/
static bool get_ptp_header(struct sk_buff *skb, unsigned int type, u8 *hdr )
{
	u8 *data = skb_mac_header(skb);
	unsigned int offset = 0;

	if (type & PTP_CLASS_VLAN)
		offset += VLAN_HLEN;

	switch (type & PTP_CLASS_PMASK) {
	case PTP_CLASS_IPV4:
		offset += ETH_HLEN + IPV4_HLEN(data + offset) + UDP_HLEN;
		break;
	case PTP_CLASS_IPV6:
		offset += ETH_HLEN + IP6_HLEN + UDP_HLEN;
		break;
	case PTP_CLASS_L2:
		offset += ETH_HLEN;
		break;
	default:
		return false;
	}

	/* Ensure that the entire header is present in this packet. */
	if (skb->len + ETH_HLEN < offset + 34)
		return false;

	hdr = data + offset;
	return true;
}

//static inline 
u64 get_time_stamp_from_list(u16 seq_id, struct list_head *list_ptr)
{
	struct bcm54210pe_fifo_item *item; 
	struct list_head *this, *next;
	
	list_for_each_safe(this, next, list_ptr) 
	{
		item = list_entry(this, struct bcm54210pe_fifo_item, list);
		
		if(item->sequence_id == seq_id)
		{ return item->time_stamp; }
	}
	
	return -1;
}

static void tx_timestamp_work(struct work_struct *w)
{
	//printk("########### Kyle Did This - tx_timestamp_work ###########\n");
	
	struct bcm54210pe_private *priv = container_of(w, struct bcm54210pe_private, txts_work);
	
	struct skb_shared_hwtstamps *shhwtstamps = NULL;
	struct sk_buff *skb;
	struct bcm54210pe_fifo_item *item; 
	struct list_head *this, *next;
	struct bcm54210_skb_info *skb_info;

	struct ptp_header *hdr;
	u8 msgtype;
	u16 seq_id;		
		
	skb = skb_dequeue(&priv->tx_queue);
	while(skb != NULL)
	{

		//Kyle - need to figure out what type and msgtype really are. update input thread to get correct values.
		
		int type = PTP_CLASS_V2_L2;
		hdr = ptp_parse_header(skb, type);
		
		if (!hdr)
		{ return; }	
			
		msgtype = ptp_get_msgtype(hdr, type);
		seq_id = be16_to_cpu(hdr->sequence_id);
		
		u64 time_stamp = get_time_stamp_from_list(seq_id, &priv->tx_fifo);
		
		shhwtstamps = skb_hwtstamps(skb);
		if (shhwtstamps)
		{
			memset(shhwtstamps, 0, sizeof(*shhwtstamps));
			shhwtstamps->hwtstamp = ns_to_ktime(time_stamp);			
			skb_complete_tx_timestamp(skb, shhwtstamps);
		
			if(0)
			{ printk("TX Time Stamp Complete - %llu\n", time_stamp); }
		
			return;
		}
				
		skb = skb_dequeue(&priv->tx_queue);
	}
	
	printk("TX Time Stamp Failed\n");
					
	return; 
}
 

static int bcm54210pe_sw_reset(struct phy_device *phydev)
{
	u16 err;
	u16 aux;
        
	err =  bcm_phy_write_exp(phydev, EXT_SOFTWARE_RESET, EXT_RESET1);
	err = bcm_phy_read_exp(phydev, EXT_ENABLE_REG1);
        if (err < 0)
                return err;
        err = bcm_phy_write_exp(phydev, EXT_SOFTWARE_RESET, EXT_RESET2);
	aux = bcm_phy_read_exp(phydev, EXT_SOFTWARE_RESET);
        return err;
}

static void bcm54210pe_get_fifo (struct work_struct *w)
{	
	//printk("########### Kyle Did This - bcm54210pe_get_fifo ###########\n");
	
	struct bcm54210pe_private *priv = container_of(w, struct bcm54210pe_private, fifo_read_work);	
	struct phy_device *phydev = priv->phydev;	
	struct bcm54210pe_fifo_item *item;
	u16 fifo_info_1, fifo_info_2;
	u16 pending_interrupt = 0;
	u8 txrx;
	u8 msgtype; 
	u16 sequence_id;
	u64 time_stamp;
	u16 Time[4];


	int TX_1;
	int TX_2;
	int RX_1;
	int RX_2;
		
	struct list_head *list_ptr = NULL;
	
	pending_interrupt = bcm_phy_read_exp(phydev, INTERRUPT_STATUS_REG); 
	pending_interrupt &= 2;


//	pr_err("Interrupt status 0x%04x", bcm_phy_read_exp(phydev, 0x85f));
//	pr_err("TXRX_1588 counter %04x", bcm_phy_read_exp(phydev, 0x884));
	while(bcm_phy_read_exp(phydev, 0x85f) & 2)
	{		
		// Flush out the FIFO
		bcm_phy_write_exp(phydev, 0x885, 1);
		
		#define TX_TS_OFFSET_LSB	0x0834
		#define TX_TS_OFFSET_MSB	0x0835
		#define RX_TS_OFFSET_LSB	0x0844
		#define RX_TS_OFFSET_MSB	0x0845

		TX_1 = bcm_phy_read_exp(phydev, TX_TS_OFFSET_LSB);
		TX_2 = bcm_phy_read_exp(phydev, TX_TS_OFFSET_MSB);
		RX_1 = bcm_phy_read_exp(phydev, RX_TS_OFFSET_LSB);
		RX_2 = bcm_phy_read_exp(phydev, RX_TS_OFFSET_MSB);
		
		printk("TX_1 = %d - TX_2 = %d - RX_1 = %d - RX_2 = %d\n", TX_1, TX_2, RX_1, RX_2);
		
		Time[3] = bcm_phy_read_exp(phydev, 0x8c4);
		Time[2] = bcm_phy_read_exp(phydev, 0x88b);
		Time[1] = bcm_phy_read_exp(phydev, 0x88a);
		Time[0] = bcm_phy_read_exp(phydev, 0x889);
		fifo_info_1 = bcm_phy_read_exp(phydev, 0x88c);
		fifo_info_2 = bcm_phy_read_exp(phydev, 0x88d);

		bcm_phy_write_exp(phydev, 0x885, 2);
		bcm_phy_write_exp(phydev, 0x885, 0);

		msgtype 	= (u8) ((fifo_info_2 & 0xE000) >> 12); // Kyle - Originally this was >> 13, but I think >> 12 is correct and seems to work
        txrx 		= (u8) ((fifo_info_2 & 0x1000) >> 12); 		
        sequence_id = fifo_info_1;

		time_stamp = ts_to_ns(Time);

		if (txrx) 
		{ list_ptr = &priv->tx_fifo; }
		else
		{
			if(msgtype == 0)
			{ list_ptr = &priv->rx_fifo_sync; }
			else if(msgtype == 2)
			{ list_ptr = &priv->rx_fifo_pdrq; }
		}
		
		item = list_first_entry_or_null(list_ptr, struct bcm54210pe_fifo_item, list);
		if(item != NULL)
		{
			list_del_init(&item->list);
			
			item->msgtype = msgtype; 
			item->sequence_id = sequence_id;
			item->time_stamp = time_stamp;
			
			list_add_tail(&item->list, list_ptr);
			
			if (txrx) 
			{ schedule_work(&priv->txts_work);}
				
			if(0)
			{
				if (txrx) 
				{ printk("Kyle - TIME_STAMP_OUT - Seq_Id = %d, Timestamp = %llu, Message_Type = %d\n", item->sequence_id, item->time_stamp, item->msgtype); }
				else
				{
					if(msgtype == 0)
					{ printk("Kyle - TIME_STAMP_INPUT_SYNC - Seq_Id = %d, Timestamp = %llu, Message Type = %d\n", item->sequence_id, item->time_stamp, item->msgtype); }
					else if(msgtype == 2)
					{ printk("Kyle - TIME_STAMP_INPUT_PDRQ - Seq_Id = %d, Timestamp = %llu, Message Type = %d\n", item->sequence_id, item->time_stamp, item->msgtype); }
					else
					{ printk("Kyle - TIME_STAMP_INPUT Unknown - Seq_Id = %d, Timestamp = %llu, Message Type = %d\n", item->sequence_id, item->time_stamp, item->msgtype); }
				}
			}
		}
			
		// Trigger sync
		bcm_phy_write_exp(phydev, 0x87f, 0xF000);
		bcm_phy_write_exp(phydev, 0x87f, 0xF020);

		// Set Heart beat time read start
		bcm_phy_write_exp(phydev, 0x88e, 0x400);
		// Set read end bit
		bcm_phy_write_exp(phydev, 0x88e, 0x800);
		bcm_phy_write_exp(phydev, 0x88e, 0x000);
	}
	
	udelay(100);
	schedule_work(&priv->fifo_read_work);
	return; 
}

irqreturn_t bcm54210pe_handle_interrupt(int irq, void * phy_dat)
{
	struct phy_device *phydev = phy_dat;
	struct phy_driver *drv = phydev->drv;


	u16 interrupt_status = 0; 


	interrupt_status = bcm_phy_read_exp(phydev, INTERRUPT_STATUS_REG);
	if (!interrupt_status)
		return IRQ_NONE;

	phy_mac_interrupt(phydev);	

	if (phydev->drv->ack_interrupt)
		phydev->drv->ack_interrupt(phydev);

	// did_interrupt() may have cleared the interrupt already
	//if (phy_clear_interrupt(phydev)) {
	//	phy_error(phydev);
	//	return IRQ_NONE;
	//}

	/*if (interrupt_status & 0x0001)
	  bcm54210pe_get_fifo(phydev);*/
	return IRQ_HANDLED;
}


static int bcm54210pe_config_1588(struct phy_device *phydev)
{
	printk("bcm54210pe_config_1588\n");
	
	int err;
	u16 aux = 0xFFFF;

	err = bcm_phy_write_exp(phydev, PTP_INTERRUPT_REG, 0x3c02 );

	err =  bcm_phy_write_exp(phydev, GLOBAL_TIMESYNC_REG, 0x0001); //Enable global timesync register.
	err =  bcm_phy_write_exp(phydev, EXT_1588_SLICE_REG, 0x0101); //ENABLE TX and RX slice 1588
	err =  bcm_phy_write_exp(phydev, TX_EVENT_MODE_REG, 0xFF00); //Add 80bit timestamp + NO CPU MODE in TX
	err =  bcm_phy_write_exp(phydev, RX_EVENT_MODE_REG, 0xFF00); //Add 32+32 bits timestamp + NO CPU mode in RX
	err =  bcm_phy_write_exp(phydev, TIMECODE_SEL_REG, 0x0101); //Select 80 bit counter


	err =  bcm_phy_write_exp(phydev, TX_TSCAPTURE_ENABLE_REG, 0x0001); //Enable timestamp capture in TX 
	err =  bcm_phy_write_exp(phydev, RX_TSCAPTURE_ENABLE_REG, 0x0001); //Enable timestamp capture in RX

	//Load Original Time Code Register
	err =  bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_0, 0x0064);
	err =  bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_1, 0x0064);
	err =  bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_2, 0x0064);
	err =  bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_3, 0x0064);
	err =  bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_4, 0x0064);

	//Enable shadow register
	err = bcm_phy_write_exp(phydev, SHADOW_REG_CONTROL, 0x0000);
	err = bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x07c0);

	//1n ts resolution
	err = bcm_phy_write_exp(phydev, DPLL_SELECT_REG, 0x0160);

	err =  bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF020); //NCO Register 6 => Enable SYNC_OUT pulse train and Internal Syncout ad framesync

	/*printk("DEBUG: GPIO %d IRQ %d\n", 41, gpio_to_irq(41));
	  err = request_threaded_irq(gpio_to_irq(41), NULL, bcm54210pe_handle_interrupt,
	  IRQF_ONESHOT | IRQF_SHARED,
	  phydev_name(phydev), phydev);

	  */
	return err; 
}

static int bcm54210pe_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	u16 Time[5];
	
	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	u64 Jiffies_Before;
	Jiffies_Before = jiffies;

	mutex_lock(&ptp->clock_lock);
	
	// Trigger sync which will capture the heartbeat counter
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF000);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF020);

	// Set Heart beat time read start
	bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x400);
	Time[4] = bcm_phy_read_exp(phydev, HEART_BEAT_REG4);
	Time[3] = bcm_phy_read_exp(phydev, HEART_BEAT_REG3);
	Time[2] = bcm_phy_read_exp(phydev, HEART_BEAT_REG2);
	Time[1] = bcm_phy_read_exp(phydev, HEART_BEAT_REG1);
	Time[0] = bcm_phy_read_exp(phydev, HEART_BEAT_REG0);

	// Set read end bit
	bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x800);
	bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x000);

	mutex_unlock(&ptp->clock_lock);
	
	//u64 Time_Stamp_NS = ptp->chosen->Current_Time;
	u64 Time_Stamp_NS = ts_to_ns(Time);
		
	int Milleseconds = jiffies_to_msecs(jiffies-Jiffies_Before);
	
	printk("bcm54210pe_gettime Elapsed = %d, Current_Time = %llu\n", Milleseconds, Time_Stamp_NS);
		
	//printk("bcm54210pe_gettime - Time_Stamp_NS = %llu\n", Time_Stamp_NS);

	ts->tv_sec = ( (u64)Time_Stamp_NS / (u64)1000000000 );
	ts->tv_nsec = ( (u64)Time_Stamp_NS % (u64)1000000000 );
	
	return 0;
}


static int bcm54210pe_settime(struct ptp_clock_info *info,
		const struct timespec64 *ts)
{
	//Kyle - Should figure / test this out.
	printk("########### Kyle Did This - bcm54210pe_settime ###########\n");
	
	int var[4];

	struct bcm54210pe_ptp *ptp =
		container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	var[4] = (int) (ts->tv_sec & 0xFFFF00000000) >> 32;
	var[3] = (int) (ts->tv_sec & 0x0000FFFF0000) >> 16; 
	var[2] = (int) (ts->tv_sec & 0x00000000FFFF);
	var[1] = (int) (ts->tv_nsec & 0x0000FFFF00000) >> 16;
	var[0] = (int) (ts->tv_nsec & 0x000000000FFFF); 


	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF000);

	//Load Original Time Code Register
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_0, var[0]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_1, var[1]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_2, var[2]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_3, var[3]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_4, var[4]);

	//Enable shadow register
	bcm_phy_write_exp(phydev, SHADOW_REG_CONTROL, 0x0000);
	bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x0400);

	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xE020); //NCO Register 6 => Enable SYNC_OUT pulse train and Internal Syncout ad framesync

	return 0; 
}

static int bcm54210pe_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{
	//Kyle - Should figure / test this out.
	
	printk("########### Kyle Did This - bcm54210pe_adjfine ###########\n");
	
	int err = 0; 
	u64 adj;
	u16 lo, hi;

	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	if (scaled_ppm < 0) {
		err = -EINVAL; 
		goto finish;
	}

	adj = scaled_ppm;
	adj <<= 13;
	adj = div_u64(adj, 15625);

	hi = (adj >> 16);
	lo = adj & 0xffff;

	mutex_lock(&ptp->timeset_lock);

	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xE000);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_1_LSB_REG, lo);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_1_MSB_REG, hi);	

	//Enable shadow register
	bcm_phy_write_exp(phydev, SHADOW_REG_CONTROL, 0x0000);
	bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x0340);
	//Force sync
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xE020); 
finish:
	mutex_unlock(&ptp->timeset_lock);
	return err;

}

static int bcm54210pe_adjtime(struct ptp_clock_info *info, s64 delta)
{
	//Kyle - Should figure / test this out.
	
	printk("########### Kyle Did This - bcm54210pe_adjtime ###########\n");
	
	int err; 
	struct timespec64 ts;
	u64 now;

	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	mutex_lock(&ptp->timeset_lock);

	err = bcm54210pe_gettime(info, &ts);
	if (err < 0)
		goto finish;	

	now = ktime_to_ns(timespec64_to_ktime(ts));
	ts = ns_to_timespec64(now + delta);

	err = bcm54210pe_settime(info, &ts);

finish:
	mutex_unlock(&ptp->timeset_lock);
	return err;
}


bool bcm54210pe_rxtstamp(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
	struct skb_shared_hwtstamps *shhwtstamps = NULL;
	struct bcm54210pe_private *priv = container_of(mii_ts, struct bcm54210pe_private, mii_ts);

	struct list_head *List_Ptr = NULL;
	
	int status = 0;
	
	//////////////////////////
	
	if (!priv->hwts_rx_en)
	{ 
		printk("zzzzzzzzzzzzzzzzzzzzzzzzzzzz bcm54210pe_rxtstamp returning !priv->hwts_rx_en\n");
		return false; 
	}
	
	int count = 0;
	
	struct ptp_header *hdr;
	u8 msgtype;
	u16 seq_id;
	u64 time_stamp;
	
	
	hdr = ptp_parse_header(skb, type);
	
	int PTP_Message_Type = hdr->tsmt & 0x0f;
	char *PTP_Message_Type_String = "UNKNOWN";

	if(PTP_Message_Type == 0)
	{ PTP_Message_Type_String = "SYNC"; }
	else if(PTP_Message_Type == 2)
	{ PTP_Message_Type_String = "PDRQ"; }
		
	
	if (!hdr)
	{ 
		printk("++++++++++++++++++++++++++++++++++ bcm54210pe_rxtstamp returning !hdr\n");
		return 0; 
	}
		
	if( PTP_Message_Type == 0)
	{ List_Ptr = &priv->rx_fifo_sync;}
	else if( PTP_Message_Type == 2)
	{ List_Ptr = &priv->rx_fifo_pdrq;}
	else
	{ 
		printk("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^bcm54210pe_rxtstamp - unknown message type!!!\n"); 
		return false;
	}
		

	msgtype = ptp_get_msgtype(hdr, type);
	seq_id = be16_to_cpu(hdr->sequence_id);
	
	//printk("########### Timestamper - Message Type = %d, Seq_Id = %d ###########\n", msgtype, seq_id);
	
	int x = 0;
	time_stamp = -1;
	
	for(x = 0; x < 10; x++)
	{
		time_stamp = get_time_stamp_from_list(seq_id, List_Ptr);
		if(time_stamp != -1)
		{ break; }
	
		mdelay(10);
	}
	
	shhwtstamps = skb_hwtstamps(skb);
	if (shhwtstamps) 
	{
		memset(shhwtstamps, 0, sizeof(*shhwtstamps));
		shhwtstamps->hwtstamp = ns_to_ktime(time_stamp);  
	}
	
	status = netif_rx_ni(skb);

	if(0)
	{
		if(time_stamp != -1)
		{ printk("TIME_STAMP_MATCH - Seq_Id = %d, Timestamp = %llu, status = %d, Message_Type = %s\n", seq_id, time_stamp, status, PTP_Message_Type_String); }
		else
		{ printk("TIME_STAMP_FAILURE - Seq_Id = %d, Message_Type = %s\n", seq_id, PTP_Message_Type_String); }
	}
	
	return true;
}


void bcm54210pe_txtstamp(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
	struct bcm54210pe_private *device = container_of(mii_ts, struct bcm54210pe_private, mii_ts);

	//printk("DEBUG: %s %d\n",__FUNCTION__, __LINE__);

	switch (device->hwts_tx_en) 
	{
		case HWTSTAMP_TX_ON:
		{
			//printk("DEBUG: %s %d Adding TX to QUEUE\n",__FUNCTION__, __LINE__);
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			skb_queue_tail(&device->tx_queue, skb);
			break;
		}
		case HWTSTAMP_TX_OFF:
		{	
			printk("DEBUG: %s %d HW TX OFF\n",__FUNCTION__, __LINE__); 
		}
		default:
		{
			kfree_skb(skb);
			break;
		}
	}
}

int bcm54210pe_ts_info(struct mii_timestamper *mii_ts, struct ethtool_ts_info *info)
{
	printk("bcm54210pe_ts_info\n");
	
	struct bcm54210pe_private *bcm54210pe = container_of(mii_ts, 
			struct bcm54210pe_private, mii_ts);


	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = ptp_clock_index(bcm54210pe->ptp->ptp_clock);
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON) ;
		/* TODO: (1 << HWTSTAMP_TX_ONESTEP_SYNC);*/
      	info->rx_filters =
                (1 << HWTSTAMP_FILTER_NONE) |
                (1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
                (1 << HWTSTAMP_FILTER_PTP_V2_L4_EVENT);
	return 0;
}

int bcm54210pe_hwtstamp(struct mii_timestamper *mii_ts, struct ifreq *ifr)
{
	printk("bcm54210pe_hwtstamp\n");
	
	struct bcm54210pe_private *device = container_of(mii_ts, struct bcm54210pe_private, mii_ts);

	struct hwtstamp_config cfg;
	u16 txcfg0, rxcfg0;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags) /* reserved for future extensions */
		return -EINVAL;

	if (cfg.tx_type < 0 || cfg.tx_type > HWTSTAMP_TX_ONESTEP_SYNC)
		return -ERANGE;

	device->hwts_tx_en = cfg.tx_type;

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		device->hwts_rx_en = 0;
		device->layer = 0;
		device->version = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		device->hwts_rx_en = 1;
		device->layer = PTP_CLASS_L4;
		device->version = PTP_CLASS_V1;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		device->hwts_rx_en = 1;
		device->layer = PTP_CLASS_L4;
		device->version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		device->hwts_rx_en = 1;
		device->layer = PTP_CLASS_L2;
		device->version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		device->hwts_rx_en = 1;
		device->layer = PTP_CLASS_L4 | PTP_CLASS_L2;
		device->version = PTP_CLASS_V2;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	default:
		return -ERANGE;
	}
	
	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}



static const struct ptp_clock_info bcm54210pe_clk_caps = {
        .owner          = THIS_MODULE,
        .name           = "BCM54210PE_PHC",
        .max_adj        = S32_MAX,
        .n_alarm        = 0,
        .n_pins         = 0,
        .n_ext_ts       = 0,
        .n_per_out      = 0,
        .pps            = 0,
        .adjtime        = &bcm54210pe_adjtime,
        .adjfine        = &bcm54210pe_adjfine,
        .gettime64      = &bcm54210pe_gettime,
        .settime64      = &bcm54210pe_settime,
};


int bcm54210pe_probe(struct phy_device *phydev)
{	
	printk("bcm54210pe_probe\n");
	
	int err = 0, i;
	struct bcm54210pe_ptp *ptp;
        struct bcm54210pe_private *bcm54210pe;

	bcm54210pe_sw_reset(phydev);
	bcm54210pe_config_1588(phydev);
	bcm54210pe = kzalloc(sizeof(struct bcm54210pe_private), GFP_KERNEL);
        if (!bcm54210pe) {
		err = -ENOMEM;
                goto error;
	}

	ptp = kzalloc(sizeof(struct bcm54210pe_ptp), GFP_KERNEL);
        if (!ptp) {
		err = -ENOMEM;
                goto error;
	}

    bcm54210pe->phydev = phydev;

	bcm54210pe->ptp = ptp;

	skb_queue_head_init(&bcm54210pe->tx_queue);
	bcm54210pe->mii_ts.rxtstamp =  bcm54210pe_rxtstamp;
	bcm54210pe->mii_ts.txtstamp = bcm54210pe_txtstamp;
	bcm54210pe->mii_ts.hwtstamp = bcm54210pe_hwtstamp;
	bcm54210pe->mii_ts.ts_info  = bcm54210pe_ts_info;


	phydev->mii_ts = &bcm54210pe->mii_ts;
	
	INIT_WORK(&bcm54210pe->txts_work, tx_timestamp_work);
	INIT_WORK(&bcm54210pe->fifo_read_work, bcm54210pe_get_fifo);
	
	INIT_LIST_HEAD(&bcm54210pe->tx_fifo);
	INIT_LIST_HEAD(&bcm54210pe->rx_fifo_pdrq);
	INIT_LIST_HEAD(&bcm54210pe->rx_fifo_sync);
	
	for (i = 0; i < MAX_POOL_SIZE; i++)
	{
		list_add(&bcm54210pe->ts_tx_data[i].list, &bcm54210pe->tx_fifo);
		list_add(&bcm54210pe->ts_rx_data_pdrq[i].list, &bcm54210pe->rx_fifo_pdrq);
		list_add(&bcm54210pe->ts_rx_data_sync[i].list, &bcm54210pe->rx_fifo_sync);
	}
	
	memcpy(&bcm54210pe->ptp->caps, &bcm54210pe_clk_caps, sizeof(bcm54210pe_clk_caps));
	mutex_init(&bcm54210pe->ptp->clock_lock);
	mutex_init(&bcm54210pe->ptp->timeset_lock);
	ptp->chosen = bcm54210pe;
        phydev->priv = bcm54210pe;
	ptp->caps.owner = THIS_MODULE;

	bcm54210pe->ptp->ptp_clock = ptp_clock_register(&bcm54210pe->ptp->caps,
			&phydev->mdio.dev);
	if (IS_ERR(bcm54210pe->ptp->ptp_clock)) {
                        err = PTR_ERR(bcm54210pe->ptp->ptp_clock);
                        goto error;
	}
	schedule_work(&bcm54210pe->fifo_read_work); //start fifo work

error:
	return err;
}
