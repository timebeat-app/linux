// SPDX-License-Identifier: GPL-2.0+
/*
 *  drivers/net/phy/bcm54210pe_ptp.c
 *
 * PTP module for BCM54210PE
 *
 * Authors: Carlos Fernandez, Kyle Judd, Lasse Johnsen
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

#include <linux/sched.h>


#include "bcm54210pe_ptp.h"
#include <linux/delay.h>
#include "bcm-phy-lib.h"

/* PTP header data offsets	*/
#define PTP_CONTROL_OFFSET		32
#define PTP_TSMT_OFFSET 		0
#define PTP_SEQUENCE_ID_OFFSET	30
#define PTP_CLOCK_ID_OFFSET		20
#define PTP_CLOCK_ID_SIZE		8
#define PTP_SEQUENCE_PORT_NUMER_OFFSET  (PTP_CLOCK_ID_OFFSET + PTP_CLOCK_ID_SIZE)


#define EXT_SELECT_REG			0x17
#define EXT_DATA_REG			0x15

#define EXT_ENABLE_REG1			0x17
#define EXT_ENABLE_DATA1		0x0F7E
#define EXT_ENABLE_REG2			0x15
#define EXT_ENABLE_DATA2		0x0000

#define EXT_1588_SLICE_REG		0x0810
#define EXT_1588_SLICE_DATA		0x0101

#define ORIGINAL_TIME_CODE_0 	0x0854
#define ORIGINAL_TIME_CODE_1 	0x0855
#define ORIGINAL_TIME_CODE_2 	0x0856
#define ORIGINAL_TIME_CODE_3 	0x0857
#define ORIGINAL_TIME_CODE_4 	0x0858

#define TIME_STAMP_REG_0		0x0889
#define TIME_STAMP_REG_1		0x088A
#define TIME_STAMP_REG_2		0x088B
#define TIME_STAMP_REG_3		0x08C4
#define TIME_STAMP_INFO_1		0x088C
#define TIME_STAMP_INFO_2		0x088D
#define INTERRUPT_STATUS_REG	0x085F
#define INTERRUPT_MASK_REG		0x085E
#define EXT_SOFTWARE_RESET		0x0F70
#define EXT_RESET1				0x0001 //RESET
#define EXT_RESET2				0x0000 //NORMAL OPERATION
#define GLOBAL_TIMESYNC_REG		0x0FF5

#define TX_EVENT_MODE_REG		0x0811
#define RX_EVENT_MODE_REG		0x0819
#define TX_TSCAPTURE_ENABLE_REG	0x0821
#define RX_TSCAPTURE_ENABLE_REG	0x0822
#define TXRX_1588_OPTION_REG	0x0823

#define TX_TS_OFFSET_LSB		0x0834
#define TX_TS_OFFSET_MSB		0x0835
#define RX_TS_OFFSET_LSB		0x0844
#define RX_TS_OFFSET_MSB		0x0845
#define NSE_DPPL_NCO_6_REG		0x087F
#define NSE_DPPL_NCO_4_REG		0x087B
#define NSE_DPPL_NCO_1_LSB_REG		0x0873
#define NSE_DPPL_NCO_1_MSB_REG		0x0874

#define NSE_DPPL_NCO_2_0_REG		0x0875
#define NSE_DPPL_NCO_2_1_REG		0x0876
#define NSE_DPPL_NCO_2_2_REG		0x0877

#define DPLL_SELECT_REG			0x085b
#define TIMECODE_SEL_REG		0x08C3
#define SHADOW_REG_CONTROL		0x085C
#define SHADOW_REG_LOAD			0x085D

#define PTP_INTERRUPT_REG		0x0D0C

#define CTR_DBG_REG				0x088E
#define HEART_BEAT_REG4			0x08ED
#define HEART_BEAT_REG3			0x08EC
#define HEART_BEAT_REG2			0x0888
#define	HEART_BEAT_REG1			0x0887
#define	HEART_BEAT_REG0			0x0886
	
#define READ_END_REG			0x0885


#define FIFO_READ_DELAY			100*HZ/1000 /* delay milliseconds in jiffies */

int force_logging = 0;

#define LOG_ENABLED 		1

#define LOG_INPUT_ 			1
#define LOG_MATCH_OUTPUT_ 	1
#define LOG_MATCH_INPUT_ 	1

#define LOG_INPUT 			((LOG_ENABLED & LOG_INPUT_) || force_logging)
#define LOG_MATCH_OUTPUT 	((LOG_ENABLED & LOG_MATCH_OUTPUT_) || force_logging)
#define LOG_MATCH_INPUT 	((LOG_ENABLED & LOG_MATCH_INPUT_)  || force_logging)

#define TIMESTAMP_NO_VALUE 	(0xFFFFFFFFFFFFFFFF)

int time_stamp_thread_cpu = 2;
int skb_input_thread_cpu = 1;


#define GET_TXRX_NAME(TXRX) TXRX==0 ? "RX" : "TX"

char *message_name_unknown = "UNKNOWN";
char *message_names[] = {"SYNC(0)         ", "PD_REQUEST(1)   ", "PD_REQUEST(2)   ", "PD_RESPONSE(3)  "};
#define GET_MESSAGE_NAME(MESSAGE_TYPE) (MESSAGE_TYPE < 0 || MESSAGE_TYPE >= sizeof(message_names) ) ? message_name_unknown : message_names[MESSAGE_TYPE]

char *event_name_unknown = "UNKNOWN";
char *event_names[] = {"TIMESTAMP_INPUT ", "TIMESTAMP_MATCH "};
#define GET_EVENT_NAME(EVENT_TYPE) (EVENT_TYPE < 0 || EVENT_TYPE >= sizeof(event_names) ) ? event_name_unknown : event_names[EVENT_TYPE]

#define POLL_INTERVAL_USECS (100)

void print_timestamp_message(int EVENT_TYPE, int TXRX, int MESSAGE_TYPE, int SEQUENCE_ID, int STATUS, uint64_t TIMESTAMP, int ERROR)
{	
	if(ERROR != 0)
	{ printk ("******** TIMESTAMP ERROR ********************************************************************\n"); }	

	printk("%s - %s - Type = %s - Seq_Id = %5d - status = %d - Timestamp = %llu\n",
			GET_EVENT_NAME(EVENT_TYPE), GET_TXRX_NAME(TXRX), GET_MESSAGE_NAME(MESSAGE_TYPE), SEQUENCE_ID, STATUS, TIMESTAMP);
	
	if(ERROR != 0)
	{ printk ("*********************************************************************************************\n"); }
	  
}
	
void print_function_message(char *FUNCTION_NAME, char *PARAM_NAME, s64 PARAM_VALUE)
{
	if(FUNCTION_NAME == NULL)
	{ return; }

	printk ("_____________________________________________________________________________________________\n");
	if(PARAM_NAME == NULL)
	{ printk ("%s\n", FUNCTION_NAME); }
	else
	{ printk ("%s - %s = %lld\n",FUNCTION_NAME, PARAM_NAME, PARAM_VALUE); }
	printk ("_____________________________________________________________________________________________\n");
	
}

static u64 ts_to_ns(u16 *ts)
{
	u64 ret_val = 0;;
	u32 nanoseconds = 0;
	u32 seconds = 0;
	
	u16 *ptr = NULL;
	
	ptr = (u16 *)&nanoseconds;
	*ptr = ts[0]; ptr++; *ptr = ts[1];
	
	ptr = (u16 *)&seconds;
	*ptr = ts[2]; ptr++; *ptr = ts[3];
	
	ret_val = ((u64)seconds * (u64)1000000000) + nanoseconds;
			
	//Kyle - Could Try using KTime and  ktime_set here.
	//Kyle - KTime, timespec64 use signed types in calculations, which may be throwing off calculations somewhere else.
	
	return ret_val; 
}

void pkt_hex_dump(struct sk_buff *skb)
{
    size_t len;
    int rowsize = 16;
    int i, l, linelen, remaining;
    int li = 0;
    uint8_t *data, ch; 

    data = (uint8_t *) skb_mac_header(skb);

    if (skb_is_nonlinear(skb)) {
        len = skb->data_len;
    } else {
        len = skb->len;
    }

    printk("Packet hex dump - packet length = %d\n", len);
	
    remaining = len;
    for (i = 0; i < len; i += rowsize) {
        printk("%06d\t", li);

        linelen = min(remaining, rowsize);
        remaining -= rowsize;

        for (l = 0; l < linelen; l++) {
            ch = data[l];
            printk("%02X ", (uint32_t) ch);
        }

        data += linelen;
        li += 10; 

        printk("\n");
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

static inline u64 get_time_stamp_from_buffers(u8 txrx, u8 message_type, u16 seq_id, int print, struct bcm54210pe_private *priv)
{
	struct bcm54210pe_circular_buffer_item *item; 
	struct list_head *this, *next;
	
	u8 index = (txrx * 4) + message_type;
	
	if(index >= CIRCULAR_BUFFER_COUNT)
	{ return TIMESTAMP_NO_VALUE; }

	list_for_each_safe(this, next, &priv->circular_buffers[index]) 
	{
		item = list_entry(this, struct bcm54210pe_circular_buffer_item, list);
		
		if(print)
		{ printk("seq_id = %d, item->sequence_id = %d\n", seq_id, item->sequence_id); }
		
		if(item->sequence_id == seq_id && item->is_valid == 1)
		{
			item->is_valid = 0;
			return item->time_stamp; 
		}
	}

	//K.J. - It is not necessarily an error to reach this point, it just means that there is no match
	//		 in the list YET. Calling functions can call this function multiple times waiting for a 
	//		 timestamp value to arrive. Calling functions should log errors if necessary.
	
	return TIMESTAMP_NO_VALUE;
}

static void read_txrx_timestamp_thread(struct work_struct *w)
{	

	struct delayed_work *dw = (struct delayed_work *)w;
	struct bcm54210pe_private *priv = container_of(dw, struct bcm54210pe_private, fifo_read_work_delayed);	
	struct phy_device *phydev = priv->phydev;	
	struct bcm54210pe_circular_buffer_item *item;
	u16 fifo_info_1, fifo_info_2;
	u16 pending_interrupt = 0;
	u8 txrx;
	u8 msg_type; 
	u16 sequence_id;
	u64 time_stamp;
	u16 Time[4];

	while(bcm_phy_read_exp(phydev, INTERRUPT_STATUS_REG) & 2)
	{
		
		// Flush out the FIFO
		bcm_phy_write_exp(phydev, READ_END_REG, 1);
		
		Time[3] = bcm_phy_read_exp(phydev, TIME_STAMP_REG_3);
		Time[2] = bcm_phy_read_exp(phydev, TIME_STAMP_REG_2);
		Time[1] = bcm_phy_read_exp(phydev, TIME_STAMP_REG_1);
		Time[0] = bcm_phy_read_exp(phydev, TIME_STAMP_REG_0);
				
		fifo_info_1 = bcm_phy_read_exp(phydev, TIME_STAMP_INFO_1);
		fifo_info_2 = bcm_phy_read_exp(phydev, TIME_STAMP_INFO_2);

		bcm_phy_write_exp(phydev, READ_END_REG, 2);
		bcm_phy_write_exp(phydev, READ_END_REG, 0);

		msg_type 	= (u8) ((fifo_info_2 & 0xF000) >> 12); 
        txrx 		= (u8) ((fifo_info_2 & 0x0800) >> 11); 		

		char *TXRX = GET_TXRX_NAME(txrx);
		char *MSG = GET_MESSAGE_NAME(msg_type);
		
        sequence_id = fifo_info_1;

		time_stamp = ts_to_ns(Time);
				
			
		if(LOG_INPUT)
		{ print_timestamp_message(0, txrx, msg_type, sequence_id, 0, time_stamp, 0); }
		
		u8 index = (txrx * 4) + msg_type;
	
		if(index < CIRCULAR_BUFFER_COUNT)
		{ item = list_first_entry_or_null(&priv->circular_buffers[index], struct bcm54210pe_circular_buffer_item, list); }
		else
		{ printk("NEW TIMESTAMP - Error 1\n"); }
	
		if(item != NULL)
		{
			list_del_init(&item->list);
			
			item->msg_type = msg_type; 
			item->sequence_id = sequence_id;
			item->time_stamp = time_stamp;
			item->is_valid = 1;
			
			list_add_tail(&item->list, &priv->circular_buffers[index]);
			
			if (txrx) 
			{ schedule_work_on(time_stamp_thread_cpu, &priv->txts_work);}			
		}
	}
	
	schedule_delayed_work_on(time_stamp_thread_cpu, &priv->fifo_read_work_delayed, usecs_to_jiffies(POLL_INTERVAL_USECS));
	
	/*
	int err = request_threaded_irq(gpio_to_irq(41), bcm54210pe_handle_interrupt, bcm54210pe_handle_interrupt_thread,
								IRQF_ONESHOT | IRQF_SHARED,
								phydev_name(phydev), phydev);
	*/
								
	return; 
}
static void match_tx_timestamp_thread(struct work_struct *w)
{
	struct bcm54210pe_private *priv = container_of(w, struct bcm54210pe_private, txts_work);
	
	struct skb_shared_hwtstamps *shhwtstamps = NULL;
	struct sk_buff *skb;
	struct bcm54210pe_circular_buffer_item *item; 
	struct list_head *this, *next;
	struct bcm54210_skb_info *skb_info;

	struct ptp_header *hdr;
	u8 msg_type;
	u16 sequence_id;		
				
	struct list_head *list_ptr = NULL;
		
	skb = skb_dequeue(&priv->tx_skb_queue);
	
	int match_found = 0;
	
	u64 time_stamp = TIMESTAMP_NO_VALUE;
	
	while(skb != NULL)
	{
		//Kyle - may need to timestamp packets with TIMESTAMP_NO_VALUE if no match found.
		
		int type = ptp_classify_raw(skb);		
		hdr = ptp_parse_header(skb, type);
		
		if (!hdr)
		{ return; }	
			
		msg_type = ptp_get_msgtype(hdr, type);
		sequence_id = be16_to_cpu(hdr->sequence_id);
		
		char *TXRX = GET_TXRX_NAME(1);
		char *MSG = GET_MESSAGE_NAME(msg_type);
		
		//Kyle - we may need to check multiple times like in input.
		time_stamp = get_time_stamp_from_buffers(1, msg_type, sequence_id, 0, priv);

		shhwtstamps = skb_hwtstamps(skb);
		
		if (shhwtstamps)
		{
			int status = 0;
			memset(shhwtstamps, 0, sizeof(*shhwtstamps));
			shhwtstamps->hwtstamp = ns_to_ktime(time_stamp);			
			
			//Kyle - check if skb_tstamp_tx is still the right function to use, documentation suggests other, but real drivers use skb_tstamp_tx
			skb_tstamp_tx(skb, shhwtstamps);
			
			match_found++;
			
			if(LOG_MATCH_OUTPUT)
			{ print_timestamp_message(1, 1, msg_type, sequence_id, 0, time_stamp, 0); }
		
		}
		//K.J. - consume_skb release the reference to the clone created in bcmgenet
		consume_skb(skb);
		skb = skb_dequeue(&priv->tx_skb_queue);
	}
	
	if(match_found == 0)
	{ print_timestamp_message(1, 1, msg_type, sequence_id, 0, time_stamp, 1); }
					
	return; 
}

bool match_rx_timestamp_callback(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
	struct skb_shared_hwtstamps *shhwtstamps = NULL;
	struct bcm54210pe_private *priv = container_of(mii_ts, struct bcm54210pe_private, mii_ts);
	
	int status = 0;
	
	//////////////////////////
	
	if (!priv->hwts_rx_en)  
	{ return false; } //Kyle - May need to timestamp with TIMESTAMP_NO_VALUE even if we return early.
	
	int count = 0;
	
	struct ptp_header *hdr;
	u8 msg_type;
	u16 sequence_id;
	u64 time_stamp;
		
	hdr = ptp_parse_header(skb, type);
	
	if (hdr == NULL)
	{ return false; } //Kyle - May need to timestamp with TIMESTAMP_NO_VALUE even if we return early.
	
	msg_type = ptp_get_msgtype(hdr, type);
	sequence_id = be16_to_cpu(hdr->sequence_id);
		
	char *message_name = GET_MESSAGE_NAME(msg_type);
		
	time_stamp = TIMESTAMP_NO_VALUE;
	
	//Kyle - figure out best way to do this...
	//Kyle - mdelay(10) may be too much, user level apps aren't waiting that long to receive timestamps.
	int x = 0;
	for(x = 0; x < 10; x++)
	{
		time_stamp = get_time_stamp_from_buffers(0, msg_type, sequence_id, 0, priv);
		if(time_stamp != TIMESTAMP_NO_VALUE)
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

	if(LOG_MATCH_INPUT)
	{
		if(time_stamp != TIMESTAMP_NO_VALUE)
		{ print_timestamp_message(1, 0, msg_type, sequence_id, status, time_stamp, 0); }
		else
		{ 
			force_logging = 1;
			print_timestamp_message(1, 0, msg_type, sequence_id, status, time_stamp, 1);
		}
	}
	
	return true;
}

irqreturn_t bcm54210pe_handle_interrupt_thread(int irq, void * phy_dat)
{
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	printk("____________bcm54210pe_handle_interrupt_thread\n");
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	
	return IRQ_HANDLED;
}

irqreturn_t bcm54210pe_handle_interrupt(int irq, void * phy_dat)
{
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	printk("___________________bcm54210pe_handle_interrupt\n");
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	printk("______________________________________________\n");
	
	
	return IRQ_WAKE_THREAD;
	
	
	//////////////////////////////////////////////////////////////
	
	
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
  
	//phy_clear_interrupt(phydev);
	return IRQ_WAKE_THREAD;
}

static int bcm54210pe_enable_pps(struct phy_device *phydev)
{

	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_4_REG, 0x0004);
	bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG,0x0003,0x0002);
	return 0;
}


static int bcm54210pe_config_1588(struct phy_device *phydev)
{
	print_function_message("bcm54210pe_config_1588", NULL, 0);
	
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


	bcm54210pe_enable_pps(phydev);

	//15, 33 or 41 - experimental
	printk("DEBUG: GPIO %d IRQ %d\n", 15, gpio_to_irq(41));
	
	err = request_threaded_irq(gpio_to_irq(41), bcm54210pe_handle_interrupt, bcm54210pe_handle_interrupt_thread,
								IRQF_ONESHOT | IRQF_SHARED,
								phydev_name(phydev), phydev);

	printk("request_threaded_irq err = %d\n", err);			

	
	return err; 
}

static int bcm54210pe_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	
	u16 Time[5] = {0,0,0,0,0};
	
	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

        // EXP approach
	// Trigger sync which will capture the heartbeat counter
	//bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF000);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF000);
	bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);
	//bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF020);

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
	
	u64 time_stamp = ts_to_ns(Time);
			
	print_function_message("ptp_clock_info", "time_stamp", time_stamp);

	ts->tv_sec = ( (u64)time_stamp / (u64)1000000000 );
	ts->tv_nsec = ( (u64)time_stamp % (u64)1000000000 );
	
	return 0;
}

static int bcm54210pe_settime(struct ptp_clock_info *info, const struct timespec64 *ts)
{
	uint64_t time_stamp = (ts->tv_sec * 1000000000) + ts->tv_nsec;
	
	print_function_message("bcm54210pe_settime", "time_stamp", time_stamp);
	
	//Kyle - Should figure / test this out.

	int var[4];

	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	var[4] = (int) (ts->tv_sec  & 0x0000FFFF00000000) >> 32;
	var[3] = (int) (ts->tv_sec  & 0x00000000FFFF0000) >> 16;
	var[2] = (int) (ts->tv_sec  & 0x000000000000FFFF);
	var[1] = (int) (ts->tv_nsec & 0x00000000FFFF0000) >> 16;
	var[0] = (int) (ts->tv_nsec & 0x000000000000FFFF); 

	phy_lock_mdio_bus(phydev);

	__bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xF000);

	//Load Original Time Code Register
	__bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_0, var[0]);
	__bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_1, var[1]);
	__bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_2, var[2]);
	__bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_3, var[3]);
	__bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_4, var[4]);

	//Enable shadow register
	__bcm_phy_write_exp(phydev, SHADOW_REG_CONTROL, 0x0000);
	__bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x0400);

	__bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xE020); //NCO Register 6 => Enable SYNC_OUT pulse train and Internal Syncout ad framesync

	phy_unlock_mdio_bus(phydev);

	struct timespec64 ts_new;

	bcm54210pe_gettime(info, &ts_new);
	
	return 0; 
}

static int bcm54210pe_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{	
	print_function_message("bcm54210pe_adjfine", "scaled_ppm", scaled_ppm);
	
	int err;
	u16 lo, hi;
	u32 corrected_8ns_interval, base_8ns_interval;
	bool negative;

	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	negative = false;
        if ( scaled_ppm < 0 ) {
		negative = true;
		scaled_ppm = -scaled_ppm;
	}

	// This is not completely accurate but very fast
	scaled_ppm >>= 7;

	base_8ns_interval = 1 << 31;

	if (negative) {
		corrected_8ns_interval = base_8ns_interval - scaled_ppm;
	} else {
		corrected_8ns_interval = base_8ns_interval + scaled_ppm;
	}

	hi = (corrected_8ns_interval & 0xFFFF0000) >> 16;
	lo = (corrected_8ns_interval & 0x0000FFFF);

	// Set freq_mdio_sel to 1
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_2_2_REG, 0x4000);

	// Load 125MHz frequency reqcntrl
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_1_MSB_REG, hi);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_1_LSB_REG, lo);

	// On next framesync load freq from freqcntrl
	bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x0040);

	// Trigger framesync
	err = bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xD020);

	return err;
}

static int bcm54210pe_adjtime(struct ptp_clock_info *info, s64 delta)
{
	print_function_message("bcm54210pe_adjtime", "delta", delta);

	//Kyle - Should figure / test this out.
	
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
{ return match_rx_timestamp_callback(mii_ts, skb, type); }

void bcm54210pe_txtstamp(struct mii_timestamper *mii_ts, struct sk_buff *skb, int type)
{
	struct bcm54210pe_private *device = container_of(mii_ts, struct bcm54210pe_private, mii_ts);

	switch (device->hwts_tx_en) 
	{
		case HWTSTAMP_TX_ON:
		{	
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			skb_queue_tail(&device->tx_skb_queue, skb);
			break;
		}
		case HWTSTAMP_TX_OFF:
		{	
		
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
	print_function_message("bcm54210pe_ts_info", NULL, 0);
	
	struct bcm54210pe_private *bcm54210pe = container_of(mii_ts, struct bcm54210pe_private, mii_ts);

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
	print_function_message("bcm54210pe_hwtstamp", NULL, 0);
	
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
        .max_adj        = 100000000,
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


static int bcm54210pe_sw_reset(struct phy_device *phydev)
{
	print_function_message("bcm54210pe_sw_reset", NULL, 0);
	
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


int bcm54210pe_probe(struct phy_device *phydev)
{	
	//print_function_message("bcm54210pe_probe", NULL, 0);
	
	int err = 0;
	int x, y;
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

	bcm54210pe->mii_ts.rxtstamp = bcm54210pe_rxtstamp;
	bcm54210pe->mii_ts.txtstamp = bcm54210pe_txtstamp;
	bcm54210pe->mii_ts.hwtstamp = bcm54210pe_hwtstamp;
	bcm54210pe->mii_ts.ts_info  = bcm54210pe_ts_info;


	phydev->mii_ts = &bcm54210pe->mii_ts;
	
	INIT_WORK(&bcm54210pe->txts_work, match_tx_timestamp_thread);
	INIT_DELAYED_WORK(&bcm54210pe->fifo_read_work_delayed, read_txrx_timestamp_thread);
	
	skb_queue_head_init(&bcm54210pe->tx_skb_queue);
	
	x = 0; y = 0;
	for (x = 0; x < CIRCULAR_BUFFER_COUNT; x++)
	{ 
		INIT_LIST_HEAD(&bcm54210pe->circular_buffers[x]);
	
		for (y = 0; y < CIRCULAR_BUFFER_ITEM_COUNT; y++)
		{ list_add(&bcm54210pe->circular_buffer_items[x][y].list, &bcm54210pe->circular_buffers[x]); }
	}
	
	memcpy(&bcm54210pe->ptp->caps, &bcm54210pe_clk_caps, sizeof(bcm54210pe_clk_caps));
	mutex_init(&bcm54210pe->ptp->clock_lock);
	mutex_init(&bcm54210pe->ptp->timeset_lock);
	ptp->chosen = bcm54210pe;
    phydev->priv = bcm54210pe;
	ptp->caps.owner = THIS_MODULE;

	bcm54210pe->ptp->ptp_clock = ptp_clock_register(&bcm54210pe->ptp->caps, &phydev->mdio.dev);
	
	if (IS_ERR(bcm54210pe->ptp->ptp_clock)) {
                        err = PTR_ERR(bcm54210pe->ptp->ptp_clock);
                        goto error;
	}
	
	
	schedule_delayed_work_on(time_stamp_thread_cpu, &bcm54210pe->fifo_read_work_delayed, usecs_to_jiffies(100));

error:
	return err;
}
