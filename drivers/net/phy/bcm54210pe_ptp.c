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
#define PTP_SEQUENCE_ID_OFFSET		30
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

#define ORIGINAL_TIME_CODE_0 		0x0854
#define ORIGINAL_TIME_CODE_1 		0x0855
#define ORIGINAL_TIME_CODE_2 		0x0856
#define ORIGINAL_TIME_CODE_3 		0x0857
#define ORIGINAL_TIME_CODE_4 		0x0858

#define TIME_STAMP_REG_0		0x0889
#define TIME_STAMP_REG_1		0x088A
#define TIME_STAMP_REG_2		0x088B
#define TIME_STAMP_REG_3		0x08C4
#define TIME_STAMP_INFO_1		0x088C
#define TIME_STAMP_INFO_2		0x088D
#define INTERRUPT_STATUS_REG		0x085F
#define INTERRUPT_MASK_REG		0x085E
#define EXT_SOFTWARE_RESET		0x0F70
#define EXT_RESET1			0x0001 //RESET
#define EXT_RESET2			0x0000 //NORMAL OPERATION
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
#define NSE_DPPL_NCO_1_LSB_REG		0x0873
#define NSE_DPPL_NCO_1_MSB_REG		0x0874

#define NSE_DPPL_NCO_2_0_REG		0x0875
#define NSE_DPPL_NCO_2_1_REG		0x0876
#define NSE_DPPL_NCO_2_2_REG		0x0877

#define NSE_DPPL_NCO_3_0_REG		0x0878
#define NSE_DPPL_NCO_3_1_REG		0x0879
#define NSE_DPPL_NCO_3_2_REG		0x087A

#define NSE_DPPL_NCO_4_REG		0x087B

#define NSE_DPPL_NCO_5_0_REG		0x087C
#define NSE_DPPL_NCO_5_1_REG		0x087D
#define NSE_DPPL_NCO_5_2_REG		0x087E

#define NSE_DPPL_NCO_6_REG		0x087F

#define DPLL_SELECT_REG			0x085b
#define TIMECODE_SEL_REG		0x08C3
#define SHADOW_REG_CONTROL		0x085C
#define SHADOW_REG_LOAD			0x085D

#define PTP_INTERRUPT_REG		0x0D0C

#define CTR_DBG_REG			0x088E
#define HEART_BEAT_REG4			0x08ED
#define HEART_BEAT_REG3			0x08EC
#define HEART_BEAT_REG2			0x0888
#define	HEART_BEAT_REG1			0x0887
#define	HEART_BEAT_REG0			0x0886
	
#define READ_END_REG			0x0885


#define FIFO_READ_DELAY			100*HZ/1000 /* delay milliseconds in jiffies */

int force_logging = 0;

#define LOG_ENABLED 		1

#define LOG_INPUT_ 		1
#define LOG_MATCH_OUTPUT_ 	1
#define LOG_MATCH_INPUT_ 	1

#define LOG_INPUT 		((LOG_ENABLED & LOG_INPUT_) || force_logging)
#define LOG_MATCH_OUTPUT 	((LOG_ENABLED & LOG_MATCH_OUTPUT_) || force_logging)
#define LOG_MATCH_INPUT 	((LOG_ENABLED & LOG_MATCH_INPUT_)  || force_logging)

#define TIMESTAMP_NO_VALUE 	(0xFFFFFFFFFFFFFFFF)
#define U48_MAX 		0xFFFFFFFFFFFF


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

		msg_type = (u8) ((fifo_info_2 & 0xF000) >> 12);
        	txrx = (u8) ((fifo_info_2 & 0x0800) >> 11);

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
	trace_printk("bcm54210pe_handle_interrupt_thread\n");

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
	trace_printk("bcm54210pe_handle_interrupt\n");
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

static int bcm54210pe_config_1588(struct phy_device *phydev)
{
	print_function_message("bcm54210pe_config_1588", NULL, 0);
	
	int err;
	u16 aux = 0xFFFF;

	//err = 0;

	err = bcm_phy_write_exp(phydev, PTP_INTERRUPT_REG, 0x3c02 );

	err |=  bcm_phy_write_exp(phydev, GLOBAL_TIMESYNC_REG, 0x0001); //Enable global timesync register.
	err |=  bcm_phy_write_exp(phydev, EXT_1588_SLICE_REG, 0x0101); //ENABLE TX and RX slice 1588
	err |=  bcm_phy_write_exp(phydev, TX_EVENT_MODE_REG, 0xFF00); //Add 80bit timestamp + NO CPU MODE in TX
	err |=  bcm_phy_write_exp(phydev, RX_EVENT_MODE_REG, 0xFF00); //Add 32+32 bits timestamp + NO CPU mode in RX
	err |=  bcm_phy_write_exp(phydev, TIMECODE_SEL_REG, 0x0101); //Select 80 bit counter
	err |=  bcm_phy_write_exp(phydev, TX_TSCAPTURE_ENABLE_REG, 0x0001); //Enable timestamp capture in TX
	err |=  bcm_phy_write_exp(phydev, RX_TSCAPTURE_ENABLE_REG, 0x0001); //Enable timestamp capture in RX

	//Enable shadow register
	err |= bcm_phy_write_exp(phydev, SHADOW_REG_CONTROL, 0x0000);
	err |= bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x07c0);

	//1n ts resolution
	//err = bcm_phy_write_exp(phydev, DPLL_SELECT_REG, 0x0160);

	// Set global mode and trigger immediate framesync to load shaddow registers
	err |=  bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xC020);

	//15, 33 or 41 - experimental
	printk("DEBUG: GPIO %d IRQ %d\n", 15, gpio_to_irq(41));

	// Enable Interrupt behaviour
	err |= bcm54210pe_enable_interrupts(phydev,true, false);

	err |= request_threaded_irq(gpio_to_irq(41), bcm54210pe_handle_interrupt, bcm54210pe_handle_interrupt_thread,
								IRQF_ONESHOT | IRQF_SHARED,
								phydev_name(phydev), phydev);

	printk("request_threaded_irq err = %d\n", err);			

	
	return err; 
}

static int bcm54210pe_gettimex(struct ptp_clock_info *info,
			       struct timespec64 *ts,
			       struct ptp_system_timestamp *sts)
{

	u16 Time[5] = {0,0,0,0,0};
	
	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;
	u16 nco_6_register_value;
	//unsigned long flags;
	int i, err;
	u64 time_stamp;

	// Capture timestamp on next framesync
	nco_6_register_value = 0x2000;

	// Heartbeat register selection. Latch 80 bit Original time coude counter into Heartbeat register
	// (this is undocumented)
	err = bcm_phy_write_exp(phydev, DPLL_SELECT_REG, 0x0040);

	// Amend to base register
	nco_6_register_value = bcm54210pe_get_base_nco6_reg(ptp->chosen, nco_6_register_value, false);

	// Set the NCO register
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, nco_6_register_value);

	// Trigger framesync
	if (sts != NULL) {

		// If we are doing a gettimex call
		ptp_read_system_prets(sts);
		bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);
		ptp_read_system_postts(sts);

		/*
		//Can't lock. unimac_mdio_write relies on bcmgenet_mii_wait. Can't sleep on spin lock. #ZZZzzz
		phy_lock_mdio_bus(phydev);
		spin_lock_irqsave(&ptp->chosen->irq_spin_lock, flags);
		ptp_read_system_prets(sts);
		__bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);
		ptp_read_system_postts(sts);
		spin_unlock_irqrestore(&ptp->chosen->irq_spin_lock, flags);
		phy_unlock_mdio_bus(phydev);
		*/
	} else {

		// or if we are doing a gettime call
		bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);
	}

	for (i = 0; i < 5;i++) {

		bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x400);
		Time[4] = bcm_phy_read_exp(phydev, HEART_BEAT_REG4);
		Time[3] = bcm_phy_read_exp(phydev, HEART_BEAT_REG3);
		Time[2] = bcm_phy_read_exp(phydev, HEART_BEAT_REG2);
		Time[1] = bcm_phy_read_exp(phydev, HEART_BEAT_REG1);
		Time[0] = bcm_phy_read_exp(phydev, HEART_BEAT_REG0);

		// Set read end bit
		bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x800);
		bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x000);

		time_stamp = ts_to_ns(Time);
		printk("Timestamp (%d): %llu\n", i, time_stamp);

		if (time_stamp != 0) {
			break;
		}
	}

	//print_function_message("ptp_clock_info", "time_stamp", time_stamp);

	ts->tv_sec = ( (u64)time_stamp / (u64)1000000000 );
	ts->tv_nsec = ( (u64)time_stamp % (u64)1000000000 );
	
	return 0;
}

static int bcm54210pe_gettime(struct ptp_clock_info *info, struct timespec64 *ts)
{
	int err;
	err = bcm54210pe_gettimex(info, ts, NULL);
	return err;
}

static int bcm54210pe_getlocaltime(struct bcm54210pe_private *private, u64 *time_stamp)
{

	///struct bcm54210pe *ptp = &bcm54210pe_ptp;


	u16 Time[3] = { 0, 0, 0};

	u16 nco_6_register_value;
	int i, err, ts;

	struct phy_device *phydev = private->phydev;

	// Capture timestamp on next framesync
	nco_6_register_value = 0x2000;

	// Heartbeat register selection. Latch 48 bit Original time coude counter into Heartbeat register
	// (this is undocumented)
	err = bcm_phy_write_exp(phydev, DPLL_SELECT_REG, 0x0000);

	// Amend to base register
	nco_6_register_value =
		bcm54210pe_get_base_nco6_reg(private, nco_6_register_value, false);

	// Set the NCO register
	err = bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, nco_6_register_value);

	// Trigger framesync
	err |= bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);

	for (i = 0; i < 5; i++) {

		err |= bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x400);
		Time[2] = bcm_phy_read_exp(phydev, HEART_BEAT_REG2);
		Time[1] = bcm_phy_read_exp(phydev, HEART_BEAT_REG1);
		Time[0] = bcm_phy_read_exp(phydev, HEART_BEAT_REG0);

		// Set read end bit
		err |= bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x800);
		err |= bcm_phy_write_exp(phydev, CTR_DBG_REG, 0x000);


		printk("LTS DGB (1): %hu:%hu:%hu\n", Time[2], Time[1], Time[0]);
		ts = 0;
		u64 ts[3];

		ts[2] = (((u64)Time[2]) << 32);
		ts[1] = (((u64)Time[1]) << 16);
		ts[0] = ((u64)Time[0]);

		u64 cumulative = 0;
		cumulative |= ts[0];
		//printk("Local Timestamp (0) (%d): %llu\n", i, cumulative);
		cumulative |= ts[1];
		//printk("Local Timestamp (1) (%d): %llu\n", i, cumulative);
		cumulative |= ts[2];
		//printk("Local Timestamp (2) (%d): %llu\n", i, cumulative);

		printk("Local Timestamp (1) (%d): %llu\n", i, cumulative);

		printk("LTS DGB (2): %llu:%llu:%llu\n", ts[2], ts[1], ts[0]);

		*time_stamp = cumulative;


		printk("Local Timestamp (2) (%d): %llu\n", i, *time_stamp);

		if (*time_stamp != 0) {
			//time_stamp = ts;
			break;
		}
	}

	return err;
}

static int bcm54210pe_settime(struct ptp_clock_info *info, const struct timespec64 *ts)
{
	//uint64_t time_stamp = (ts->tv_sec * 1000000000) + ts->tv_nsec;
	u16 shadow_load_register;
	int original_time_codes[5], local_time_codes[3];

	shadow_load_register = 0;

	//print_function_message("bcm54210pe_settime", "time_stamp", time_stamp);

	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	struct phy_device *phydev = ptp->chosen->phydev;

	// Assign original time codes
	original_time_codes[4] = (int) ((ts->tv_sec & 0x0000FFFF00000000) >> 32);
	original_time_codes[3] = (int) ((ts->tv_sec  & 0x00000000FFFF0000) >> 16);
	original_time_codes[2] = (int) (ts->tv_sec  & 0x000000000000FFFF);
	original_time_codes[1] = (int) ((ts->tv_nsec & 0x00000000FFFF0000) >> 16);
	original_time_codes[0] = (int) (ts->tv_nsec & 0x000000000000FFFF);

	// Write Original Time Code Register
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_0, original_time_codes[0]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_1, original_time_codes[1]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_2, original_time_codes[2]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_3, original_time_codes[3]);
	bcm_phy_write_exp(phydev, ORIGINAL_TIME_CODE_4, original_time_codes[4]);

	// Set Local Time Code Register
	local_time_codes[2] = 0x4000; 	               // Set FREQ_MDIO_SEL to 1
	local_time_codes[1] = original_time_codes[1];
	local_time_codes[0] = original_time_codes[0];

	// Write Local Time Code Register
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_2_0_REG, local_time_codes[0]);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_2_1_REG, local_time_codes[1]);
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_2_1_REG, local_time_codes[2]);

	// Set Time Code load bit in the shadow load register
	shadow_load_register |= 0x0400;

	// Set Local Time load bit in the shadow load register
	shadow_load_register |= 0x0080;

	// Write Shadow register
	bcm_phy_write_exp(phydev, SHADOW_REG_CONTROL, 0x0000);
	bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, shadow_load_register);

	// Set global mode and nse_init
	bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xD000);

	// Trigger framesync
	bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);

	// Set the second on set
	ptp->chosen->second_on_set = ts->tv_sec;

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

	// FIXME: This does not look right (at least an explanation needs to accompany)
	//scaled_ppm *=2147  //2^31 divided by millin (ppm)	
	//or use the faster bitwise operation approx
	//scaled_ppm <<= 11;
	
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
	//err = bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, 0xD020);
	err = bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);

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
	//struct phy_device *phydev = ptp->chosen->phydev;

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


static int bcm54210pe_perout_en(struct bcm54210pe_ptp *ptp, s64 period, s64 pulsewidth, int on)
{
	int err;
	struct phy_device *phydev;
	u16 nco_6_register_value, frequency_hi, frequency_lo, pulsewidth_reg, pulse_start_hi, pulse_start_lo;

	phydev = ptp->chosen->phydev;

	if (on) {
		frequency_hi = 0;
		frequency_lo = 0;
		pulsewidth_reg = 0;
		pulse_start_hi = 0;
		pulse_start_lo = 0;

		// Convert interval pulse spacing (period) and pulsewidth to 8 ns units
		period /= 8;
		pulsewidth /= 8;

		// IF pulsewidth is not explicitly set with PTP_PEROUT_DUTY_CYCLE
		if (pulsewidth == 0) {
			if (period < 2500) {
				// At a frequency at less than 20us (2500 x 8ns) set pulse length to 1/10th of the interval pulse spacing
				pulsewidth = period / 10;

				// Where the interval pulse spacing is short, ensure we set a pulse length of 8ns
				if (pulsewidth == 0)
					pulsewidth = 1;

			} else {
				// Otherwise set pulse with to 4us (8ns x 500 = 4us)
				pulsewidth = 500;
			}
		}

		frequency_lo 	 = (u16)period; 			// Lowest 16 bits of 8ns interval pulse spacing [15:0]
		frequency_hi	 = (u16) (0x3FFF & (period >> 16));	// Highest 14 bits of 8ns interval pulse spacing [29:16]
		frequency_hi	|= (u16) pulsewidth << 14; 		// 2 lowest bits of 8ns pulse length [1:0]
		pulsewidth_reg	 = (u16) (0x7F & (pulsewidth >> 2));	// 7 highest bit  of 8 ns pulse length [8:2]

		// Set enable per_out
		ptp->chosen->per_out_en = true;

		/* Replaced by thread

		// Get base value
		nco_6_register_value = bcm54210pe_get_base_nco6_reg(ptp->chosen, nco_6_register_value, true);

		// Write to register
		err = bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, nco_6_register_value);

		// Set sync out pulse interval spacing and pulse length
		err |= bcm_phy_write_exp(phydev, NSE_DPPL_NCO_3_0_REG, frequency_lo);
		err |= bcm_phy_write_exp(phydev, NSE_DPPL_NCO_3_1_REG, frequency_hi);
		err |= bcm_phy_write_exp(phydev, NSE_DPPL_NCO_3_2_REG, pulsewidth_reg);

		// On next framesync load sync out frequency
		err |= bcm_phy_write_exp(phydev, SHADOW_REG_LOAD, 0x0200);

		// Trigger immediate framesync framesync
		err |= bcm_phy_modify_exp(phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);
		*/

		schedule_work(&ptp->chosen->perout_ws);

	} else {

		// Set disable pps
		ptp->chosen->per_out_en = false;

		// Get base value
		nco_6_register_value = bcm54210pe_get_base_nco6_reg(ptp->chosen, nco_6_register_value, false);

		// Write to register
		err = bcm_phy_write_exp(phydev, NSE_DPPL_NCO_6_REG, nco_6_register_value);
	}

	return err;
}

static void bcm54210pe_run_perout_thread(struct work_struct *perout_ws)
{
	struct bcm54210pe_private *private = container_of(perout_ws, struct bcm54210pe_private, perout_ws);

	//struct bcm54210pe_ptp *ptp = container_of(perout_ws, struct bcm54210pe_ptp, work_struct);

	u64 i, time_stamp;
	i = 0;
	time_stamp = 0;
	u64 local_time_stamp, next_event, time_before_next_pulse, period;
	u16 pulsewidth, nco_6_register_value;

	pulsewidth = 250;
	period = 1000000000;
	nco_6_register_value = 0;

	// Get base value
	nco_6_register_value = bcm54210pe_get_base_nco6_reg(private, nco_6_register_value, false);

	// Write to register
	bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_6_REG, nco_6_register_value);

	while(true) {
		//printk("run_perout %lli\n", i);

		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_3_1_REG, pulsewidth << 14);
		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_3_2_REG, pulsewidth >> 2);

		//printk("run_perout (1) (%llu): %hu:%hu\n", pulsewidth << 14, pulsewidth >> 2);

		bcm54210pe_getlocaltime(private, &local_time_stamp);
		time_before_next_pulse =  period - (local_time_stamp % period);
		next_event = local_time_stamp + time_before_next_pulse;

		printk("run_perout (2) (%llu): %llu : %llu\n", i, local_time_stamp, next_event);

		// Set sync out pulse interval spacing and pulse length
		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_0_REG, next_event & 0xFFF0);
		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_1_REG, next_event >> 16);
		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_2_REG, next_event >> 32);

		// 0 x 3B9A CA00
		//bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_0_REG, 0xFFFF);
		//bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_1_REG, 0xFFFF);
		//bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_2_REG, 0x0000);

		// On next framesync load sync out frequency
		bcm_phy_write_exp(private->phydev, SHADOW_REG_LOAD, 0x0200);

		// Trigger immediate framesync framesync
		bcm_phy_modify_exp(private->phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);

		/*
		udelay(time_before_next_pulse / 1000 + 100);

		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_0_REG, 0xFFFF);
		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_1_REG, 0xFFFF);
		bcm_phy_write_exp(private->phydev, NSE_DPPL_NCO_5_2_REG, 0xFFFF);

		// On next framesync load sync out frequency
		bcm_phy_write_exp(private->phydev, SHADOW_REG_LOAD, 0x0200);

		// Trigger immediate framesync framesync
		bcm_phy_modify_exp(private->phydev, NSE_DPPL_NCO_6_REG, 0x003C, 0x0020);
		 */

		i++;

		do_softirq();
		if (!private->per_out_en) {
			break;
		}
		mdelay(1000);
	}
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

static int bcm54210pe_feature_enable(struct ptp_clock_info *info, struct ptp_clock_request *req, int on)
{
	struct bcm54210pe_ptp *ptp = container_of(info, struct bcm54210pe_ptp, caps);
	s64 period, pulsewidth;
	struct timespec64 ts;

	switch (req->type) {

	case PTP_CLK_REQ_PEROUT :

		period = 0;
		pulsewidth = 0;

		// Check if pin func is set correctly
		if (ptp->chosen->sdp_config[SYNC_OUT_PIN].func != PTP_PF_PEROUT) {
			return -EOPNOTSUPP;
		}

		// No other flags supported
		if (req->perout.flags & ~PTP_PEROUT_DUTY_CYCLE) {
			return -EOPNOTSUPP;
		}
		// Check if a specific pulsewidth is set
		if ((req->perout.flags & PTP_PEROUT_DUTY_CYCLE) > 0) {

			// Extract pulsewidth
			ts.tv_sec = req->perout.on.sec;
			ts.tv_nsec = req->perout.on.nsec;
			pulsewidth = timespec64_to_ns(&ts);

			// 9 bits in 8ns units, so max = 4,088ns
			if (pulsewidth > 511 * 8) {
				return -ERANGE;
			}
		}

		// Extract pulse spacing interval (period)
		ts.tv_sec = req->perout.period.sec;
		ts.tv_nsec = req->perout.period.nsec;
		period = timespec64_to_ns(&ts);

		// 16ns is minimum pulse spacing interval (a value of 16 will result in 8ns high followed by 8 ns low)
		if (period != 0 && period < 16) {
			return -ERANGE;
		}


		return bcm54210pe_perout_en(ptp, period, pulsewidth, on);

	case PTP_CLK_REQ_EXTTS:
		if (ptp->chosen->sdp_config[SYNC_IN_PIN].func != PTP_PF_EXTTS) {
			return -EOPNOTSUPP;
		}

	// Not the right thing #Ooops
	case PTP_CLK_REQ_PPS :
		// This is what we call kpps in Timebeat - will get back to later

	default:
		break;
	}

	return -EOPNOTSUPP;
}


static int bcm54210pe_ptp_verify_pin(struct ptp_clock_info *info, unsigned int pin,
			      enum ptp_pin_function func, unsigned int chan)
{
	switch (func) {
	case PTP_PF_NONE:
		return 0;
		break;
	case PTP_PF_EXTTS:
		if (pin == SYNC_IN_PIN)
			return 0;
		break;
	case PTP_PF_PEROUT:
		if (pin == SYNC_OUT_PIN)
			return 0;
		break;
	case PTP_PF_PHYSYNC:
		break;
	}
	return -1;
}

static const struct ptp_clock_info bcm54210pe_clk_caps = {
        .owner          = THIS_MODULE,
        .name           = "BCM54210PE_PHC",
        .max_adj        = 100000000,
        .n_alarm        = 0,
        .n_pins         = 2,
        .n_ext_ts       = 1,
        .n_per_out      = 1,
        .pps            = 0,
        .adjtime        = &bcm54210pe_adjtime,
        .adjfine        = &bcm54210pe_adjfine,
        .gettime64      = &bcm54210pe_gettime,
	.gettimex64	= &bcm54210pe_gettimex,
        .settime64      = &bcm54210pe_settime,
	.enable		= &bcm54210pe_feature_enable,
	.verify		= &bcm54210pe_ptp_verify_pin,
};

static int bcm54210pe_enable_interrupts(struct phy_device *phydev, bool fsync_en, bool sop_en)
{
	u16 interrupt_mask;

	interrupt_mask = 0;

	if (fsync_en) {
		interrupt_mask |= 0x0001;
	}

	if (sop_en) {
		interrupt_mask |= 0x0002;
	}

	return bcm_phy_write_exp(phydev, INTERRUPT_MASK_REG, interrupt_mask);
}

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
	struct ptp_pin_desc *sync_in_pin_desc, *sync_out_pin_desc;

	bcm54210pe_sw_reset(phydev);
	bcm54210pe_config_1588(phydev);

	printk("PHY_DEV INTERRUPTS: %d\n", phydev->interrupts);
	printk("PHY_DEV IRQ: %d\n", phydev->irq);
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

	// Initialisation of work_structs and similar
	INIT_WORK(&bcm54210pe->txts_work, match_tx_timestamp_thread);
	INIT_DELAYED_WORK(&bcm54210pe->fifo_read_work_delayed, read_txrx_timestamp_thread);
	INIT_WORK(&bcm54210pe->perout_ws, bcm54210pe_run_perout_thread);

	skb_queue_head_init(&bcm54210pe->tx_skb_queue);
	
	x = 0; y = 0;
	for (x = 0; x < CIRCULAR_BUFFER_COUNT; x++)
	{ 
		INIT_LIST_HEAD(&bcm54210pe->circular_buffers[x]);
	
		for (y = 0; y < CIRCULAR_BUFFER_ITEM_COUNT; y++)
		{ list_add(&bcm54210pe->circular_buffer_items[x][y].list, &bcm54210pe->circular_buffers[x]); }
	}

	// Caps
	memcpy(&bcm54210pe->ptp->caps, &bcm54210pe_clk_caps, sizeof(bcm54210pe_clk_caps));
	bcm54210pe->ptp->caps.pin_config = bcm54210pe->sdp_config;

	// Mutex
	mutex_init(&bcm54210pe->ptp->clock_lock);
	mutex_init(&bcm54210pe->ptp->timeset_lock);

	// Spinlock
	spin_lock_init(&bcm54210pe->irq_spin_lock);

	// Features
	bcm54210pe->ts_capture = true;
	bcm54210pe->one_step = false;
	bcm54210pe->extts_en = false;
	bcm54210pe->per_out_en = false;

	// Pin descriptions
	sync_in_pin_desc = &bcm54210pe->sdp_config[SYNC_IN_PIN];
	snprintf(sync_in_pin_desc->name, sizeof(sync_in_pin_desc->name), "SYNC_IN");
	sync_in_pin_desc->index = SYNC_IN_PIN;
	sync_in_pin_desc->func = PTP_PF_NONE;

	sync_out_pin_desc = &bcm54210pe->sdp_config[SYNC_OUT_PIN];
	snprintf(sync_out_pin_desc->name, sizeof(sync_out_pin_desc->name), "SYNC_OUT");
	sync_out_pin_desc->index = SYNC_OUT_PIN;
	sync_out_pin_desc->func = PTP_PF_NONE;

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

static u16 bcm54210pe_get_base_nco6_reg(struct bcm54210pe_private *private, u16 val, bool do_nse_init)
{

	// Set Global mode to CPU system
	val |= 0xC000;

	// NSE init
	if (do_nse_init) {
		val |= 0x1000;
	}

	// TS Capture
	/*
	if (ptp->chosen->ts_capture) {
		val |= 0x2000;
	}
	*/

	// PPS out
	if (private->per_out_en) {
		// val |= 0x0002; // We've moved to mode 1
		val |= 0x0001;
	}

	return val;
}
