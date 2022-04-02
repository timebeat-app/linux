// SPDX-License-Identifier: GPL-2.0+
/*
 *  drivers/net/phy/bcm54210pe_ptp.h
 *
 * PTP for BCM54210PE header file
 *
 * Authors: Carlos Fernandez, Kyle Judd, Lasse Johnsen
 * License: GPL
 * Copyright (C) 2021 Technica-Electronics GmbH
 */


#include <linux/ptp_clock_kernel.h>                                                                  
#include <linux/list.h>

#define CIRCULAR_BUFFER_COUNT		8
#define CIRCULAR_BUFFER_ITEM_COUNT	32

#define SYNC_IN_PIN  0
#define SYNC_OUT_PIN 1


struct bcm54210pe_ptp {
	struct ptp_clock_info caps;
	struct ptp_clock *ptp_clock;
	struct mutex clock_lock;
	struct bcm54210pe_private *chosen;
	struct mutex timeset_lock;
};

struct bcm54210pe_circular_buffer_item
{
	struct list_head list;
	
	u8 domain_number;
	u8 msg_type;
	u16 sequence_id;
    u16 source_clock_id[4];
	u16 port_number;
	
	u64 time_stamp;	
	
	int is_valid;
};


struct bcm54210pe_private {
	
	struct phy_device *phydev;
	struct bcm54210pe_ptp *ptp;
	struct mii_timestamper mii_ts;
	struct ptp_pin_desc sdp_config[2];

	int ts_tx_config;
	int tx_rx_filter;

	bool ts_capture;
	bool one_step;
	bool per_out_en;
	bool extts_en;

	struct sk_buff_head tx_skb_queue;
		
	struct bcm54210pe_circular_buffer_item	circular_buffer_items[CIRCULAR_BUFFER_COUNT][CIRCULAR_BUFFER_ITEM_COUNT];
	struct list_head 						circular_buffers[CIRCULAR_BUFFER_COUNT];

	struct work_struct txts_work;
	struct delayed_work fifo_read_work_delayed;
	
	int hwts_tx_en;
	int hwts_rx_en;
	int layer;
	int version;
};

irqreturn_t bcm54210pe_handle_interrupt(int irq, void *phy_dat);
irqreturn_t bcm54210pe_handle_interrupt_thread(int irq, void *phy_dat);

static int bcm54210pe_per_out_en(struct bcm54210pe_ptp *ptp, int freq, int on);
static u16 bcm54210pe_get_base_nco6_reg(struct bcm54210pe_ptp *ptp, u16 val, bool do_nse_init);

