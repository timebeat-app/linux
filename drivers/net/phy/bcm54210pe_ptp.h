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

#define SYNC_OUT_MODE_1 1
#define SYNC_OUT_MODE_2 2

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
	spinlock_t irq_spin_lock;

	int ts_tx_config;
	int tx_rx_filter;

	bool ts_capture;
	bool one_step;
	bool perout_en;
	bool extts_en;
	int  second_on_set;
	int  perout_mode;

	struct sk_buff_head tx_skb_queue;
		
	struct bcm54210pe_circular_buffer_item	circular_buffer_items[CIRCULAR_BUFFER_COUNT][CIRCULAR_BUFFER_ITEM_COUNT];
	struct list_head 						circular_buffers[CIRCULAR_BUFFER_COUNT];

	struct work_struct txts_work, perout_ws;
	struct delayed_work fifo_read_work_delayed;

	int hwts_tx_en;
	int hwts_rx_en;
	int layer;
	int version;
};

irqreturn_t bcm54210pe_handle_interrupt(int irq, void *phy_dat);
irqreturn_t bcm54210pe_handle_interrupt_thread(int irq, void *phy_dat);

static int bcm54210pe_perout_enable(struct bcm54210pe_private *private, s64 period, s64 pulsewidth, int on);
static u16 bcm54210pe_get_base_nco6_reg(struct bcm54210pe_private *private, u16 val, bool do_nse_init);
static int bcm54210pe_interrupts_enable(struct phy_device *phydev, bool fsync_en, bool sop_en);
static int bcm54210pe_gettimex(struct ptp_clock_info *info, struct timespec64 *ts, struct ptp_system_timestamp *sts);
static int bcm54210pe_get80bittime(struct bcm54210pe_private *private, struct timespec64 *ts, struct ptp_system_timestamp *sts);
static int bcm54210pe_get48bittime(struct bcm54210pe_private *private, u64 *time_stamp);
static void bcm54210pe_run_perout_mode_one_thread(struct work_struct *perout_ws);
static u64 four_u16_to_ns(u16 *four_u16);
static u64 ts_to_ns(struct timespec64 *ts);
