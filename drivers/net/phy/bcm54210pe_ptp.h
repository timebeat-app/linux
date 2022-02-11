// SPDX-License-Identifier: GPL-2.0+
/*
 *  drivers/net/phy/bcm54210pe_ptp.h
 *
 * PTP for BCM54210PE header file
 *
 * Authors: Carlos Fernandez, Kyle Judd
 * License: GPL
 * Copyright (C) 2021 Technica-Electronics GmbH
 */


#include <linux/ptp_clock_kernel.h>                                                                  
#include <linux/list.h>

#define MAX_POOL_SIZE	32

struct bcm54210pe_ptp {
        struct ptp_clock_info caps;
        struct ptp_clock *ptp_clock;
	struct mutex clock_lock;
	struct bcm54210pe_private *chosen;
	struct mutex timeset_lock;
};

struct bcm54210pe_fifo_item
{
	struct list_head list;
	
	u8 domain_number;
	u8 msgtype;
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
	int ts_tx_config;
	int tx_rx_filter;
	bool one_step;
	struct sk_buff_head tx_queue;
	
	struct list_head tx_fifo_sync;
	struct list_head tx_fifo_pd_request;
	struct list_head tx_fifo_pd_response;
	
	struct list_head rx_fifo_sync;
	struct list_head rx_fifo_pd_request;
	struct list_head rx_fifo_pd_response;
	
	struct bcm54210pe_fifo_item ts_tx_data_sync[MAX_POOL_SIZE];
	struct bcm54210pe_fifo_item ts_tx_data_pd_request[MAX_POOL_SIZE];	
	struct bcm54210pe_fifo_item ts_tx_data_pd_response[MAX_POOL_SIZE];
		
	struct bcm54210pe_fifo_item ts_rx_data_sync[MAX_POOL_SIZE];
	struct bcm54210pe_fifo_item ts_rx_data_pd_request[MAX_POOL_SIZE];	
	struct bcm54210pe_fifo_item ts_rx_data_pd_response[MAX_POOL_SIZE];
	
	struct work_struct txts_work;
	//struct work_struct fifo_read_work;
	struct delayed_work fifo_read_work_delayed;
	
	int hwts_tx_en;
	int hwts_rx_en;
	int layer;
	int version;
	
	u64 Current_Time;
};
