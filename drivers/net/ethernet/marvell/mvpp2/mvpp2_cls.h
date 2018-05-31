/*
 * RSS and Classifier definitions for Marvell PPv2 Network Controller
 *
 * Copyright (C) 2014 Marvell
 *
 * Marcin Wojtas <mw@semihalf.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef _MVPP2_CLS_H_
#define _MVPP2_CLS_H_

#include "mvpp2.h"

/* Classifier constants */
#define MVPP2_CLS_FLOWS_TBL_SIZE	512
#define MVPP2_CLS_FLOWS_TBL_DATA_WORDS	3
#define MVPP2_CLS_LKP_TBL_SIZE		64
#define MVPP2_CLS_RX_QUEUES		256

/* Classifier flow constants */

#define MVPP2_FLOW_N_FIELDS		4

enum mvpp2_rss_proto {
	MVPP2_RSS_IP4,
	MVPP2_RSS_IP6,
};

enum mvpp2_cls_engine {
	MVPP22_CLS_ENGINE_C2 = 1,
	MVPP22_CLS_ENGINE_C3A,
	MVPP22_CLS_ENGINE_C3B,
	MVPP22_CLS_ENGINE_C4,
	MVPP22_CLS_ENGINE_C3HA = 6,
	MVPP22_CLS_ENGINE_C3HB = 7,
};

enum mvpp2_cls_field_id {
	MVPP22_CLS_FIELD_VLAN = 0x06,
	MVPP22_CLS_FIELD_IP4SA = 0x10,
	MVPP22_CLS_FIELD_IP4DA = 0x11,
	MVPP22_CLS_FIELD_IP6SA = 0x17,
	MVPP22_CLS_FIELD_IP6DA = 0x1A,
	MVPP22_CLS_FIELD_L4SIP = 0x1D,
	MVPP22_CLS_FIELD_L4DIP = 0x1E,
};

enum mvpp2_cls_flow_seq {
	MVPP2_CLS_FLOW_SEQ_NORMAL = 0,
	MVPP2_CLS_FLOW_SEQ_FIRST1,
	MVPP2_CLS_FLOW_SEQ_FIRST2,
	MVPP2_CLS_FLOW_SEQ_LAST,
	MVPP2_CLS_FLOW_SEQ_MIDDLE
};

/* Classifier C2 engine constants */
#define MVPP22_CLS_C2_TCAM_EN(data)		((data) << 16)

enum mvpp22_cls_c2_action {
	MVPP22_C2_NO_UPD = 0,
	MVPP22_C2_NO_UPD_LOCK,
	MVPP22_C2_UPD,
	MVPP22_C2_UPD_LOCK,
};

enum mvpp22_cls_c2_fwd_action {
	MVPP22_C2_FWD_NO_UPD = 0,
	MVPP22_C2_FWD_NO_UPD_LOCK,
	MVPP22_C2_FWD_SW,
	MVPP22_C2_FWD_SW_LOCK,
	MVPP22_C2_FWD_HW,
	MVPP22_C2_FWD_HW_LOCK,
	MVPP22_C2_FWD_HW_LOW_LAT,
	MVPP22_C2_FWD_HW_LOW_LAT_LOCK,
};

/* Classifier C2 engine entries */
#define MVPP22_CLS_C2_MATCH_ALL_IDX	0

/* RSS flow entries in the flow table. We have 2 entries per port for RSS.
 *
 * The first performs a dummy lookup using the C2 TCAM engine, to tag the
 * packet for software forwarding (needed for RSS)
 *
 * The second configures the hash generation, by specifying which fields of the
 * packet header are used to generate the hash, and specifies the relevant hash
 * engine to use.
 */
#define MVPP22_RSS_FLOW_C2_OFFS		0
#define MVPP22_RSS_FLOW_HASH_OFFS	1
#define MVPP22_RSS_FLOW_SIZE		(MVPP22_RSS_FLOW_HASH_OFFS + 1)

#define MVPP22_RSS_FLOW_C2(port)	((port) * MVPP22_RSS_FLOW_SIZE + \
					 MVPP22_RSS_FLOW_C2_OFFS)
#define MVPP22_RSS_FLOW_HASH(port)	((port) * MVPP22_RSS_FLOW_SIZE + \
					 MVPP22_RSS_FLOW_HASH_OFFS)
#define MVPP22_RSS_FLOW_FIRST(port)	MVPP22_RSS_FLOW_C2(port)

struct mvpp2_cls_flow_entry {
	u32 index;
	u32 data[MVPP2_CLS_FLOWS_TBL_DATA_WORDS];
};

struct mvpp2_cls_lookup_entry {
	u32 lkpid;
	u32 way;
	u32 data;
};

void mvpp22_rss_fill_table(struct mvpp2_port *port, u32 table);

void mvpp22_rss_enable(struct mvpp2_port *port);

void mvpp22_rss_disable(struct mvpp2_port *port);

int mvpp2_rss_get_flow(struct mvpp2_port *port, struct ethtool_rxnfc *info);

int mvpp2_rss_set_flow(struct mvpp2_port *port, struct ethtool_rxnfc *info);

void mvpp2_init_rss(struct mvpp2 *priv);

void mvpp2_port_init_rss(struct mvpp2_port *port);

void mvpp2_cls_init(struct mvpp2 *priv);

void mvpp2_cls_port_config(struct mvpp2_port *port);

void mvpp2_cls_oversize_rxq_set(struct mvpp2_port *port);

#endif
