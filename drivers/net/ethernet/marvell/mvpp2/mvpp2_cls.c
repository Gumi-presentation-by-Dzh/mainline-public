/*
 * RSS and Classifier helpers for Marvell PPv2 Network Controller
 *
 * Copyright (C) 2014 Marvell
 *
 * Marcin Wojtas <mw@semihalf.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "mvpp2.h"
#include "mvpp2_cls.h"

static void mvpp2_cls_flow_read(struct mvpp2 *priv, int index,
				struct mvpp2_cls_flow_entry *fe)
{
	fe->index = index;
	mvpp2_write(priv, MVPP2_CLS_FLOW_INDEX_REG, index);
	fe->data[0] = mvpp2_read(priv, MVPP2_CLS_FLOW_TBL0_REG);
	fe->data[1] = mvpp2_read(priv, MVPP2_CLS_FLOW_TBL1_REG);
	fe->data[2] = mvpp2_read(priv, MVPP2_CLS_FLOW_TBL2_REG);
}

/* Update classification flow table registers */
static void mvpp2_cls_flow_write(struct mvpp2 *priv,
				 struct mvpp2_cls_flow_entry *fe)
{
	mvpp2_write(priv, MVPP2_CLS_FLOW_INDEX_REG, fe->index);
	mvpp2_write(priv, MVPP2_CLS_FLOW_TBL0_REG,  fe->data[0]);
	mvpp2_write(priv, MVPP2_CLS_FLOW_TBL1_REG,  fe->data[1]);
	mvpp2_write(priv, MVPP2_CLS_FLOW_TBL2_REG,  fe->data[2]);
}

static void mvpp2_cls_lookup_read(struct mvpp2 *priv, int lkpid, int way,
				  struct mvpp2_cls_lookup_entry *le)
{
	u32 val;

	val = (way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | lkpid;
	mvpp2_write(priv, MVPP2_CLS_LKP_INDEX_REG, val);
	le->way = way;
	le->lkpid = lkpid;
	le->data = mvpp2_read(priv, MVPP2_CLS_LKP_TBL_REG);
}

/* Update classification lookup table register */
static void mvpp2_cls_lookup_write(struct mvpp2 *priv,
				   struct mvpp2_cls_lookup_entry *le)
{
	u32 val;

	val = (le->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | le->lkpid;
	mvpp2_write(priv, MVPP2_CLS_LKP_INDEX_REG, val);
	mvpp2_write(priv, MVPP2_CLS_LKP_TBL_REG, le->data);
}

/* Operations on flow entry */
static int mvpp2_cls_sw_flow_hek_num_get(struct mvpp2_cls_flow_entry *fe)
{
	return fe->data[1] & MVPP2_CLS_FLOW_TBL1_N_FIELDS_MASK;
}

static void mvpp2_cls_sw_flow_hek_num_set(struct mvpp2_cls_flow_entry *fe,
					  int num_of_fields)
{
	fe->data[1] &= ~MVPP2_CLS_FLOW_TBL1_N_FIELDS_MASK;
	fe->data[1] |= MVPP2_CLS_FLOW_TBL1_N_FIELDS(num_of_fields);
}

static int mvpp2_cls_sw_flow_hek_get(struct mvpp2_cls_flow_entry *fe,
				     int field_index)
{
	return (fe->data[2] >> MVPP2_CLS_FLOW_TBL2_FLD_OFFS(field_index)) &
		MVPP2_CLS_FLOW_TBL2_FLD_MASK;
}

static void mvpp2_cls_sw_flow_hek_set(struct mvpp2_cls_flow_entry *fe,
				      int field_index, int field_id)
{
	fe->data[2] &= ~MVPP2_CLS_FLOW_TBL2_FLD(field_index,
						MVPP2_CLS_FLOW_TBL2_FLD_MASK);
	fe->data[2] |= MVPP2_CLS_FLOW_TBL2_FLD(field_index, field_id);
}

static void mvpp2_cls_sw_flow_eng_set(struct mvpp2_cls_flow_entry *fe,
				      int engine)
{
	fe->data[0] &= ~MVPP2_CLS_FLOW_TBL0_ENG(MVPP2_CLS_FLOW_TBL0_ENG_MASK);
	fe->data[0] |= MVPP2_CLS_FLOW_TBL0_ENG(engine);
}

static void mvpp2_cls_sw_flow_port_id_sel(struct mvpp2_cls_flow_entry *fe,
					  bool from_packet)
{
	if (from_packet)
		fe->data[0] |= MVPP2_CLS_FLOW_TBL0_PORT_ID_SEL;
	else
		fe->data[0] &= ~MVPP2_CLS_FLOW_TBL0_PORT_ID_SEL;
}

static void mvpp2_cls_sw_flow_seq_set(struct mvpp2_cls_flow_entry *fe, u32 seq)
{
	fe->data[1] &= ~MVPP2_CLS_FLOW_TBL1_SEQ(MVPP2_CLS_FLOW_TBL1_SEQ_MASK);
	fe->data[1] |= MVPP2_CLS_FLOW_TBL1_SEQ(seq);
}

static void mvpp2_cls_sw_flow_last_set(struct mvpp2_cls_flow_entry *fe,
				       bool is_last)
{
	fe->data[0] &= ~MVPP2_CLS_FLOW_TBL0_LAST;
	fe->data[0] |= !!is_last;
}

static void mvpp2_cls_sw_flow_pri_set(struct mvpp2_cls_flow_entry *fe, int prio)
{
	fe->data[1] &= ~MVPP2_CLS_FLOW_TBL1_PRIO(MVPP2_CLS_FLOW_TBL1_PRIO_MASK);
	fe->data[1] |= MVPP2_CLS_FLOW_TBL1_PRIO(prio);
}

static void mvpp2_cls_sw_flow_port_add(struct mvpp2_cls_flow_entry *fe,
				       u32 port)
{
	fe->data[0] |= MVPP2_CLS_FLOW_TBL0_PORT_ID(port);
}

/* Classifier default initialization */
void mvpp2_cls_init(struct mvpp2 *priv)
{
	struct mvpp2_cls_lookup_entry le;
	struct mvpp2_cls_flow_entry fe;
	int index;

	/* Enable classifier */
	mvpp2_write(priv, MVPP2_CLS_MODE_REG, MVPP2_CLS_MODE_ACTIVE_MASK);

	/* Clear classifier flow table */
	memset(&fe.data, 0, sizeof(fe.data));
	for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
		fe.index = index;
		mvpp2_cls_flow_write(priv, &fe);
	}

	/* Clear classifier lookup table */
	le.data = 0;
	for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
		le.lkpid = index;
		le.way = 0;
		mvpp2_cls_lookup_write(priv, &le);

		le.way = 1;
		mvpp2_cls_lookup_write(priv, &le);
	}
}

void mvpp2_cls_port_config(struct mvpp2_port *port)
{
	struct mvpp2_cls_lookup_entry le;
	u32 val;

	/* Set way for the port */
	val = mvpp2_read(port->priv, MVPP2_CLS_PORT_WAY_REG);
	val &= ~MVPP2_CLS_PORT_WAY_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_CLS_PORT_WAY_REG, val);

	/* Pick the entry to be accessed in lookup ID decoding table
	 * according to the way and lkpid.
	 */
	le.lkpid = port->id;
	le.way = 0;
	le.data = 0;

	/* Set initial CPU queue for receiving packets */
	le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
	le.data |= port->first_rxq;

	/* Disable classification engines */
	le.data &= ~MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

	/* Update lookup ID table entry */
	mvpp2_cls_lookup_write(port->priv, &le);
}

void mvpp22_rss_enable(struct mvpp2_port *port)
{
	struct mvpp2_cls_lookup_entry le;

	mvpp2_cls_lookup_read(port->priv, port->id, 0, &le);

	/* Enable classification lookup */
	le.data |= MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

	/* In this mode, the default RxQ is used as an index in the RxQ2RSS
	 * Table. We use the port_id to determine which RSS Table to use, so
	 * we need to update the default RxQ.
	 */
	le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
	le.data |= port->id;

	/* Update lookup ID table entry */
	mvpp2_cls_lookup_write(port->priv, &le);
}

void mvpp22_rss_disable(struct mvpp2_port *port)
{
	struct mvpp2_cls_lookup_entry le;

	mvpp2_cls_lookup_read(port->priv, port->id, 0, &le);

	/* Disable classification lookup */
	le.data &= ~MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

	/* We won't be performing classification actions, so the default RxQ
	 * in this entry need to be updated to the real first_rxq associated
	 * with this port
	 */
	le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
	le.data |= port->first_rxq;

	/* Update lookup ID table entry */
	mvpp2_cls_lookup_write(port->priv, &le);
}

/* Set CPU queue number for oversize packets */
void mvpp2_cls_oversize_rxq_set(struct mvpp2_port *port)
{
	u32 val;

	mvpp2_write(port->priv, MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port->id),
		    port->first_rxq & MVPP2_CLS_OVERSIZE_RXQ_LOW_MASK);

	mvpp2_write(port->priv, MVPP2_CLS_SWFWD_P2HQ_REG(port->id),
		    (port->first_rxq >> MVPP2_CLS_OVERSIZE_RXQ_LOW_BITS));

	val = mvpp2_read(port->priv, MVPP2_CLS_SWFWD_PCTRL_REG);
	val |= MVPP2_CLS_SWFWD_PCTRL_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_CLS_SWFWD_PCTRL_REG, val);
}

static inline u32 mvpp22_rxfh_indir(struct mvpp2_port *port, u32 rxq)
{
	int nrxqs, cpus = num_present_cpus();

	/* Number of RXQs per CPU */
	nrxqs = port->nrxqs / cpus;

	/* Indirection to better distribute the paquets on the CPUs when
	 * configuring the RSS queues.
	 */
	return (rxq * nrxqs + rxq / cpus) % port->nrxqs;
}

void mvpp22_rss_fill_table(struct mvpp2_port *port, u32 table)
{
	struct mvpp2 *priv = port->priv;
	int i;

	for (i = 0; i < MVPP22_RSS_TABLE_ENTRIES; i++) {
		u32 sel = MVPP22_RSS_INDEX_TABLE(table) |
			  MVPP22_RSS_INDEX_TABLE_ENTRY(i);
		mvpp2_write(priv, MVPP22_RSS_INDEX, sel);

		mvpp2_write(priv, MVPP22_RSS_TABLE_ENTRY,
			    mvpp22_rxfh_indir(port, port->indir[i]));
	}
}

static int mvpp2_flow_add_hek_field(struct mvpp2_cls_flow_entry *fe,
				    u32 field_id)
{
	int nb_fields = mvpp2_cls_sw_flow_hek_num_get(fe);

	if (nb_fields == MVPP2_FLOW_N_FIELDS)
		return -EINVAL;

	mvpp2_cls_sw_flow_hek_set(fe, nb_fields, field_id);

	mvpp2_cls_sw_flow_hek_num_set(fe, nb_fields + 1);

	return 0;
}

/* Add HEK fields to the flow command */
static int mvpp2_rss_hash_opts_set(struct mvpp2_cls_flow_entry *fe, u32 cmd,
				   int proto)
{
	/* It's a requirement to add HEK fields to the flow in ascending
	 * field_id order
	 */
	if (cmd & RXH_VLAN)
		if (mvpp2_flow_add_hek_field(fe, MVPP22_CLS_FIELD_VLAN))
			return -EINVAL;

	if (proto == MVPP2_RSS_IP4) {
		if (cmd & RXH_IP_SRC)
			if (mvpp2_flow_add_hek_field(fe,
						     MVPP22_CLS_FIELD_IP4SA))
				return -EINVAL;

		if (cmd & RXH_IP_DST)
			if (mvpp2_flow_add_hek_field(fe,
						     MVPP22_CLS_FIELD_IP4DA))
				return -EINVAL;
	} else {
		if (cmd & RXH_IP_SRC)
			if (mvpp2_flow_add_hek_field(fe,
						     MVPP22_CLS_FIELD_IP6SA))
				return -EINVAL;

		if (cmd & RXH_IP_DST)
			if (mvpp2_flow_add_hek_field(fe,
						     MVPP22_CLS_FIELD_IP6DA))
				return -EINVAL;
	}

	if (cmd & RXH_L4_B_0_1)
		if (mvpp2_flow_add_hek_field(fe, MVPP22_CLS_FIELD_L4SIP))
			return -EINVAL;

	if (cmd & RXH_L4_B_2_3)
		if (mvpp2_flow_add_hek_field(fe, MVPP22_CLS_FIELD_L4DIP))
			return -EINVAL;
	return 0;
}

int mvpp2_rss_set_flow(struct mvpp2_port *port, struct ethtool_rxnfc *info)
{
	int ret = 0, engine = MVPP22_CLS_ENGINE_C3HB;
	int flow_id = MVPP22_RSS_FLOW_HASH(port->id);
	struct mvpp2_cls_flow_entry fe;

	mvpp2_cls_flow_read(port->priv, flow_id, &fe);

	/* Clear former HEK parameters */
	mvpp2_cls_sw_flow_hek_num_set(&fe, 0);
	fe.data[2] = 0;

	switch (info->flow_type) {
	case IPV4_FLOW:
		/* When L4 data isn't needed, we use C3HA engine. C3HB engine
		 * automatically adds the L4 info field to the hash.
		 * The L4 info field comes from the Header Parser.
		 */
		engine = MVPP22_CLS_ENGINE_C3HA;
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
		ret = mvpp2_rss_hash_opts_set(&fe, info->data, MVPP2_RSS_IP4);
		break;
	case IPV6_FLOW:
		engine = MVPP22_CLS_ENGINE_C3HA;
	case TCP_V6_FLOW:
	case UDP_V6_FLOW:
		ret = mvpp2_rss_hash_opts_set(&fe, info->data, MVPP2_RSS_IP6);
		break;
	default: return -EOPNOTSUPP;
	}

	mvpp2_cls_sw_flow_eng_set(&fe, engine);

	mvpp2_cls_flow_write(port->priv, &fe);

	return ret;
}

int mvpp2_rss_get_flow(struct mvpp2_port *port, struct ethtool_rxnfc *info)
{
	int n_fields, field, flow_id, i;
	struct mvpp2_cls_flow_entry fe;

	flow_id = MVPP22_RSS_FLOW_HASH(port->id);

	mvpp2_cls_flow_read(port->priv, flow_id, &fe);

	n_fields = mvpp2_cls_sw_flow_hek_num_get(&fe);
	info->data = 0;

	for (i = 0; i < n_fields; i++) {
		field = mvpp2_cls_sw_flow_hek_get(&fe, i);

		switch (field) {
		case MVPP22_CLS_FIELD_VLAN:
			info->data |= RXH_VLAN;
			break;
		case MVPP22_CLS_FIELD_IP4SA:
		case MVPP22_CLS_FIELD_IP6SA:
			info->data |= RXH_IP_SRC;
			break;
		case MVPP22_CLS_FIELD_IP6DA:
		case MVPP22_CLS_FIELD_IP4DA:
			info->data |= RXH_IP_DST;
			break;
		case MVPP22_CLS_FIELD_L4SIP:
			info->data |= RXH_L4_B_0_1;
			break;
		case MVPP22_CLS_FIELD_L4DIP:
			info->data |= RXH_L4_B_2_3;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

/* Initial configuration of the classifier C2 engine, used to tag packets for
 * RSS
 */
static void mvpp2_init_cls_c2(struct mvpp2 *priv)
{
	u32 val;

	/* Select the TCAM entry */
	mvpp2_write(priv, MVPP22_CLS_C2_TCAM_IDX, MVPP22_CLS_C2_MATCH_ALL_IDX);

	/* We want to match everything with this C2 TCAM entry, so we don't
	 * enable any specific pattern matching
	 */
	mvpp2_write(priv, MVPP22_CLS_C2_TCAM_DATA0, 0);
	mvpp2_write(priv, MVPP22_CLS_C2_TCAM_DATA1, 0);
	mvpp2_write(priv, MVPP22_CLS_C2_TCAM_DATA2, 0);
	mvpp2_write(priv, MVPP22_CLS_C2_TCAM_DATA3, 0);

	/* Enable all ports. This means that we filter not port, that's why we
	 * don't enable any PORT_ID bit in the TCAM
	 */
	mvpp2_write(priv, MVPP22_CLS_C2_TCAM_DATA4, 0);

	/* Configure C2 action to enable RSS, soft forwarding and lock that */
	val = MVPP22_CLS_C2_ACT_RSS_EN(MVPP22_C2_UPD_LOCK);
	val |= MVPP22_CLS_C2_ACT_FWD(MVPP22_C2_FWD_SW_LOCK);
	mvpp2_write(priv, MVPP22_CLS_C2_ACT, val);

	/* Enable RSS after a lookup */
	mvpp2_write(priv, MVPP22_CLS_C2_ATTR2, MVPP22_CLS_C2_ATTR2_RSS_EN);
}

void mvpp2_port_init_rss(struct mvpp2_port *port)
{
	struct mvpp2_cls_lookup_entry le;
	struct mvpp2 *priv = port->priv;
	struct mvpp2_cls_flow_entry fe;
	int i;

	/* Set the table width: replace the whole classifier Rx queue number
	 * with the ones configured in RSS table entries.
	 */
	mvpp2_write(priv, MVPP22_RSS_INDEX, MVPP22_RSS_INDEX_TABLE(port->id));
	mvpp2_write(priv, MVPP22_RSS_WIDTH, 8);

	/* Loop through the classifier Rx Queues and map them to a RSS table.
	 * Map them all to the first table (0) by default.
	 */
	for (i = 0; i < MVPP2_CLS_RX_QUEUES; i++) {
		mvpp2_write(priv, MVPP22_RSS_INDEX, MVPP22_RSS_INDEX_QUEUE(i));
		mvpp2_write(priv, MVPP22_RXQ2RSS_TABLE,
			    MVPP22_RSS_TABLE_POINTER(port->id));
	}

	/* Configure the first table to evenly distribute the packets across
	 * real Rx Queues. The table entries map a hash to a port Rx Queue.
	 */
	for (i = 0; i < MVPP22_RSS_TABLE_ENTRIES; i++)
		port->indir[i] = ethtool_rxfh_indir_default(i, port->nrxqs);

	mvpp22_rss_fill_table(port, port->id);

	/* Select the relevant flow in the flow table, according to the RSS
	 * hash used
	 */
	memset(&fe, 0, sizeof(fe));

	fe.index = MVPP22_RSS_FLOW_C2(port->id);

	mvpp2_cls_sw_flow_eng_set(&fe, MVPP22_CLS_ENGINE_C2);
	mvpp2_cls_sw_flow_port_id_sel(&fe, true);
	mvpp2_cls_sw_flow_last_set(&fe, 0);
	mvpp2_cls_sw_flow_pri_set(&fe, 0);
	mvpp2_cls_sw_flow_seq_set(&fe, MVPP2_CLS_FLOW_SEQ_FIRST1);
	mvpp2_cls_sw_flow_port_add(&fe, BIT(port->id));

	mvpp2_cls_flow_write(priv, &fe);

	/* Add the relevant flows for RSS */
	memset(&fe, 0, sizeof(fe));

	fe.index = MVPP22_RSS_FLOW_HASH(port->id);

	/* Default hash generation parmeters : Use 2T generation */
	mvpp2_cls_sw_flow_hek_num_set(&fe, 2);
	mvpp2_cls_sw_flow_hek_set(&fe, 0, MVPP22_CLS_FIELD_IP4SA);
	mvpp2_cls_sw_flow_hek_set(&fe, 1, MVPP22_CLS_FIELD_IP4DA);
	mvpp2_cls_sw_flow_eng_set(&fe, MVPP22_CLS_ENGINE_C3HA);
	mvpp2_cls_sw_flow_port_id_sel(&fe, true);
	mvpp2_cls_sw_flow_last_set(&fe, 1);
	mvpp2_cls_sw_flow_pri_set(&fe, 1);
	mvpp2_cls_sw_flow_seq_set(&fe, MVPP2_CLS_FLOW_SEQ_LAST);
	mvpp2_cls_sw_flow_port_add(&fe, BIT(port->id));

	mvpp2_cls_flow_write(priv, &fe);

	/* Configure lookup */
	le.lkpid = port->id;
	/* We only use way 0 */
	le.way = 0;
	le.data = 0;

	/* Set initial CPU queue for receiving packets */
	le.data |= port->first_rxq;

	/* Set flow id */
	le.data |= MVPP2_CLS_LKP_FLOW_PTR(MVPP22_RSS_FLOW_FIRST(port->id));

	mvpp2_cls_lookup_write(port->priv, &le);
}

void mvpp2_init_rss(struct mvpp2 *priv)
{
	mvpp2_init_cls_c2(priv);
}
