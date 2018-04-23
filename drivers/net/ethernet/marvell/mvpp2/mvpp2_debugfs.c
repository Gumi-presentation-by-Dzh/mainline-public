/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Maxime Chevallier <maxime.chevallier@bootlin.com>
 *
 * Debugfs interface for Marvell PPv2 driver
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#include "mvpp2.h"
#include "mvpp2_prs.h"

struct mvpp2_dbgfs_prs_entry {
	int tid;
	struct mvpp2 *priv;
};

static int mvpp2_dbgfs_port_vid_show(struct seq_file *s, void *unused)
{
	struct mvpp2_port *port = s->private;
	unsigned char byte[2], enable[2];
	struct mvpp2 *priv = port->priv;
	struct mvpp2_prs_entry pe;
	unsigned long pmap;
	u16 rvid;
	int tid;

	for (tid = MVPP2_PRS_VID_PORT_FIRST(port->id);
	     tid <= MVPP2_PRS_VID_PORT_LAST(port->id); tid++) {

		mvpp2_prs_init_from_hw(priv, &pe, tid);

		pmap = mvpp2_prs_tcam_port_map_get(&pe);

		if (!priv->prs_shadow[tid].valid)
			continue;

		if (!test_bit(port->id, &pmap))
			continue;

		mvpp2_prs_tcam_data_byte_get(&pe, 2, &byte[0], &enable[0]);
		mvpp2_prs_tcam_data_byte_get(&pe, 3, &byte[1], &enable[1]);

		rvid = ((byte[0] & 0xf) << 8) + byte[1];

		seq_printf(s, "%u\n", rvid);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mvpp2_dbgfs_port_vid);

static int mvpp2_dbgfs_port_parser_show(struct seq_file *s, void *unused)
{
	struct mvpp2_port *port = s->private;
	struct mvpp2 *priv = port->priv;
	struct mvpp2_prs_entry pe;
	unsigned long pmap;
	int i;

	for (i = 0; i < MVPP2_PRS_TCAM_SRAM_SIZE; i++) {
		mvpp2_prs_init_from_hw(port->priv, &pe, i);

		pmap = mvpp2_prs_tcam_port_map_get(&pe);
		if (priv->prs_shadow[i].valid && test_bit(port->id, &pmap))
			seq_printf(s, "%03d\n", i);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mvpp2_dbgfs_port_parser);

static int mvpp2_dbgfs_filter_show(struct seq_file *s, void *unused)
{
	struct mvpp2_port *port = s->private;
	struct mvpp2 *priv = port->priv;
	struct mvpp2_prs_entry pe;
	unsigned long pmap;
	int index, tid;

	for (tid = MVPP2_PE_MAC_RANGE_START;
	     tid <= MVPP2_PE_MAC_RANGE_END; tid++) {
		unsigned char da[ETH_ALEN], da_mask[ETH_ALEN];

		if (!priv->prs_shadow[tid].valid ||
		    (priv->prs_shadow[tid].lu != MVPP2_PRS_LU_MAC) ||
		    (priv->prs_shadow[tid].udf != MVPP2_PRS_UDF_MAC_DEF))
			continue;

		mvpp2_prs_init_from_hw(priv, &pe, tid);

		pmap = mvpp2_prs_tcam_port_map_get(&pe);

		/* We only want entries active on this port */
		if (!test_bit(port->id, &pmap))
			continue;

		/* Read mac addr from entry */
		for (index = 0; index < ETH_ALEN; index++)
			mvpp2_prs_tcam_data_byte_get(&pe, index, &da[index],
						     &da_mask[index]);

		seq_printf(s, "%pM\n", da);
	}

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mvpp2_dbgfs_filter);



static int mvpp2_dbgfs_prs_lu_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	struct mvpp2 *priv = entry->priv;

	seq_printf(s, "%x\n", priv->prs_shadow[entry->tid].lu);

	return 0;
}

static ssize_t mvpp2_dbgfs_prs_lu_write(struct file *file,
					const char __user *buf, size_t len,
					loff_t *offp)
{
	struct mvpp2_dbgfs_prs_entry *entry = file->f_inode->i_private;
	struct mvpp2 *priv = entry->priv;
	struct mvpp2_prs_entry pe;
	unsigned int lu;
	int ret;

	ret = kstrtouint_from_user(buf, len, 0, &lu);
	if (ret)
		return ret;

	lu &= MVPP2_PRS_LU_MASK;

	mvpp2_prs_init_from_hw(priv, &pe, entry->tid);

	mvpp2_prs_tcam_lu_set(&pe, lu);
	priv->prs_shadow[entry->tid].lu = lu;

	mvpp2_prs_hw_write(priv, &pe);

	return len;
}

static int mvpp2_dbgfs_prs_lu_open(struct inode *inode, struct file *file)
{
	return single_open(file, mvpp2_dbgfs_prs_lu_show, inode->i_private);
}

static const struct file_operations mvpp2_dbgfs_prs_lu_fops = {
	.open = mvpp2_dbgfs_prs_lu_open,
	.read = seq_read,
	.write = mvpp2_dbgfs_prs_lu_write,
	.release = seq_release,
};

static int mvpp2_dbgfs_prs_pmap_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	struct mvpp2_prs_entry pe;
	unsigned int pmap;

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	pmap = mvpp2_prs_tcam_port_map_get(&pe);
	pmap &= MVPP2_PRS_PORT_MASK;

	seq_printf(s, "%02x\n", pmap);

	return 0;
}

static ssize_t mvpp2_dbgfs_prs_pmap_write(struct file *file,
					  const char __user *buf,
					  size_t len, loff_t *offp)
{
	struct mvpp2_dbgfs_prs_entry *entry = file->f_inode->i_private;
	struct mvpp2_prs_entry pe;
	unsigned int pmap;
	int ret;

	ret = kstrtouint_from_user(buf, len, 0, &pmap);
	if (ret)
		return ret;

	pmap &= MVPP2_PRS_PORT_MASK;

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	mvpp2_prs_tcam_port_map_set(&pe, pmap);
	mvpp2_prs_hw_write(entry->priv, &pe);

	return len;
}

static int mvpp2_dbgfs_prs_pmap_open(struct inode *inode, struct file *file)
{
	return single_open(file, mvpp2_dbgfs_prs_pmap_show, inode->i_private);
}

static const struct file_operations mvpp2_dbgfs_prs_pmap_fops = {
	.open = mvpp2_dbgfs_prs_pmap_open,
	.read = seq_read,
	.write = mvpp2_dbgfs_prs_pmap_write,
	.release = seq_release,
};

static int mvpp2_dbgfs_prs_ai_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	struct mvpp2_prs_entry pe;
	unsigned char ai, ai_mask;

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	ai = pe.tcam.byte[MVPP2_PRS_TCAM_AI_BYTE];
	ai_mask = pe.tcam.byte[MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_AI_BYTE)];

	seq_printf(s, "%02x %02x\n", ai, ai_mask);

	return 0;
}

static ssize_t mvpp2_dbgfs_prs_ai_write(struct file *file,
					  const char __user *user_buf,
					  size_t len, loff_t *offp)
{
	struct mvpp2_dbgfs_prs_entry *entry = file->f_inode->i_private;
	struct mvpp2_prs_entry pe;
	int ai, ai_mask, ret = len;
	char *buf;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, len)) {
		ret = -EINVAL;
		goto out;
	}

	if (sscanf(buf, "%x %x", &ai, &ai_mask) != 2) {
		ret = -EINVAL;
		goto out;
	}

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	pe.tcam.byte[MVPP2_PRS_TCAM_AI_BYTE] = ai;
	pe.tcam.byte[MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_AI_BYTE)] = ai_mask;

	mvpp2_prs_hw_write(entry->priv, &pe);

out :
	kfree(buf);
	return ret;

}

static int mvpp2_dbgfs_prs_ai_open(struct inode *inode, struct file *file)
{
	return single_open(file, mvpp2_dbgfs_prs_ai_show, inode->i_private);
}

static const struct file_operations mvpp2_dbgfs_prs_ai_fops = {
	.open = mvpp2_dbgfs_prs_ai_open,
	.read = seq_read,
	.write = mvpp2_dbgfs_prs_ai_write,
	.release = seq_release,
};

static int mvpp2_dbgfs_prs_hdata_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	struct mvpp2_prs_entry pe;
	unsigned char data[8], mask[8];
	int i;

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	for (i = 0; i < 8; i++)
		mvpp2_prs_tcam_data_byte_get(&pe, i, &data[i], &mask[i]);

	seq_printf(s, "%*phN %*phN\n", 8, data, 8, mask);

	return 0;
}

#define GET_BYTE(data, i) ((data >> (8 * i)) & 0xff)

static ssize_t mvpp2_dbgfs_prs_hdata_write(struct file *file,
					   const char __user *user_buf,
					   size_t len, loff_t *offp)
{

	struct mvpp2_dbgfs_prs_entry *entry = file->f_inode->i_private;
	struct mvpp2_prs_entry pe;
	u64 hdata, hmask;
	int i, ret = len;
	char *buf;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, len)) {
		ret = -EINVAL;
		goto out;
	}

	if (sscanf(buf, "%llx %llx", &hdata, &hmask) != 2) {
		ret = -EINVAL;
		goto out;
	}

	hdata = cpu_to_be64(hdata);
	hmask = cpu_to_be64(hmask);

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	for (i = 0; i < 8; i++)
		mvpp2_prs_tcam_data_byte_set(&pe, i, GET_BYTE(hdata, i),
					     GET_BYTE(hmask, i));

	mvpp2_prs_hw_write(entry->priv, &pe);

out:
	kfree(buf);

	return ret;
}

static int mvpp2_dbgfs_prs_hdata_open(struct inode *inode, struct file *file)
{
	return single_open(file, mvpp2_dbgfs_prs_hdata_show, inode->i_private);
}

static const struct file_operations mvpp2_dbgfs_prs_hdata_fops = {
	.open = mvpp2_dbgfs_prs_hdata_open,
	.read = seq_read,
	.write = mvpp2_dbgfs_prs_hdata_write,
	.release = seq_release,
};

static int mvpp2_dbgfs_prs_sram_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	struct mvpp2_prs_entry pe;

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	seq_printf(s, "%*phN\n", 14, pe.sram.byte);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mvpp2_dbgfs_prs_sram);

static int mvpp2_dbgfs_prs_hits_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	int val;

	val = mvpp2_prs_hits(entry->priv, entry->tid);
	if (val < 0)
		return val;

	seq_printf(s, "%d\n", val);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(mvpp2_dbgfs_prs_hits);

static int mvpp2_dbgfs_prs_valid_show(struct seq_file *s, void *unused)
{
	struct mvpp2_dbgfs_prs_entry *entry = s->private;
	struct mvpp2 *priv = entry->priv;
	int tid = entry->tid;

	seq_printf(s, "%d\n", priv->prs_shadow[tid].valid ? 1 : 0);

	return 0;
}

static ssize_t mvpp2_dbgfs_prs_valid_write(struct file *file,
					  const char __user *buf,
					  size_t len, loff_t *offp)
{
	struct mvpp2_dbgfs_prs_entry *entry = file->f_inode->i_private;
	struct mvpp2 *priv = entry->priv;
	struct mvpp2_prs_entry pe;
	int valid, ret;
	u32 val;

	mvpp2_prs_init_from_hw(entry->priv, &pe, entry->tid);

	ret = kstrtoint_from_user(buf, len, 0, &valid);
	if (ret)
		return ret;

	/* We want to invalidate the entry without overriding the lookup id,
	 * that's why we don't use mvpp2_prs_hw_inv
	 */
	mvpp2_write(entry->priv, MVPP2_PRS_TCAM_IDX_REG, entry->tid);

	val = mvpp2_read(entry->priv,
			 MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD));

	if (valid == 0) {
		val |= MVPP2_PRS_TCAM_INV_MASK;
		priv->prs_shadow[entry->tid].valid = false;
	} else if (valid == 1) {
		val &= ~MVPP2_PRS_TCAM_INV_MASK;
		priv->prs_shadow[entry->tid].valid = true;
	} else {
		return -EINVAL;
	}

	mvpp2_write(entry->priv,
		    MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD),
		    MVPP2_PRS_TCAM_INV_MASK);

	return len;
}

static int mvpp2_dbgfs_prs_valid_open(struct inode *inode, struct file *file)
{
	return single_open(file, mvpp2_dbgfs_prs_valid_show, inode->i_private);
}

static const struct file_operations mvpp2_dbgfs_prs_valid_fops = {
	.open = mvpp2_dbgfs_prs_valid_open,
	.read = seq_read,
	.write = mvpp2_dbgfs_prs_valid_write,
};

static int mvpp2_dbgfs_prs_entry_init(struct dentry *parent,
				       struct mvpp2 *priv, int tid)
{
	struct mvpp2_dbgfs_prs_entry *entry;
	struct dentry *prs_entry_dir;
	char prs_entry_name[10];

	if (tid >= MVPP2_PRS_TCAM_SRAM_SIZE)
		return -EINVAL;

	sprintf(prs_entry_name, "%03d", tid);

	prs_entry_dir = debugfs_create_dir(prs_entry_name, parent);
	if (!prs_entry_dir)
		return -ENOMEM;

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->tid = tid;
	entry->priv = priv;

	/* Create each attr */
	debugfs_create_file("sram", 0444, prs_entry_dir, entry,
			    &mvpp2_dbgfs_prs_sram_fops);

	debugfs_create_file("valid", 0644, prs_entry_dir, entry,
			    &mvpp2_dbgfs_prs_valid_fops);

	debugfs_create_file("lookup_id", 0644, prs_entry_dir, entry,
			    &mvpp2_dbgfs_prs_lu_fops);

	debugfs_create_file("ai", 0644, prs_entry_dir, entry,
			    &mvpp2_dbgfs_prs_ai_fops);

	debugfs_create_file("header_data", 0644, prs_entry_dir, entry,
			    &mvpp2_dbgfs_prs_hdata_fops);

	debugfs_create_file("hits", 0444, prs_entry_dir, entry,
			    &mvpp2_dbgfs_prs_hits_fops);

	return 0;
}

static int mvpp2_dbgfs_prs_init(struct dentry *parent, struct mvpp2 *priv)
{
	struct dentry *prs_dir;
	int i, ret;

	prs_dir = debugfs_create_dir("parser", parent);
	if (!prs_dir)
		return -ENOMEM;

	for (i = 0; i < MVPP2_PRS_TCAM_SRAM_SIZE; i++) {
		ret = mvpp2_dbgfs_prs_entry_init(prs_dir, priv, i);
		if (ret)
			return ret;
	}

	return 0;
}

static int mvpp2_dbgfs_port_init(struct dentry *parent,
				 struct mvpp2_port *port)
{
	struct dentry *port_dir;

	port_dir = debugfs_create_dir(port->dev->name, parent);
	if (IS_ERR(port_dir))
		return PTR_ERR(port_dir);

	debugfs_create_file("parser_entries", 0444, port_dir, port,
			    &mvpp2_dbgfs_port_parser_fops);

	debugfs_create_file("mac_filter", 0444, port_dir, port,
			    &mvpp2_dbgfs_filter_fops);

	debugfs_create_file("vid_filter", 0444, port_dir, port,
			    &mvpp2_dbgfs_port_vid_fops);

	return 0;
}

void mvpp2_dbgfs_cleanup(struct mvpp2 *priv)
{
	if (priv->dbgfs_dir)
		debugfs_remove_recursive(priv->dbgfs_dir);
}

int mvpp2_dbgfs_init(struct mvpp2 *priv, const char *name)
{
	struct dentry *mvpp2_dir;
	int ret, i;

	mvpp2_dir = debugfs_create_dir(name, NULL);
	if (IS_ERR(mvpp2_dir))
		return PTR_ERR(mvpp2_dir);

	priv->dbgfs_dir = mvpp2_dir;

	ret = mvpp2_dbgfs_prs_init(mvpp2_dir, priv);
	if (ret)
		goto err;

	for (i = 0; i < priv->port_count; i++) {
		ret = mvpp2_dbgfs_port_init(mvpp2_dir, priv->port_list[i]);
		if (ret)
			goto err;
	}

	return 0;
err:
	mvpp2_dbgfs_cleanup(priv);

	return ret;
}
