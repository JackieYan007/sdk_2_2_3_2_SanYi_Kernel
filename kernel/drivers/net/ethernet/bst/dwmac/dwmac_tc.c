// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (c) 2018 Synopsys, Inc. and/or its affiliates.
 * bstgmac TC Handling (HW only)
 */

#include <net/pkt_cls.h>
#include <net/tc_act/tc_gact.h>
#include "common.h"
#include "dwmac4.h"
#include "dwmac5.h"
#include "bstgmac.h"

static void tc_fill_all_pass_entry(struct bstgmac_tc_entry *entry)
{
	memset(entry, 0, sizeof(*entry));
	entry->in_use = true;
	entry->is_last = true;
	entry->is_frag = false;
	entry->prio = ~0x0;
	entry->handle = 0;
	entry->val.match_data = 0x0;
	entry->val.match_en = 0x0;
	entry->val.af = 1;
	entry->val.dma_ch_no = 0x0;
}

static struct bstgmac_tc_entry *tc_find_entry(struct bstgmac_priv *priv,
					     struct tc_cls_u32_offload *cls,
					     bool free)
{
	struct bstgmac_tc_entry *entry, *first = NULL, *dup = NULL;
	u32 loc = cls->knode.handle;
	int i;

	for (i = 0; i < priv->tc_entries_max; i++) {
		entry = &priv->tc_entries[i];
		if (!entry->in_use && !first && free)
			first = entry;
		if ((entry->handle == loc) && !free && !entry->is_frag)
			dup = entry;
	}

	if (dup)
		return dup;
	if (first) {
		first->handle = loc;
		first->in_use = true;

		/* Reset HW values */
		memset(&first->val, 0, sizeof(first->val));
	}

	return first;
}

static int tc_fill_actions(struct bstgmac_tc_entry *entry,
			   struct bstgmac_tc_entry *frag,
			   struct tc_cls_u32_offload *cls)
{
	struct bstgmac_tc_entry *action_entry = entry;
	const struct tc_action *act;
	struct tcf_exts *exts;
	int i;

	exts = cls->knode.exts;
	if (!tcf_exts_has_actions(exts))
		return -EINVAL;
	if (frag)
		action_entry = frag;

	tcf_exts_for_each_action(i, act, exts) {
		/* Accept */
		if (is_tcf_gact_ok(act)) {
			action_entry->val.af = 1;
			break;
		}
		/* Drop */
		if (is_tcf_gact_shot(act)) {
			action_entry->val.rf = 1;
			break;
		}

		/* Unsupported */
		return -EINVAL;
	}

	return 0;
}

static int tc_fill_entry(struct bstgmac_priv *priv,
			 struct tc_cls_u32_offload *cls)
{
	struct bstgmac_tc_entry *entry, *frag = NULL;
	struct tc_u32_sel *sel = cls->knode.sel;
	u32 off, data, mask, real_off, rem;
	u32 prio = cls->common.prio << 16;
	int ret;

	/* Only 1 match per entry */
	if (sel->nkeys <= 0 || sel->nkeys > 1)
		return -EINVAL;

	off = sel->keys[0].off << sel->offshift;
	data = sel->keys[0].val;
	mask = sel->keys[0].mask;

	switch (ntohs(cls->common.protocol)) {
	case ETH_P_ALL:
		break;
	case ETH_P_IP:
		off += ETH_HLEN;
		break;
	default:
		return -EINVAL;
	}

	if (off > priv->tc_off_max)
		return -EINVAL;

	real_off = off / 4;
	rem = off % 4;

	entry = tc_find_entry(priv, cls, true);
	if (!entry)
		return -EINVAL;

	if (rem) {
		frag = tc_find_entry(priv, cls, true);
		if (!frag) {
			ret = -EINVAL;
			goto err_unuse;
		}

		entry->frag_ptr = frag;
		entry->val.match_en = (mask << (rem * 8)) &
			GENMASK(31, rem * 8);
		entry->val.match_data = (data << (rem * 8)) &
			GENMASK(31, rem * 8);
		entry->val.frame_offset = real_off;
		entry->prio = prio;

		frag->val.match_en = (mask >> (rem * 8)) &
			GENMASK(rem * 8 - 1, 0);
		frag->val.match_data = (data >> (rem * 8)) &
			GENMASK(rem * 8 - 1, 0);
		frag->val.frame_offset = real_off + 1;
		frag->prio = prio;
		frag->is_frag = true;
	} else {
		entry->frag_ptr = NULL;
		entry->val.match_en = mask;
		entry->val.match_data = data;
		entry->val.frame_offset = real_off;
		entry->prio = prio;
	}

	ret = tc_fill_actions(entry, frag, cls);
	if (ret)
		goto err_unuse;

	return 0;

err_unuse:
	if (frag)
		frag->in_use = false;
	entry->in_use = false;
	return ret;
}

static void tc_unfill_entry(struct bstgmac_priv *priv,
			    struct tc_cls_u32_offload *cls)
{
	struct bstgmac_tc_entry *entry;

	entry = tc_find_entry(priv, cls, false);
	if (!entry)
		return;

	entry->in_use = false;
	if (entry->frag_ptr) {
		entry = entry->frag_ptr;
		entry->is_frag = false;
		entry->in_use = false;
	}
}

static int tc_config_knode(struct bstgmac_priv *priv,
			   struct tc_cls_u32_offload *cls)
{
	int ret;

	ret = tc_fill_entry(priv, cls);
	if (ret)
		return ret;

	ret = bstgmac_rxp_config(priv, priv->hw->pcsr, priv->tc_entries,
			priv->tc_entries_max);
	if (ret)
		goto err_unfill;

	return 0;

err_unfill:
	tc_unfill_entry(priv, cls);
	return ret;
}

static int tc_delete_knode(struct bstgmac_priv *priv,
			   struct tc_cls_u32_offload *cls)
{
	int ret;

	/* Set entry and fragments as not used */
	tc_unfill_entry(priv, cls);

	ret = bstgmac_rxp_config(priv, priv->hw->pcsr, priv->tc_entries,
			priv->tc_entries_max);
	if (ret)
		return ret;

	return 0;
}

static int tc_setup_cls_u32(struct bstgmac_priv *priv,
			    struct tc_cls_u32_offload *cls)
{
	switch (cls->command) {
	case TC_CLSU32_REPLACE_KNODE:
		tc_unfill_entry(priv, cls);
		/* Fall through */
	case TC_CLSU32_NEW_KNODE:
		return tc_config_knode(priv, cls);
	case TC_CLSU32_DELETE_KNODE:
		return tc_delete_knode(priv, cls);
	default:
		return -EOPNOTSUPP;
	}
}

static int tc_init(struct bstgmac_priv *priv)
{
	struct dma_features *dma_cap = &priv->dma_cap;
	unsigned int count;
	int i;

	if (dma_cap->l3l4fnum) {
		priv->flow_entries_max = dma_cap->l3l4fnum;
		priv->flow_entries = devm_kcalloc(priv->device,
						  dma_cap->l3l4fnum,
						  sizeof(*priv->flow_entries),
						  GFP_KERNEL);
		if (!priv->flow_entries)
			return -ENOMEM;

		for (i = 0; i < priv->flow_entries_max; i++)
			priv->flow_entries[i].idx = i;

		dev_info(priv->device, "Enabled Flow TC (entries=%d)\n",
			 priv->flow_entries_max);
	}

	/* Fail silently as we can still use remaining features, e.g. CBS */
	if (!dma_cap->frpsel)
		return 0;

	switch (dma_cap->frpbs) {
	case 0x0:
		priv->tc_off_max = 64;
		break;
	case 0x1:
		priv->tc_off_max = 128;
		break;
	case 0x2:
		priv->tc_off_max = 256;
		break;
	default:
		return -EINVAL;
	}

	switch (dma_cap->frpes) {
	case 0x0:
		count = 64;
		break;
	case 0x1:
		count = 128;
		break;
	case 0x2:
		count = 256;
		break;
	default:
		return -EINVAL;
	}

	/* Reserve one last filter which lets all pass */
	priv->tc_entries_max = count;
	priv->tc_entries = devm_kcalloc(priv->device,
			count, sizeof(*priv->tc_entries), GFP_KERNEL);
	if (!priv->tc_entries)
		return -ENOMEM;

	tc_fill_all_pass_entry(&priv->tc_entries[count - 1]);

	dev_info(priv->device, "Enabling HW TC (entries=%d, max_off=%d)\n",
			priv->tc_entries_max, priv->tc_off_max);
	return 0;
}

static int tc_setup_cbs(struct bstgmac_priv *priv,
			struct tc_cbs_qopt_offload *qopt)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queue = qopt->queue;
	u32 ptr, speed_div;
	u32 mode_to_use;
	u64 value;
	int ret;

	/* Queue 0 is not AVB capable */
	if (queue <= 0 || queue >= tx_queues_count)
		return -EINVAL;
	if (!priv->dma_cap.av)
		return -EOPNOTSUPP;

	mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
	if (mode_to_use == MTL_QUEUE_DCB && qopt->enable) {
		ret = bstgmac_dma_qmode(priv, priv->ioaddr, queue, MTL_QUEUE_AVB);
		if (ret)
			return ret;

		priv->plat->tx_queues_cfg[queue].mode_to_use = MTL_QUEUE_AVB;
	} else if (!qopt->enable) {
		return bstgmac_dma_qmode(priv, priv->ioaddr, queue, MTL_QUEUE_DCB);
	}

	/* Port Transmit Rate and Speed Divider */
	ptr = (priv->speed == SPEED_100) ? 4 : 8;
	speed_div = (priv->speed == SPEED_100) ? 100000 : 1000000;

	/* Final adjustments for HW */
	value = div_s64(qopt->idleslope * 1024ll * ptr, speed_div);
	priv->plat->tx_queues_cfg[queue].idle_slope = value & GENMASK(31, 0);

	value = div_s64(-qopt->sendslope * 1024ll * ptr, speed_div);
	priv->plat->tx_queues_cfg[queue].send_slope = value & GENMASK(31, 0);

	value = qopt->hicredit * 1024ll * 8;
	priv->plat->tx_queues_cfg[queue].high_credit = value & GENMASK(31, 0);

	value = qopt->locredit * 1024ll * 8;
	priv->plat->tx_queues_cfg[queue].low_credit = value & GENMASK(31, 0);

	ret = bstgmac_config_cbs(priv, priv->hw,
				priv->plat->tx_queues_cfg[queue].send_slope,
				priv->plat->tx_queues_cfg[queue].idle_slope,
				priv->plat->tx_queues_cfg[queue].high_credit,
				priv->plat->tx_queues_cfg[queue].low_credit,
				queue);
	if (ret)
		return ret;

	dev_info(priv->device, "CBS queue %d: send %d, idle %d, hi %d, lo %d\n",
			queue, qopt->sendslope, qopt->idleslope,
			qopt->hicredit, qopt->locredit);
	return 0;
}

struct timespec64 bstgmac_calc_tas_basetime(ktime_t old_base_time,
					   ktime_t current_time,
					   u64 cycle_time)
{
	struct timespec64 time;

	if (ktime_after(old_base_time, current_time)) {
		time = ktime_to_timespec64(old_base_time);
	} else {
		s64 n;
		ktime_t base_time;

		n = div64_s64(ktime_sub_ns(current_time, old_base_time),
			      cycle_time);
		base_time = ktime_add_ns(old_base_time,
					 (n + 1) * cycle_time);

		time = ktime_to_timespec64(base_time);
	}

	return time;
}

static int tc_setup_taprio(struct bstgmac_priv *priv,
			   struct tc_taprio_qopt_offload *qopt)
{
	u32 size, wid = priv->dma_cap.estwid, dep = priv->dma_cap.estdep;
	struct plat_stmmacenet_data *plat = priv->plat;
	struct timespec64 time, current_time, qopt_time;
	ktime_t current_time_ns;
	int i, ret = 0;
	u64 ctr;

	if (!priv->dma_cap.estsel)
		return -EOPNOTSUPP;

	switch (wid) {
	case 0x1:
		wid = 16;
		break;
	case 0x2:
		wid = 20;
		break;
	case 0x3:
		wid = 24;
		break;
	default:
		return -EOPNOTSUPP;
	}

	switch (dep) {
	case 0x1:
		dep = 64;
		break;
	case 0x2:
		dep = 128;
		break;
	case 0x3:
		dep = 256;
		break;
	case 0x4:
		dep = 512;
		break;
	case 0x5:
		dep = 1024;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (!qopt->enable)
		goto disable;
	if (qopt->num_entries >= dep)
		return -EINVAL;
	if (!qopt->base_time)
		return -ERANGE;
	if (!qopt->cycle_time)
		return -ERANGE;

	if (!plat->est) {
		plat->est = devm_kzalloc(priv->device, sizeof(*plat->est),
					 GFP_KERNEL);
		if (!plat->est)
			return -ENOMEM;
			
		mutex_init(&priv->plat->est->lock);
	} else {
		memset(plat->est, 0, sizeof(*plat->est));
	}

	size = qopt->num_entries;

	mutex_lock(&priv->plat->est->lock);
	priv->plat->est->gcl_size = size;
	priv->plat->est->enable = qopt->enable;
	mutex_unlock(&priv->plat->est->lock);

	for (i = 0; i < size; i++) {
		s64 delta_ns = qopt->entries[i].interval;
		u32 gates = qopt->entries[i].gate_mask;

		if (delta_ns > GENMASK(wid, 0))
			return -ERANGE;
		if (gates > GENMASK(31 - wid, 0))
			return -ERANGE;
		if (qopt->entries[i].command != TC_TAPRIO_CMD_SET_GATES)
			return -EOPNOTSUPP;

		priv->plat->est->gcl[i] = delta_ns | (gates << wid);
	}

	mutex_lock(&priv->plat->est->lock);
	/* Adjust for real system time */
	priv->ptp_clock_ops.gettime64(&priv->ptp_clock_ops, &current_time);
	current_time_ns = timespec64_to_ktime(current_time);
	time = bstgmac_calc_tas_basetime(qopt->base_time, current_time_ns,
					qopt->cycle_time);
	priv->plat->est->btr[0] = (u32)time.tv_nsec;
	priv->plat->est->btr[1] = (u32)time.tv_sec;
	qopt_time = ktime_to_timespec64(qopt->base_time);
	priv->plat->est->btr_reserve[0] = (u32)qopt_time.tv_nsec;
	priv->plat->est->btr_reserve[1] = (u32)qopt_time.tv_sec;

	ctr = qopt->cycle_time;
	priv->plat->est->ctr[0] = do_div(ctr, NSEC_PER_SEC);
	priv->plat->est->ctr[1] = (u32)ctr;

	ret = bstgmac_est_configure(priv, priv->ioaddr, priv->plat->est,
				   priv->plat->clk_ptp_rate);
	mutex_unlock(&priv->plat->est->lock);
	if (ret) {
		netdev_err(priv->dev, "failed to configure EST\n");
		goto disable;
	}

	netdev_info(priv->dev, "configured EST\n");
	return 0;

disable:
	if (!plat->est)
		return ret;
		
	mutex_lock(&priv->plat->est->lock);
	priv->plat->est->enable = false;
	bstgmac_est_configure(priv, priv->ioaddr, priv->plat->est,
			     priv->plat->clk_ptp_rate);
	mutex_unlock(&priv->plat->est->lock);
	return ret;
}

static int tc_parse_flow_actions(struct bstgmac_priv  *priv,
				 struct flow_action *action,
				 struct bstgmac_flow_entry *entry,
				 struct netlink_ext_ack *extack)
{
	struct flow_action_entry *act;
	int i;

	if (!flow_action_has_entries(action))
		return -EINVAL;

	if (!flow_action_basic_hw_stats_check(action, extack))
			return -EOPNOTSUPP;

	flow_action_for_each(i, act, action) {
		switch (act->id) {
		case FLOW_ACTION_DROP:
			entry->action |= BSTGMAC_FLOW_ACTION_DROP;
			return 0;
		default:
			break;
		}
	}

	/* Nothing to do, maybe inverse filter ? */
	return 0;
}

static int tc_add_basic_flow(struct bstgmac_priv  *priv,
				 struct flow_cls_offload *cls,
				 struct bstgmac_flow_entry *entry)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct flow_dissector *dissector = rule->match.dissector;
	struct flow_match_basic match;

	/* Nothing to do here */
	if (!dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_BASIC))
		return -EINVAL;

	flow_rule_match_basic(rule, &match);
	entry->ip_proto = match.key->ip_proto;
	return 0;
}

static int tc_add_ip4_flow(struct bstgmac_priv  *priv,
			   struct flow_cls_offload *cls,
			   struct bstgmac_flow_entry *entry)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct flow_dissector *dissector = rule->match.dissector;
	bool inv = entry->action & BSTGMAC_FLOW_ACTION_DROP;
	struct flow_match_ipv4_addrs match;
	u32 hw_match;
	int ret;

	/* Nothing to do here */
	if (!dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_IPV4_ADDRS))
		return -EINVAL;

	flow_rule_match_ipv4_addrs(rule, &match);
	hw_match = ntohl(match.key->src) & ntohl(match.mask->src);
	if (hw_match) {
		ret = bstgmac_config_l3_filter(priv, priv->hw, entry->idx, true,
						  false, true, inv, hw_match);
		if (ret)
			return ret;
	}

	hw_match = ntohl(match.key->dst) & ntohl(match.mask->dst);
	if (hw_match) {
		ret = bstgmac_config_l3_filter(priv, priv->hw, entry->idx, true,
						  false, false, inv, hw_match);
		if (ret)
			return ret;
	}

	return 0;
}

static int tc_add_ports_flow(struct bstgmac_priv  *priv,
				 struct flow_cls_offload *cls,
				 struct bstgmac_flow_entry *entry)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct flow_dissector *dissector = rule->match.dissector;
	bool inv = entry->action & BSTGMAC_FLOW_ACTION_DROP;
	struct flow_match_ports match;
	u32 hw_match;
	bool is_udp;
	int ret;

	/* Nothing to do here */
	if (!dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_PORTS))
		return -EINVAL;

	switch (entry->ip_proto) {
	case IPPROTO_TCP:
		is_udp = false;
		break;
	case IPPROTO_UDP:
		is_udp = true;
		break;
	default:
		return -EINVAL;
	}

	flow_rule_match_ports(rule, &match);

	hw_match = ntohs(match.key->src) & ntohs(match.mask->src);
	if (hw_match) {
		ret = bstgmac_config_l4_filter(priv, priv->hw, entry->idx, true,
						  is_udp, true, inv, hw_match);
		if (ret)
			return ret;
	}

	hw_match = ntohs(match.key->dst) & ntohs(match.mask->dst);
	if (hw_match) {
		ret = bstgmac_config_l4_filter(priv, priv->hw, entry->idx, true,
						  is_udp, false, inv, hw_match);
		if (ret)
			return ret;
	}

	entry->is_l4 = true;
	return 0;
}

static struct bstgmac_flow_entry *tc_find_flow(struct bstgmac_priv  *priv,
						  struct flow_cls_offload *cls,
						  bool get_free)
{
	int i;

	for (i = 0; i < priv->flow_entries_max; i++) {
		struct bstgmac_flow_entry *entry = &priv->flow_entries[i];

		if (entry->cookie == cls->cookie)
			return entry;
		if (get_free && (entry->in_use == false))
			return entry;
	}

	return NULL;
}

struct {
	int (*fn)(struct bstgmac_priv  *priv, struct flow_cls_offload *cls,
		  struct bstgmac_flow_entry *entry);
} tc_flow_parsers[] = {
	{ .fn = tc_add_basic_flow },
	{ .fn = tc_add_ip4_flow },
	{ .fn = tc_add_ports_flow },
};

static int tc_add_flow(struct bstgmac_priv  *priv,
			   struct flow_cls_offload *cls)
{
	struct bstgmac_flow_entry *entry = tc_find_flow(priv, cls, false);
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	int i, ret;
	
	if (!entry) {
		entry = tc_find_flow(priv, cls, true);
		if (!entry)
			return -ENOENT;
	}

	ret = tc_parse_flow_actions(priv, &rule->action, entry,
				    cls->common.extack);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(tc_flow_parsers); i++) {
		ret = tc_flow_parsers[i].fn(priv, cls, entry);
		if (!ret) {
			entry->in_use = true;
			continue;
		}
	}

	if (!entry->in_use)
		return -EINVAL;

	entry->cookie = cls->cookie;
	return 0;
}

static int tc_del_flow(struct bstgmac_priv  *priv,
			   struct flow_cls_offload *cls)
{
	struct bstgmac_flow_entry *entry = tc_find_flow(priv, cls, false);
	int ret;

	if (!entry || !entry->in_use)
		return -ENOENT;

	if (entry->is_l4) {
		ret = bstgmac_config_l4_filter(priv, priv->hw, entry->idx, false,
						  false, false, false, 0);
	} else {
		ret = bstgmac_config_l3_filter(priv, priv->hw, entry->idx, false,
						  false, false, false, 0);
	}

	entry->in_use = false;
	entry->cookie = 0;
	entry->is_l4 = false;
	return ret;
}

#define VLAN_PRIO_FULL_MASK (0x07)
			   
static int tc_add_vlan_flow(struct bstgmac_priv *priv,
			   struct flow_cls_offload *cls)
{
   struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
   struct flow_dissector *dissector = rule->match.dissector;
   int tc = tc_classid_to_hwtc(priv->dev, cls->classid);
   struct flow_match_vlan match;
   
   dump_stack();

   /* Nothing to do here */
   if (!dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_VLAN))
	   return -EINVAL;

   if (tc < 0) {
	   netdev_err(priv->dev, "Invalid traffic class\n");
	   return -EINVAL;
   }

   flow_rule_match_vlan(rule, &match);

   if (match.mask->vlan_priority) {
	   u32 prio;

	   if (match.mask->vlan_priority != VLAN_PRIO_FULL_MASK) {
		   netdev_err(priv->dev, "Only full mask is supported for VLAN priority");
		   return -EINVAL;
	   }

	   prio = BIT(match.key->vlan_priority);
	   bstgmac_rx_queue_prio(priv, priv->hw, prio, tc);
   }

   return 0;
}

static int tc_del_vlan_flow(struct bstgmac_priv *priv,
			   struct flow_cls_offload *cls)
{
   struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
   struct flow_dissector *dissector = rule->match.dissector;
   int tc = tc_classid_to_hwtc(priv->dev, cls->classid);

   /* Nothing to do here */
   if (!dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_VLAN))
	   return -EINVAL;

   if (tc < 0) {
	   netdev_err(priv->dev, "Invalid traffic class\n");
	   return -EINVAL;
   }

   bstgmac_rx_queue_prio(priv, priv->hw, 0, tc);

   return 0;
}

static int tc_add_flow_cls(struct bstgmac_priv *priv,
			  struct flow_cls_offload *cls)
{
   int ret;

   ret = tc_add_flow(priv, cls);
   if (!ret)
		   return ret;

   return tc_add_vlan_flow(priv, cls);
}

static int tc_del_flow_cls(struct bstgmac_priv *priv,
			  struct flow_cls_offload *cls)
{
   int ret;

   ret = tc_del_flow(priv, cls);
   if (!ret)
		   return ret;

   return tc_del_vlan_flow(priv, cls);
}

static int tc_setup_cls(struct bstgmac_priv  *priv,
			struct flow_cls_offload *cls)
{
	int ret = 0;

	switch (cls->command) {
	case FLOW_CLS_REPLACE:
		ret = tc_add_flow_cls(priv, cls);
		break;
	case FLOW_CLS_DESTROY:
		ret = tc_del_flow_cls(priv, cls);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

const struct bstgmac_tc_ops dwmac510_tc_ops = {
	.init = tc_init,
	.setup_cls_u32 = tc_setup_cls_u32,
	.setup_cbs = tc_setup_cbs,
	.setup_taprio = tc_setup_taprio,
	.setup_cls = tc_setup_cls,
};