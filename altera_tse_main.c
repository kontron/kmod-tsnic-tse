/* Altera Triple-Speed Ethernet MAC driver for the Kontron TSNIC
 * Copyright (C) 2017-2018 Kontron Europe GmbH. All rights reserved
 *
 * Altera Triple-Speed Ethernet MAC driver
 * Copyright (C) 2008-2014 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Dalon Westergreen
 *   Thomas Chou
 *   Ian Abbott
 *   Yuriy Kozlov
 *   Tobias Klauser
 *   Andriy Smolskyy
 *   Roman Bulgakov
 *   Dmytro Mytarchuk
 *   Matthew Gerlach
 *
 * Original driver contributed by SLS.
 * Major updates contributed by GlobalLogic
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>

#include "altera_utils.h"
#include "altera_tse.h"
#include "altera_msgdma.h"

static const struct altera_dmaops altera_dtype_msgdma;

/* Module parameters */
static int debug = -1;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
					NETIF_MSG_LINK | NETIF_MSG_IFUP |
					NETIF_MSG_IFDOWN);

#define RX_DESCRIPTORS 64
static int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 64
static int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");

#define MAC_LOOPBACK 0
static int mac_loopback = MAC_LOOPBACK;
module_param(mac_loopback, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mac_loopback, "Enable MAC Loopback");

#define TSN_QUEUE_THRESHOLD TC_PRIO_CONTROL
static int tsn_queue_threshold = TSN_QUEUE_THRESHOLD;
module_param(tsn_queue_threshold, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tsn_queue_threshold, "Set threshold for tsn queue");

#define MAX_TX_QUEUE_COUNT 2
static int tx_queue_count = MAX_TX_QUEUE_COUNT;
module_param(tx_queue_count, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tx_queue_count, "Set number of tx queues");


#define POLL_PHY (-1)

/* Make sure DMA buffer size is larger than the max frame size
 * plus some alignment offset and a VLAN header. If the max frame size is
 * 1518, a VLAN header would be additional 4 bytes and additional
 * headroom for alignment is 2 bytes, 2048 is just fine.
 */
#define ALTERA_RXDMABUFFER_SIZE	2048

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define TSE_TX_THRESH(x)	(x->tx_ring_size / 4)

#define TXQUEUESTOP_THRESHHOLD	2

struct altera_tse_kontron_driver_data
{
	int rx_dma_resp_offset;
	int rx_dma_desc_offset;
	int mac_dev_offset;
	int rx_dma_csr_offset;
	int tx_dma_desc_offset[2];
	int tx_dma_csr_offset[2];
	int mgmt_offset;

	const struct altera_dmaops *dmaops;
	int dmamask;

	int rx_irq;
	int tx_irq;

	int rx_fifo_depth;
	int tx_fifo_depth;
	int hash_filter;
	int added_unicast;
	int max_mtu;
};

static const struct of_device_id altera_tse_ids[];
static const struct altera_tse_kontron_driver_data kontron_drv_data[];


static inline u32 tse_tx_avail(struct altera_tse_private *priv, int queue)
{
	return priv->tx_cons[queue] + priv->tx_ring_size - priv->tx_prod[queue] - 1;
}

static int tse_init_rx_buffer(struct altera_tse_private *priv,
			      struct tse_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb_ip_align(priv->dev, len);
	if (!rxbuffer->skb)
		return -ENOMEM;

	rxbuffer->dma_addr = dma_map_single(priv->device, rxbuffer->skb->data,
						len,
						DMA_FROM_DEVICE);

	if (dma_mapping_error(priv->device, rxbuffer->dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(rxbuffer->skb);
		return -EINVAL;
	}
	rxbuffer->dma_addr &= (dma_addr_t)~3;
	rxbuffer->len = len;
	return 0;
}

static void tse_free_rx_buffer(struct altera_tse_private *priv,
			       struct tse_buffer *rxbuffer)
{
	struct sk_buff *skb = rxbuffer->skb;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	if (skb != NULL) {
		if (dma_addr)
			dma_unmap_single(priv->device, dma_addr,
					 rxbuffer->len,
					 DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		rxbuffer->skb = NULL;
		rxbuffer->dma_addr = 0;
	}
}

/* Unmap and free Tx buffer resources
 */
static void tse_free_tx_buffer(struct altera_tse_private *priv,
			       struct tse_buffer *buffer)
{
	if (buffer->dma_addr) {
		if (buffer->mapped_as_page)
			dma_unmap_page(priv->device, buffer->dma_addr,
				       buffer->len, DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device, buffer->dma_addr,
					 buffer->len, DMA_TO_DEVICE);
		buffer->dma_addr = 0;
	}
	if (buffer->skb) {
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}
}

static int alloc_init_skbufs(struct altera_tse_private *priv)
{
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->rx_ring = kcalloc(rx_descs, sizeof(struct tse_buffer),
				GFP_KERNEL);
	if (!priv->rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffers */
	for (i = 0; i < priv->num_queues; i++) {
		priv->tx_ring[i] = kcalloc(tx_descs, sizeof(struct tse_buffer),
				GFP_KERNEL);
		if (!priv->tx_ring[i])
			goto err_tx_ring;

		priv->tx_cons[i] = 0;
		priv->tx_prod[i] = 0;
	}

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = tse_init_rx_buffer(priv, &priv->rx_ring[i],
					 priv->rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->rx_cons = 0;
	priv->rx_prod = 0;

	return 0;
err_init_rx_buffers:
	while (--i >= 0)
		tse_free_rx_buffer(priv, &priv->rx_ring[i]);
	for (i = 0; i < priv->num_queues; i++)
		kfree(priv->tx_ring[i]);
err_tx_ring:
	kfree(priv->rx_ring);
err_rx_ring:
	return ret;
}

static void free_skbufs(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int i, q;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		tse_free_rx_buffer(priv, &priv->rx_ring[i]);
	for (q = 0; q < priv->num_queues; q++) {
		for (i = 0; i < tx_descs; i++)
			tse_free_tx_buffer(priv, &priv->tx_ring[q][i]);

		kfree(priv->tx_ring[q]);
	}
}

/* Reallocate the skb for the reception process
 */
static inline void tse_rx_refill(struct altera_tse_private *priv)
{
	unsigned int rxsize = priv->rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->rx_cons - priv->rx_prod > 0;
			priv->rx_prod++) {
		entry = priv->rx_prod % rxsize;
		if (likely(priv->rx_ring[entry].skb == NULL)) {
			ret = tse_init_rx_buffer(priv, &priv->rx_ring[entry],
				priv->rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->dmaops->add_rx_desc(priv, &priv->rx_ring[entry]);
		}
	}
}

/* Pull out the VLAN tag and fix up the packet
 */
static inline void tse_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct ethhdr *eth_hdr;
	u16 vid;
	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
	    !__vlan_get_tag(skb, &vid)) {
		eth_hdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, eth_hdr, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
	}
}

/* Receive a packet: retrieve and pass over to upper levels
 */
static int tse_rx(struct altera_tse_private *priv, int limit)
{
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry = priv->rx_cons % priv->rx_ring_size;
	u32 rxstatus;
	u16 pktlength;
	u16 pktstatus;

	/* Check for count < limit first as get_rx_status is changing
	* the response-fifo so we must process the next packet
	* after calling get_rx_status if a response is pending.
	* (reading the last byte of the response pops the value from the fifo.)
	*/
	while ((count < limit) &&
	       ((rxstatus = priv->dmaops->get_rx_status(priv)) != 0)) {
		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		if ((pktstatus & 0xFF) || (pktlength == 0))
			netdev_err(priv->dev,
				   "RCV pktstatus %08X pktlength %08X\n",
				   pktstatus, pktlength);

		count++;
		next_entry = (++priv->rx_cons) % priv->rx_ring_size;

		skb = priv->rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}
		priv->rx_ring[entry].skb = NULL;

		skb_put(skb, pktlength);
		/* DMA trasfer from TSE starts with 2 aditional bytes for
		 * IP payload alignment. Status returned by get_rx_status()
		 * contains DMA transfer length. Packet is 2 bytes shorter.
		 */
		skb_pull(skb, 2);

		dma_unmap_single(priv->device, priv->rx_ring[entry].dma_addr,
				 priv->rx_ring[entry].len, DMA_FROM_DEVICE);

		if (netif_msg_pktdata(priv)) {
			netdev_info(priv->dev, "frame received %d bytes\n",
				    pktlength);
			print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}

		tse_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);

		napi_gro_receive(&priv->napi, skb);

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;

		entry = next_entry;

		tse_rx_refill(priv);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int tse_tx_complete(struct altera_tse_private *priv, int q)
{
	unsigned int txsize = priv->tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct tse_buffer *tx_buff;

	ready = priv->dmaops->tx_completions(priv, q);

	/* Free sent buffers */
	while (ready && (priv->tx_cons[q] != priv->tx_prod[q])) {
		entry = priv->tx_cons[q] % txsize;
		tx_buff = &priv->tx_ring[q][entry];

		if (netif_msg_tx_done(priv))
			netdev_dbg(priv->dev, "%s: curr %d, dirty %d\n",
				   __func__, priv->tx_prod[q], priv->tx_cons[q]);

		if (likely(tx_buff->skb))
			priv->dev->stats.tx_packets++;

		tse_free_tx_buffer(priv, tx_buff);
		priv->tx_cons[q]++;
		ready = priv->dmaops->tx_completions(priv, q);
	}

	return (ready == 0);
}

/* TX NAPI polling function
 */
static int tse_poll_tx(struct napi_struct *napi, int budget)
{
	struct altera_tse_q_vector *q_vector =
			container_of(napi, struct altera_tse_q_vector, napi);
	struct altera_tse_private *priv = q_vector->priv;
	unsigned long int flags;
	int q = q_vector->queue;
	int work = budget;

	spin_lock(&q_vector->tx_lock);
	spin_lock_irqsave(&priv->txdma_irq_lock, flags);
	priv->dmaops->clear_txirq(priv, q);
	spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);

	(void) tse_tx_complete(priv, q);

	if (unlikely(netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev, q)) &&
		     tse_tx_avail(priv, q) > TSE_TX_THRESH(priv))) {
		if (netif_msg_tx_done(priv))
			netdev_dbg(priv->dev, "%s: Queue %d restart transmit\n",
				   __func__, q);
		netif_wake_subqueue(priv->dev, q);
		work = budget;

	} else {
		napi_complete(napi);

		spin_lock_irqsave(&priv->txdma_irq_lock, flags);
		priv->dmaops->enable_txirq(priv, q);
		spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);
		work = 0;
	}

	spin_unlock(&q_vector->tx_lock);

	return work;
}

/* RX NAPI polling function
 */
static int tse_poll(struct napi_struct *napi, int budget)
{
	struct altera_tse_private *priv =
			container_of(napi, struct altera_tse_private, napi);
	const struct altera_dmaops *dmaops = priv->dmaops;
	int work_done = 0;
	int work = budget;
	unsigned long int flags;

	work_done = tse_rx(priv, budget);

	if (work_done < budget && (dmaops->get_rsp_level(priv) == 0)) {
		napi_complete_done(napi, work_done);

		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   work_done, budget);

		spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
		dmaops->clear_rxirq(priv);
		dmaops->enable_rxirq(priv);
		spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);
		return 0;
	}

	return work;
}

/* DMA RX FIFO interrupt routing
 */
static irqreturn_t altera_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct altera_tse_private *priv;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

	if (likely(napi_schedule_prep(&priv->napi))) {
		spin_lock(&priv->rxdma_irq_lock);
		priv->dmaops->disable_rxirq(priv);
		spin_unlock(&priv->rxdma_irq_lock);
		__napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

/* DMA TX FIFO interrupt routing
 */
static irqreturn_t altera_isr_tx(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct altera_tse_private *priv;
	int q;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}

	priv = netdev_priv(dev);

	for (q = 0; q < priv->num_queues; q++) {
		spin_lock(&priv->txdma_irq_lock);
		/* reset TX IRQ */
		priv->dmaops->clear_txirq(priv, q);
		spin_unlock(&priv->txdma_irq_lock);

		if (likely(napi_schedule_prep(&priv->q_vector[q].napi))) {
			spin_lock(&priv->txdma_irq_lock);
			priv->dmaops->disable_txirq(priv, q);
			spin_unlock(&priv->txdma_irq_lock);
			__napi_schedule(&priv->q_vector[q].napi);
		}
	}

	return IRQ_HANDLED;
}

static inline int tse_tx_queue_mapping(struct altera_tse_private *priv,
						    struct sk_buff *skb)
{
	int q = skb->queue_mapping;

	if (q >= priv->num_queues)
		q = q % priv->num_queues;

	return q;
}

/* Transmit a packet (called by the kernel). Dispatches
 * either the SGDMA method for transmitting or the
 * MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 */
static int tse_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->tx_ring_size;
	unsigned int entry;
	struct tse_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	enum netdev_tx ret = NETDEV_TX_OK;
	dma_addr_t dma_addr;
	int q = 0;

	q = tse_tx_queue_mapping(priv, skb);

	spin_lock(&priv->q_vector[q].tx_lock);

	if (unlikely(tse_tx_avail(priv, q) < nfrags + 1)) {
		if (!netif_subqueue_stopped(dev, skb)) {
			netif_stop_subqueue(dev, q);
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx list full when queue %d awake\n",
				   __func__, q);
		}
		ret = NETDEV_TX_BUSY;
		dev->stats.tx_fifo_errors++;
		goto out;
	}

	/* Map the first skb fragment */
	entry = priv->tx_prod[q] % txsize;
	buffer = &priv->tx_ring[q][entry];

	dma_addr = dma_map_single(priv->device, skb->data, nopaged_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		ret = NETDEV_TX_OK;
		goto out;
	}

	if (dma_addr & 7)
		priv->cnt_unaligned_xmit++;

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = nopaged_len;

	priv->dmaops->tx_buffer(priv, q, buffer);

	skb_tx_timestamp(skb);

	priv->tx_prod[q]++;
	dev->stats.tx_bytes += skb->len;
	priv->cnt_queue_xmit[q]++;

	if (unlikely(tse_tx_avail(priv, q) <= TXQUEUESTOP_THRESHHOLD)) {
		if (netif_msg_hw(priv))
			netdev_dbg(priv->dev, "%s: stop transmitted packets\n",
				   __func__);
		netif_stop_subqueue(dev, q);
		if (likely(napi_schedule_prep(&priv->q_vector[q].napi))) {
			unsigned long int flags;
			spin_lock_irqsave(&priv->txdma_irq_lock, flags);
			priv->dmaops->disable_txirq(priv, q);
			spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);
			__napi_schedule(&priv->q_vector[q].napi);
		}
	}

out:
	spin_unlock(&priv->q_vector[q].tx_lock);

	return ret;
}

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
static void altera_tse_adjust_link(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;
	int new_state = 0;

	/* only change config if there is a link */
	spin_lock(&priv->mac_cfg_lock);
	if (phydev->link) {
		/* Read old config */
		u32 cfg_reg = ioread32(&priv->mac_dev->command_config);

		/* Check duplex */
		if (phydev->duplex != priv->oldduplex) {
			new_state = 1;
			if (!(phydev->duplex))
				cfg_reg |= MAC_CMDCFG_HD_ENA;
			else
				cfg_reg &= ~MAC_CMDCFG_HD_ENA;

			netdev_dbg(priv->dev, "%s: Link duplex = 0x%x\n",
				   dev->name, phydev->duplex);

			priv->oldduplex = phydev->duplex;
		}

		/* Check speed */
		if (phydev->speed != priv->oldspeed) {
			new_state = 1;
			switch (phydev->speed) {
			case 1000:
				cfg_reg |= MAC_CMDCFG_ETH_SPEED;
				cfg_reg &= ~MAC_CMDCFG_ENA_10;
				break;
			case 100:
				cfg_reg &= ~MAC_CMDCFG_ETH_SPEED;
				cfg_reg &= ~MAC_CMDCFG_ENA_10;
				break;
			case 10:
				cfg_reg &= ~MAC_CMDCFG_ETH_SPEED;
				cfg_reg |= MAC_CMDCFG_ENA_10;
				break;
			default:
				if (netif_msg_link(priv))
					netdev_warn(dev, "Speed (%d) is not 10/100/1000!\n",
						    phydev->speed);
				break;
			}
			priv->oldspeed = phydev->speed;
		}
		iowrite32(cfg_reg, &priv->mac_dev->command_config);

		if (!priv->oldlink) {
			new_state = 1;
			priv->oldlink = 1;
		}
	} else if (priv->oldlink) {
		new_state = 1;
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);

	spin_unlock(&priv->mac_cfg_lock);
}

/* Initialize driver's PHY state, and attach to the PHY
 */
static int init_phy_direct(struct net_device *dev)
{
	struct fixed_phy_status fphy_status = {
		.link = 1,
		.speed = SPEED_1000,
		.duplex = DUPLEX_FULL,
	};
	struct phy_device *phy_dev;
	int err;

	phy_dev = fixed_phy_register(PHY_POLL, &fphy_status, -1, NULL);
	if (!phy_dev || IS_ERR(phy_dev)) {
		dev_err(&dev->dev, "Failed to register fixed PHY device\n");
		return -ENODEV;
	}

	err = phy_connect_direct(dev, phy_dev, altera_tse_adjust_link,
				 PHY_INTERFACE_MODE_GMII);
	if (err) {
		dev_err(&dev->dev, "Connecting PHY failed\n");
		return err;
	}

	return err;
}

static void tse_update_mac_addr(struct altera_tse_private *priv, u8 *addr)
{
	u32 msb;
	u32 lsb;

	msb = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	lsb = ((addr[5] << 8) | addr[4]) & 0xffff;

	/* Set primary MAC address */
	csrwr32(msb, priv->mac_dev, tse_csroffs(mac_addr_0));
	csrwr32(lsb, priv->mac_dev, tse_csroffs(mac_addr_1));
}

static int reset_scheduler(struct altera_tse_private *priv)
{
	int counter;
	u32 dat;

	dat = csrrd32(priv->mgmt_dev, SCHED_CFG_OFFSET);
	dat |= SCHED_CFG_RESET;
	csrwr32(dat, priv->mgmt_dev, SCHED_CFG_OFFSET);

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if ((csrrd32(priv->mgmt_dev, SCHED_CFG_OFFSET) & SCHED_CFG_RESET) == 0) 
			break;
		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		dat = csrrd32(priv->mgmt_dev, SCHED_CFG_OFFSET);
		dat &= ~SCHED_CFG_RESET;
		csrwr32(dat, priv->mgmt_dev, SCHED_CFG_OFFSET);
		return -1;
	}

	return 0;
}

/* MAC software reset.
 * When reset is triggered, the MAC function completes the current
 * transmission or reception, and subsequently disables the transmit and
 * receive logic, flushes the receive FIFO buffer, and resets the statistics
 * counters.
 */
static int reset_mac(struct altera_tse_private *priv)
{
	int counter;
	u32 dat;

	dat = csrrd32(priv->mac_dev, tse_csroffs(command_config));
	dat &= ~(MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);
	dat |= MAC_CMDCFG_SW_RESET | MAC_CMDCFG_CNT_RESET;
	csrwr32(dat, priv->mac_dev, tse_csroffs(command_config));

	counter = 0;
	while (counter++ < ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		if (tse_bit_is_clear(priv->mac_dev, tse_csroffs(command_config),
				     MAC_CMDCFG_SW_RESET))
			break;
		udelay(1);
	}

	if (counter >= ALTERA_TSE_SW_RESET_WATCHDOG_CNTR) {
		dat = csrrd32(priv->mac_dev, tse_csroffs(command_config));
		dat &= ~MAC_CMDCFG_SW_RESET;
		csrwr32(dat, priv->mac_dev, tse_csroffs(command_config));
		return -1;
	}
	return 0;
}

/* Initialize MAC core registers
*/
static int init_mac(struct altera_tse_private *priv)
{
	unsigned int cmd = 0;
	u32 frm_length;

	/* Setup Rx FIFO */
	csrwr32(priv->rx_fifo_depth - ALTERA_TSE_RX_SECTION_EMPTY,
		priv->mac_dev, tse_csroffs(rx_section_empty));

	csrwr32(ALTERA_TSE_RX_SECTION_FULL, priv->mac_dev,
		tse_csroffs(rx_section_full));

	csrwr32(ALTERA_TSE_RX_ALMOST_EMPTY, priv->mac_dev,
		tse_csroffs(rx_almost_empty));

	csrwr32(ALTERA_TSE_RX_ALMOST_FULL, priv->mac_dev,
		tse_csroffs(rx_almost_full));

	/* Setup Tx FIFO */
	csrwr32(priv->tx_fifo_depth - ALTERA_TSE_TX_SECTION_EMPTY,
		priv->mac_dev, tse_csroffs(tx_section_empty));

	csrwr32(ALTERA_TSE_TX_SECTION_FULL, priv->mac_dev,
		tse_csroffs(tx_section_full));

	csrwr32(ALTERA_TSE_TX_ALMOST_EMPTY, priv->mac_dev,
		tse_csroffs(tx_almost_empty));

	csrwr32(ALTERA_TSE_TX_ALMOST_FULL, priv->mac_dev,
		tse_csroffs(tx_almost_full));

	/* MAC Address Configuration */
	tse_update_mac_addr(priv, priv->dev->dev_addr);

	/* MAC Function Configuration */
	frm_length = ETH_HLEN + priv->dev->mtu + ETH_FCS_LEN;
	csrwr32(frm_length, priv->mac_dev, tse_csroffs(frm_length));

	csrwr32(ALTERA_TSE_TX_IPG_LENGTH, priv->mac_dev,
		tse_csroffs(tx_ipg_length));

	/* Disable RX/TX shift 16 for alignment of all received frames on 16-bit
	 * start address
	 */
	tse_set_bit(priv->mac_dev, tse_csroffs(rx_cmd_stat),
		    ALTERA_TSE_RX_CMD_STAT_RX_SHIFT16);

	tse_clear_bit(priv->mac_dev, tse_csroffs(tx_cmd_stat),
		      ALTERA_TSE_TX_CMD_STAT_TX_SHIFT16 |
		      ALTERA_TSE_TX_CMD_STAT_OMIT_CRC);

	/* Set the MAC options */
	cmd = csrrd32(priv->mac_dev, tse_csroffs(command_config));
	cmd &= ~MAC_CMDCFG_PAD_EN;	/* No padding Removal on Receive */
	cmd &= ~MAC_CMDCFG_CRC_FWD;	/* CRC Removal */
	cmd |= MAC_CMDCFG_RX_ERR_DISC;	/* Automatically discard frames
					 * with CRC errors
					 */
	cmd |= MAC_CMDCFG_CNTL_FRM_ENA;
	cmd &= ~MAC_CMDCFG_TX_ENA;
	cmd &= ~MAC_CMDCFG_RX_ENA;

	/* Default speed and duplex setting, full/100 */
	cmd &= ~MAC_CMDCFG_HD_ENA;
	cmd &= ~MAC_CMDCFG_ETH_SPEED;
	cmd &= ~MAC_CMDCFG_ENA_10;

	csrwr32(cmd, priv->mac_dev, tse_csroffs(command_config));

	csrwr32(ALTERA_TSE_PAUSE_QUANTA, priv->mac_dev,
		tse_csroffs(pause_quanta));

	if (netif_msg_hw(priv))
		dev_dbg(priv->device,
			"MAC post-initialization: CMD_CONFIG = 0x%08x\n", cmd);

	return 0;
}

/* Start/stop MAC transmission logic
 */
static void tse_set_mac(struct altera_tse_private *priv, bool enable)
{
	u32 value = csrrd32(priv->mac_dev, tse_csroffs(command_config));

	if (enable)
		value |= MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA;
	else
		value &= ~(MAC_CMDCFG_TX_ENA | MAC_CMDCFG_RX_ENA);

	if (mac_loopback && enable)
		value |= MAC_CMDCFG_LOOP_ENA;
	else
		value &= ~MAC_CMDCFG_LOOP_ENA;

	csrwr32(value, priv->mac_dev, tse_csroffs(command_config));
}

/* Change the MTU
 */
static int tse_change_mtu(struct net_device *dev, int new_mtu)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int max_mtu = priv->max_mtu;
	unsigned int min_mtu = ETH_ZLEN + ETH_FCS_LEN;

	if (netif_running(dev)) {
		netdev_err(dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	if ((new_mtu < min_mtu) || (new_mtu > max_mtu)) {
		netdev_err(dev, "invalid MTU, max MTU is: %u\n", max_mtu);
		return -EINVAL;
	}

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

static void altera_tse_set_mcfilter(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int i;
	struct netdev_hw_addr *ha;

	/* clear the hash filter */
	for (i = 0; i < 64; i++)
		csrwr32(0, priv->mac_dev, tse_csroffs(hash_table) + i * 4);

	netdev_for_each_mc_addr(ha, dev) {
		unsigned int hash = 0;
		int mac_octet;

		for (mac_octet = 5; mac_octet >= 0; mac_octet--) {
			unsigned char xor_bit = 0;
			unsigned char octet = ha->addr[mac_octet];
			unsigned int bitshift;

			for (bitshift = 0; bitshift < 8; bitshift++)
				xor_bit ^= ((octet >> bitshift) & 0x01);

			hash = (hash << 1) | xor_bit;
		}
		csrwr32(1, priv->mac_dev, tse_csroffs(hash_table) + hash * 4);
	}
}


static void altera_tse_set_mcfilterall(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int i;

	/* set the hash filter */
	for (i = 0; i < 64; i++)
		csrwr32(1, priv->mac_dev, tse_csroffs(hash_table) + i * 4);
}

/* Set or clear the multicast filter for this adaptor
 */
static void tse_set_rx_mode_hashfilter(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	spin_lock(&priv->mac_cfg_lock);

	if (dev->flags & IFF_PROMISC)
		tse_set_bit(priv->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);

	if (dev->flags & IFF_ALLMULTI)
		altera_tse_set_mcfilterall(dev);
	else
		altera_tse_set_mcfilter(dev);

	spin_unlock(&priv->mac_cfg_lock);
}

/* Set or clear the multicast filter for this adaptor
 */
static void tse_set_rx_mode(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	spin_lock(&priv->mac_cfg_lock);

	if ((dev->flags & IFF_PROMISC) || (dev->flags & IFF_ALLMULTI) ||
	    !netdev_mc_empty(dev) || !netdev_uc_empty(dev))
		tse_set_bit(priv->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);
	else
		tse_clear_bit(priv->mac_dev, tse_csroffs(command_config),
			      MAC_CMDCFG_PROMIS_EN);

	spin_unlock(&priv->mac_cfg_lock);
}

u16 tse_select_queue(struct net_device *dev, struct sk_buff *skb,
		       void *accel_priv, select_queue_fallback_t fallback)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	if (priv->num_queues > 1)
		if (skb->priority > TSN_QUEUE_THRESHOLD)
			return 1;
	return 0;
}

/* Open and initialize the interface
 */
static int tse_open(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret = 0;
	int i;
	unsigned long int flags;

	/* Reset and configure TSE MAC and probe associated PHY */
	ret = priv->dmaops->init_dma(priv);
	if (ret != 0) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto phy_error;
	}

	if (netif_msg_ifup(priv))
		netdev_warn(dev, "device MAC address %pM\n",
			    dev->dev_addr);
	if ((priv->revision < 0xd00) || (priv->revision > 0xe00))
		netdev_warn(dev, "TSE revision %x\n", priv->revision);

	spin_lock(&priv->mac_cfg_lock);

	/* no PCS initialization, MAC does not operate in SGMII mode*/

	/* reset priority scheduler */
	ret = reset_scheduler(priv);
	if (ret)
		netdev_err(dev, "Cannot reset scheduler core (error: %d)\n", ret);

	ret = reset_mac(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
	if (ret)
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);

	ret = init_mac(priv);
	spin_unlock(&priv->mac_cfg_lock);
	if (ret) {
		netdev_err(dev, "Cannot init MAC core (error: %d)\n", ret);
		goto alloc_skbuf_error;
	}

	priv->dmaops->reset_dma(priv);

	/* Create and initialize the TX/RX descriptors chains. */
	priv->rx_ring_size = dma_rx_num;
	priv->tx_ring_size = dma_tx_num;
	ret = alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}

	/* Enable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->enable_rxirq(priv);
	/* Setup RX descriptor chain */
	for (i = 0; i < priv->rx_ring_size; i++)
		priv->dmaops->add_rx_desc(priv, &priv->rx_ring[i]);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	spin_lock_irqsave(&priv->txdma_irq_lock, flags);
	for (i = 0; i < priv->num_queues; i++)
		priv->dmaops->enable_txirq(priv, i);
	spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);

	if (dev->phydev)
		phy_start(dev->phydev);

	napi_enable(&priv->napi);
	for (i = 0; i < priv->num_queues; i++) {
		napi_enable(&priv->q_vector[i].napi);
	}

	netif_tx_start_all_queues(dev);

	priv->dmaops->start_rxdma(priv);

	/* Start MAC Rx/Tx */
	spin_lock(&priv->mac_cfg_lock);
	tse_set_mac(priv, true);
	spin_unlock(&priv->mac_cfg_lock);

	/* Register RX interrupt */
	ret = request_irq(priv->rx_irq, altera_isr, IRQF_SHARED,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register RX interrupt %d\n",
			   priv->rx_irq);
		goto init_error;
	}

	/* Register TX interrupt */
	ret = request_irq(priv->tx_irq, altera_isr_tx, IRQF_SHARED,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register TX interrupt %d\n",
			   priv->tx_irq);
		goto tx_request_irq_error;
	}

	return 0;

tx_request_irq_error:
	free_irq(priv->rx_irq, dev);
init_error:
	free_skbufs(dev);
alloc_skbuf_error:
phy_error:
	return ret;
}

/* Stop TSE MAC interface and put the device in an inactive state
 */
static int tse_shutdown(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret;
	unsigned long int flags;
	int q;

	/* Stop the PHY */
	if (dev->phydev)
		phy_stop(dev->phydev);

	netif_tx_stop_all_queues(dev);
	napi_disable(&priv->napi);
	for (q = 0; q < priv->num_queues; q++)
		napi_disable(&priv->q_vector[q].napi);

	/* Disable RX DMA interrupt */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->disable_rxirq(priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	/* Disable TX DMA interrupt */
	spin_lock_irqsave(&priv->txdma_irq_lock, flags);
	for (q = 0; q < priv->num_queues; q++)
		priv->dmaops->disable_txirq(priv, q);
	spin_unlock_irqrestore(&priv->txdma_irq_lock, flags);


	/* Free the IRQ lines */
	free_irq(priv->rx_irq, dev);
	free_irq(priv->tx_irq, dev);

	/* disable and reset the MAC, empties fifo */
	spin_lock(&priv->mac_cfg_lock);
	for (q = 0; q < priv->num_queues; q++)
		spin_lock(&priv->q_vector[q].tx_lock);

	ret = reset_mac(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
	if (ret)
		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);
	priv->dmaops->reset_dma(priv);
	free_skbufs(dev);

	for (q = 0; q < priv->num_queues; q++)
		spin_unlock(&priv->q_vector[q].tx_lock);
	spin_unlock(&priv->mac_cfg_lock);

	priv->dmaops->uninit_dma(priv);
	return 0;
}

static struct net_device_ops altera_tse_netdev_ops = {
	.ndo_open		= tse_open,
	.ndo_stop		= tse_shutdown,
	.ndo_start_xmit		= tse_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_set_rx_mode	= tse_set_rx_mode,
	.ndo_change_mtu		= tse_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_select_queue	= tse_select_queue,
};

static int altera_tse_platform_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	int ret = -ENODEV;
	struct altera_tse_private *priv;
	struct resource *res;
	struct resource *region;
	const struct altera_tse_kontron_driver_data *driver_data =
			&kontron_drv_data[0];
	void __iomem *base;
	int i;

	ndev = alloc_etherdev_mqs(sizeof(struct altera_tse_private), 2, 1);
	if (!ndev) {
		dev_err(&pdev->dev, "Could not allocate network device\n");
		return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	priv = netdev_priv(ndev);
	priv->device = &pdev->dev;
	priv->dev = ndev;
	priv->msg_enable = netif_msg_init(debug, default_msg_level);

	if (tx_queue_count < 1 || tx_queue_count > MAX_TX_QUEUE_COUNT) {
		ret= -ENODEV;
		goto error;
	}
	priv->num_queues = tx_queue_count;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "memory resource not defined\n");
		ret = -ENOMEM;
		goto error;
	}

	region = devm_request_mem_region(&pdev->dev, res->start,
					 resource_size(res), dev_name(&pdev->dev));
	if (region == NULL) {
		dev_err(&pdev->dev, "unable to request region\n");
		ret = -ENOMEM;
		goto error;
	}

	base = devm_ioremap_nocache(&pdev->dev, region->start,
			resource_size(region));
	if (base == NULL) {
		dev_err(&pdev->dev, "ioremap_nocache failed!");
		ret = -ENOMEM;
		goto error;
	}

	/* single supported DMA mode, for now? */
	priv->dmaops = driver_data->dmaops;

	if (priv->dmaops &&
			priv->dmaops->altera_dtype != ALTERA_DTYPE_MSGDMA) {
		ret = -ENODEV;
		goto err_free_netdev;
	}

	/* MAC address space */
	priv->mac_dev = base + driver_data->mac_dev_offset;
	/* management address space */
	priv->mgmt_dev = base + driver_data->mgmt_offset;
	/* xSGDMA Rx Dispatcher, CSR and response buffer address spaces */
	priv->rx_dma_csr = base + driver_data->rx_dma_csr_offset;
	priv->rx_dma_desc = base + driver_data->rx_dma_desc_offset;
	priv->rx_dma_resp = base + driver_data->rx_dma_resp_offset;

	/* setup per queue xmit counter */
	priv->cnt_queue_xmit = kzalloc(priv->num_queues* sizeof(*priv->cnt_queue_xmit),
			GFP_KERNEL);

	/* xSGDMA Tx Dispatcher address space */
	for (i = 0; i < priv->num_queues; i++) {
		priv->tx_dma_csr[i] = base + driver_data->tx_dma_csr_offset[i];
		priv->tx_dma_desc[i] = base + driver_data->tx_dma_desc_offset[i];
	}

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(driver_data->dmamask))) {
		if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
			dev_err(&pdev->dev, "No usable DMA configuration, aborting\n");
			goto err_free_netdev;
		}
	}

	priv->rx_irq = platform_get_irq(pdev, 0);
	if (priv->rx_irq < 0) {
		dev_err(&pdev->dev, "IRQ0 (%d) resource not defined\n", priv->tx_irq);
		ret = -ENOMEM;
		goto error;
	}

	priv->tx_irq = platform_get_irq(pdev, 1);
	if (priv->tx_irq < 0) {
		dev_err(&pdev->dev, "IRQ1 (%d) resource not defined\n", priv->tx_irq);
		ret = -ENOMEM;
		goto error;
	}

	/* FIFO depths must match with design */
	priv->rx_fifo_depth = driver_data->rx_fifo_depth;
	priv->tx_fifo_depth = driver_data->tx_fifo_depth;

	/* hash filter settings for this instance */
	priv->hash_filter = driver_data->hash_filter;

	/* Set hash filter to not set for now until the
	 * multicast filter receive issue is debugged
	 */
	priv->hash_filter = 0;

	/* supplemental address settings for this instance must match with design */
	priv->added_unicast = driver_data->added_unicast;

	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->max_mtu = driver_data->max_mtu;
	/* Setup max_mtu by default */
	ndev->mtu = priv->max_mtu;

	/* The DMA buffer size already accounts for an alignment bias
	 * to avoid unaligned access exceptions for the host processor,
	 */
	priv->rx_dma_buf_sz = ALTERA_RXDMABUFFER_SIZE;

	/* initialize netdev */
	ndev->mem_start = res->start;
	ndev->mem_end = res->end;

	ndev->netdev_ops = &altera_tse_netdev_ops;
	altera_tse_set_ethtool_ops(ndev);

	altera_tse_netdev_ops.ndo_set_rx_mode = tse_set_rx_mode;

	if (priv->hash_filter)
		altera_tse_netdev_ops.ndo_set_rx_mode =
				tse_set_rx_mode_hashfilter;

	/* Scatter/gather IO is not supported,
	 * so it is turned off
	 */
	ndev->hw_features &= ~NETIF_F_SG;
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;

	/* VLAN offloading of tagging, stripping and filtering is not
	 * supported by hardware, but driver will accommodate the
	 * extra 4-byte VLAN tag for processing by upper layers
	 */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;

	/* setup eth addr */
	res = platform_get_resource(pdev, IORESOURCE_REG, 0);
	if (res == NULL) {
		/* set random MAC address */
		eth_hw_addr_random(ndev);
		dev_warn(&pdev->dev, "Generating random eth_addr\n");
	} else {
		ether_addr_copy(ndev->dev_addr, (const u8*)res->start);
	}

	/* setup NAPI interface */
	netif_napi_add(ndev, &priv->napi, tse_poll, NAPI_POLL_WEIGHT);
	for (i = 0; i < priv->num_queues; i++) {
		netif_tx_napi_add(ndev, &priv->q_vector[i].napi, tse_poll_tx, 64);
		spin_lock_init(&priv->q_vector[i].tx_lock);
		priv->q_vector[i].queue = i;
		priv->q_vector[i].priv = priv;
	}

	spin_lock_init(&priv->mac_cfg_lock);
	spin_lock_init(&priv->rxdma_irq_lock);
	spin_lock_init(&priv->txdma_irq_lock);

	netif_carrier_off(ndev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register TSE net device\n");
		goto err_register_netdev;
	}

	priv->revision = ioread32(&priv->mac_dev->megacore_revision);

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "Altera TSE MAC version %d.%d, %d tx_queues at 0x%08lx irq %d/%d\n",
				(priv->revision >> 8) & 0xff,
				priv->revision & 0xff,
				priv->num_queues,
				(unsigned long) ndev->mem_start, priv->rx_irq,
				priv->tx_irq);

	ret = init_phy_direct(ndev);
	if (ret != 0) {
		netdev_err(ndev, "Cannot attach to PHY (error: %d)\n", ret);
		goto err_init_phy;
	}

	return 0;

err_init_phy:
	unregister_netdev(ndev);
err_register_netdev:
	netif_napi_del(&priv->napi);
err_free_netdev:
	free_netdev(ndev);
error:
	return ret;
}

static int altera_tse_platform_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	if (ndev->phydev) {
		phy_disconnect(ndev->phydev);
	}

	unregister_netdev(ndev);
	platform_set_drvdata(pdev, NULL);
	free_netdev(ndev);

	return 0;
}

static const struct altera_dmaops altera_dtype_msgdma = {
	.altera_dtype = ALTERA_DTYPE_MSGDMA,
	.dmamask = 64,
	.reset_dma = msgdma_reset,
	.enable_txirq = msgdma_enable_txirq,
	.enable_rxirq = msgdma_enable_rxirq,
	.disable_txirq = msgdma_disable_txirq,
	.disable_rxirq = msgdma_disable_rxirq,
	.clear_txirq = msgdma_clear_txirq,
	.clear_rxirq = msgdma_clear_rxirq,
	.tx_buffer = msgdma_tx_buffer,
	.tx_completions = msgdma_tx_completions,
	.add_rx_desc = msgdma_add_rx_desc,
	.get_rx_status = msgdma_rx_status,
	.init_dma = msgdma_initialize,
	.uninit_dma = msgdma_uninitialize,
	.start_rxdma = msgdma_start_rxdma,
	.get_rsp_level = msgdma_get_rsp_level,
};

static const struct altera_tse_kontron_driver_data kontron_drv_data[] = {
	{
		.mac_dev_offset = 0,
		.rx_dma_csr_offset = 0x840,
		.rx_dma_desc_offset = 0x860,
		.rx_dma_resp_offset = 0x880,
		.tx_dma_csr_offset = {0x800, 0x900},
		.tx_dma_desc_offset = {0x820, 0x920},
		.mgmt_offset = 0x400,

		.dmaops = &altera_dtype_msgdma,

		.rx_irq = 3,
		.tx_irq = 2,

		.rx_fifo_depth = 2048,
		.tx_fifo_depth = 2048,
		.hash_filter = 1,
		.added_unicast = 1,
		.max_mtu = ETH_DATA_LEN + 2,
	}
};

static struct platform_driver altera_tse_platform_driver = {
    .driver = {
        .name = "tsnic-tse",
    },
    .probe = altera_tse_platform_probe,
    .remove = altera_tse_platform_remove,
};

static int __init altera_tse_init_module(void)
{
	int rv;

	rv = platform_driver_register(&altera_tse_platform_driver);
	if (rv)
		pr_err("Unable to register mfd platform driver: %d\n", rv);


	return rv;
}

module_init(altera_tse_init_module);


static void altera_tse_exit_module(void)
{
	platform_driver_unregister(&altera_tse_platform_driver);
}

module_exit(altera_tse_exit_module);


MODULE_AUTHOR("Altera Corporation");
MODULE_DESCRIPTION("Altera Triple Speed Ethernet MAC driver");
MODULE_LICENSE("GPL v2");
