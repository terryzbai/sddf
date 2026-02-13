/*
 * Copyright 2026, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * This driver is based on the Linux driver:
 *      drivers/net/ethernet/broadcom/genet/bcmgenet.c
 *      which is: Copyright (c) 2014-2017 Broadcom
 *
 * Also referred to:
 *      https://github.com/u-boot/u-boot/blob/master/drivers/net/bcmgenet.c
 *      https://github.com/RT-Thread/rt-thread/blob/master/bsp/raspberry-pi/raspi4-32/driver/drv_eth.c
 *      BCM54213PE Datasheet
 */

#include <stdbool.h>
#include <stdint.h>
#include <os/sddf.h>
#include <sddf/resources/device.h>
#include <sddf/network/queue.h>
#include <sddf/network/config.h>
#include <sddf/util/util.h>
#include <sddf/util/fence.h>
#include <sddf/util/printf.h>
#include <sddf/timer/config.h>
#include <sddf/timer/client.h>

#include <sddf/serial/queue.h> // TODO: remove
#include <sddf/serial/config.h>

serial_queue_handle_t serial_tx_queue_handle;

#include "ethernet.h"

__attribute__((__section__(".device_resources"))) device_resources_t device_resources;
__attribute__((__section__(".net_driver_config"))) net_driver_config_t config;
__attribute__((__section__(".serial_client_config"))) serial_client_config_t serial_config;
__attribute__((__section__(".timer_client_config"))) timer_client_config_t timer_config;

volatile struct genet_regs *dev_regs;
volatile struct mbox_regs *mbox_regs;
volatile uint32_t *mbox;

struct genet_dma_ring_rx *ring_rx;
struct genet_dma_ring_tx *ring_tx;

uint32_t tx_index = 0;
uint32_t rx_index = 0;
uint32_t index_flag = 0;

static void bcmgenet_mdio_write(uint8_t reg_addr, uint16_t val)
{
    uint32_t cmd = MDIO_WR | (GENET_PHY_ID << MDIO_PMD_SHIFT) | ((reg_addr & MDIO_REG_MASK) << MDIO_REG_SHIFT) | val;
    dev_regs->umac_mdio_cmd = cmd;

    uint32_t reg = dev_regs->umac_mdio_cmd | MDIO_START_BUSY;
    dev_regs->umac_mdio_cmd = reg;

    // TODO: proper way
    while (dev_regs->umac_mdio_cmd & MDIO_START_BUSY);
}

static uint16_t bcmgenet_mdio_read(uint8_t reg_addr)
{
    uint32_t cmd = MDIO_RD | (GENET_PHY_ID << MDIO_PMD_SHIFT) | ((reg_addr & MDIO_REG_MASK) << MDIO_REG_SHIFT);
    dev_regs->umac_mdio_cmd = cmd;

    uint32_t reg = dev_regs->umac_mdio_cmd | MDIO_START_BUSY;
    dev_regs->umac_mdio_cmd = reg;

    // TODO: proper way
    while (dev_regs->umac_mdio_cmd & MDIO_START_BUSY);

    return dev_regs->umac_mdio_cmd & 0xFFFF;
}

static void sleep_us(uint32_t us)
{
    uint64_t start = sddf_timer_time_now(timer_config.driver_id);
    while ((sddf_timer_time_now(timer_config.driver_id) - start) < us);
}


static void rx_return(void)
{
    uint32_t prod_index = ring_rx->prod_index;
    sddf_dprintf("Rx prod_index: %d\n", prod_index);
    sddf_dprintf("Rx cons_index: %d\n", ring_rx->cons_index);
    sddf_dprintf("Tx prod_index: %d\n", ring_tx->prod_index);
    sddf_dprintf("Tx cons_index: %d\n", ring_tx->cons_index);

    sddf_dprintf("desc[0] status: 0x%x\n", dev_regs->dma_rx.descs[0].status);
    sddf_dprintf("desc[0] addr_lo: 0x%x\n", dev_regs->dma_rx.descs[0].addr_lo);
    sddf_dprintf("desc[0] addr_hi: 0x%x\n", dev_regs->dma_rx.descs[0].addr_hi);
}


static void handle_irq(void)
{
    rx_return();
}

static void eth_setup(void)
{
    sddf_printf("ethernet driver\n");
    dev_regs = device_resources.regions[0].region.vaddr;

    uint8_t version_major = (dev_regs->sys_rev_ctrl >> 24) & 0x0f;
    if (version_major != 6) {
        sddf_printf("Unsupported GENET version\n");
    }
    sddf_printf("GENET HW version: %d\n", version_major);

    // set interface
    dev_regs->sys_port_ctrl = PORT_MODE_EXT_GPHY;

    // rbuf clear
    dev_regs->sys_rbuf_flush_ctrl = 0;

    // disable MAC while updating its registers
    dev_regs->umac_cmd = 0;
    // issue soft reset with (rg)mii loopback to ensure a stable rxclk
    dev_regs->umac_cmd = CMD_SW_RESET | CMD_LCL_LOOP_EN;

    // ========== MDIO init ==========
    // get ethernet uid
    uint32_t uid_high = bcmgenet_mdio_read(BCM54213PE_PHY_IDENTIFIER_HIGH);
    uint32_t uid_low = bcmgenet_mdio_read(BCM54213PE_PHY_IDENTIFIER_LOW);
    if (((uid_high << 16) | (uid_low & 0xFFFF)) == 0) {
        sddf_dprintf("ERROR: invalid ethernet UID '0'\n");
        return;
    }

    // reset phy
    bcmgenet_mdio_write(BCM54213PE_MII_CONTROL, MII_CONTROL_PHY_RESET);
    // read control reg
    bcmgenet_mdio_read(BCM54213PE_MII_CONTROL);
    // reset phy again
    bcmgenet_mdio_write(BCM54213PE_MII_CONTROL, MII_CONTROL_PHY_RESET);
    // read control reg
    bcmgenet_mdio_read(BCM54213PE_MII_CONTROL);
    // read status reg
    bcmgenet_mdio_read(BCM54213PE_MII_STATUS);
    // read status reg
    bcmgenet_mdio_read(BCM54213PE_IEEE_EXTENDED_STATUS);
    bcmgenet_mdio_read(BCM54213PE_AUTO_NEGOTIATION_ADV);

    bcmgenet_mdio_read(BCM54213PE_MII_STATUS);
    bcmgenet_mdio_read(BCM54213PE_CONTROL);
    // half full duplex capability
    bcmgenet_mdio_write(BCM54213PE_CONTROL, (CONTROL_HALF_DUPLEX_CAPABILITY | CONTROL_FULL_DUPLEX_CAPABILITY));
    bcmgenet_mdio_read(BCM54213PE_MII_CONTROL);

    // set mii control
    bcmgenet_mdio_write(BCM54213PE_MII_CONTROL, (MII_CONTROL_AUTO_NEGOTIATION_ENABLED | MII_CONTROL_AUTO_NEGOTIATION_RESTART | MII_CONTROL_PHY_FULL_DUPLEX | MII_CONTROL_SPEED_SELECTION));

    while (~bcmgenet_mdio_read(BCM54213PE_MII_STATUS) & MII_STATUS_AUTO_NEGOTIATION_COMPLETE);
    sddf_dprintf("finished MDIO init\n");

    // ========== Set MAC address ==========
    mbox[0] = 8*4;                                 // length of the message
    mbox[1] = MBOX_REQUEST;                        // this is a request message
    mbox[2] = MBOX_TAG_HARDWARE_GET_MAC_ADDRESS;
    mbox[3] = 6;                                   // buffer size
    mbox[4] = 0;                                   // len
    mbox[5] = 0;
    mbox[6] = 0;
    mbox[7] = MBOX_TAG_LAST;

    // 0x8 is the channel identifier for Property Tags
    unsigned int r = device_resources.regions[3].io_addr | 0x8;
    /* wait until we can write to the mailbox */
    while (mbox_regs->status & MBOX_FULL);
    // write the address of our message to the mailbox with channel identifier
    mbox_regs->write = r;
    // wait for the response
    while(1)
    {
        // check if response is ready
        while (mbox_regs->status & MBOX_EMPTY);
        // check if response is for us
        if (r == mbox_regs->read) {
            if (mbox[1] != MBOX_RESPONSE){
                sddf_dprintf("ERROR: Invalid mbox response\n");
                return;
            }
            break;
        }
    }
    char *mac = (char *)&mbox[5];
    sddf_dprintf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                 mac[0], mac[1], mac[2],
                 mac[3], mac[4], mac[5]);
    dev_regs->umac_mac0 = mac[0] << 24 | mac[1] << 16 | mac[2] << 8 | mac[3];
    dev_regs->umac_mac1 = mac[4] << 8 | mac[5];

    sddf_dprintf("mii_contrl: 0x%x\n", bcmgenet_mdio_read(BCM54213PE_MII_CONTROL));
    sddf_dprintf("mii_status: 0x%x\n", bcmgenet_mdio_read(BCM54213PE_MII_STATUS));

    // ========== Check Link Speed ==========
    uint32_t link_status = bcmgenet_mdio_read(BCM54213PE_STATUS);
    sddf_dprintf("status: 0x%x\n", link_status);
    if ((link_status & BIT(10)) | (link_status & BIT(11))) {
        sddf_dprintf("Support link mode speed 1000M\n");
    }

    // ========== UMAC Reset ==========
    sddf_dprintf("rbug_ctrl: 0x%x\n", dev_regs->sys_rbuf_flush_ctrl);
    dev_regs->sys_rbuf_flush_ctrl |= BIT(1);
    dev_regs->sys_rbuf_flush_ctrl &= ~BIT(1);
    sleep_us(10);

    dev_regs->sys_rbuf_flush_ctrl = 0;
    sleep_us(10);

    dev_regs->umac_cmd = 0;
    dev_regs->umac_cmd = CMD_SW_RESET | CMD_LCL_LOOP_EN;
    sleep_us(2);

    dev_regs->umac_cmd = 0;
    dev_regs->umac_mib_ctrl = MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT;
    dev_regs->umac_mib_ctrl = 0;
    dev_regs->umac_max_frame_len = ENET_MAX_MTU_SIZE;

    dev_regs->rbuf_ctrl |= RBUF_ALIGN_2B;
    dev_regs->rbuf_tbuf_size_ctrl = 1;
    sddf_dprintf("UMAC reset finished\n");

    // ========== Disable DMA ==========
    dev_regs->dma_tx.ctrl &= ~BIT(DMA_EN);
    dev_regs->dma_rx.ctrl &= ~BIT(DMA_EN);
    dev_regs->umac_tx_flush = 1;
    sleep_us(100);
    dev_regs->umac_tx_flush = 0;

    // ========== Rx Ring Init ==========
    sddf_dprintf("dma_rx: 0x%lx\n", &dev_regs->dma_rx);
    ring_rx = (struct genet_dma_ring_rx *)&dev_regs->dma_rx.ring_base;
    dev_regs->dma_rx.burst_size = DMA_MAX_BURST_LENGTH;
    ring_rx->start_addr = 0;
    ring_rx->read_prt = 0;
    ring_rx->write_ptr = 0;
    ring_rx->end_addr = NUM_DESCS * DESC_SIZE / 4 - 1;
    ring_rx->prod_index = 0;
    ring_rx->cons_index = 0;
    ring_rx->buf_size = (NUM_DESCS << 16) | RX_BUF_LENGTH;
    ring_rx->xon_xoff_thresh = (NUM_DESCS >> 4) | (5 << 16);
    dev_regs->dma_rx.ring_cfg = BIT(DEFAULT_Q);

    // ========== Rx Descs Init ==========
    uintptr_t rx_buf_ptr = (uintptr_t)device_resources.regions[1].region.vaddr;
    sddf_dprintf("rx_buf_prt: 0x%lx\n", rx_buf_ptr);
    for (int i = 0; i < NUM_DESCS; i++) {
        dev_regs->dma_rx.descs[i].addr_lo = (rx_buf_ptr + i * RX_BUF_LENGTH) & 0xFFFFFFFF;
        dev_regs->dma_rx.descs[i].addr_hi = (rx_buf_ptr + i * RX_BUF_LENGTH) >> 32;
        dev_regs->dma_rx.descs[i].status = (RX_BUF_LENGTH << DMA_BUFLEN_SHIFT) | DMA_OWN;
    }

    // ========== Tx Ring Init ==========
    sddf_dprintf("dma_tx: 0x%lx\n", &dev_regs->dma_tx);
    ring_tx = (struct genet_dma_ring_tx *)&dev_regs->dma_tx.ring_base;
    dev_regs->dma_tx.burst_size = DMA_MAX_BURST_LENGTH;
    ring_tx->start_addr = 0;
    // TODO: why set read_ptr multiple times in rt-thread?
    ring_tx->read_ptr = 0;
    ring_tx->write_ptr = 0;
    ring_tx->end_addr = NUM_DESCS * DESC_SIZE / 4 - 1;
    ring_tx->prod_index = 0;
    ring_tx->cons_index = 0;
    ring_tx->mbuf_done_thresh = 0x1;
    ring_tx->flow_period = 0;
    ring_tx->buf_size = (NUM_DESCS << 16) | RX_BUF_LENGTH;

    // ========== Enable DMA ==========
    uint32_t dma_ctrl = (1 << (DEFAULT_Q + DMA_RING_BUF_EN_SHIFT)) | DMA_EN;
    dev_regs->dma_tx.ctrl = dma_ctrl;
    dev_regs->dma_rx.ctrl |= dma_ctrl;

    // ========== Adjust Link ==========
    uint32_t oob_ctrl = dev_regs->ext_rgmii_oob_ctrl | RGMII_LINK | RGMII_MODE_EN | ID_MODE_DIS;
    dev_regs->ext_rgmii_oob_ctrl = oob_ctrl;
    sleep_us(1000);
    dev_regs->umac_cmd = UMAC_SPEED_1000 << CMD_SPEED_SHIFT;

    // ========== Index Reset ==========
    sddf_dprintf("Tx cons_index: 0x%x\n", ring_tx->cons_index);
    while (ring_tx->cons_index != 0);
    tx_index = ring_tx->cons_index;
    ring_tx->prod_index = tx_index;

    index_flag = ring_rx->prod_index;
    rx_index = index_flag % NUM_DESCS;
    ring_rx->cons_index = index_flag;
    ring_rx->prod_index = index_flag;

    // ========== Enable Rx/Tx ==========
    dev_regs->umac_cmd |= (CMD_TX_EN | CMD_RX_EN);

    // ========== Enable IRQ ==========
    dev_regs->intrl2_cpu_clear_mask = GENET_IRQ_RXDMA_DONE;
    sddf_dprintf("Ready\n");
}

void init(void)
{
    serial_queue_init(&serial_tx_queue_handle, serial_config.tx.queue.vaddr, serial_config.tx.data.size,
                      serial_config.tx.data.vaddr);
    serial_putchar_init(serial_config.tx.id, &serial_tx_queue_handle);

    mbox_regs = (struct mbox_regs *)0x3000880;
    mbox = device_resources.regions[3].region.vaddr;

    eth_setup();

}

void notified(sddf_channel ch)
{
    sddf_printf("notified by ch %d\n", ch);
    if (ch == device_resources.irqs[0].id) {
        handle_irq();
        sddf_deferred_irq_ack(ch);
    }
}
