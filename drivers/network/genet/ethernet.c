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

#include <sddf/serial/queue.h> // TODO: remove
#include <sddf/serial/config.h>

serial_queue_handle_t serial_tx_queue_handle;

#include "ethernet.h"

__attribute__((__section__(".device_resources"))) device_resources_t device_resources;
__attribute__((__section__(".net_driver_config"))) net_driver_config_t config;
__attribute__((__section__(".serial_client_config"))) serial_client_config_t serial_config;

volatile struct genet_regs *dev_regs;
volatile struct mbox_regs *mbox_regs;
volatile uint32_t *mbox;

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

    sddf_dprintf("reg: 0x%x\n", dev_regs->umac_mdio_cmd);
    return dev_regs->umac_mdio_cmd & 0xFFFF;
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

    // TODO: link up

    // ========== Check Link Speed ==========
    uint32_t link_status = bcmgenet_mdio_read(BCM54213PE_STATUS);
    sddf_dprintf("mii_status: 0x%lx\n", link_status);
    if ((link_status & BIT(10)) | (link_status & BIT(11))) {
        sddf_dprintf("Support link mode speed 1000M\n");
    }


    // ========== UMAC Reset ==========


    // ========== Disable DMA ==========
    // ========== Rx Ring Init ==========
    // ========== Rx Desc Init ==========
    // ========== Tx Ring Init ==========
    // ========== Enable DMA ==========
    // ========== Adjust Link ==========
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
}
