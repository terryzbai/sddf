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

static void eth_setup(void)
{
    serial_queue_init(&serial_tx_queue_handle, serial_config.tx.queue.vaddr, serial_config.tx.data.size,
                      serial_config.tx.data.vaddr);
    serial_putchar_init(serial_config.tx.id, &serial_tx_queue_handle);

    sddf_printf("ethernet driver\n");
    dev_regs = device_resources.regions[0].region.vaddr;

    uint8_t version_major = (dev_regs->sys_rev_ctrl >> 24) & 0x0f;
    if (version_major != 6) {
        sddf_printf("Unsupported GENET version\n");
    }
    sddf_printf("GENET HW version: %d\n", version_major);

    // set interface
    dev_regs->sys_port_ctrl = PORT_MODE_EXT_GPHY;

    // disable MAC while updating its registers
    dev_regs->umac_cmd = 0;
    // issue soft reset with (rg)mii loopback to ensure a stable rxclk
    dev_regs->umac_cmd = CMD_SW_RESET | CMD_LCL_LOOP_EN;
}

void init(void)
{
    eth_setup();
}

void notified(sddf_channel ch)
{
    sddf_printf("notified by ch %d\n", ch);
}
