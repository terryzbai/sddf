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

#pragma once

#define PORT_MODE_EXT_GPHY 3
#define CMD_SW_RESET                BIT(13)
#define CMD_LCL_LOOP_EN             BIT(15)

struct genet_regs {
    uint32_t sys_rev_ctrl;           // 0x00
    uint32_t sys_port_ctrl;          // 0x04
    uint32_t sys_rbuf_flush_ctrl;    // 0x08
    uint32_t sys_tbuf_flush_ctrl;    // 0x0C
    uint32_t sys_unused[28];         // 0x10-0x80
    uint32_t ext_pwr_mgmt;           // 0x80
    uint32_t ext_unused1;            // 0x40
    uint32_t ext_rgmii_oob_ctrl;     // 0x8C
    uint32_t ext_unused2[3];         // 0x90-0x9C
    uint32_t ext_gphy_ctrl;          // 0x9C
    uint32_t ext_unused3[88];        // 0xA0-0x200
    uint32_t intrl2_cpu_start;       // 0x200
    uint32_t intrl2_unused1;         // 0x204-0x208
    uint32_t intrl2_cpu_clear;       // 0x208
    uint32_t intrl2_cpu_stat_mask;   // 0x20C
    uint32_t intrl2_cpu_set_mask;    // 0x210
    uint32_t intrl2_cpu_clear_mask;  // 0x214
    uint32_t intrl2_unused2[58];     // 0x218-0x300
    uint32_t rbuf_ctrl;              // 0x300
    uint32_t rbuf_unused1[45];       // 0x304-0x3B4
    uint32_t rbuf_tbuf_size_ctrl;    // 0x3B4
    uint32_t rbuf_unused2[286];      // 0x388-0x800
    uint32_t umac_unused[2];         // 0x800-0x808
    uint32_t umac_cmd;               // 0x808
    uint32_t umac_mac0;              // 0x80C
    uint32_t umac_mac1;              // 0x810
    uint32_t umac_max_frame_len;     // 0x814
    uint32_t umac_tx_flush;          // 0xB48
    uint32_t umac_mib_ctrl;          // 0xD80
    uint32_t umac_mdio_cmd;          // 0x614
};
