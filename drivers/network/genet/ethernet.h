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

#define GENET_PHY_ID 1
#define MDIO_START_BUSY             BIT(29)
#define MDIO_READ_FAIL              BIT(28)
#define MDIO_RD                     (2 << 26)
#define MDIO_WR                     BIT(26)
#define MDIO_PMD_SHIFT              (21)
#define MDIO_PMD_MASK               (0x1f)
#define MDIO_REG_SHIFT              (16)
#define MDIO_REG_MASK               (0x1f)

#define  BCM54213PE_MII_CONTROL                 (0x00)
#define  BCM54213PE_MII_STATUS                  (0x01)
#define  BCM54213PE_PHY_IDENTIFIER_HIGH         (0x02)
#define  BCM54213PE_PHY_IDENTIFIER_LOW          (0x03)

#define  BCM54213PE_AUTO_NEGOTIATION_ADV        (0x04)
#define  BCM54213PE_AUTO_NEGOTIATION_LINK       (0x05)
#define  BCM54213PE_AUTO_NEGOTIATION_EXPANSION  (0x06)

#define  BCM54213PE_NEXT_PAGE_TX                (0x07)

#define  BCM54213PE_PARTNER_RX                  (0x08)

#define  BCM54213PE_CONTROL                     (0x09)
#define  BCM54213PE_STATUS                      (0x0A)

#define  BCM54213PE_IEEE_EXTENDED_STATUS        (0x0F)
#define  BCM54213PE_PHY_EXTENDED_CONTROL        (0x10)
#define  BCM54213PE_PHY_EXTENDED_STATUS         (0x11)

#define  BCM54213PE_RECEIVE_ERROR_COUNTER       (0x12)
#define  BCM54213PE_FALSE_C_S_COUNTER           (0x13)
#define  BCM54213PE_RECEIVE_NOT_OK_COUNTER      (0x14)

#define BCM54213PE_VERSION_B1                   (0x600d84a2)
#define BCM54213PE_VERSION_X                    (0x600d84a0)

//BCM54213PE_MII_CONTROL
#define MII_CONTROL_PHY_RESET                   (1 << 15)
#define MII_CONTROL_AUTO_NEGOTIATION_ENABLED    (1 << 12)
#define MII_CONTROL_AUTO_NEGOTIATION_RESTART    (1 << 9)
#define MII_CONTROL_PHY_FULL_DUPLEX             (1 << 8)
#define MII_CONTROL_SPEED_SELECTION             (1 << 6)

//BCM54213PE_MII_STATUS
#define MII_STATUS_LINK_UP                      (1 << 2)

//BCM54213PE_CONTROL
#define CONTROL_FULL_DUPLEX_CAPABILITY          (1 << 9)
#define CONTROL_HALF_DUPLEX_CAPABILITY          (1 << 8)

struct genet_regs {
    uint32_t sys_rev_ctrl;           // 0x00
    uint32_t sys_port_ctrl;          // 0x04
    uint32_t sys_rbuf_flush_ctrl;    // 0x08
    uint32_t sys_tbuf_flush_ctrl;    // 0x0C
    uint32_t sys_unused[28];         // 0x10-0x80
    uint32_t ext_pwr_mgmt;           // 0x80
    uint32_t ext_unused1[2];         // 0x84
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
    uint32_t rbuf_unused1[44];       // 0x304-0x3B4
    uint32_t rbuf_tbuf_size_ctrl;    // 0x3B4
    uint32_t rbuf_unused2[274];      // 0x3B8-0x800
    uint32_t umac_unused1[2];        // 0x800-0x808
    uint32_t umac_cmd;               // 0x808
    uint32_t umac_mac0;              // 0x80C
    uint32_t umac_mac1;              // 0x810
    uint32_t umac_max_frame_len;     // 0x814
    uint32_t umac_unused2[204];      // 0x818-0xB48
    uint32_t umac_tx_flush;          // 0xB48
    uint32_t umac_unused3[141];      // 0xB4C-0xD80
    uint32_t umac_mib_ctrl;          // 0xD80
    uint32_t umac_unused4[36];       // 0xD80-0xE14
    uint32_t umac_mdio_cmd;          // 0xE14
};
#define MBOX_REQUEST    0
#define MBOX_TAG_HARDWARE_GET_MAC_ADDRESS  0x00010003
#define MBOX_TAG_LAST           0
#define MBOX_ADDR       0x08000000
#define MBOX_RESPONSE   0x80000000
#define MBOX_FULL       0x80000000
#define MBOX_EMPTY      0x40000000

struct mbox_regs {
    uint32_t read;                   // 0x00
    uint32_t unused1[3];             // 0x04-0x10
    uint32_t poll;                   // 0x10
    uint32_t sender;                 // 0x14
    uint32_t status;                 // 0x18
    uint32_t config;                 // 0x1C
    uint32_t write;                  // 0x20
};
