/* -*- linux-c -*- */
/*
 * USB at76c503 driver
 *
 * Copyright (c) 2002 - 2003 Oliver Kurth <oku@masqmail.cx>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 */

#define DEVICE_VENDOR_REQUEST_OUT    0x40
#define DEVICE_VENDOR_REQUEST_IN     0xc0
#define INTERFACE_VENDOR_REQUEST_OUT 0x41
#define INTERFACE_VENDOR_REQUEST_IN  0xc1
#define CLASS_REQUEST_OUT            0x21
#define CLASS_REQUEST_IN             0xa1

#define CMD_STATUS_IDLE                   0x00
#define CMD_STATUS_COMPLETE               0x01
#define CMD_STATUS_UNKNOWN                0x02
#define CMD_STATUS_INVALID_PARAMETER      0x03
#define CMD_STATUS_FUNCTION_NOT_SUPPORTED 0x04
#define CMD_STATUS_TIME_OUT               0x07
#define CMD_STATUS_IN_PROGRESS            0x08
#define CMD_STATUS_HOST_FAILURE           0xff
#define CMD_STATUS_SCAN_FAILED            0xf0

#define CMD_SET_MIB    0x01
#define CMD_GET_MIB    0x02
#define CMD_SCAN       0x03
#define CMD_JOIN       0x04
#define CMD_START_IBSS 0x05
#define CMD_RADIO      0x06
#define CMD_STARTUP    0x0B

#define MIB_LOCAL      0x01
#define MIB_MAC_ADD    0x02
#define MIB_MAC        0x03
#define MIB_MAC_MGMT   0x05
#define MIB_MAC_WEP    0x06
#define MIB_PHY        0x07
#define MIB_FW_VERSION 0x08
#define MIB_MDOMAIN    0x09

#define ADHOC_MODE 1
#define INFRASTRUCTURE_MODE 2

/* values for struct mib_local, field preamble_type */
#define PREAMBLE_TYPE_SHORT 1
#define PREAMBLE_TYPE_LONG  0

/* values for tx_rate */
#define TX_RATE_1MBIT 0
#define TX_RATE_2MBIT 1
#define TX_RATE_5_5MBIT 2
#define TX_RATE_11MBIT 3
#define TX_RATE_AUTO 4

/* offsets into the MIBs we use to configure the device */
#define TX_AUTORATE_FALLBACK_OFFSET offsetof(struct mib_local,txautorate_fallback)
#define FRAGMENTATION_OFFSET        offsetof(struct mib_mac,frag_threshold)
#define PREAMBLE_TYPE_OFFSET        offsetof(struct mib_local,preamble_type)
#define RTS_OFFSET                  offsetof(struct mib_mac, rts_threshold)      

/* valid only for rfmd and 505 !*/
#define IBSS_CHANGE_OK_OFFSET       offsetof(struct mib_mac_mgmt, ibss_change)
#define IROAMING_OFFSET \
  offsetof(struct mib_mac_mgmt, multi_domain_capability_enabled)

struct hwcfg_r505 {
	u8 cr39_values[14];
	u8 reserved1[14];
	u8 bb_cr[14];
	u8 pidvid[4];
	u8 mac_addr[ETH_ALEN];
	u8 regulatory_domain;
	u8 reserved2[14];
	u8 cr15_values[14];
	u8 reserved3[3];
} __attribute__ ((packed));

struct hwcfg_rfmd {
	u8 cr20_values[14]; 
	u8 cr21_values[14]; 
	u8 bb_cr[14]; 
	u8 pidvid[4]; 
	u8 mac_addr[ETH_ALEN]; 
	u8 regulatory_domain; 
	u8 low_power_values[14];     
	u8 normal_power_values[14]; 
	u8 reserved1[3];   
} __attribute__ ((packed));

struct hwcfg_other {
	u8   mac_addr[ETH_ALEN];
	u8   cr31[14];
	u8   cr58[14];
	u8   vidpid[4];
	u8   reg_domain;
	u8   reserved[1];
} __attribute__ ((packed));

#define WEP_KEY_SIZE 13
#define NR_WEP_KEYS 4
#define WEP_SMALL_KEY_LEN (40/8)
#define WEP_LARGE_KEY_LEN (104/8)

struct at76c503_card_config{
	u8 exclude_unencrypted;
	u8 promiscuous_mode;
	u8 short_retry_limit;
	u8 encryption_type;
	u16 rts_threshold;
	u16 fragmentation_threshold;         // 256..2346
	u8 basic_rate_set[4];
	u8 auto_rate_fallback;                       //0,1
	u8 channel;
	u8 privacy_invoked;
	u8 wep_default_key_id;                        // 0..3
	u8 current_ssid[32];
	u8 wep_default_key_value[4][WEP_KEY_SIZE];
	u8 ssid_len;
	u8 short_preamble;
	u16 beacon_period;
} __attribute__ ((packed));

struct at76c503_command{
	u8 cmd;
	u8 reserved;
	u16 size;
} __attribute__ ((packed));

#define MAX_PACKET 1536

struct at76c503_rx_buffer {
	u16 wlength;
	u8 rx_rate;
	u8 newbss;
	u8 fragmentation;
	u8 rssi;
	u8 link_quality;
	u8 noise_level;
	u8 rx_time[4];
	u8 packet[MAX_PACKET];
} __attribute__ ((packed));

struct at76c503_tx_buffer {
	u16 wlength;
	u8 tx_rate;
	u8 padding;
	u8 reserved[4];
	u8 packet[MAX_PACKET];
} __attribute__ ((packed));

/* defines for scan_type below */
#define SCAN_TYPE_ACTIVE  0
#define SCAN_TYPE_PASSIVE 1

struct at76c503_start_scan {
	u8   bssid[ETH_ALEN];
	u8   essid[32];
	u8   scan_type;
	u8   channel;
	u16  probe_delay;
	u16  min_channel_time;
	u16  max_channel_time;
	u8   essid_size;
	u8   international_scan;
} __attribute__ ((packed));

struct at76c503_start_bss {
	u8 bssid[ETH_ALEN];
	u8 essid[32];
	u8 bss_type;
	u8 channel;
	u8 essid_size;
	u8 reserved[3];
} __attribute__ ((packed));

struct at76c503_join {
	u8 bssid[ETH_ALEN];
	u8 essid[32];
	u8 bss_type;
	u8 channel;
	u16 timeout;
	u8 essid_size;
	u8 reserved;
} __attribute__ ((packed));

struct set_mib_buffer {
	u8 type;
	u8 size;
	u8 index;
	u8 reserved;
	u8 data[72];
} __attribute__ ((packed));

struct mib_local {
        u16 reserved0;
        u8  beacon_enable;
        u8  txautorate_fallback;
        u8  reserved1;
        u8  ssid_size;
        u8  promiscuous_mode;
        u16 reserved2;
        u8  preamble_type;
        u16 reserved3;
} __attribute__ ((packed));

struct mib_mac_addr {
	u8 mac_addr[ETH_ALEN];
        u8 res[2]; /* ??? */
        u8 group_addr[4][ETH_ALEN];
        u8 group_addr_status[4];
} __attribute__ ((packed));

struct mib_mac {
        u32 max_tx_msdu_lifetime;
        u32 max_rx_lifetime;
        u16 frag_threshold;
        u16 rts_threshold;
        u16 cwmin;
        u16 cwmax;
        u8  short_retry_time;
        u8  long_retry_time;
        u8  scan_type; /* active or passive */
        u8  scan_channel;
        u16 probe_delay; /* delay before sending a ProbeReq in active scan, RO */
        u16 min_channel_time;
        u16 max_channel_time;
        u16 listen_interval;
        u8  desired_ssid[32];
        u8  desired_bssid[ETH_ALEN];
        u8  desired_bsstype; /* ad-hoc or infrastructure */
        u8  reserved2;
} __attribute__ ((packed));

struct mib_mac_mgmt {
	u16 beacon_period;
	u16 CFP_max_duration;
	u16 medium_occupancy_limit;
	u16 station_id;
	u16 ATIM_window;
	u8  CFP_mode;
	u8  privacy_option_implemented;
	u8  DTIM_period;
	u8  CFP_period;
	u8  current_bssid[ETH_ALEN];
	u8  current_essid[32];
	u8  current_bss_type;
	u8  power_mgmt_mode;
	/* rfmd and 505 */
	u8  ibss_change;
	u8  res;
	u8  multi_domain_capability_implemented;
	u8  multi_domain_capability_enabled;
	u8  country_string[3];
	u8  reserved[3];
} __attribute__ ((packed));

struct mib_mac_wep {
        u8 privacy_invoked; /* 0 disable encr., 1 enable encr */
        u8 wep_default_key_id;
        u8 wep_key_mapping_len;
        u8 exclude_unencrypted;
        u32 wep_icv_error_count;
        u32 wep_excluded_count;
        u8 wep_default_keyvalue[NR_WEP_KEYS][WEP_KEY_SIZE];
        u8 encryption_level; /* 1 for 40bit, 2 for 104bit encryption */
} __attribute__ ((packed));

struct mib_phy {
	u32 ed_threshold;
  
	u16 slot_time;
	u16 sifs_time;
	u16 preamble_length;
	u16 plcp_header_length;
	u16 mpdu_max_length;
	u16 cca_mode_supported;
  
	u8 operation_rate_set[4];
	u8 channel_id;
	u8 current_cca_mode;
	u8 phy_type;
	u8 current_reg_domain;
} __attribute__ ((packed));

struct mib_fw_version {
        u8 major;
        u8 minor;
        u8 patch;
        u8 build;
} __attribute__ ((packed));

struct mib_mdomain {
        u8 tx_powerlevel[14];
        u8 channel_list[14]; /* 0 for invalid channels */
} __attribute__ ((packed));


