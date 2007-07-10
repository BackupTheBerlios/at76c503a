/*
 * Copyright (c) 2002,2003 Oliver Kurth
 *           (c) 2003,2004 Jörg Albert <joerg.albert@gmx.de>
 *           (c) 2007 Guido Guenther <agx@sigxcpu.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This driver was based on information from the Sourceforge driver
 * released and maintained by Atmel:
 *
 *  http://sourceforge.net/projects/atmelwlandriver/
 *
 * Although the code was completely re-written,
 * it would have been impossible without Atmel's decision to
 * release an Open Source driver (unfortunately the firmware was
 * kept binary only). Thanks for that decision to Atmel!
 */

#ifndef _AT76_USB_H
#define _AT76_USB_H

#include <net/ieee80211.h>

/* current driver version */
#define DRIVER_VERSION "0.15dev"

/* our private ioctl's */
/* preamble length (0 - long, 1 - short, 2 - auto) */
#define AT76_SET_SHORT_PREAMBLE  (SIOCIWFIRSTPRIV + 0)
#define AT76_GET_SHORT_PREAMBLE  (SIOCIWFIRSTPRIV + 1)
/* which debug channels are enabled */
#define AT76_SET_DEBUG           (SIOCIWFIRSTPRIV + 2)
#define AT76_GET_DEBUG           (SIOCIWFIRSTPRIV + 3)
/* power save mode (incl. the Atmel proprietary smart save mode) */
#define AT76_SET_POWERSAVE_MODE  (SIOCIWFIRSTPRIV + 4)
#define AT76_GET_POWERSAVE_MODE  (SIOCIWFIRSTPRIV + 5)
/* min and max channel times for scan */
#define AT76_SET_SCAN_TIMES      (SIOCIWFIRSTPRIV + 6)
#define AT76_GET_SCAN_TIMES      (SIOCIWFIRSTPRIV + 7)
/* scan mode (0 - active, 1 - passive) */
#define AT76_SET_SCAN_MODE       (SIOCIWFIRSTPRIV + 8)
#define AT76_GET_SCAN_MODE       (SIOCIWFIRSTPRIV + 9)
/* international roaming (0 - disabled, 1 - enabled */
#define AT76_SET_INTL_ROAMING    (SIOCIWFIRSTPRIV + 10)
#define AT76_GET_INTL_ROAMING    (SIOCIWFIRSTPRIV + 11)

#define CMD_STATUS_IDLE                   0x00
#define CMD_STATUS_COMPLETE               0x01
#define CMD_STATUS_UNKNOWN                0x02
#define CMD_STATUS_INVALID_PARAMETER      0x03
#define CMD_STATUS_FUNCTION_NOT_SUPPORTED 0x04
#define CMD_STATUS_TIME_OUT               0x07
#define CMD_STATUS_IN_PROGRESS            0x08
#define CMD_STATUS_HOST_FAILURE           0xff
#define CMD_STATUS_SCAN_FAILED            0xf0

/* answers to get op mode */
#define OPMODE_NONE                         0x00
#define OPMODE_NORMAL_NIC_WITH_FLASH        0x01
#define OPMODE_HW_CONFIG_MODE               0x02
#define OPMODE_DFU_MODE_WITH_FLASH          0x03
#define OPMODE_NORMAL_NIC_WITHOUT_FLASH     0x04

#define CMD_SET_MIB    0x01
#define CMD_GET_MIB    0x02
#define CMD_SCAN       0x03
#define CMD_JOIN       0x04
#define CMD_START_IBSS 0x05
#define CMD_RADIO      0x06
#define CMD_STARTUP    0x0B
#define CMD_GETOPMODE  0x33

#define MIB_LOCAL      0x01
#define MIB_MAC_ADD    0x02
#define MIB_MAC        0x03
#define MIB_MAC_MGMT   0x05
#define MIB_MAC_WEP    0x06
#define MIB_PHY        0x07
#define MIB_FW_VERSION 0x08
#define MIB_MDOMAIN    0x09

#define ADHOC_MODE          1
#define INFRASTRUCTURE_MODE 2

/* values for struct mib_local, field preamble_type */
#define PREAMBLE_TYPE_LONG  0
#define PREAMBLE_TYPE_SHORT 1
#define PREAMBLE_TYPE_AUTO 2

/* values for tx_rate */
#define TX_RATE_1MBIT   0
#define TX_RATE_2MBIT   1
#define TX_RATE_5_5MBIT 2
#define TX_RATE_11MBIT  3
#define TX_RATE_AUTO    4

/* power management modes */
#define AT76_PM_OFF   1
#define AT76_PM_ON    2
#define AT76_PM_SMART 3

/* international roaming state */
#define IR_OFF        0
#define IR_ON         1

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

struct hwcfg_intersil {
	u8 mac_addr[ETH_ALEN];
	u8 cr31_values[14];
	u8 cr58_values[14];
	u8 pidvid[4];
	u8 regulatory_domain;
	u8 reserved[1];
} __attribute__ ((packed));

union at76_hwcfg {
	struct hwcfg_intersil i;
	struct hwcfg_rfmd r3;
	struct hwcfg_r505 r5;
};

#define WEP_SMALL_KEY_LEN (40/8)
#define WEP_LARGE_KEY_LEN (104/8)

struct at76_card_config {
	u8 exclude_unencrypted;
	u8 promiscuous_mode;
	u8 short_retry_limit;
	u8 encryption_type;
	__le16 rts_threshold;
	__le16 fragmentation_threshold;	/* 256..2346 */
	u8 basic_rate_set[4];
	u8 auto_rate_fallback;	/* 0,1 */
	u8 channel;
	u8 privacy_invoked;
	u8 wep_default_key_id;	/* 0..3 */
	u8 current_ssid[32];
	u8 wep_default_key_value[4][WEP_KEY_LEN];
	u8 ssid_len;
	u8 short_preamble;
	__le16 beacon_period;
} __attribute__ ((packed));

struct at76_command {
	u8 cmd;
	u8 reserved;
	__le16 size;
	u8 data[0];
} __attribute__ ((packed));

/* the length of the Atmel firmware specific rx header before IEEE 802.11 starts */
#define AT76_RX_HDRLEN offsetof(struct at76_rx_buffer, packet)

struct at76_rx_buffer {
	__le16 wlength;
	u8 rx_rate;
	u8 newbss;
	u8 fragmentation;
	u8 rssi;
	u8 link_quality;
	u8 noise_level;
	__le32 rx_time;
	u8 packet[IEEE80211_FRAME_LEN + IEEE80211_FCS_LEN];
} __attribute__ ((packed));

/* the length of the Atmel firmware specific tx header before IEEE 802.11 starts */
#define AT76_TX_HDRLEN offsetof(struct at76_tx_buffer, packet)

struct at76_tx_buffer {
	__le16 wlength;
	u8 tx_rate;
	u8 padding;
	u8 reserved[4];
	u8 packet[IEEE80211_FRAME_LEN + IEEE80211_FCS_LEN];
} __attribute__ ((packed));

/* defines for scan_type below */
#define SCAN_TYPE_ACTIVE  0
#define SCAN_TYPE_PASSIVE 1

struct at76_req_scan {
	u8 bssid[ETH_ALEN];
	u8 essid[32];
	u8 scan_type;
	u8 channel;
	__le16 probe_delay;
	__le16 min_channel_time;
	__le16 max_channel_time;
	u8 essid_size;
	u8 international_scan;
} __attribute__ ((packed));

struct at76_req_ibss {
	u8 bssid[ETH_ALEN];
	u8 essid[32];
	u8 bss_type;
	u8 channel;
	u8 essid_size;
	u8 reserved[3];
} __attribute__ ((packed));

struct at76_req_join {
	u8 bssid[ETH_ALEN];
	u8 essid[32];
	u8 bss_type;
	u8 channel;
	__le16 timeout;
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
	u8 beacon_enable;
	u8 txautorate_fallback;
	u8 reserved1;
	u8 ssid_size;
	u8 promiscuous_mode;
	u16 reserved2;
	u8 preamble_type;
	u16 reserved3;
} __attribute__ ((packed));

struct mib_mac_addr {
	u8 mac_addr[ETH_ALEN];
	u8 res[2];		/* ??? */
	u8 group_addr[4][ETH_ALEN];
	u8 group_addr_status[4];
} __attribute__ ((packed));

struct mib_mac {
	__le32 max_tx_msdu_lifetime;
	__le32 max_rx_lifetime;
	__le16 frag_threshold;
	__le16 rts_threshold;
	__le16 cwmin;
	__le16 cwmax;
	u8 short_retry_time;
	u8 long_retry_time;
	u8 scan_type;		/* active or passive */
	u8 scan_channel;
	__le16 probe_delay;	/* delay before sending a ProbeReq in active scan, RO */
	__le16 min_channel_time;
	__le16 max_channel_time;
	__le16 listen_interval;
	u8 desired_ssid[32];
	u8 desired_bssid[ETH_ALEN];
	u8 desired_bsstype;	/* ad-hoc or infrastructure */
	u8 reserved2;
} __attribute__ ((packed));

struct mib_mac_mgmt {
	__le16 beacon_period;
	__le16 CFP_max_duration;
	__le16 medium_occupancy_limit;
	__le16 station_id;	/* assoc id */
	__le16 ATIM_window;
	u8 CFP_mode;
	u8 privacy_option_implemented;
	u8 DTIM_period;
	u8 CFP_period;
	u8 current_bssid[ETH_ALEN];
	u8 current_essid[32];
	u8 current_bss_type;
	u8 power_mgmt_mode;
	/* rfmd and 505 */
	u8 ibss_change;
	u8 res;
	u8 multi_domain_capability_implemented;
	u8 multi_domain_capability_enabled;
	u8 country_string[3];
	u8 reserved[3];
} __attribute__ ((packed));

struct mib_mac_wep {
	u8 privacy_invoked;	/* 0 disable encr., 1 enable encr */
	u8 wep_default_key_id;
	u8 wep_key_mapping_len;
	u8 exclude_unencrypted;
	__le32 wep_icv_error_count;
	__le32 wep_excluded_count;
	u8 wep_default_keyvalue[WEP_KEYS][WEP_KEY_LEN];
	u8 encryption_level;	/* 1 for 40bit, 2 for 104bit encryption */
} __attribute__ ((packed));

struct mib_phy {
	__le32 ed_threshold;

	__le16 slot_time;
	__le16 sifs_time;
	__le16 preamble_length;
	__le16 plcp_header_length;
	__le16 mpdu_max_length;
	__le16 cca_mode_supported;

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
	u8 channel_list[14];	/* 0 for invalid channels */
} __attribute__ ((packed));

struct at76_fw_header {
	__le32 crc;		/* CRC32 of the whole image */
	__le32 board_type;	/* firmware compatibility code */
	u8 build;		/* firmware build number */
	u8 patch;		/* firmware patch level */
	u8 minor;		/* firmware minor version */
	u8 major;		/* firmware major version */
	__le32 str_offset;	/* offset of the copyright string */
	__le32 int_fw_offset;	/* internal firmware image offset */
	__le32 int_fw_len;	/* internal firmware image length */
	__le32 ext_fw_offset;	/* external firmware image offset */
	__le32 ext_fw_len;	/* external firmware image length */
} __attribute__ ((packed));

/* states in infrastructure mode */
enum infra_state {
	INIT,
	SCANNING,
	AUTHENTICATING,
	ASSOCIATING,
	REASSOCIATING,
	DISASSOCIATING,
	JOINING,
	CONNECTED,
	STARTIBSS,
	MONITORING,
};

/* a description of a regulatory domain and the allowed channels */
struct reg_domain {
	u16 code;
	char const *name;
	u32 channel_map;	/* if bit N is set, channel (N+1) is allowed */
};

/* how long do we keep a (I)BSS in the bss_list in jiffies 
   this should be long enough for the user to retrieve the table
   (by iwlist ?) after the device started, because all entries from
   other channels than the one the device locks on get removed, too */
#define BSS_LIST_TIMEOUT (120*HZ)
/* struct to store BSS info found during scan */
#define BSS_LIST_MAX_RATE_LEN 32	/* 32 rates should be enough ... */

struct bss_info {
	struct list_head list;

	u8 bssid[ETH_ALEN];	/* bssid */
	u8 ssid[IW_ESSID_MAX_SIZE + 1];	/* ssid, +1 for trailing \0 
					   to make it printable */
	u8 ssid_len;		/* length of ssid above */
	u8 channel;
	u16 capa;		/* BSS capabilities */
	u16 beacon_interval;	/* beacon interval in Kus (1024 microseconds) */
	u8 rates[BSS_LIST_MAX_RATE_LEN];	/* supported rates (list of bytes: 
						   (basic_rate ? 0x80 : 0) + rate/(500 Kbit/s); e.g. 
						   x82,x84,x8b,x96 for basic rates 1,2,5.5,11 MBit/s) */
	u8 rates_len;

	/* quality of received beacon */
	u8 rssi;
	u8 link_qual;
	u8 noise_level;

	unsigned long last_rx;	/* time (jiffies) of last beacon received */
	u16 assoc_id;		/* if this is priv->curr_bss this is the assoc id we got
				   in a successful AssocResponse */
};

/* a rx data buffer to collect rx fragments */
struct rx_data_buf {
	u8 sender[ETH_ALEN];	/* sender address */
	u16 seqnr;		/* sequence number */
	u16 fragnr;		/* last fragment received */
	unsigned long last_rx;	/* jiffies of last rx */
	struct sk_buff *skb;	/* == NULL if entry is free */
};

#define NR_RX_DATA_BUF 8
/* how often do we try to submit a rx urb until giving up */
#define NR_SUBMIT_RX_TRIES 8

/* Data for one loaded firmware file */
struct fwentry {
	const char *const fwname;
	const struct firmware *fw;
	int extfw_size;
	int intfw_size;
	/* these point into a buffer managed by the firmware dl functions, no need to dealloc */
	u8 *extfw;		/* points to external firmware part, extfw_size bytes long */
	u8 *intfw;		/* points to internal firmware part, intfw_size bytes long */
	u32 board_type;		/* BOARDTYPE_* in at76_usb_ids.h */
	struct mib_fw_version fw_version;
	int loaded;		/* Loaded and parsed successfully */
};

struct at76_priv {
	struct usb_device *udev;	/* USB device pointer */
	struct net_device *netdev;	/* net device pointer */
	struct net_device_stats stats;
	struct iw_statistics wstats;

	struct sk_buff *rx_skb;	/* skbuff for receiving packets */
	unsigned int rx_bulk_pipe;	/* bulk in endpoint */

	void *bulk_out_buffer;	/* the buffer to send data */
	struct urb *write_urb;	/* the urb used to send data */
	struct urb *read_urb;
	unsigned int tx_bulk_pipe;	/* bulk out endpoint */

	int open_count;		/* number of times this port has been opened */
	struct mutex mtx;	/* locks this structure */

	/* work queues */
	struct work_struct work_assoc_done;
	struct work_struct work_join;
	struct work_struct work_new_bss;
	struct work_struct work_scan;
	struct work_struct work_set_promisc;
	struct work_struct work_submit_rx;
	struct delayed_work dwork_restart;
	struct delayed_work dwork_mgmt;

	int nr_submit_rx_tries;	/* number of tries to submit an rx urb left */
	struct tasklet_struct rx_tasklet;

	/* the WEP stuff */
	int wep_enabled;	/* 1 if WEP is enabled */
	int wep_key_id;		/* key id to be used */
	u8 wep_keys[WEP_KEYS][WEP_KEY_LEN];	/* the four WEP keys,
						   5 or 13 bytes are used */
	u8 wep_keys_len[WEP_KEYS];	/* the length of the above keys */

	int channel;
	int iw_mode;
	u8 bssid[ETH_ALEN];
	u8 essid[IW_ESSID_MAX_SIZE];
	int essid_size;
	int radio_on;
	int promisc;

	int preamble_type;	/* 0 - long, 1 - short, 2 - auto */
	int auth_mode;		/* authentication type: 0 open, 1 shared key */
	int txrate;		/* 0,1,2,3 = 1,2,5.5,11 MBit, 4 is auto-fallback */
	int frag_threshold;	/* threshold for fragmentation of tx packets */
	int rts_threshold;	/* threshold for RTS mechanism */
	int short_retry_limit;

	int scan_min_time;	/* scan min channel time */
	int scan_max_time;	/* scan max channel time */
	int scan_mode;		/* SCAN_TYPE_ACTIVE, SCAN_TYPE_PASSIVE */
	int scan_runs;		/* counts how many scans are started */

	/* the list we got from scanning */
	spinlock_t bss_list_spinlock;	/* protects bss_list operations and setting
					   curr_bss and new_bss */
	struct list_head bss_list;	/* the list of bss we received beacons from */
	struct timer_list bss_list_timer;	/* a timer removing old entries from
						   the bss_list. It must acquire bss_list_spinlock
						   before and must not remove curr_bss nor
						   new_bss ! */
	struct bss_info *curr_bss;	/* if istate == AUTH, ASSOC, REASSOC, JOIN or CONN 
					   priv->bss[curr_bss] is the currently selected BSS
					   we operate on */
	struct bss_info *new_bss;	/* if istate == REASSOC priv->new_bss
					   is the new bss we want to reassoc to */

	u8 wanted_bssid[ETH_ALEN];
	int wanted_bssid_valid;	/* != 0 if wanted_bssid is to be used */

	/* some data for infrastructure mode only */
	spinlock_t mgmt_spinlock;	/* this spinlock protects access to
					   next_mgmt_bulk */

	struct at76_tx_buffer *next_mgmt_bulk;	/* pending management msg to
						   send via bulk out */
	enum infra_state istate;
	enum {
		SCAN_IDLE,
		SCAN_IN_PROGRESS,
		SCAN_COMPLETED
	} scan_state;
	time_t last_scan;

	int retries;		/* counts backwards while re-trying to send auth/assoc_req's */
	u8 pm_mode;		/* power management mode: AT76_PM_{OFF, ON, SMART} */
	u32 pm_period;		/* power manag. period in us */

	struct reg_domain const *domain;	/* the description of the regulatory domain */
	int international_roaming;

	/* iwspy support */
	spinlock_t spy_spinlock;
	struct iw_spy_data spy_data;

	struct iw_public_data wireless_data;

	/* These fields contain HW config provided by the device (not all of
	 * these fields are used by all board types) */
	u8 mac_addr[ETH_ALEN];
	u8 regulatory_domain;

	struct at76_card_config card_config;

	int rx_data_fcs_len;	/* length of the trailing FCS 
				   (0 for fw <= 0.84.x, 4 otherwise) */

	/* store rx fragments until complete */
	struct rx_data_buf rx_data[NR_RX_DATA_BUF];

	struct fwentry *fwe;
	unsigned int device_unplugged:1;
	unsigned int netdev_registered:1;
	struct set_mib_buffer mib_buf;	/* global buffer for set_mib calls */

	/* beacon counting */
	int beacon_period;	/* period of mgmt beacons */
	int beacons_received;
	unsigned long beacons_last_qual;	/* last time we reset beacons_received = 0 */
};

struct at76_rx_radiotap {
	struct ieee80211_radiotap_header rt_hdr;
	__le64 rt_tsft;
	u8 rt_flags;
	u8 rt_rate;
	s8 rt_signal;
	s8 rt_noise;
};

#define AT76_RX_RADIOTAP_PRESENT (		  \
	(1 << IEEE80211_RADIOTAP_TSFT)		| \
	(1 << IEEE80211_RADIOTAP_FLAGS))	| \
	(1 << IEEE80211_RADIOTAP_RATE)		| \
	(1 << IEEE80211_RADIOTAP_DB_ANTSIGNAL)	| \
	(1 << IEEE80211_RADIOTAP_DB_ANTNOISE)

#define BEACON_MAX_DATA_LENGTH 1500

#define DISASSOC_FRAME_SIZE \
  (AT76_TX_HDRLEN + sizeof(struct ieee80211_disassoc))

/* the maximum size of an AssocReq packet */
#define ASSOCREQ_MAX_SIZE \
  (AT76_TX_HDRLEN + sizeof(struct ieee80211_assoc_request) + \
   1+1+IW_ESSID_MAX_SIZE + 1+1+4)

/* the maximum size of a ReAssocReq packet */
#define REASSOCREQ_MAX_SIZE \
  (AT76_TX_HDRLEN + sizeof(struct ieee80211_reassoc_request) + \
   1+1+IW_ESSID_MAX_SIZE + 1+1+4)

/* for shared secret auth, add the challenge text size */
#define AUTH_FRAME_SIZE (AT76_TX_HDRLEN + sizeof(struct ieee80211_auth))

/* how often do we re-try these packets ? */
#define AUTH_RETRIES  3
#define ASSOC_RETRIES 3
#define DISASSOC_RETRIES 3

/* the beacon timeout in infra mode when we are connected (in seconds) */
#define BEACON_TIMEOUT 10
/* the interval in ticks we poll if scan is completed */
#define SCAN_POLL_INTERVAL (HZ/4)
/* the interval in ticks to wait for a command to be completed */
#define CMD_COMPLETION_TIMEOUT (5 * HZ)

#define DEF_RTS_THRESHOLD 1536
#define DEF_FRAG_THRESHOLD 1536
#define DEF_SHORT_RETRY_LIMIT 8
#define DEF_CHANNEL 10

#define MAX_RTS_THRESHOLD (MAX_FRAG_THRESHOLD + 1)

/* the max padding size for tx in bytes (see calc_padding) */
#define MAX_PADDING_SIZE 53

/* at76_debug bits */
#define DBG_PROGRESS        0x00000001	/* progress of scan-join-(auth-assoc)-connected */
#define DBG_BSS_TABLE       0x00000002	/* show the bss table after scans */
#define DBG_IOCTL           0x00000004	/* ioctl calls / settings */
#define DBG_DEVENT          0x00000008	/* at76_devents */
#define DBG_TX_DATA         0x00000010	/* tx header */
#define DBG_TX_DATA_CONTENT 0x00000020	/* tx content */
#define DBG_TX_MGMT         0x00000040
#define DBG_RX_DATA         0x00000080	/* rx data header */
#define DBG_RX_DATA_CONTENT 0x00000100	/* rx data content */
#define DBG_RX_MGMT         0x00000200	/* rx mgmt header except beacon and probe responses */
#define DBG_RX_BEACON       0x00000400	/* rx beacon */
#define DBG_RX_CTRL         0x00000800	/* rx control */
#define DBG_RX_MGMT_CONTENT 0x00001000	/* rx mgmt content */
#define DBG_RX_FRAGS        0x00002000	/* rx data fragment handling */
#define DBG_DEVSTART        0x00004000	/* fw download, device start */
#define DBG_URB             0x00008000	/* rx urb status, ... */
#define DBG_RX_ATMEL_HDR    0x00010000	/* the Atmel specific header of each rx packet */
#define DBG_PROC_ENTRY      0x00020000	/* procedure entries and exits */
#define DBG_PM              0x00040000	/* power management settings */
#define DBG_BSS_MATCH       0x00080000	/* show why a certain bss did not match */
#define DBG_PARAMS          0x00100000	/* show the configured parameters */
#define DBG_WAIT_COMPLETE   0x00200000	/* show the wait_completion progress */
#define DBG_RX_FRAGS_SKB    0x00400000	/* show skb header for incoming rx fragments */
#define DBG_BSS_TABLE_RM    0x00800000	/* inform on removal of old bss table entries */
#define DBG_MONITOR_MODE    0x01000000	/* debugs from monitor mode */
#define DBG_MIB             0x02000000	/* dump all MIBs in startup_device */
#define DBG_MGMT_TIMER      0x04000000	/* dump mgmt_timer ops */
#define DBG_WE_EVENTS       0x08000000	/* dump wireless events */
#define DBG_FW              0x10000000	/* firmware download */
#define DBG_DFU             0x20000000	/* device firmware upgrade */

#define DBG_DEFAULTS 0

/* Use our own dbg macro */
#define at76_dbg(bits, format, arg...) \
	do { \
		if (at76_debug & (bits)) \
		printk(KERN_DEBUG DRIVER_NAME ": " format "\n" , ## arg);\
	} while (0)

#define at76_assert(x) \
  do {\
   if (!(x)) \
     err("%d: assertion " #x " failed", __LINE__);\
  } while (0)

#endif				/* _AT76_USB_H */
