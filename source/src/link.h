/* unofficial gameplaySP kai
 *
 * Link cable emulation over PSP ad-hoc Wi-Fi.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Design notes
 * ------------
 * - Everything network-related lives on one dedicated "Link thread". The
 *   emulation thread never blocks on the network: link_send() is a
 *   non-blocking datagram send, and link_recv() pops from a lock-free
 *   single-producer/single-consumer ring filled by the link thread.
 * - Nothing is initialised until link_start() is called, so the Wi-Fi stack
 *   and its memory pool cost nothing when link mode is off.
 * - The link thread answers protocol-level traffic itself (HELLO); game data
 *   (LINK_MSG_SIO) is only queued for the emulation core. Replies that must
 *   be fast (e.g. a slave answering a master's poll) are meant to be
 *   generated from the link thread, not from the frame loop: the measured
 *   PDP round trip is ~3.4 ms median with an occasional ~15 ms tail, while a
 *   frame-polled round trip costs two whole frames (~33 ms).
 */

#ifndef LINK_H
#define LINK_H

#include <stdint.h>

#define LINK_MAX_PLAYERS    (4)
#define LINK_PAYLOAD_MAX    (16)
#define LINK_GROUP_NAME     "GBALINK"   /* ad-hoc group, max 8 alphanumerics */

#define LINK_SLOT_NONE      (-1)
#define LINK_SLOT_BROADCAST (0xFF)

typedef enum
{
  LINK_STATE_OFF,          /* nothing initialised */
  LINK_STATE_STARTING,     /* loading modules / bringing the stack up */
  LINK_STATE_CONNECTING,   /* joining the ad-hoc group */
  LINK_STATE_CONNECTED,    /* in the group, PDP socket open */
  LINK_STATE_ERROR         /* see link_get_error() */
} LINK_STATE;

typedef enum
{
  LINK_ROLE_HOST,          /* becomes player 0 (the "cable master") */
  LINK_ROLE_JOIN
} LINK_ROLE;

typedef enum
{
  LINK_ERR_NONE = 0,
  LINK_ERR_WLAN_OFF,       /* PSP WLAN switch is off */
  LINK_ERR_MODULE_LOAD,    /* sceUtilityLoadNetModule failed */
  LINK_ERR_STACK_INIT,     /* sceNetInit / Adhoc / Adhocctl init failed */
  LINK_ERR_CONNECT,        /* could not join the ad-hoc group */
  LINK_ERR_SOCKET,         /* sceNetAdhocPdpCreate failed */
  LINK_ERR_THREAD          /* could not create the link thread */
} LINK_ERROR;

/* Message types carried in link_msg.type */
typedef enum
{
  LINK_MSG_HELLO = 1,      /* internal: peer discovery (never queued) */
  LINK_MSG_SIO   = 2,      /* SIO register exchange payload */
  LINK_MSG_USER  = 3       /* reserved for later protocol work */
} LINK_MSG_TYPE;

/* A received message, as handed to the emulation core. */
typedef struct
{
  uint8_t  type;                     /* LINK_MSG_TYPE */
  int8_t   src_slot;                 /* sender's player slot, LINK_SLOT_NONE if unknown */
  uint16_t seq;                      /* sender's sequence number */
  uint8_t  len;                      /* valid bytes in data[] */
  uint8_t  data[LINK_PAYLOAD_MAX];
  uint32_t rx_time_us;               /* sceKernelGetSystemTimeLow() at arrival */
} link_msg;

typedef struct
{
  uint32_t tx_packets;
  uint32_t rx_packets;
  uint32_t rx_dropped_full;          /* ring was full */
  uint32_t rx_dropped_bad;           /* wrong magic / short / oversized */
  uint32_t tx_errors;
} link_stats;

/* ---- lifecycle (call from the emulation/GUI thread) ---- */

/* Begin bringing the link up. Returns 0 immediately if the link thread was
 * started (progress is reported by link_get_state()), <0 on failure. */
int  link_start(LINK_ROLE role);

/* Tear everything down and free the Wi-Fi stack. Safe to call at any time,
 * including when the link is off. */
void link_stop(void);

/* ---- status ---- */
LINK_STATE link_get_state(void);
LINK_ERROR link_get_error(void);
int        link_is_connected(void);      /* 1 if CONNECTED */
int        link_peer_count(void);        /* remote peers currently seen (0..3) */
int        link_local_slot(void);        /* our player slot 0..3, or LINK_SLOT_NONE */
void       link_get_stats(link_stats *out);

/* Microsecond timestamp on the same clock as link_msg.rx_time_us (wraps). */
uint32_t   link_now_us(void);

/* ---- data path (non-blocking) ---- */

/* Send a message to one slot (0..3) or LINK_SLOT_BROADCAST. Returns 0 if the
 * datagram was queued to the Wi-Fi driver, <0 otherwise. */
int  link_send(uint8_t dest_slot, uint8_t type, const void *data, uint8_t len);

/* Pop the oldest received message. Returns 1 and fills *out if one was
 * available, 0 if the queue is empty. */
int  link_recv(link_msg *out);

#endif /* LINK_H */
