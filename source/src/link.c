/* unofficial gameplaySP kai
 *
 * Link cable emulation over PSP ad-hoc Wi-Fi. See link.h for the design.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <pspkernel.h>
#include <pspnet.h>
#include <pspnet_adhoc.h>
#include <pspnet_adhocctl.h>
#include <psputility_netmodules.h>
#include <pspwlan.h>

#include <string.h>
#include <stddef.h>

#include "link.h"


/* ------------------------------------------------------------------------ */
/* Constants                                                                */
/* ------------------------------------------------------------------------ */

#define LINK_PRODUCT_ID       "ULUS10000"  /* must match on every unit */
#define LINK_PDP_PORT         (0x4742)
#define LINK_PDP_BUFSIZE      (0x2000)

#define LINK_NET_POOL_SIZE    (0x20000)
#define LINK_ADHOCCTL_STACK   (0x2000)
#define LINK_ADHOCCTL_PRIO    (0x20)

#define LINK_THREAD_PRIO      (0x18)       /* below the sound thread (0x08) */
#define LINK_THREAD_STACK     (0x4000)

#define LINK_CONNECT_TIMEOUT_US   (30 * 1000 * 1000)
#define LINK_RECV_TIMEOUT_US      (100 * 1000)   /* also the quit-flag latency */
#define LINK_HELLO_INTERVAL_US    (500 * 1000)
#define LINK_PEER_TIMEOUT_US      (3 * 1000 * 1000)
#define LINK_STOP_WAIT_US         (4 * 1000 * 1000)

#define LINK_MAX_PEERS        (LINK_MAX_PLAYERS - 1)

#define LINK_WIRE_MAGIC       (0x4C)       /* 'L' */

#define LINK_RX_RING_SIZE     (32)         /* power of two */
#define LINK_RX_RING_MASK     (LINK_RX_RING_SIZE - 1)

#define COMPILER_BARRIER()    __asm__ volatile ("" ::: "memory")


/* ------------------------------------------------------------------------ */
/* Wire format                                                              */
/* ------------------------------------------------------------------------ */

typedef struct
{
  uint8_t  magic;
  uint8_t  type;
  uint8_t  src_slot;                 /* sender's slot, LINK_SLOT_NONE-as-0xFF if unknown */
  uint8_t  dest_slot;                /* slot or LINK_SLOT_BROADCAST */
  uint16_t seq;
  uint8_t  role;                     /* LINK_ROLE, meaningful in HELLO */
  uint8_t  len;
  uint8_t  data[LINK_PAYLOAD_MAX];
} __attribute__((packed)) link_wire;

#define LINK_WIRE_HEADER      ((int)offsetof(link_wire, data))


/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

typedef struct
{
  uint8_t  mac[6];
  uint8_t  role;
  uint32_t last_seen_us;
} link_peer;

static const uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* status, written by the link thread, read by anyone */
static volatile LINK_STATE g_state = LINK_STATE_OFF;
static volatile LINK_ERROR g_error = LINK_ERR_NONE;
static volatile int        g_peer_count = 0;
static volatile int        g_local_slot = LINK_SLOT_NONE;

/* thread control */
static SceUID       g_thread = -1;
static volatile int g_quit = 0;
static LINK_ROLE    g_role = LINK_ROLE_JOIN;

/* net resources (only touched by the link thread once it is running) */
static int g_pdp = -1;
static uint8_t g_my_mac[6];

/* peer table + slot map (link thread writes; g_slot_mac read by link_send) */
static link_peer     g_peers[LINK_MAX_PEERS];
static uint8_t       g_slot_mac[LINK_MAX_PLAYERS][6];
static uint8_t       g_slot_valid[LINK_MAX_PLAYERS];
static volatile uint32_t g_map_version = 0;   /* seqlock: odd while updating */

/* receive ring: single producer (link thread), single consumer (emulation) */
static link_msg          g_rx_ring[LINK_RX_RING_SIZE];
static volatile uint32_t g_rx_head = 0;
static volatile uint32_t g_rx_tail = 0;

static volatile uint16_t g_tx_seq = 0;
static link_stats        g_stats;


/* ------------------------------------------------------------------------ */
/* Peer table / slot assignment                                             */
/* ------------------------------------------------------------------------ */

typedef struct
{
  uint8_t mac[6];
  uint8_t role;
} link_member;

/* Hosts first, then joiners; ties broken by MAC. Every unit sees the same set
 * of members once discovery settles, so every unit computes the same slots
 * without any negotiation. */
static int member_before(const link_member *a, const link_member *b)
{
  int a_key = (a->role == LINK_ROLE_HOST) ? 0 : 1;
  int b_key = (b->role == LINK_ROLE_HOST) ? 0 : 1;

  if (a_key != b_key)
    return a_key < b_key;

  return memcmp(a->mac, b->mac, 6) < 0;
}

static void recompute_slots(void)
{
  link_member members[LINK_MAX_PLAYERS];
  int count = 0;
  int i, j;
  int peers = 0;
  int local = LINK_SLOT_NONE;

  memcpy(members[count].mac, g_my_mac, 6);
  members[count].role = (uint8_t)g_role;
  count++;

  for (i = 0; i < LINK_MAX_PEERS; i++)
  {
    if (g_peers[i].last_seen_us != 0)
    {
      memcpy(members[count].mac, g_peers[i].mac, 6);
      members[count].role = g_peers[i].role;
      count++;
      peers++;
    }
  }

  /* insertion sort, count <= 4 */
  for (i = 1; i < count; i++)
  {
    link_member key = members[i];

    for (j = i - 1; j >= 0 && member_before(&key, &members[j]); j--)
      members[j + 1] = members[j];

    members[j + 1] = key;
  }

  g_map_version++;                    /* odd: update in progress */
  COMPILER_BARRIER();

  memset(g_slot_valid, 0, sizeof(g_slot_valid));

  for (i = 0; i < count; i++)
  {
    memcpy(g_slot_mac[i], members[i].mac, 6);
    g_slot_valid[i] = 1;

    if (memcmp(members[i].mac, g_my_mac, 6) == 0)
      local = i;
  }

  COMPILER_BARRIER();
  g_map_version++;                    /* even: consistent again */

  g_local_slot = local;
  g_peer_count = peers;
}

static link_peer *find_peer(const uint8_t *mac)
{
  int i;

  for (i = 0; i < LINK_MAX_PEERS; i++)
  {
    if (g_peers[i].last_seen_us != 0 && memcmp(g_peers[i].mac, mac, 6) == 0)
      return &g_peers[i];
  }

  return NULL;
}

static link_peer *add_peer(const uint8_t *mac, uint8_t role, uint32_t now)
{
  int i;

  for (i = 0; i < LINK_MAX_PEERS; i++)
  {
    if (g_peers[i].last_seen_us == 0)
    {
      memcpy(g_peers[i].mac, mac, 6);
      g_peers[i].role = role;
      g_peers[i].last_seen_us = now ? now : 1;   /* 0 means "unused" */
      return &g_peers[i];
    }
  }

  return NULL;                        /* group is full */
}

static int slot_of_mac(const uint8_t *mac)
{
  int i;

  for (i = 0; i < LINK_MAX_PLAYERS; i++)
  {
    if (g_slot_valid[i] && memcmp(g_slot_mac[i], mac, 6) == 0)
      return i;
  }

  return LINK_SLOT_NONE;
}

static void expire_peers(uint32_t now)
{
  int i;
  int changed = 0;

  for (i = 0; i < LINK_MAX_PEERS; i++)
  {
    if (g_peers[i].last_seen_us != 0 &&
        (uint32_t)(now - g_peers[i].last_seen_us) > LINK_PEER_TIMEOUT_US)
    {
      g_peers[i].last_seen_us = 0;
      changed = 1;
    }
  }

  if (changed)
    recompute_slots();
}


/* ------------------------------------------------------------------------ */
/* Receive ring                                                             */
/* ------------------------------------------------------------------------ */

static void rx_push(const link_msg *msg)
{
  uint32_t head = g_rx_head;

  if ((uint32_t)(head - g_rx_tail) >= LINK_RX_RING_SIZE)
  {
    g_stats.rx_dropped_full++;
    return;
  }

  g_rx_ring[head & LINK_RX_RING_MASK] = *msg;
  COMPILER_BARRIER();
  g_rx_head = head + 1;
}

int link_recv(link_msg *out)
{
  uint32_t tail = g_rx_tail;

  if (tail == g_rx_head)
    return 0;

  *out = g_rx_ring[tail & LINK_RX_RING_MASK];
  COMPILER_BARRIER();
  g_rx_tail = tail + 1;

  return 1;
}


/* ------------------------------------------------------------------------ */
/* Sending                                                                  */
/* ------------------------------------------------------------------------ */

static int send_wire(const uint8_t *dest_mac, const link_wire *wire)
{
  int size = LINK_WIRE_HEADER + wire->len;
  int r = sceNetAdhocPdpSend(g_pdp, (unsigned char *)dest_mac, LINK_PDP_PORT,
                             (void *)wire, (unsigned int)size, 0, 1);

  if (r < 0)
  {
    g_stats.tx_errors++;
    return r;
  }

  g_stats.tx_packets++;
  return 0;
}

/* Copy a slot's MAC using the seqlock so a concurrent membership change is
 * never seen half-written. */
static int lookup_slot_mac(uint8_t slot, uint8_t *mac_out)
{
  int attempt;

  if (slot >= LINK_MAX_PLAYERS)
    return -1;

  for (attempt = 0; attempt < 4; attempt++)
  {
    uint32_t v1 = g_map_version;
    uint8_t valid;

    COMPILER_BARRIER();

    if (v1 & 1)
      continue;

    valid = g_slot_valid[slot];
    memcpy(mac_out, g_slot_mac[slot], 6);

    COMPILER_BARRIER();

    if (v1 == g_map_version)
      return valid ? 0 : -1;
  }

  return -1;
}

int link_send(uint8_t dest_slot, uint8_t type, const void *data, uint8_t len)
{
  link_wire wire;
  uint8_t mac[6];

  if (g_state != LINK_STATE_CONNECTED)
    return -1;

  if (len > LINK_PAYLOAD_MAX || (len != 0 && data == NULL))
    return -1;

  if (dest_slot == LINK_SLOT_BROADCAST)
  {
    memcpy(mac, broadcast_mac, 6);
  }
  else if (lookup_slot_mac(dest_slot, mac) < 0)
  {
    return -1;
  }

  wire.magic     = LINK_WIRE_MAGIC;
  wire.type      = type;
  wire.src_slot  = (uint8_t)g_local_slot;
  wire.dest_slot = dest_slot;
  wire.seq       = g_tx_seq++;
  wire.role      = (uint8_t)g_role;
  wire.len       = len;

  if (len != 0)
    memcpy(wire.data, data, len);

  return send_wire(mac, &wire);
}

static void send_hello(void)
{
  link_wire wire;

  wire.magic     = LINK_WIRE_MAGIC;
  wire.type      = LINK_MSG_HELLO;
  wire.src_slot  = (uint8_t)g_local_slot;
  wire.dest_slot = LINK_SLOT_BROADCAST;
  wire.seq       = 0;
  wire.role      = (uint8_t)g_role;
  wire.len       = 0;

  send_wire(broadcast_mac, &wire);
}


/* ------------------------------------------------------------------------ */
/* Receive handling (link thread)                                           */
/* ------------------------------------------------------------------------ */

static void handle_packet(const uint8_t *src_mac, const link_wire *wire,
                          int received, uint32_t now)
{
  link_peer *peer;
  link_msg msg;

  if (received < LINK_WIRE_HEADER || wire->magic != LINK_WIRE_MAGIC ||
      wire->len > LINK_PAYLOAD_MAX || received < LINK_WIRE_HEADER + wire->len)
  {
    g_stats.rx_dropped_bad++;
    return;
  }

  if (memcmp(src_mac, g_my_mac, 6) == 0)
    return;                                       /* our own broadcast */

  g_stats.rx_packets++;

  peer = find_peer(src_mac);

  if (peer == NULL)
  {
    /* only HELLO may introduce a new peer */
    if (wire->type != LINK_MSG_HELLO)
    {
      g_stats.rx_dropped_bad++;
      return;
    }

    peer = add_peer(src_mac, wire->role, now);

    if (peer == NULL)
      return;                                     /* group full */

    recompute_slots();
  }
  else
  {
    peer->last_seen_us = now ? now : 1;

    if (wire->type == LINK_MSG_HELLO && peer->role != wire->role)
    {
      peer->role = wire->role;
      recompute_slots();
    }
  }

  if (wire->type == LINK_MSG_HELLO)
    return;                                       /* protocol-internal */

  if (wire->dest_slot != LINK_SLOT_BROADCAST &&
      wire->dest_slot != (uint8_t)g_local_slot)
    return;                                       /* not for us */

  msg.type       = wire->type;
  msg.src_slot   = (int8_t)slot_of_mac(src_mac);
  msg.seq        = wire->seq;
  msg.len        = wire->len;
  msg.rx_time_us = now;
  memcpy(msg.data, wire->data, wire->len);

  rx_push(&msg);
}


/* ------------------------------------------------------------------------ */
/* Link thread: bring-up, run loop, tear-down                               */
/* ------------------------------------------------------------------------ */

typedef struct
{
  int modules_common;
  int modules_adhoc;
  int net;
  int adhoc;
  int adhocctl;
  int connected;
  int pdp;
} link_progress;

static void link_teardown(link_progress *p)
{
  if (p->pdp)
  {
    sceNetAdhocPdpDelete(g_pdp, 0);
    g_pdp = -1;
  }

  if (p->connected)
    sceNetAdhocctlDisconnect();

  if (p->adhocctl)
    sceNetAdhocctlTerm();

  if (p->adhoc)
    sceNetAdhocTerm();

  if (p->net)
    sceNetTerm();

  if (p->modules_adhoc)
    sceUtilityUnloadNetModule(PSP_NET_MODULE_ADHOC);

  if (p->modules_common)
    sceUtilityUnloadNetModule(PSP_NET_MODULE_COMMON);

  memset(p, 0, sizeof(*p));
}

static int link_fail(link_progress *p, LINK_ERROR err)
{
  g_error = err;
  g_state = LINK_STATE_ERROR;
  link_teardown(p);

  return -1;
}

static int link_bring_up(link_progress *p)
{
  struct productStruct product;
  int state = 0;
  uint32_t start;

  if (sceWlanGetSwitchState() != 1)
    return link_fail(p, LINK_ERR_WLAN_OFF);

  if (sceUtilityLoadNetModule(PSP_NET_MODULE_COMMON) < 0)
    return link_fail(p, LINK_ERR_MODULE_LOAD);
  p->modules_common = 1;

  if (sceUtilityLoadNetModule(PSP_NET_MODULE_ADHOC) < 0)
    return link_fail(p, LINK_ERR_MODULE_LOAD);
  p->modules_adhoc = 1;

  if (sceNetInit(LINK_NET_POOL_SIZE, 0x20, 0x1000, 0x20, 0x1000) < 0)
    return link_fail(p, LINK_ERR_STACK_INIT);
  p->net = 1;

  if (sceNetAdhocInit() < 0)
    return link_fail(p, LINK_ERR_STACK_INIT);
  p->adhoc = 1;

  memset(&product, 0, sizeof(product));
  memcpy(product.product, LINK_PRODUCT_ID, 9);

  if (sceNetAdhocctlInit(LINK_ADHOCCTL_STACK, LINK_ADHOCCTL_PRIO, &product) < 0)
    return link_fail(p, LINK_ERR_STACK_INIT);
  p->adhocctl = 1;

  g_state = LINK_STATE_CONNECTING;

  if (sceNetAdhocctlConnect(LINK_GROUP_NAME) < 0)
    return link_fail(p, LINK_ERR_CONNECT);
  p->connected = 1;

  start = sceKernelGetSystemTimeLow();

  for (;;)
  {
    if (g_quit)
      return -1;                                  /* link_stop() in progress */

    sceNetAdhocctlGetState(&state);

    if (state == 1)
      break;

    if ((uint32_t)(sceKernelGetSystemTimeLow() - start) > LINK_CONNECT_TIMEOUT_US)
      return link_fail(p, LINK_ERR_CONNECT);

    sceKernelDelayThread(50 * 1000);
  }

  sceWlanGetEtherAddr(g_my_mac);

  g_pdp = sceNetAdhocPdpCreate(g_my_mac, LINK_PDP_PORT, LINK_PDP_BUFSIZE, 0);

  if (g_pdp < 0)
  {
    g_pdp = -1;
    return link_fail(p, LINK_ERR_SOCKET);
  }
  p->pdp = 1;

  return 0;
}

static int link_thread_main(SceSize args, void *argp)
{
  link_progress progress;
  uint32_t last_hello;

  memset(&progress, 0, sizeof(progress));

  if (link_bring_up(&progress) < 0)
  {
    link_teardown(&progress);                     /* no-op if link_fail ran */
    return 0;
  }

  memset(g_peers, 0, sizeof(g_peers));
  recompute_slots();

  g_state = LINK_STATE_CONNECTED;

  send_hello();
  last_hello = sceKernelGetSystemTimeLow();

  while (!g_quit)
  {
    link_wire wire;
    uint8_t src_mac[6];
    unsigned short src_port = 0;
    int len = sizeof(wire);
    int r;
    int conn_state = 0;
    uint32_t now;

    r = sceNetAdhocPdpRecv(g_pdp, src_mac, &src_port, &wire, &len,
                           LINK_RECV_TIMEOUT_US, 0);
    now = sceKernelGetSystemTimeLow();            /* stamp before other work */

    if (r >= 0 && len > 0)
      handle_packet(src_mac, &wire, len, now);

    expire_peers(now);

    if ((uint32_t)(now - last_hello) >= LINK_HELLO_INTERVAL_US)
    {
      send_hello();
      last_hello = now;
    }

    /* dropped out of the ad-hoc group (e.g. WLAN switched off) */
    sceNetAdhocctlGetState(&conn_state);

    if (conn_state != 1)
    {
      g_error = LINK_ERR_CONNECT;
      g_state = LINK_STATE_ERROR;
      break;
    }
  }

  /* free the stack on the thread that created it; the state stays ERROR if
   * we left because of a failure, until link_stop() resets it */
  link_teardown(&progress);

  return 0;
}


/* ------------------------------------------------------------------------ */
/* Public lifecycle / status                                                */
/* ------------------------------------------------------------------------ */

int link_start(LINK_ROLE role)
{
  if (g_thread >= 0)
    link_stop();                                  /* restart cleanly */

  g_quit = 0;
  g_role = role;
  g_error = LINK_ERR_NONE;
  g_state = LINK_STATE_STARTING;
  g_peer_count = 0;
  g_local_slot = LINK_SLOT_NONE;
  g_rx_head = 0;
  g_rx_tail = 0;
  g_tx_seq = 0;
  memset(&g_stats, 0, sizeof(g_stats));
  memset(g_peers, 0, sizeof(g_peers));
  memset(g_slot_valid, 0, sizeof(g_slot_valid));

  g_thread = sceKernelCreateThread("Link thread", link_thread_main,
                                   LINK_THREAD_PRIO, LINK_THREAD_STACK,
                                   0, NULL);

  if (g_thread < 0)
  {
    g_thread = -1;
    g_error = LINK_ERR_THREAD;
    g_state = LINK_STATE_ERROR;
    return -1;
  }

  sceKernelStartThread(g_thread, 0, NULL);

  return 0;
}

void link_stop(void)
{
  if (g_thread >= 0)
  {
    SceUInt timeout = LINK_STOP_WAIT_US;

    g_quit = 1;

    if (sceKernelWaitThreadEnd(g_thread, &timeout) < 0)
      sceKernelTerminateDeleteThread(g_thread);   /* stuck in the driver */
    else
      sceKernelDeleteThread(g_thread);

    g_thread = -1;
  }

  g_state = LINK_STATE_OFF;
  g_error = LINK_ERR_NONE;
  g_peer_count = 0;
  g_local_slot = LINK_SLOT_NONE;
  g_rx_head = 0;
  g_rx_tail = 0;
}

LINK_STATE link_get_state(void)
{
  return g_state;
}

LINK_ERROR link_get_error(void)
{
  return g_error;
}

int link_is_connected(void)
{
  return g_state == LINK_STATE_CONNECTED;
}

int link_peer_count(void)
{
  return g_peer_count;
}

int link_local_slot(void)
{
  return g_local_slot;
}

void link_get_stats(link_stats *out)
{
  *out = g_stats;
}

uint32_t link_now_us(void)
{
  return sceKernelGetSystemTimeLow();
}
