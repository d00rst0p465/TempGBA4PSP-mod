/* unofficial gameplaySP kai
 *
 * GBA serial port (SIO) semantics on top of the link transport.
 * See sio_link.h for scope and design.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <string.h>
#include <stdio.h>

#include "sio_link.h"


/* ------------------------------------------------------------------------ */
/* Payload layout of a LINK_MSG_SIO message (8 bytes, little endian)         */
/*   [0] kind   SIO_KIND_REQ (master -> slave) / SIO_KIND_RSP (slave -> master)
 *   [1] flags  bit0: 32-bit transfer, bit1: NAK (no armed slave)
 *   [2] request sequence number (low)   [3] (high)
 *   [4..7] data word (8-bit transfers use byte 4 only)
 * ------------------------------------------------------------------------ */

#define SIO_KIND_REQ      (1)
#define SIO_KIND_RSP      (2)

#define SIO_FLAG_32BIT    (0x01)
#define SIO_FLAG_NAK      (0x02)

/* Multiplayer messages (12 bytes): [0] kind, [1] flags (bit1 = NAK: not in
 * multiplayer mode), [2..3] seq, [4..11] four little endian words.
 *   MREQ master -> all   word0 = master's SIOMLT_SEND
 *   MRSP slave -> master word0 = slave's SIOMLT_SEND
 *   MRES master -> all   words = SIOMULTI0..3 (0xFFFF = absent) */
#define SIO_KIND_MREQ     (3)
#define SIO_KIND_MRSP     (4)
#define SIO_KIND_MRES     (5)
#define SIO_MP_PAYLOAD_LEN (12)

#define SIOCNT_MODE_MASK  (0x3000)
#define SIOCNT_MODE_MULTI (0x2000)
#define SIOCNT_MP_STATUS  (0x007C)      /* SD, SI, ID, error */

#define SIO_PAYLOAD_LEN   (8)
#define SIO_MAX_MSGS_PER_POLL (16)

#define SIOCNT_START      (0x0080)
#define SIOCNT_INTERNAL   (0x0001)
#define SIOCNT_IRQ_ENABLE (0x4000)
#define SIOCNT_LEN_MASK   (0x3000)   /* bits 12-13: 00=normal8 01=normal32 */


static void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v);
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t read_data(const sio_link_ctx *c, int is32)
{
  if (is32)
    return (uint32_t)c->io[SIO_REG_DATA32_LO] |
           ((uint32_t)c->io[SIO_REG_DATA32_HI] << 16);

  return (uint32_t)(c->io[SIO_REG_DATA8] & 0xFF);
}

/* Store the received word, clear the busy bit, and report whether the game
 * asked for a serial interrupt. */
static uint16_t complete_transfer(sio_link_ctx *c, int is32, uint32_t data)
{
  if (is32)
  {
    c->io[SIO_REG_DATA32_LO] = (uint16_t)(data & 0xFFFF);
    c->io[SIO_REG_DATA32_HI] = (uint16_t)(data >> 16);
  }
  else
  {
    c->io[SIO_REG_DATA8] = (uint16_t)(data & 0xFF);
  }

  c->io[SIO_REG_SIOCNT] &= (uint16_t)~SIOCNT_START;

  return (c->io[SIO_REG_SIOCNT] & SIOCNT_IRQ_ENABLE) ? SIO_IRQ_SERIAL : 0;
}

static uint32_t no_partner_word(int is32)
{
  /* an unconnected SI line reads as all ones */
  return is32 ? 0xFFFFFFFFu : 0xFFu;
}


void sio_link_ctx_init(sio_link_ctx *c, uint16_t *io, const sio_link_ops *ops)
{
  memset(c, 0, sizeof(*c));
  c->io  = io;
  c->ops = ops;
}

void sio_link_ctx_reset(sio_link_ctx *c)
{
  c->pending = 0;
  c->pending_deadline_us = 0;
  c->mp_pending = 0;
  c->mp_slave_busy = 0;
}


int sio_link_ctx_start(sio_link_ctx *c, uint16_t value, int is32)
{
  const sio_link_ops *o = c->ops;
  uint8_t payload[SIO_PAYLOAD_LEN];

  if (!o->connected(o->user) || o->peer_count(o->user) < 1)
    return 0;                                   /* original behaviour */

  if ((value & SIOCNT_INTERNAL) == 0)
    return 1;                                   /* slave: armed, wait for master */

  if (c->pending)
    return 1;                                   /* transfer already in flight */

  payload[0] = SIO_KIND_REQ;
  payload[1] = is32 ? SIO_FLAG_32BIT : 0;
  payload[2] = (uint8_t)(c->next_seq);
  payload[3] = (uint8_t)(c->next_seq >> 8);
  put_u32(&payload[4], read_data(c, is32));

  if (o->send(o->user, LINK_SLOT_BROADCAST, LINK_MSG_SIO,
              payload, SIO_PAYLOAD_LEN) < 0)
    return 0;                                   /* could not send: fake it */

  c->pending             = 1;
  c->pending_is32        = is32;
  c->pending_seq         = c->next_seq++;
  c->pending_deadline_us = o->now_us(o->user) + SIO_LINK_TIMEOUT_US;

  return 1;
}


/* A master asked for an exchange. */
static uint16_t handle_request(sio_link_ctx *c, const link_msg *m)
{
  const sio_link_ops *o = c->ops;
  int is32 = (m->data[1] & SIO_FLAG_32BIT) != 0;
  uint32_t master_word = get_u32(&m->data[4]);
  uint16_t siocnt = c->io[SIO_REG_SIOCNT];
  uint16_t rcnt   = c->io[SIO_REG_RCNT];
  uint8_t reply[SIO_PAYLOAD_LEN];
  uint8_t dest;
  int armed;

  /* armed = start bit set, external clock, still in normal mode, same width */
  armed = (siocnt & SIOCNT_START) != 0 &&
          (siocnt & SIOCNT_INTERNAL) == 0 &&
          (rcnt & 0x8000) == 0 &&
          ((siocnt & SIOCNT_LEN_MASK) == (is32 ? 0x1000 : 0x0000));

  reply[0] = SIO_KIND_RSP;
  reply[1] = (uint8_t)((is32 ? SIO_FLAG_32BIT : 0) | (armed ? 0 : SIO_FLAG_NAK));
  reply[2] = m->data[2];
  reply[3] = m->data[3];
  put_u32(&reply[4], armed ? read_data(c, is32) : no_partner_word(is32));

  dest = (m->src_slot == LINK_SLOT_NONE) ? (uint8_t)LINK_SLOT_BROADCAST
                                         : (uint8_t)m->src_slot;
  o->send(o->user, dest, LINK_MSG_SIO, reply, SIO_PAYLOAD_LEN);

  if (!armed)
    return 0;

  return complete_transfer(c, is32, is32 ? master_word : (master_word & 0xFF));
}

/* The slave answered our request. */
static uint16_t handle_response(sio_link_ctx *c, const link_msg *m)
{
  int is32 = (m->data[1] & SIO_FLAG_32BIT) != 0;
  uint16_t seq = (uint16_t)(m->data[2] | (m->data[3] << 8));
  uint32_t word;

  if (!c->pending || seq != c->pending_seq || is32 != c->pending_is32)
    return 0;                                   /* stale or unexpected */

  c->pending = 0;

  word = (m->data[1] & SIO_FLAG_NAK) ? no_partner_word(is32)
                                     : get_u32(&m->data[4]);
  if (!is32)
    word &= 0xFF;

  return complete_transfer(c, is32, word);
}



/* ------------------------------------------------------------------------ */
/* Multiplayer mode                                                         */
/* ------------------------------------------------------------------------ */

static int in_mp_mode(const sio_link_ctx *c)
{
  return (c->io[SIO_REG_RCNT] & 0x8000) == 0 &&
         (c->io[SIO_REG_SIOCNT] & SIOCNT_MODE_MASK) == SIOCNT_MODE_MULTI;
}

/* Read-only bits: SD (bit2) = all units ready, SI (bit3) = 1 on slaves,
 * ID (bits 4-5) = player number. */
static uint16_t mp_status_bits(int slot)
{
  return (uint16_t)(0x0004 | (slot != 0 ? 0x0008 : 0) | ((slot & 3) << 4));
}

static void mp_put_words(uint8_t *p, const uint16_t *w)
{
  int i;
  for (i = 0; i < 4; i++)
  {
    p[i * 2]     = (uint8_t)(w[i]);
    p[i * 2 + 1] = (uint8_t)(w[i] >> 8);
  }
}

static void mp_store_result(sio_link_ctx *c, const uint16_t *w, int slot)
{
  uint16_t cnt;

  c->io[SIO_REG_DATA32_LO]     = w[0];
  c->io[SIO_REG_DATA32_HI]     = w[1];
  c->io[SIO_REG_DATA32_HI + 1] = w[2];
  c->io[SIO_REG_DATA32_HI + 2] = w[3];

  cnt = c->io[SIO_REG_SIOCNT];
  cnt &= (uint16_t)~(SIOCNT_START | SIOCNT_MP_STATUS);
  cnt |= mp_status_bits(slot);
  c->io[SIO_REG_SIOCNT] = cnt;
}

static uint16_t mp_irq(const sio_link_ctx *c)
{
  return (c->io[SIO_REG_SIOCNT] & SIOCNT_IRQ_ENABLE) ? SIO_IRQ_SERIAL : 0;
}

/* Master: all replies in (or timed out) -> publish the result to everyone. */
static uint16_t mp_master_finish(sio_link_ctx *c)
{
  const sio_link_ops *o = c->ops;
  uint8_t payload[SIO_MP_PAYLOAD_LEN];

  c->mp_pending = 0;
  c->dbg_mp_done++;

  payload[0] = SIO_KIND_MRES;
  payload[1] = 0;
  payload[2] = (uint8_t)c->mp_seq;
  payload[3] = (uint8_t)(c->mp_seq >> 8);
  mp_put_words(&payload[4], c->mp_words);
  o->send(o->user, LINK_SLOT_BROADCAST, LINK_MSG_SIO, payload, SIO_MP_PAYLOAD_LEN);

  mp_store_result(c, c->mp_words, 0);
  return mp_irq(c);
}

int sio_link_ctx_mp_control(sio_link_ctx *c, uint16_t *value)
{
  const sio_link_ops *o = c->ops;
  uint16_t v = *value;
  int slot = o->local_slot(o->user);
  int i, busy;

  if (!o->connected(o->user) || o->peer_count(o->user) < 1 ||
      slot < 0 || slot > 3)
    return 0;                                   /* original stub */

  if (slot == 0)
  {
    if ((v & SIOCNT_START) != 0 && !c->mp_pending)
    {
      uint8_t payload[SIO_MP_PAYLOAD_LEN];
      int peers = o->peer_count(o->user);

      memset(payload, 0, sizeof(payload));
      payload[0] = SIO_KIND_MREQ;
      payload[2] = (uint8_t)c->mp_next_seq;
      payload[3] = (uint8_t)(c->mp_next_seq >> 8);
      payload[4] = (uint8_t)(c->io[SIO_REG_DATA8]);
      payload[5] = (uint8_t)(c->io[SIO_REG_DATA8] >> 8);

      if (o->send(o->user, LINK_SLOT_BROADCAST, LINK_MSG_SIO,
                  payload, SIO_MP_PAYLOAD_LEN) < 0)
        return 0;

      for (i = 0; i < 4; i++)
        c->mp_words[i] = 0xFFFF;
      c->mp_words[0]    = c->io[SIO_REG_DATA8];
      c->mp_expected    = (uint8_t)(((1 << (peers + 1)) - 1) & 0x0E);
      c->mp_got         = 0;
      c->mp_seq         = c->mp_next_seq++;
      c->mp_pending     = 1;
      c->dbg_mp_start++;
      c->mp_deadline_us = o->now_us(o->user) + SIO_MP_TIMEOUT_US;
    }
    busy = c->mp_pending;
  }
  else
  {
    busy = c->mp_slave_busy;                    /* start bit is read-only */
  }

  v &= (uint16_t)~(SIOCNT_START | SIOCNT_MP_STATUS);
  v |= mp_status_bits(slot);
  if (busy)
    v |= SIOCNT_START;

  *value = v;
  return 1;
}

static uint16_t mp_handle_request(sio_link_ctx *c, const link_msg *m)
{
  const sio_link_ops *o = c->ops;
  int slot = o->local_slot(o->user);
  uint8_t reply[SIO_MP_PAYLOAD_LEN];
  uint8_t dest;

  if (slot <= 0)
    return 0;                                   /* masters ignore requests */

  memset(reply, 0, sizeof(reply));
  reply[0] = SIO_KIND_MRSP;
  reply[2] = m->data[2];
  reply[3] = m->data[3];

  c->dbg_slave_req++;

  if (in_mp_mode(c))
  {
    reply[4] = (uint8_t)(c->io[SIO_REG_DATA8]);
    reply[5] = (uint8_t)(c->io[SIO_REG_DATA8] >> 8);
    c->mp_slave_busy = 1;
    c->mp_slave_deadline_us = o->now_us(o->user) + SIO_MP_SLAVE_TIMEOUT_US;
    c->io[SIO_REG_SIOCNT] |= SIOCNT_START;
  }
  else
  {
    reply[1] = SIO_FLAG_NAK;
    c->dbg_slave_nak++;
  }

  dest = (m->src_slot == LINK_SLOT_NONE) ? (uint8_t)LINK_SLOT_BROADCAST
                                         : (uint8_t)m->src_slot;
  o->send(o->user, dest, LINK_MSG_SIO, reply, SIO_MP_PAYLOAD_LEN);
  return 0;
}

static uint16_t mp_handle_response(sio_link_ctx *c, const link_msg *m)
{
  int src = m->src_slot;
  uint16_t seq = (uint16_t)(m->data[2] | (m->data[3] << 8));

  if (!c->mp_pending || seq != c->mp_seq || src < 1 || src > 3)
    return 0;

  if (!(m->data[1] & SIO_FLAG_NAK))
    c->mp_words[src] = (uint16_t)(m->data[4] | (m->data[5] << 8));

  c->mp_got |= (uint8_t)(1 << src);

  if ((c->mp_got & c->mp_expected) == c->mp_expected)
    return mp_master_finish(c);

  return 0;
}

static uint16_t mp_handle_result(sio_link_ctx *c, const link_msg *m)
{
  int slot = c->ops->local_slot(c->ops->user);
  uint16_t w[4];
  int i;

  if (slot <= 0 || !in_mp_mode(c))
    return 0;

  for (i = 0; i < 4; i++)
    w[i] = (uint16_t)(m->data[4 + i * 2] | (m->data[5 + i * 2] << 8));

  c->mp_slave_busy = 0;
  c->dbg_slave_res++;
  mp_store_result(c, w, slot);
  return mp_irq(c);
}

static uint16_t mp_tick(sio_link_ctx *c)
{
  const sio_link_ops *o = c->ops;
  uint16_t irq = 0;
  int slot;

  if (c->mp_pending &&
      (int32_t)(o->now_us(o->user) - c->mp_deadline_us) >= 0)
    { c->dbg_mp_timeout++; irq |= mp_master_finish(c); }  /* missing slaves read 0xFFFF */

  if (c->mp_slave_busy &&
      (int32_t)(o->now_us(o->user) - c->mp_slave_deadline_us) >= 0)
  {
    c->mp_slave_busy = 0;
    c->io[SIO_REG_SIOCNT] &= (uint16_t)~SIOCNT_START;
  }

  /* keep SD/SI/ID current as peers come and go */
  if (in_mp_mode(c) && o->connected(o->user) && o->peer_count(o->user) >= 1)
  {
    slot = o->local_slot(o->user);
    if (slot >= 0 && slot <= 3)
    {
      c->io[SIO_REG_SIOCNT] = (uint16_t)((c->io[SIO_REG_SIOCNT] &
                              ~0x003C) | mp_status_bits(slot));
    }
  }

  return irq;
}

uint16_t sio_link_ctx_poll(sio_link_ctx *c)
{
  const sio_link_ops *o = c->ops;
  uint16_t irq = 0;
  link_msg m;
  int n;

  for (n = 0; n < SIO_MAX_MSGS_PER_POLL && o->recv(o->user, &m); n++)
  {
    if (m.type != LINK_MSG_SIO || m.len < SIO_PAYLOAD_LEN)
      continue;

    if (m.data[0] == SIO_KIND_REQ)
      irq |= handle_request(c, &m);
    else if (m.data[0] == SIO_KIND_RSP)
      irq |= handle_response(c, &m);
    else if (m.data[0] == SIO_KIND_MREQ)
      irq |= mp_handle_request(c, &m);
    else if (m.data[0] == SIO_KIND_MRSP)
      irq |= mp_handle_response(c, &m);
    else if (m.data[0] == SIO_KIND_MRES)
      irq |= mp_handle_result(c, &m);
  }

  irq |= mp_tick(c);

  if (c->pending &&
      (int32_t)(o->now_us(o->user) - c->pending_deadline_us) >= 0)
  {
    /* slave never answered: behave like an unconnected cable */
    c->pending = 0;
    irq |= complete_transfer(c, c->pending_is32, no_partner_word(c->pending_is32));
  }

  return irq;
}


/* ------------------------------------------------------------------------ */
/* Production instance                                                      */
/* ------------------------------------------------------------------------ */

#ifndef SIO_LINK_HOST_TEST

#include "common.h"
#include "memory.h"

volatile int sio_link_enabled = 0;
volatile unsigned sio_dbg_siocnt_reads = 0;

static sio_link_ctx sio_ctx;

static int prod_connected(void *u)  { (void)u; return link_is_connected(); }
static int prod_peer_count(void *u) { (void)u; return link_peer_count(); }
static int prod_local_slot(void *u) { (void)u; return link_local_slot(); }
static uint32_t prod_now_us(void *u) { (void)u; return link_now_us(); }

static int prod_send(void *u, uint8_t dest, uint8_t type,
                     const void *data, uint8_t len)
{
  (void)u;
  return link_send(dest, type, data, len);
}

static int prod_recv(void *u, link_msg *out)
{
  (void)u;
  return link_recv(out);
}

static const sio_link_ops prod_ops =
{
  prod_connected, prod_peer_count, prod_local_slot, prod_send, prod_recv, prod_now_us, NULL
};

int sio_link_enable(LINK_ROLE role)
{
  int r;

  sio_link_ctx_init(&sio_ctx, io_registers, &prod_ops);

  r = link_start(role);

  if (r == 0)
    sio_link_enabled = 1;

  return r;
}

void sio_link_disable(void)
{
  sio_link_enabled = 0;                         /* hooks go quiet first */
  link_stop();
  sio_link_ctx_reset(&sio_ctx);
}

void sio_link_reset(void)
{
  sio_link_ctx_reset(&sio_ctx);
}

int sio_link_start(uint16_t value, int is32)
{
  if (!sio_link_enabled)
    return 0;

  return sio_link_ctx_start(&sio_ctx, value, is32);
}

void sio_link_note_write(uint16_t value, uint16_t rcnt)
{
  if (!sio_link_enabled)
    return;

  {
    int i;
    for (i = 5; i > 0; i--)
      sio_ctx.dbg_hist[i] = sio_ctx.dbg_hist[i - 1];
    sio_ctx.dbg_hist[0] = value;
  }
  sio_ctx.dbg_last_siocnt = value;
  sio_ctx.dbg_last_rcnt   = rcnt;

  if (rcnt & 0x8000)
    sio_ctx.dbg_wr_other++;
  else if ((value & 0x3000) == 0x2000)
    sio_ctx.dbg_wr_multi++;
  else if ((value & 0x3000) < 0x2000)
    sio_ctx.dbg_wr_normal++;
  else
    sio_ctx.dbg_wr_other++;
}

void sio_link_debug_text(char *buf, int n, int line)
{
  const sio_link_ctx *c = &sio_ctx;

  switch (line)
  {
    case 0:
      snprintf(buf, n, "SIOCNT %04X RCNT %04X", c->dbg_last_siocnt, c->dbg_last_rcnt);
      break;
    case 1:
      snprintf(buf, n, "wr n%u m%u o%u", (unsigned)c->dbg_wr_normal,
               (unsigned)c->dbg_wr_multi, (unsigned)c->dbg_wr_other);
      break;
    case 3:
      snprintf(buf, n, "%04X %04X %04X %04X %04X %04X",
               c->dbg_hist[0], c->dbg_hist[1], c->dbg_hist[2],
               c->dbg_hist[3], c->dbg_hist[4], c->dbg_hist[5]);
      break;
    case 4:
      snprintf(buf, n, "IE %04X IME %X TM3 %04X rd %u", c->io[0x200 / 2],
               c->io[0x208 / 2], c->io[0x10E / 2], sio_dbg_siocnt_reads);
      break;
    default:
      snprintf(buf, n, "M s%u d%u t%u S q%u r%u x%u",
               (unsigned)c->dbg_mp_start, (unsigned)c->dbg_mp_done,
               (unsigned)c->dbg_mp_timeout, (unsigned)c->dbg_slave_req,
               (unsigned)c->dbg_slave_res, (unsigned)c->dbg_slave_nak);
      break;
  }
}

int sio_link_mp_control(uint16_t *value)
{
  if (!sio_link_enabled)
    return 0;

  return sio_link_ctx_mp_control(&sio_ctx, value);
}

uint16_t sio_link_poll(void)
{
  if (!sio_link_enabled)
    return 0;

  return sio_link_ctx_poll(&sio_ctx);
}

#endif /* SIO_LINK_HOST_TEST */
