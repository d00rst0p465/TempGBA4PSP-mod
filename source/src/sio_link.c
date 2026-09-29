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
  }

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

static sio_link_ctx sio_ctx;

static int prod_connected(void *u)  { (void)u; return link_is_connected(); }
static int prod_peer_count(void *u) { (void)u; return link_peer_count(); }
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
  prod_connected, prod_peer_count, prod_send, prod_recv, prod_now_us, NULL
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

uint16_t sio_link_poll(void)
{
  if (!sio_link_enabled)
    return 0;

  return sio_link_ctx_poll(&sio_ctx);
}

#endif /* SIO_LINK_HOST_TEST */
