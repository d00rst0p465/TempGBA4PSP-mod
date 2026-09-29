/* unofficial gameplaySP kai
 *
 * GBA serial port (SIO) semantics on top of the link transport (link.h).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Scope of this module
 * --------------------
 * Normal-8 / Normal-32 mode, two units: one master (SIOCNT bit0 = internal
 * clock) exchanges a word with one slave (bit0 = external clock, start bit
 * armed). Multiplayer, UART, JoyBus and GP modes are NOT handled here and keep
 * the original emulator behaviour.
 *
 * This file has no PSP dependencies: it works on a caller-owned copy of the
 * io register array and talks to the network through an ops table, so the
 * state machine can be tested on a PC with two simulated units.
 *
 * When the link is not connected every entry point is a no-op that reports
 * "not handled", so the emulator behaves exactly as before.
 */

#ifndef SIO_LINK_H
#define SIO_LINK_H

#include <stdint.h>
#include "link.h"

/* Half-word indices into the io register array (byte address / 2). */
#define SIO_REG_DATA32_LO   (0x120 / 2)   /* SIODATA32 low  / SIOMULTI0 */
#define SIO_REG_DATA32_HI   (0x122 / 2)   /* SIODATA32 high / SIOMULTI1 */
#define SIO_REG_DATA8       (0x12A / 2)   /* SIODATA8 / SIOMLT_SEND     */
#define SIO_REG_SIOCNT      (0x128 / 2)
#define SIO_REG_RCNT        (0x134 / 2)

#define SIO_IRQ_SERIAL      (0x0080)      /* same bit as IRQ_SERIAL in cpu.h */

/* How long a master waits for the slave before giving up and reading 1s. */
#define SIO_LINK_TIMEOUT_US (250 * 1000)

/* Multiplayer: how long the master waits for all slave replies, and how long a
 * slave stays "busy" without a result before giving up. */
#define SIO_MP_TIMEOUT_US   (50 * 1000)
#define SIO_MP_SLAVE_TIMEOUT_US (250 * 1000)

/* Transport used by the state machine. link_* wrappers are provided for
 * production; tests supply their own. */
typedef struct
{
  int      (*connected)(void *user);
  int      (*peer_count)(void *user);
  int      (*local_slot)(void *user);   /* 0..3, or LINK_SLOT_NONE */
  int      (*send)(void *user, uint8_t dest_slot, uint8_t type,
                   const void *data, uint8_t len);
  int      (*recv)(void *user, link_msg *out);
  uint32_t (*now_us)(void *user);
  void      *user;
} sio_link_ops;

typedef struct
{
  uint16_t            *io;          /* half-word indexed io register array */
  const sio_link_ops  *ops;

  /* master transfer in flight */
  int                  pending;
  int                  pending_is32;
  uint16_t             pending_seq;
  uint32_t             pending_deadline_us;

  uint16_t             next_seq;

  /* multiplayer master transfer in flight */
  int                  mp_pending;
  uint16_t             mp_seq;
  uint8_t              mp_expected;     /* bitmask of slave slots we wait for */
  uint8_t              mp_got;
  uint16_t             mp_words[4];
  uint32_t             mp_deadline_us;
  uint16_t             mp_next_seq;

  /* multiplayer slave: busy between request and result */
  int                  mp_slave_busy;
  uint32_t             mp_slave_deadline_us;

  /* diagnostics (shown in the Link menu) */
  uint32_t             dbg_wr_normal, dbg_wr_multi, dbg_wr_other;
  uint32_t             dbg_mp_start, dbg_mp_done, dbg_mp_timeout;
  uint32_t             dbg_slave_req, dbg_slave_res, dbg_slave_nak;
  uint16_t             dbg_last_siocnt, dbg_last_rcnt;
  uint16_t             dbg_hist[6];
} sio_link_ctx;

/* ---- core (host-testable) ---- */
void     sio_link_ctx_init(sio_link_ctx *c, uint16_t *io, const sio_link_ops *ops);
void     sio_link_ctx_reset(sio_link_ctx *c);

/* Called from the SIOCNT write handler when a Normal-mode write has the start
 * bit (0x80) set. `value` is the value being written, `is32` selects
 * Normal-32. Returns 1 if the link took over the transfer (the caller must
 * then keep the start/busy bit set and must NOT fake a completion), or 0 to
 * fall back to the original instant-completion behaviour. */
int      sio_link_ctx_start(sio_link_ctx *c, uint16_t value, int is32);

/* Multiplayer mode (SIOCNT bits 12-13 = 10). Called from the SIOCNT write
 * handler with the value being written; adjusts *value (read-only status bits
 * SD/SI/ID, busy bit) and, on the master (player slot 0) with the start bit
 * set, starts an exchange of the four SIOMLT words. Returns 1 if the link
 * handled it, or 0 to use the original stub (link off / no partner). */
int      sio_link_ctx_mp_control(sio_link_ctx *c, uint16_t *value);

/* Pump the receive queue and time out a stalled transfer. Returns a mask of
 * IRQ bits to raise (SIO_IRQ_SERIAL or 0). Cheap when idle. */
uint16_t sio_link_ctx_poll(sio_link_ctx *c);

/* ---- production instance (used by memory.c / main.c / GUI) ---- */

/* Non-zero while the link is enabled; checked in the scanline loop so the
 * link-off cost is one load and one branch. */
extern volatile int sio_link_enabled;
extern volatile unsigned sio_dbg_siocnt_reads;

int      sio_link_enable(LINK_ROLE role);   /* link_start + hook up SIO */
void     sio_link_disable(void);            /* hook down + link_stop    */
void     sio_link_reset(void);              /* emulator reset: drop pending */

int      sio_link_start(uint16_t value, int is32);

/* Register-access trace (production only). Records SIO / TM3 / IE accesses in
 * a ring buffer while the link is enabled and writes link_trace.txt to the
 * application folder on disconnect / quit. kind: 0 r8 1 r16 2 r32 3 w16 4 w32
 * 5 event marker. */
void     sio_trace(int kind, uint32_t addr, uint32_t val, uint32_t pc);
void     sio_trace_flush(const char *path);
void     sio_link_trace_reads(int on);   /* memory.c: route SIO reads via C */
/* Record every SIOCNT write (any mode) for the diagnostics line. */
void     sio_link_note_write(uint16_t value, uint16_t rcnt);
void     sio_link_debug_text(char *buf, int n, int line);   /* line 0..5 */
int      sio_link_mp_control(uint16_t *value);
uint16_t sio_link_poll(void);

#endif /* SIO_LINK_H */
