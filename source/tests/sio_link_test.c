/*
 * Host-side test for sio_link.c (no PSP needed).
 *
 * Two simulated GBAs (A and B) each own an io register array and a
 * sio_link_ctx. A fake network with optional latency connects them, so the
 * Normal-8 / Normal-32 state machine can be exercised on a PC.
 *
 *   cc -DSIO_LINK_HOST_TEST -I../src -o sio_link_test sio_link_test.c ../src/sio_link.c
 *   ./sio_link_test
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "sio_link.h"

static int g_checks, g_failed;

#define CHECK(cond, ...)                                                      \
  do {                                                                        \
    g_checks++;                                                               \
    if (!(cond)) {                                                            \
      g_failed++;                                                             \
      printf("  FAIL line %d: %s  ", __LINE__, #cond);                        \
      printf(__VA_ARGS__);                                                    \
      printf("\n");                                                           \
    }                                                                         \
  } while (0)

/* ---------------- fake network ---------------- */

#define QMAX 64

typedef struct node node;

struct node
{
  int        slot;
  node      *peer;
  int        connected;
  int        peers;
  int        sent_count;
  int        fail_send;
  link_msg   q[QMAX];
  uint32_t   deliver_at[QMAX];
  int        qn;
  uint32_t  *clock;
  uint32_t   latency_us;
};

static int n_connected(void *u)  { return ((node *)u)->connected; }
static int n_peer_count(void *u) { return ((node *)u)->peers; }
static uint32_t n_now(void *u)   { return *((node *)u)->clock; }

static int n_send(void *u, uint8_t dest, uint8_t type, const void *data, uint8_t len)
{
  node *me = (node *)u;
  node *to = me->peer;
  link_msg m;

  (void)dest;

  if (me->fail_send)
    return -1;

  me->sent_count++;

  memset(&m, 0, sizeof(m));
  m.type     = type;
  m.src_slot = (int8_t)me->slot;
  m.len      = len;
  memcpy(m.data, data, len);

  if (to->qn < QMAX)
  {
    to->q[to->qn]          = m;
    to->deliver_at[to->qn] = *me->clock + me->latency_us;
    to->qn++;
  }
  return 0;
}

static int n_recv(void *u, link_msg *out)
{
  node *me = (node *)u;
  int i;

  for (i = 0; i < me->qn; i++)
  {
    if ((int32_t)(*me->clock - me->deliver_at[i]) >= 0)
    {
      *out = me->q[i];
      memmove(&me->q[i], &me->q[i + 1], (me->qn - i - 1) * sizeof(link_msg));
      memmove(&me->deliver_at[i], &me->deliver_at[i + 1],
              (me->qn - i - 1) * sizeof(uint32_t));
      me->qn--;
      return 1;
    }
  }
  return 0;
}

/* ---------------- simulated unit ---------------- */

typedef struct
{
  uint16_t     io[0x400];
  node         net;
  sio_link_ops ops;
  sio_link_ctx ctx;
} unit;

static uint32_t g_clock;

static void unit_init(unit *u, int slot)
{
  memset(u, 0, sizeof(*u));
  u->net.slot      = slot;
  u->net.connected = 1;
  u->net.peers     = 1;
  u->net.clock     = &g_clock;
  u->ops.connected  = n_connected;
  u->ops.peer_count = n_peer_count;
  u->ops.send       = n_send;
  u->ops.recv       = n_recv;
  u->ops.now_us     = n_now;
  u->ops.user       = &u->net;
  sio_link_ctx_init(&u->ctx, u->io, &u->ops);
}

static void pair(unit *a, unit *b)
{
  unit_init(a, 0);
  unit_init(b, 1);
  a->net.peer = &b->net;
  b->net.peer = &a->net;
  g_clock = 1000;
}

/* What sio_control() does around the hook: if the link takes the transfer the
 * start bit stays set, otherwise the original code clears it. */
static int game_writes_siocnt(unit *u, uint16_t value)
{
  int is32 = ((value & 0x3000) == 0x1000);
  int handled = 0;

  if (value & 0x80)
    handled = sio_link_ctx_start(&u->ctx, value, is32);

  if ((value & 0x80) && !handled)
    value &= 0xFF7F;

  u->io[SIO_REG_SIOCNT] = value;
  return handled;
}

static void set32(unit *u, uint32_t v)
{
  u->io[SIO_REG_DATA32_LO] = (uint16_t)v;
  u->io[SIO_REG_DATA32_HI] = (uint16_t)(v >> 16);
}

static uint32_t get32(const unit *u)
{
  return u->io[SIO_REG_DATA32_LO] | ((uint32_t)u->io[SIO_REG_DATA32_HI] << 16);
}

/* SIOCNT values: 0x1000 = normal32, 0x0000 = normal8, bit14 irq, bit0 master */
#define M32_IRQ   (0x1000 | 0x4000 | 0x0001 | 0x0080)
#define S32_IRQ   (0x1000 | 0x4000 | 0x0080)

/* ---------------- tests ---------------- */

static void test_link_off_is_unchanged(void)
{
  unit a, b;
  printf("link off / no partner behaves like the original stub\n");
  pair(&a, &b);

  a.net.connected = 0;
  CHECK(game_writes_siocnt(&a, M32_IRQ) == 0, "master, link off");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) == 0, "start bit cleared (fake completion)");
  CHECK(a.net.sent_count == 0, "nothing sent");

  a.net.connected = 1; a.net.peers = 0;
  CHECK(game_writes_siocnt(&a, M32_IRQ) == 0, "master, connected but alone");
  CHECK(game_writes_siocnt(&a, S32_IRQ) == 0, "slave, connected but alone");
  CHECK(sio_link_ctx_poll(&a.ctx) == 0, "poll with nothing pending");
}

static void test_normal32_exchange(void)
{
  unit a, b;
  uint16_t irq_a, irq_b;
  printf("normal32 exchange, both sides get the other's word + IRQ\n");
  pair(&a, &b);

  set32(&a, 0x11223344);
  set32(&b, 0xAABBCCDD);

  CHECK(game_writes_siocnt(&b, S32_IRQ) == 1, "slave armed");
  CHECK((b.io[SIO_REG_SIOCNT] & 0x80) != 0, "slave stays busy/armed");
  CHECK(game_writes_siocnt(&a, M32_IRQ) == 1, "master started");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) != 0, "master busy until reply");
  CHECK(a.net.sent_count == 1, "one request sent");

  CHECK(sio_link_ctx_poll(&a.ctx) == 0, "master still waiting");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) != 0, "still busy");

  irq_b = sio_link_ctx_poll(&b.ctx);
  CHECK(irq_b == SIO_IRQ_SERIAL, "slave IRQ, got 0x%x", irq_b);
  CHECK(get32(&b) == 0x11223344, "slave received master word: %08x", get32(&b));
  CHECK((b.io[SIO_REG_SIOCNT] & 0x80) == 0, "slave busy cleared");

  irq_a = sio_link_ctx_poll(&a.ctx);
  CHECK(irq_a == SIO_IRQ_SERIAL, "master IRQ, got 0x%x", irq_a);
  CHECK(get32(&a) == 0xAABBCCDD, "master received slave word: %08x", get32(&a));
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) == 0, "master busy cleared");
}

static void test_normal8_exchange(void)
{
  unit a, b;
  printf("normal8 exchange uses only the low byte\n");
  pair(&a, &b);

  a.io[SIO_REG_DATA8] = 0xFF5A;
  b.io[SIO_REG_DATA8] = 0x00C3;

  CHECK(game_writes_siocnt(&b, 0x4080) == 1, "slave armed");
  CHECK(game_writes_siocnt(&a, 0x4081 | 0x0080) == 1, "master started");
  sio_link_ctx_poll(&b.ctx);
  sio_link_ctx_poll(&a.ctx);

  CHECK(b.io[SIO_REG_DATA8] == 0x005A, "slave got %04x", b.io[SIO_REG_DATA8]);
  CHECK(a.io[SIO_REG_DATA8] == 0x00C3, "master got %04x", a.io[SIO_REG_DATA8]);
}

static void test_no_irq_when_disabled(void)
{
  unit a, b;
  printf("no IRQ when SIOCNT bit14 is clear, busy bit still clears\n");
  pair(&a, &b);
  set32(&a, 1); set32(&b, 2);

  game_writes_siocnt(&b, 0x1000 | 0x0080);
  game_writes_siocnt(&a, 0x1000 | 0x0001 | 0x0080);
  CHECK(sio_link_ctx_poll(&b.ctx) == 0, "slave: no irq");
  CHECK(sio_link_ctx_poll(&a.ctx) == 0, "master: no irq");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) == 0 && (b.io[SIO_REG_SIOCNT] & 0x80) == 0,
        "both completed");
  CHECK(get32(&a) == 2 && get32(&b) == 1, "data swapped");
}

static void test_unarmed_slave_reads_ones(void)
{
  unit a, b;
  uint16_t irq;
  printf("slave not armed: master reads all ones, slave untouched\n");
  pair(&a, &b);
  set32(&a, 0x01020304);
  set32(&b, 0x0BADF00D);
  b.io[SIO_REG_SIOCNT] = 0x1000 | 0x4000;            /* normal32, NOT armed */

  game_writes_siocnt(&a, M32_IRQ);
  CHECK(sio_link_ctx_poll(&b.ctx) == 0, "slave ignores request");
  CHECK(get32(&b) == 0x0BADF00D, "slave data untouched");
  irq = sio_link_ctx_poll(&a.ctx);
  CHECK(irq == SIO_IRQ_SERIAL, "master completes with IRQ");
  CHECK(get32(&a) == 0xFFFFFFFF, "master reads ones: %08x", get32(&a));
}

static void test_width_mismatch_is_nak(void)
{
  unit a, b;
  printf("32-bit master vs 8-bit armed slave is refused\n");
  pair(&a, &b);
  set32(&a, 0x12345678);
  b.io[SIO_REG_DATA8] = 0x77;

  game_writes_siocnt(&b, 0x4000 | 0x0080);           /* normal8 armed */
  game_writes_siocnt(&a, M32_IRQ);
  sio_link_ctx_poll(&b.ctx);
  sio_link_ctx_poll(&a.ctx);
  CHECK((b.io[SIO_REG_SIOCNT] & 0x80) != 0, "slave still armed");
  CHECK(b.io[SIO_REG_DATA8] == 0x77, "slave data untouched");
  CHECK(get32(&a) == 0xFFFFFFFF, "master reads ones");
}

static void test_latency_and_timeout(void)
{
  unit a, b;
  uint16_t irq;
  printf("latency: completes only after the reply arrives; timeout otherwise\n");
  pair(&a, &b);
  a.net.latency_us = b.net.latency_us = 1700;         /* ~3.4 ms round trip */
  set32(&a, 0xDEAD0001); set32(&b, 0xBEEF0002);

  game_writes_siocnt(&b, S32_IRQ);
  game_writes_siocnt(&a, M32_IRQ);

  g_clock += 1000;
  CHECK(sio_link_ctx_poll(&b.ctx) == 0, "request not delivered yet");
  g_clock += 800;
  CHECK(sio_link_ctx_poll(&b.ctx) == SIO_IRQ_SERIAL, "delivered after 1.8ms");
  g_clock += 1000;
  CHECK(sio_link_ctx_poll(&a.ctx) == 0, "reply still in flight");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) != 0, "master busy");
  g_clock += 800;
  irq = sio_link_ctx_poll(&a.ctx);
  CHECK(irq == SIO_IRQ_SERIAL && get32(&a) == 0xBEEF0002, "master done, %08x", get32(&a));

  /* partner never polls: timeout */
  pair(&a, &b);
  set32(&a, 5);
  game_writes_siocnt(&a, M32_IRQ);
  g_clock += SIO_LINK_TIMEOUT_US - 1;
  CHECK(sio_link_ctx_poll(&a.ctx) == 0, "not timed out yet");
  g_clock += 2;
  irq = sio_link_ctx_poll(&a.ctx);
  CHECK(irq == SIO_IRQ_SERIAL, "timeout raises IRQ");
  CHECK(get32(&a) == 0xFFFFFFFF, "timeout reads ones");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) == 0, "busy cleared on timeout");
}

static void test_stale_response_ignored(void)
{
  unit a, b;
  printf("a late reply after a timeout does not clobber data\n");
  pair(&a, &b);
  set32(&a, 1); set32(&b, 0x22222222);

  game_writes_siocnt(&b, S32_IRQ);
  game_writes_siocnt(&a, M32_IRQ);
  sio_link_ctx_poll(&b.ctx);                          /* reply now queued for A */

  g_clock += SIO_LINK_TIMEOUT_US + 10;
  sio_link_ctx_poll(&a.ctx);                          /* times out, drains reply */
  CHECK(get32(&a) == 0xFFFFFFFF || get32(&a) == 0x22222222, "a completed once");

  /* now craft an old reply and start a fresh transfer */
  set32(&a, 0x33333333);
  game_writes_siocnt(&a, M32_IRQ);
  {
    link_msg stale;
    memset(&stale, 0, sizeof(stale));
    stale.type = LINK_MSG_SIO; stale.len = 8; stale.src_slot = 1;
    stale.data[0] = 2; stale.data[1] = 1;
    stale.data[2] = 0xEE; stale.data[3] = 0xEE;       /* wrong sequence */
    stale.data[4] = 0x99;
    a.net.q[a.net.qn] = stale; a.net.deliver_at[a.net.qn++] = g_clock;
  }
  CHECK(sio_link_ctx_poll(&a.ctx) == 0, "stale reply ignored");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) != 0, "still waiting for the real one");
}

static void test_second_start_while_pending(void)
{
  unit a, b;
  printf("second start while a transfer is pending sends nothing new\n");
  pair(&a, &b);
  game_writes_siocnt(&a, M32_IRQ);
  CHECK(a.net.sent_count == 1, "first request");
  CHECK(game_writes_siocnt(&a, M32_IRQ) == 1, "still handled");
  CHECK(a.net.sent_count == 1, "no second request");
}

static void test_send_failure_falls_back(void)
{
  unit a, b;
  printf("send failure falls back to instant completion\n");
  pair(&a, &b);
  a.net.fail_send = 1;
  CHECK(game_writes_siocnt(&a, M32_IRQ) == 0, "not handled");
  CHECK((a.io[SIO_REG_SIOCNT] & 0x80) == 0, "start bit cleared");
}

static void test_many_transfers(void)
{
  unit a, b;
  int i, ok = 1;
  printf("100 back-to-back transfers stay in sync\n");
  pair(&a, &b);

  for (i = 0; i < 100; i++)
  {
    uint32_t ma = 0x10000000u + i, sb = 0x20000000u + i;
    set32(&a, ma); set32(&b, sb);
    game_writes_siocnt(&b, S32_IRQ);
    game_writes_siocnt(&a, M32_IRQ);
    sio_link_ctx_poll(&b.ctx);
    sio_link_ctx_poll(&a.ctx);
    if (get32(&a) != sb || get32(&b) != ma ||
        (a.io[SIO_REG_SIOCNT] & 0x80) || (b.io[SIO_REG_SIOCNT] & 0x80))
      ok = 0;
  }
  CHECK(ok, "all 100 exchanges correct");
  CHECK(a.net.sent_count == 100 && b.net.sent_count == 100, "one request + one reply each");
}

int main(void)
{
  test_link_off_is_unchanged();
  test_normal32_exchange();
  test_normal8_exchange();
  test_no_irq_when_disabled();
  test_unarmed_slave_reads_ones();
  test_width_mismatch_is_nak();
  test_latency_and_timeout();
  test_stale_response_ignored();
  test_second_start_while_pending();
  test_send_failure_falls_back();
  test_many_transfers();

  printf("\n%d checks, %d failed\n", g_checks, g_failed);
  return g_failed ? 1 : 0;
}
