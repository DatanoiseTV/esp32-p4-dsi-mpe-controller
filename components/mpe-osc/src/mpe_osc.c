/*
 * mpe_osc.c — OSC 1.0 builder + UDP client.
 *
 * Wire format reminders (OSC 1.0):
 *   - All integers/floats are big-endian (network byte order).
 *   - Strings are null-terminated and padded with zeros to a 4-byte
 *     boundary. Both the address and type-tag string follow this
 *     rule, as do 's' arguments.
 *   - Bundle format: "#bundle\0" (8 B) + uint64 NTP timestamp +
 *     repeated [int32 size, message bytes]. Timestamp = 1 means
 *     "immediately".
 */
#include "mpe_osc.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "osc";

/* --- writer ---------------------------------------------------- */

static bool ensure_(mpe_osc_writer *w, size_t need)
{
    if (w->overflow) return false;
    if (w->pos + need > w->cap) {
        w->overflow = true;
        return false;
    }
    return true;
}

static void put_u32_be_(mpe_osc_writer *w, uint32_t v)
{
    if (!ensure_(w, 4)) return;
    w->buf[w->pos++] = (uint8_t)(v >> 24);
    w->buf[w->pos++] = (uint8_t)(v >> 16);
    w->buf[w->pos++] = (uint8_t)(v >> 8);
    w->buf[w->pos++] = (uint8_t)(v);
}

static void put_u64_be_(mpe_osc_writer *w, uint64_t v)
{
    put_u32_be_(w, (uint32_t)(v >> 32));
    put_u32_be_(w, (uint32_t)(v & 0xFFFFFFFFu));
}

static void put_padded_str_(mpe_osc_writer *w, const char *s)
{
    if (!s) s = "";
    size_t len = strlen(s);
    size_t total = (len + 4) & ~(size_t)3;  /* room for NUL + pad */
    if (!ensure_(w, total)) return;
    memcpy(w->buf + w->pos, s, len);
    /* zero pad up to 'total' */
    for (size_t i = len; i < total; i++) w->buf[w->pos + i] = 0;
    w->pos += total;
}

void mpe_osc_writer_init(mpe_osc_writer *w, uint8_t *buf, size_t cap)
{
    w->buf      = buf;
    w->cap      = cap;
    w->pos      = 0;
    w->overflow = false;
}

size_t mpe_osc_bundle_begin(mpe_osc_writer *w)
{
    put_padded_str_(w, "#bundle");
    put_u64_be_(w, 1);  /* immediate */
    return w->pos;
}

size_t mpe_osc_msg_begin(mpe_osc_writer *w, const char *address,
                        const char *typetags)
{
    /* Reserve 4 bytes for the size prefix; we'll patch it in
       mpe_osc_msg_end. Whether the message is inside a bundle or
       top-level, we always reserve the prefix slot — at the top
       level the caller can simply skip the first 4 bytes when
       sending if it doesn't want a prefixed packet. We don't do
       that here: a UDP datagram that contains exactly one OSC
       message doesn't need a size prefix, but our flow always
       wraps in a bundle, so the prefix is always wanted. */
    if (!ensure_(w, 4)) return w->pos;
    const size_t msg_start = w->pos;
    w->pos += 4;
    put_padded_str_(w, address);
    /* Type tag string starts with ','. */
    char tag[32];
    size_t tl = typetags ? strlen(typetags) : 0;
    if (tl + 1 >= sizeof tag) tl = sizeof tag - 2;
    tag[0] = ',';
    if (tl) memcpy(tag + 1, typetags, tl);
    tag[tl + 1] = '\0';
    put_padded_str_(w, tag);
    return msg_start;
}

void mpe_osc_arg_i(mpe_osc_writer *w, int32_t v)
{
    put_u32_be_(w, (uint32_t)v);
}

void mpe_osc_arg_f(mpe_osc_writer *w, float v)
{
    union { float f; uint32_t u; } x;
    x.f = v;
    put_u32_be_(w, x.u);
}

void mpe_osc_arg_s(mpe_osc_writer *w, const char *s)
{
    put_padded_str_(w, s ? s : "");
}

void mpe_osc_msg_end(mpe_osc_writer *w, size_t msg_start)
{
    if (w->overflow) return;
    /* Size = (current pos) - (msg_start + 4). The 4-byte prefix
       itself is not part of the OSC message body. */
    if (w->pos < msg_start + 4) return;  /* corrupt — give up */
    const uint32_t body = (uint32_t)(w->pos - (msg_start + 4));
    w->buf[msg_start + 0] = (uint8_t)(body >> 24);
    w->buf[msg_start + 1] = (uint8_t)(body >> 16);
    w->buf[msg_start + 2] = (uint8_t)(body >> 8);
    w->buf[msg_start + 3] = (uint8_t)(body);
}

/* --- client ---------------------------------------------------- */

struct mpe_osc_client_s {
    char     host[64];
    uint16_t port;
    int      sock;            /* -1 if not opened */
    struct   sockaddr_in addr;
    bool     addr_resolved;
    int      consecutive_errors;
};

mpe_osc_client *mpe_osc_client_new(const char *host, uint16_t port)
{
    if (!host || !host[0]) return NULL;
    mpe_osc_client *c = (mpe_osc_client *)calloc(1, sizeof(*c));
    if (!c) return NULL;
    strncpy(c->host, host, sizeof c->host - 1);
    c->port = port;
    c->sock = -1;
    return c;
}

void mpe_osc_client_free(mpe_osc_client *c)
{
    if (!c) return;
    if (c->sock >= 0) close(c->sock);
    free(c);
}

static int resolve_(mpe_osc_client *c)
{
    /* Try numeric first — it's free and handles the configured
       defaults without DNS roundtrip. */
    memset(&c->addr, 0, sizeof c->addr);
    c->addr.sin_family = AF_INET;
    c->addr.sin_port   = htons(c->port);
    if (inet_aton(c->host, &c->addr.sin_addr) == 1) {
        c->addr_resolved = true;
        return 0;
    }
    /* Fall back to DNS. */
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(c->host, NULL, &hints, &res) != 0 || !res) {
        if (res) freeaddrinfo(res);
        return -1;
    }
    c->addr.sin_addr = ((struct sockaddr_in *)res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    c->addr_resolved = true;
    return 0;
}

static int open_sock_(mpe_osc_client *c)
{
    if (c->sock >= 0) return 0;
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        ESP_LOGW(TAG, "socket() failed: %d", errno);
        return -1;
    }
    c->sock = s;
    return 0;
}

esp_err_t mpe_osc_client_send(mpe_osc_client *c,
                              const uint8_t *data, size_t len)
{
    if (!c || !data || len == 0) return ESP_OK;
    if (!c->addr_resolved && resolve_(c) != 0) return ESP_OK;
    if (open_sock_(c) != 0) return ESP_OK;

    ssize_t n = sendto(c->sock, data, len, 0,
                       (struct sockaddr *)&c->addr, sizeof c->addr);
    if (n < 0) {
        c->consecutive_errors++;
        /* The lwIP send path returns EHOSTUNREACH/ENETUNREACH if the
           ARP entry hasn't materialised yet — recoverable; just drop
           the packet. After many in a row, recycle the socket. */
        if (c->consecutive_errors >= 16) {
            ESP_LOGW(TAG, "many send errors; reopening socket");
            close(c->sock);
            c->sock = -1;
            c->consecutive_errors = 0;
        }
    } else {
        c->consecutive_errors = 0;
    }
    return ESP_OK;
}
