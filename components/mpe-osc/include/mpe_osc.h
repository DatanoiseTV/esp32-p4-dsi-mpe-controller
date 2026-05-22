/*
 * mpe_osc — minimal OSC 1.0 builder + UDP sender.
 *
 * Scope: just what the MPE controller actually emits. That's a
 * /touch message per active finger plus a /clear when all fingers
 * lift, all bundled together at the configured rate. Types covered:
 *
 *   'i'  int32   (big-endian)
 *   'f'  float32 (big-endian; IEEE 754 wire bytes)
 *   's'  null-terminated ASCII string, padded to 4-byte boundary
 *
 * The bundle path uses the standard "#bundle" + uint64 NTP timestamp
 * + size-prefixed messages format. We send "immediately" (timestamp
 * = 1) — the host plays back as-soon-as-received, which is what an
 * instrument wants.
 *
 * The "client" wraps an unconnected UDP socket; we sendto() the
 * resolved peer address on every emit. The socket is created lazily
 * and reopened on send errors that look fatal (ENOTCONN, EHOSTUNREACH
 * after multiple consecutive failures).
 *
 * Thread safety: the builder API is stateless (all state lives in
 * the caller's buffer). The client API is single-writer; the
 * controller calls it from the render task only.
 */
#ifndef MPE_OSC_H_
#define MPE_OSC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Builder ----------------------------------------------------- */

typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   pos;
    bool     overflow;
} mpe_osc_writer;

void mpe_osc_writer_init(mpe_osc_writer *w, uint8_t *buf, size_t cap);

/* Start a new top-level bundle. Subsequent mpe_osc_msg_* calls go
   into it. Each message is automatically size-prefixed by
   mpe_osc_msg_end(). Returns the bundle's body start offset (for
   eventual rewinding if you want; usually ignored). */
size_t mpe_osc_bundle_begin(mpe_osc_writer *w);

/* Start a new message inside the bundle (or as a top-level message
   if not bundled). The address is copied; the type tag string is
   built by mpe_osc_arg_* calls. Returns the offset where the message
   begins (used by mpe_osc_msg_end to write the size prefix). */
size_t mpe_osc_msg_begin(mpe_osc_writer *w, const char *address,
                        const char *typetags);

/* Append one argument; type must match the typetag string passed
   to mpe_osc_msg_begin (the API does not verify — callers are
   expected to keep them in sync). */
void mpe_osc_arg_i(mpe_osc_writer *w, int32_t v);
void mpe_osc_arg_f(mpe_osc_writer *w, float v);
void mpe_osc_arg_s(mpe_osc_writer *w, const char *s);

/* Close the current message, writing its 4-byte size prefix if it
   sits inside a bundle (pass the offset returned by msg_begin). */
void mpe_osc_msg_end(mpe_osc_writer *w, size_t msg_start);

/* --- UDP sender -------------------------------------------------- */

typedef struct mpe_osc_client_s mpe_osc_client;

/* Create a client targeting host:port. host can be a dotted IPv4 or
   a DNS name; resolution happens lazily on first send. NULL or empty
   host disables the client (sends become no-ops). */
mpe_osc_client *mpe_osc_client_new(const char *host, uint16_t port);
void            mpe_osc_client_free(mpe_osc_client *c);

/* Send a fully-built packet via UDP. Returns ESP_OK even when the
   packet was dropped at the kernel boundary — the controller doesn't
   want one bad packet to stall the render loop. Hard errors that
   indicate the socket needs reopening are handled internally. */
esp_err_t mpe_osc_client_send(mpe_osc_client *c,
                              const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
