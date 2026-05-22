/*
 * mpe_applemidi — RTP-MIDI ("AppleMIDI") session client. Pairs with
 * the macOS Audio MIDI Setup network session, Windows rtpMIDI driver,
 * or Linux rtpmidid.
 *
 * Protocol shape (RFC 6295 + the AppleMIDI session sub-protocol):
 *
 *   - Two adjacent UDP ports: control (P) and data (P + 1).
 *   - Initiator (us) sends a 16-byte "IN" (invitation) to the peer's
 *     control port. Peer responds with "OK" (accept). We then send
 *     another IN to the peer's data port and again wait for OK.
 *   - With the session open, both sides exchange 36-byte "CK" (clock
 *     sync) messages periodically. The exchange is a 3-message
 *     round-trip from which both sides derive the timestamp offset
 *     between them and the round-trip latency. macOS will not play
 *     received MIDI until the first CK exchange has completed.
 *   - MIDI events are sent on the data port as RTP packets (12-byte
 *     header, payload-type 0x61) followed by an RTP-MIDI command
 *     section. We use the short (B=0) header with no journal and no
 *     per-command delta time — the RTP timestamp drives playback.
 *
 * Threading: a small worker task (one) handles invitations, CK echo
 * and the receive loop on both sockets. The render task calls into
 * `mpe_applemidi_*` send helpers, which build packets on the caller's
 * stack and sendto() directly under a short mutex (one-writer at a
 * time on the data socket). MIDI sends become no-ops until the
 * session is fully open AND the first CK exchange has completed.
 *
 * MPE convention: we expose a small helper to send the standard MPE
 * configuration RPN. Caller is responsible for staying within the
 * configured lower-zone member-channel range.
 */
#ifndef MPE_APPLEMIDI_H_
#define MPE_APPLEMIDI_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the session task and begin inviting `host`:`port` (control;
   data is port+1). Empty/NULL host disables the client. `session_name`
   is advertised in the IN packet. Returns immediately; the session
   transitions to "connected" in the background, observable via
   mpe_applemidi_connected(). */
esp_err_t mpe_applemidi_start(const char *host, uint16_t port,
                              const char *session_name);

void      mpe_applemidi_stop(void);

/* True once the peer has accepted both control and data invitations
   AND the first CK exchange has completed (i.e. it is safe to send
   MIDI events). */
bool      mpe_applemidi_connected(void);

/* Round-trip latency of the most recent CK exchange, in milliseconds.
   0 if no exchange has completed yet. Useful for the status overlay. */
int       mpe_applemidi_latency_ms(void);

/* Short-message send. The high nibble of `status` carries the message
   class (0x80..0xE0); `channel` is the low 4 bits (0..15). Anything
   above 0x7F in d1/d2 is masked off. d2 is ignored for 2-byte
   messages (Program Change, Channel Pressure). */
void      mpe_applemidi_send3(uint8_t status, uint8_t d1, uint8_t d2);
void      mpe_applemidi_send2(uint8_t status, uint8_t d1);

/* MPE-flavoured helpers. `channel` is 0..15 (i.e. the wire-level
   channel; the MPE master is 0, members start at 1 by convention). */
void      mpe_applemidi_note_on(uint8_t channel, uint8_t note, uint8_t vel);
void      mpe_applemidi_note_off(uint8_t channel, uint8_t note, uint8_t vel);
void      mpe_applemidi_pitch_bend(uint8_t channel, uint16_t v14);
void      mpe_applemidi_cc(uint8_t channel, uint8_t cc, uint8_t value);
void      mpe_applemidi_channel_pressure(uint8_t channel, uint8_t value);

/* Push the MPE Configuration RPN to the host on master channel 1
   (wire channel 0): RPN 6 with data = `member_count` (1..15). Sent
   as four 3-byte CCs (RPN MSB/LSB then DATA MSB/LSB). Should be
   called once after the session opens. */
void      mpe_applemidi_send_mpe_config(uint8_t member_count);

/* Push a per-channel pitch-bend range RPN (RPN 0, data = semitones).
   Used to inform the host that members will use the configured wide
   bend range; sent on each member channel. */
void      mpe_applemidi_send_pb_range(uint8_t channel, uint8_t semitones);

#ifdef __cplusplus
}
#endif

#endif
