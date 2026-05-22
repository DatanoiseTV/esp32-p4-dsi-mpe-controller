/*
 * mpe_applemidi.c — AppleMIDI / RTP-MIDI session client.
 *
 * Architecture: one worker task plus a send-path that runs on the
 * caller (render) thread. Receive, invitation, and CK-echo all live
 * in the worker; render task calls into mpe_applemidi_send3() which
 * builds a 14-byte RTP-MIDI packet on the stack and sendto()'s it
 * on the data socket under a short mutex.
 *
 * Why one writer + one reader thread: we want minimum jitter on the
 * MIDI hot-path. Queue-and-dequeue between the render task and a
 * dedicated sender task would add ~1 ms of context-switch latency
 * and pin two cache lines for no real benefit. sendto() is
 * thread-safe in lwIP, but only one writer at a time can advance
 * the sequence number — hence the mutex.
 *
 * Wire format notes:
 *
 *   AppleMIDI control-channel packets all start with the 0xFFFF magic.
 *   The 16-bit big-endian command word that follows is interpreted as
 *   two ASCII chars:
 *      "IN" = Invitation
 *      "OK" = Accept invitation
 *      "NO" = Reject invitation
 *      "BY" = End session
 *      "CK" = Clock sync (sent on either control or data port)
 *
 *   RTP-MIDI packet (data port):
 *      byte 0  = 0x80           (V=2, P=0, X=0, CC=0)
 *      byte 1  = 0x61           (M=0, PT=97 = rtp-midi)
 *      bytes 2-3 = seq (BE)
 *      bytes 4-7 = ts  (BE, 100 µs ticks)
 *      bytes 8-11 = SSRC (BE)
 *      byte 12 = 0bBJZPLLLL where LLLL = MIDI command bytes count.
 *                We always use B=0/J=0/Z=0/P=0 with a 1..15-byte cmd
 *                section.
 *      bytes 13.. = raw MIDI command bytes.
 */
#include "mpe_applemidi.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdatomic.h>
#include <errno.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "applemidi";

#define APPLEMIDI_TASK_STACK 4096
#define APPLEMIDI_TASK_PRIO  5
#define SESSION_NAME_MAX     32
#define CK_INTERVAL_US       (10ULL * 1000 * 1000)   /* 10 s */
#define INVITE_RETRY_MS      1500
#define INVITE_RETRY_MAX     20

enum {
    ST_OFFLINE = 0,
    ST_CTRL_INVITED,
    ST_CTRL_ACCEPTED,
    ST_DATA_INVITED,
    ST_DATA_ACCEPTED,   /* both UDP channels open, awaiting first CK */
    ST_SYNCED,          /* CK round-trip done; OK to send MIDI */
};

typedef struct {
    char                 host[64];
    uint16_t             port;            /* control port; data is port+1 */
    char                 session_name[SESSION_NAME_MAX];

    int                  sock_ctrl;       /* -1 if not opened */
    int                  sock_data;

    struct sockaddr_in   peer_ctrl;
    struct sockaddr_in   peer_data;

    uint32_t             ssrc;            /* our source ID */
    uint32_t             token;           /* initiator token used in IN */
    uint16_t             seq;             /* RTP sequence */
    SemaphoreHandle_t    send_lock;       /* protects sock_data sendto + seq */

    _Atomic int          state;
    _Atomic bool         running;
    _Atomic int          latency_ms;
    int                  invite_retry;
    int64_t              last_ck_us;
    TaskHandle_t         task;
} client_t;

static client_t s_c;

/* --- big-endian helpers --------------------------------------------- */

static inline void put_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static inline void put_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static inline void put_u64_be(uint8_t *p, uint64_t v)
{
    put_u32_be(p, (uint32_t)(v >> 32));
    put_u32_be(p + 4, (uint32_t)(v & 0xFFFFFFFFu));
}

static inline uint16_t get_u16_be(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static inline uint32_t get_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static inline uint64_t get_u64_be(const uint8_t *p)
{
    return ((uint64_t)get_u32_be(p) << 32) | get_u32_be(p + 4);
}

/* 100-µs ticks since boot — matches the AppleMIDI clock-sync unit
   and is also a sensible RTP timestamp source for MIDI events. */
static uint64_t midi_ticks_now(void)
{
    return (uint64_t)esp_timer_get_time() / 100;
}

/* --- session-protocol packet builders -------------------------------- */

static int build_invitation(uint8_t *buf, uint32_t ssrc, uint32_t token,
                            const char *name)
{
    /* layout:
       0..1   = 0xFFFF
       2..3   = "IN"
       4..7   = protocol version (=2)
       8..11  = initiator token
       12..15 = SSRC
       16..   = session name (null-terminated; not padded) */
    put_u16_be(buf + 0, 0xFFFF);
    buf[2] = 'I'; buf[3] = 'N';
    put_u32_be(buf + 4, 2);
    put_u32_be(buf + 8, token);
    put_u32_be(buf + 12, ssrc);
    size_t nlen = name ? strlen(name) : 0;
    if (nlen >= SESSION_NAME_MAX) nlen = SESSION_NAME_MAX - 1;
    memcpy(buf + 16, name ? name : "", nlen);
    buf[16 + nlen] = '\0';
    return 16 + (int)nlen + 1;
}

static int build_ck(uint8_t *buf, uint32_t ssrc, uint8_t count,
                    uint64_t ts1, uint64_t ts2, uint64_t ts3)
{
    put_u16_be(buf + 0, 0xFFFF);
    buf[2] = 'C'; buf[3] = 'K';
    put_u32_be(buf + 4, ssrc);
    buf[8]  = count;
    buf[9]  = 0;
    buf[10] = 0;
    buf[11] = 0;
    put_u64_be(buf + 12, ts1);
    put_u64_be(buf + 20, ts2);
    put_u64_be(buf + 28, ts3);
    return 36;
}

/* --- socket helpers ------------------------------------------------- */

static int open_udp_socket(uint16_t *bound_port_out)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return -1;
    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);

    /* Non-blocking sends. At 250 Hz touch + 5 fingers we can fire
       ~30 small UDP packets per second on this socket; lwIP's
       sendto is normally fast but can stall the calling task
       (which is the touch task — holding the controller lock) when
       the WiFi/SDIO queue is briefly full. EAGAIN from a non-block
       send is fine for an instrument: dropping one PB / CC update
       under congestion is inaudible, and getting the controller
       lock back to the render task quickly is what matters. */
    int flags = fcntl(s, F_GETFL, 0);
    if (flags != -1) fcntl(s, F_SETFL, flags | O_NONBLOCK);

    /* Bigger send buffer also helps absorb bursts. */
    int sndbuf = 16 * 1024;
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof sndbuf);

    /* Bind to ephemeral local port. */
    struct sockaddr_in addr = { .sin_family = AF_INET,
                                 .sin_addr.s_addr = htonl(INADDR_ANY),
                                 .sin_port = 0 };
    if (bind(s, (struct sockaddr *)&addr, sizeof addr) < 0) {
        close(s);
        return -1;
    }
    if (bound_port_out) {
        socklen_t alen = sizeof addr;
        if (getsockname(s, (struct sockaddr *)&addr, &alen) == 0) {
            *bound_port_out = ntohs(addr.sin_port);
        }
    }
    return s;
}

static int resolve_peer(const char *host, uint16_t port, struct in_addr *out)
{
    if (inet_aton(host, out) == 1) return 0;
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM };
    struct addrinfo *res = NULL;
    char service[8];
    snprintf(service, sizeof service, "%u", port);
    if (getaddrinfo(host, service, &hints, &res) != 0 || !res) {
        if (res) freeaddrinfo(res);
        return -1;
    }
    *out = ((struct sockaddr_in *)res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    return 0;
}

static void send_invitation(int sock, struct sockaddr_in *peer)
{
    uint8_t buf[64];
    int n = build_invitation(buf, s_c.ssrc, s_c.token, s_c.session_name);
    sendto(sock, buf, n, 0, (struct sockaddr *)peer, sizeof *peer);
}

static void send_ck_initial(int sock, struct sockaddr_in *peer)
{
    uint8_t buf[64];
    uint64_t t1 = midi_ticks_now();
    int n = build_ck(buf, s_c.ssrc, 0, t1, 0, 0);
    sendto(sock, buf, n, 0, (struct sockaddr *)peer, sizeof *peer);
}

/* --- session-task receive loop -------------------------------------- */

static void handle_session_packet(int sock, struct sockaddr_in *from,
                                  uint8_t *buf, int n)
{
    if (n < 4) return;
    if (get_u16_be(buf) != 0xFFFF) return;
    const char c0 = (char)buf[2];
    const char c1 = (char)buf[3];

    if (c0 == 'O' && c1 == 'K') {
        /* OK to our invitation. Update the peer source addr in case
           the responder chose a different port for replies. */
        if (sock == s_c.sock_ctrl) {
            s_c.peer_ctrl = *from;
            atomic_store(&s_c.state, ST_CTRL_ACCEPTED);
            ESP_LOGI(TAG, "control accepted, inviting data");
            /* Immediately invite on the data socket. */
            s_c.peer_data = *from;
            s_c.peer_data.sin_port = htons(s_c.port + 1);
            send_invitation(s_c.sock_data, &s_c.peer_data);
            atomic_store(&s_c.state, ST_DATA_INVITED);
        } else {
            s_c.peer_data = *from;
            atomic_store(&s_c.state, ST_DATA_ACCEPTED);
            ESP_LOGI(TAG, "data accepted, sending CK0");
            send_ck_initial(s_c.sock_data, &s_c.peer_data);
            s_c.last_ck_us = esp_timer_get_time();
        }
    } else if (c0 == 'N' && c1 == 'O') {
        ESP_LOGW(TAG, "invitation rejected by peer");
        atomic_store(&s_c.state, ST_OFFLINE);
    } else if (c0 == 'B' && c1 == 'Y') {
        ESP_LOGW(TAG, "peer ended session");
        atomic_store(&s_c.state, ST_OFFLINE);
    } else if (c0 == 'C' && c1 == 'K' && n >= 36) {
        const uint32_t peer_ssrc = get_u32_be(buf + 4);
        (void)peer_ssrc;
        const uint8_t count = buf[8];
        const uint64_t t1 = get_u64_be(buf + 12);
        const uint64_t t2 = get_u64_be(buf + 20);
        const uint64_t t3 = get_u64_be(buf + 28);
        if (count == 0) {
            /* Peer-initiated CK: respond with count=1, adding our
               timestamp as t2, echoing t1, leaving t3 zero. */
            uint64_t our_ts = midi_ticks_now();
            uint8_t reply[36];
            int rn = build_ck(reply, s_c.ssrc, 1, t1, our_ts, 0);
            sendto(sock, reply, rn, 0, (struct sockaddr *)from, sizeof *from);
        } else if (count == 1) {
            /* This is the response to our own CK0; we now have all
               three timestamps so we can close the round-trip. */
            uint64_t our_ts3 = midi_ticks_now();
            uint8_t reply[36];
            int rn = build_ck(reply, s_c.ssrc, 2, t1, t2, our_ts3);
            sendto(sock, reply, rn, 0, (struct sockaddr *)from, sizeof *from);
            /* round-trip latency = (t3 - t1) in 100-µs ticks; convert
               to ms for the status overlay. */
            int rtt_ms = (int)((our_ts3 - t1) / 10);
            atomic_store(&s_c.latency_ms, rtt_ms);
            if (atomic_load(&s_c.state) < ST_SYNCED) {
                ESP_LOGI(TAG, "synced (rtt=%d ms)", rtt_ms);
            }
            atomic_store(&s_c.state, ST_SYNCED);
        } else if (count == 2) {
            /* Peer is closing a round-trip we participated in as
               responder. Nothing to do. */
            (void)t3;
        }
    }
}

static void session_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    while (atomic_load(&s_c.running)) {
        /* Drive invitation retries if we're stuck before fully open. */
        const int st = atomic_load(&s_c.state);
        const int64_t now = esp_timer_get_time();

        if (st == ST_OFFLINE) {
            if (s_c.invite_retry < INVITE_RETRY_MAX) {
                send_invitation(s_c.sock_ctrl, &s_c.peer_ctrl);
                atomic_store(&s_c.state, ST_CTRL_INVITED);
                s_c.invite_retry++;
                s_c.last_ck_us = now;
            }
        } else if (st == ST_CTRL_INVITED || st == ST_DATA_INVITED) {
            /* Retry the in-flight invitation if no response after
               INVITE_RETRY_MS. */
            if ((now - s_c.last_ck_us) > (int64_t)INVITE_RETRY_MS * 1000) {
                if (st == ST_CTRL_INVITED) {
                    send_invitation(s_c.sock_ctrl, &s_c.peer_ctrl);
                } else {
                    send_invitation(s_c.sock_data, &s_c.peer_data);
                }
                s_c.last_ck_us = now;
            }
        } else if (st >= ST_DATA_ACCEPTED) {
            /* Periodic CK0 from us to maintain liveness. */
            if ((uint64_t)(now - s_c.last_ck_us) > CK_INTERVAL_US) {
                send_ck_initial(s_c.sock_data, &s_c.peer_data);
                s_c.last_ck_us = now;
            }
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s_c.sock_ctrl, &rfds);
        FD_SET(s_c.sock_data, &rfds);
        int maxfd = (s_c.sock_ctrl > s_c.sock_data ? s_c.sock_ctrl : s_c.sock_data) + 1;
        struct timeval tv = { .tv_sec = 0, .tv_usec = 250 * 1000 };
        int r = select(maxfd, &rfds, NULL, NULL, &tv);
        if (r <= 0) continue;

        for (int which = 0; which < 2; which++) {
            int sock = (which == 0) ? s_c.sock_ctrl : s_c.sock_data;
            if (!FD_ISSET(sock, &rfds)) continue;
            struct sockaddr_in from = {};
            socklen_t flen = sizeof from;
            int n = recvfrom(sock, buf, sizeof buf, 0,
                             (struct sockaddr *)&from, &flen);
            if (n <= 0) continue;
            handle_session_packet(sock, &from, buf, n);
        }
    }
    /* Teardown: send "BY" on both ports best-effort. */
    if (s_c.sock_ctrl >= 0) {
        uint8_t bye[16];
        put_u16_be(bye + 0, 0xFFFF);
        bye[2] = 'B'; bye[3] = 'Y';
        put_u32_be(bye + 4, 2);
        put_u32_be(bye + 8, s_c.token);
        put_u32_be(bye + 12, s_c.ssrc);
        sendto(s_c.sock_ctrl, bye, 16, 0,
               (struct sockaddr *)&s_c.peer_ctrl, sizeof s_c.peer_ctrl);
        sendto(s_c.sock_data, bye, 16, 0,
               (struct sockaddr *)&s_c.peer_data, sizeof s_c.peer_data);
        close(s_c.sock_ctrl);
        close(s_c.sock_data);
        s_c.sock_ctrl = s_c.sock_data = -1;
    }
    s_c.task = NULL;
    vTaskDelete(NULL);
}

/* --- public API ----------------------------------------------------- */

esp_err_t mpe_applemidi_start(const char *host, uint16_t port,
                              const char *session_name)
{
    if (!host || !host[0]) {
        ESP_LOGW(TAG, "no host configured — MIDI disabled");
        return ESP_ERR_INVALID_ARG;
    }
    if (atomic_load(&s_c.running)) return ESP_ERR_INVALID_STATE;

    memset(&s_c, 0, sizeof s_c);
    strncpy(s_c.host, host, sizeof s_c.host - 1);
    s_c.port = port;
    strncpy(s_c.session_name,
            (session_name && session_name[0]) ? session_name : "ESP32-MPE",
            sizeof s_c.session_name - 1);
    s_c.ssrc  = esp_random();
    s_c.token = esp_random();
    s_c.seq   = (uint16_t)(esp_random() & 0xFFFF);
    s_c.send_lock = xSemaphoreCreateMutex();
    s_c.sock_ctrl = s_c.sock_data = -1;
    atomic_store(&s_c.state, ST_OFFLINE);
    atomic_store(&s_c.latency_ms, 0);

    struct in_addr peer_ip;
    if (resolve_peer(host, port, &peer_ip) != 0) {
        ESP_LOGE(TAG, "could not resolve %s", host);
        return ESP_FAIL;
    }
    memset(&s_c.peer_ctrl, 0, sizeof s_c.peer_ctrl);
    s_c.peer_ctrl.sin_family = AF_INET;
    s_c.peer_ctrl.sin_addr   = peer_ip;
    s_c.peer_ctrl.sin_port   = htons(port);

    s_c.sock_ctrl = open_udp_socket(NULL);
    s_c.sock_data = open_udp_socket(NULL);
    if (s_c.sock_ctrl < 0 || s_c.sock_data < 0) {
        ESP_LOGE(TAG, "socket() failed");
        if (s_c.sock_ctrl >= 0) close(s_c.sock_ctrl);
        if (s_c.sock_data >= 0) close(s_c.sock_data);
        return ESP_FAIL;
    }

    atomic_store(&s_c.running, true);
    /* Pinned to CPU 1: this task does select() + UDP I/O, blocking
       calls would block the render task if scheduled on CPU 0. */
    BaseType_t ok = xTaskCreatePinnedToCore(session_task, "applemidi",
                                APPLEMIDI_TASK_STACK, NULL,
                                APPLEMIDI_TASK_PRIO, &s_c.task, 1);
    if (ok != pdPASS) {
        atomic_store(&s_c.running, false);
        close(s_c.sock_ctrl);
        close(s_c.sock_data);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "inviting %s:%u (data %u), ssrc=0x%08x",
             host, port, (unsigned)(port + 1), (unsigned)s_c.ssrc);
    return ESP_OK;
}

void mpe_applemidi_stop(void)
{
    atomic_store(&s_c.running, false);
    /* The worker handles teardown + close + self-delete. */
}

bool mpe_applemidi_connected(void)
{
    return atomic_load(&s_c.state) >= ST_SYNCED;
}

int mpe_applemidi_latency_ms(void)
{
    return atomic_load(&s_c.latency_ms);
}

/* --- send path ------------------------------------------------------ */

static void send_midi_bytes(const uint8_t *cmd, int cmd_len)
{
    if (!mpe_applemidi_connected() || cmd_len <= 0 || cmd_len > 15) return;
    if (!s_c.send_lock) return;

    uint8_t pkt[12 + 1 + 16];
    /* RTP header */
    pkt[0] = 0x80;
    pkt[1] = 0x61;
    xSemaphoreTake(s_c.send_lock, portMAX_DELAY);
    uint16_t seq = ++s_c.seq;
    xSemaphoreGive(s_c.send_lock);
    put_u16_be(pkt + 2, seq);
    put_u32_be(pkt + 4, (uint32_t)midi_ticks_now());
    put_u32_be(pkt + 8, s_c.ssrc);
    /* MIDI command header: B=0 J=0 Z=0 P=0, length = cmd_len. */
    pkt[12] = (uint8_t)(cmd_len & 0x0F);
    memcpy(pkt + 13, cmd, cmd_len);
    const int total = 12 + 1 + cmd_len;

    sendto(s_c.sock_data, pkt, total, 0,
           (struct sockaddr *)&s_c.peer_data, sizeof s_c.peer_data);
}

void mpe_applemidi_send3(uint8_t status, uint8_t d1, uint8_t d2)
{
    uint8_t cmd[3] = { status, (uint8_t)(d1 & 0x7F), (uint8_t)(d2 & 0x7F) };
    send_midi_bytes(cmd, 3);
}

void mpe_applemidi_send2(uint8_t status, uint8_t d1)
{
    uint8_t cmd[2] = { status, (uint8_t)(d1 & 0x7F) };
    send_midi_bytes(cmd, 2);
}

void mpe_applemidi_note_on(uint8_t ch, uint8_t note, uint8_t vel)
{
    mpe_applemidi_send3((uint8_t)(0x90 | (ch & 0x0F)), note, vel);
}

void mpe_applemidi_note_off(uint8_t ch, uint8_t note, uint8_t vel)
{
    mpe_applemidi_send3((uint8_t)(0x80 | (ch & 0x0F)), note, vel);
}

void mpe_applemidi_pitch_bend(uint8_t ch, uint16_t v14)
{
    if (v14 > 0x3FFF) v14 = 0x3FFF;
    mpe_applemidi_send3((uint8_t)(0xE0 | (ch & 0x0F)),
                        (uint8_t)(v14 & 0x7F),
                        (uint8_t)((v14 >> 7) & 0x7F));
}

void mpe_applemidi_cc(uint8_t ch, uint8_t cc, uint8_t value)
{
    mpe_applemidi_send3((uint8_t)(0xB0 | (ch & 0x0F)), cc, value);
}

void mpe_applemidi_channel_pressure(uint8_t ch, uint8_t value)
{
    mpe_applemidi_send2((uint8_t)(0xD0 | (ch & 0x0F)), value);
}

void mpe_applemidi_send_mpe_config(uint8_t member_count)
{
    /* MPE config = RPN 0x0006 with data = member_count, sent on master
       channel (here wire-channel 0 = MIDI ch 1). */
    if (member_count > 15) member_count = 15;
    mpe_applemidi_cc(0, 101, 0);             /* RPN MSB = 0 */
    mpe_applemidi_cc(0, 100, 6);             /* RPN LSB = 6 (MPE) */
    mpe_applemidi_cc(0, 6,   member_count);  /* data MSB */
    mpe_applemidi_cc(0, 38,  0);             /* data LSB */
    mpe_applemidi_cc(0, 101, 0x7F);          /* RPN null (deselect) */
    mpe_applemidi_cc(0, 100, 0x7F);
}

void mpe_applemidi_send_pb_range(uint8_t ch, uint8_t semitones)
{
    if (semitones > 96) semitones = 96;
    mpe_applemidi_cc(ch, 101, 0);            /* RPN MSB = 0 */
    mpe_applemidi_cc(ch, 100, 0);            /* RPN LSB = 0 (PB range) */
    mpe_applemidi_cc(ch, 6,   semitones);    /* data MSB = semitones */
    mpe_applemidi_cc(ch, 38,  0);            /* data LSB = cents */
    mpe_applemidi_cc(ch, 101, 0x7F);
    mpe_applemidi_cc(ch, 100, 0x7F);
}
