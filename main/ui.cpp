/*
 * ui.cpp — animated MPE controller UI for the 1024x600 panel.
 *
 * Visual language:
 *   - Dark navy background with a slow, almost-imperceptible vertical
 *     hue drift driven off esp_timer.
 *   - Top status bar (translucent dark panel) with WiFi/MIDI/OSC/FPS.
 *   - LinnStrument-style isomorphic grid. Cell tint encodes pitch-
 *     class: C (root) is warm amber, black notes (sharps) are deep
 *     violet, naturals are dark teal. Subtle gradient inside each
 *     cell adds depth without distracting from the touch glow.
 *   - Per-channel activity halo: each finger lights up its cell while
 *     active, decaying smoothly after release.
 *   - Touch rendering: additive radial glow tinted by the channel
 *     colour, with a small comet trail of previous positions so
 *     swipes read as motion.
 *
 * Rendering order (back to front):
 *   1. background gradient
 *   2. grid cells (fill + label)
 *   3. activity halos (alpha)
 *   4. trail glows (additive, oldest first)
 *   5. current finger glows (additive)
 *   6. status bar (alpha) + text
 *
 * No floating-point in hot per-pixel paths; the alpha and additive
 * blends are integer-only inside mpe_paint.
 */
#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "mpe_font.h"
#include "mpe_paint.h"

/* --- channel palette ------------------------------------------------ */
typedef struct { uint8_t r, g, b; } rgb;

static const rgb kChannelColor[16] = {
    /* Master channel + members 1..5 used; rest match for safety. */
    {0x00, 0xCC, 0xFF},  /* 0  cyan         (master / fallback) */
    {0x00, 0xCC, 0xFF},  /* 1  cyan         */
    {0xFF, 0x33, 0x99},  /* 2  hot pink     */
    {0x66, 0xFF, 0x33},  /* 3  lime         */
    {0xFF, 0x99, 0x00},  /* 4  amber        */
    {0xBB, 0x44, 0xFF},  /* 5  violet       */
    {0xFF, 0xFF, 0x66},  /* 6  yellow       */
    {0x44, 0xFF, 0xCC},  /* 7  mint         */
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
    {0x44, 0xFF, 0xCC},
};

/* The pitch-class colour table + note-name helper used to live here
   for per-frame rendering; both moved into main.cpp's template
   pre-render after the static-UI/dynamic-overlay split. */

/* --- public API ----------------------------------------------------- */

void mpe_ui_init(mpe_ui *u)
{
    memset(u, 0, sizeof *u);
    u->anim_t0_us = esp_timer_get_time();
    u->top_bar_h  = MPE_UI_TOP_BAR_H;
}

void mpe_ui_push_trails(mpe_ui *u, const mpe_controller *c)
{
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        const mpe_finger *f = &c->fingers[i];
        if (!f->active || f->is_ui) continue;
        const int W = c->cfg.ui_w, H = c->cfg.ui_h;
        const int x0 = c->cfg.ui_x, y0 = c->cfg.ui_y;
        const int px = x0 + (int)(f->x_norm * (float)W);
        const int py = y0 + (int)((1.0f - f->y_norm) * (float)H);
        const int idx = (u->trails.head[i] + 1) % MPE_UI_TRAIL_LEN;
        u->trails.pts[i][idx] = (mpe_ui_trail_pt){ px, py, 1.0f };
        u->trails.head[i]    = idx;
        for (int k = 0; k < MPE_UI_TRAIL_LEN; k++) {
            if (k == idx) continue;
            u->trails.pts[i][k].intensity *= 0.85f;
        }
    }
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        if (c->fingers[i].active && !c->fingers[i].is_ui) continue;
        for (int k = 0; k < MPE_UI_TRAIL_LEN; k++) {
            u->trails.pts[i][k].intensity *= 0.55f;
        }
    }
}

void mpe_ui_render(mpe_ui *u, const mp_target *t,
                   const mpe_controller *c,
                   const mpe_touch_frame *latest,
                   const mpe_ui_status *st)
{
    (void)u;

    /* The static template carries the background, the keyboard, the
       title and the control chips. Per-frame dynamics are: key
       activity halos, finger trails, finger glows, status text. */

    /* 1. Activity halo: light up the touched key in channel colour.
          Only the TOP HALF of the key — that's where the player's
          eye lives during play, and halving the alpha-blend area
          across 5 fingers saves multiple ms of per-pixel work per
          frame (the bottleneck under chord load). Black keys get
          a smaller portion since they're already shorter. */
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        const mpe_finger *f = &c->fingers[i];
        if (f->is_ui) continue;
        const float recency = (f->ch >= 0) ? c->ch_busy_recency[f->ch] : 0.0f;
        if (recency < 0.02f) continue;
        if (f->key_idx < 0 || f->key_idx >= c->kb.n_keys) continue;
        const mpe_key *k = &c->kb.keys[f->key_idx];
        const rgb col = kChannelColor[f->ch & 0x0F];
        const uint8_t a = (uint8_t)(recency * (k->is_black ? 150.0f : 110.0f));
        const int halo_h = k->is_black ? (k->h * 50) / 100
                                       : (k->h * 50) / 100;
        mp_fill_rect_a(t, k->x + 1, k->y + 1,
                       k->w - 2, halo_h - 1,
                       col.r, col.g, col.b, a);
    }

    /* 2. Solid finger dots, pressure-scaled. The previous soft-glow
          stack (outer additive halo + inner alpha core + AA spec +
          per-frame decaying trail of 4-8 glow_add passes per finger)
          was the dominant per-pixel cost — 5 fingers tanked FPS to
          single digits because every pixel inside ~10K-pixel discs
          went through a PSRAM read/blend/write cycle. The new model:
          one opaque circle per finger whose radius encodes Z, plus
          a small white centre pip at the exact touch position. Pure
          RGB565 writes, no blends, no per-pixel sqrt — typically
          ~5× cheaper at 5-finger load. */
    if (latest) {
        for (int i = 0; i < latest->count && i < MPE_TOUCH_MAX_POINTS; i++) {
            const mpe_touch_point *p = &latest->points[i];
            int slot = -1;
            for (int s = 0; s < MPE_CTRL_MAX_FINGERS; s++) {
                if (c->fingers[s].active &&
                    c->fingers[s].tracking_id == p->tracking_id) {
                    slot = s;
                    break;
                }
            }
            if (slot >= 0 && c->fingers[slot].is_ui) continue;
            const int ch_idx = (slot >= 0 ? c->fingers[slot].ch : 0);
            const rgb col = kChannelColor[ch_idx & 0x0F];

            /* Pressure → radius. GT911 strength is the contact-area
               proxy. Floor radius 16 px so a light tap is still
               obviously a dot; ceiling 32 px so the dirty-rect
               margin (44 px below) always provably covers the
               drawn pixels. */
            const int z = p->strength;
            int radius = 16 + (z * 16 / 80);
            if (radius < 16) radius = 16;
            if (radius > 32) radius = 32;

            mp_fill_circle(t, p->x, p->y, radius,
                           mp_rgb565(col.r, col.g, col.b));
            /* Small white centre to mark the exact touch point. */
            mp_fill_circle(t, p->x, p->y, radius / 4 + 2,
                           mp_rgb565(0xff, 0xff, 0xff));
        }
    }

    /* The per-finger floating info chip (note name + ch + z% +
       bend bar) used to live here. Two problems made it untenable:
         (a) It extended 35-75 px outside the touched key, which
             put its drawn pixels OUTSIDE the residual-dirty rect
             (which only covers the key). On release the chip
             stayed onscreen as a ghost (the user's "stuck graphic
             bugs" symptom).
         (b) Rendering it was the next-biggest per-finger cost
             after the soft glow we already removed: backdrop alpha
             rect + 4 border lines + 2 alpha-blended text lines +
             bend bar = ~30K alpha-blends per finger per frame.
       The dot's size + channel colour already communicates pressure
       + which channel, and the player feels the bend on the synth
       output; the chip was strictly debug-helpful. Dropping it. */

    /* 5. Status overlay (text only — panel + buttons live in the
          template). One compact line across the lower row of the
          top bar. */
    if (st) {
        const int small = 14;
        const int dot_r = 4;
        const int row_y = MPE_UI_STATUS_Y;
        const int dot_y = row_y + 6 + dot_r;
        const int text_y = row_y + 4;

        auto chip = [&](int x, const char *label, const char *value,
                        bool ok, rgb dot_col) -> int {
            if (ok) {
                mp_fill_circle_soft(t, x + dot_r, dot_y, dot_r,
                                    dot_col.r, dot_col.g, dot_col.b, 255);
            } else {
                mp_fill_circle_soft(t, x + dot_r, dot_y, dot_r,
                                    0x80, 0x14, 0x24, 255);
            }
            int tx = mpe_font_draw_text(t->fb, t->width, t->height,
                                        x + 2 * dot_r + 6, text_y,
                                        label, -1, small,
                                        mp_rgb565(0x70, 0x80, 0x98));
            tx = mpe_font_draw_text(t->fb, t->width, t->height,
                                    tx + 6, text_y, value, -1,
                                    small, mp_rgb565(0xea, 0xee, 0xff));
            return tx;
        };

        /* Helper: extract :port from "host:port" string, return -1
           if there's no colon. Keeps the status row narrow by
           dropping the peer IP (it's a Kconfig constant; the dot
           colour already says "configured + reachable"). */
        auto port_of = [](const char *s) -> int {
            if (!s) return -1;
            const char *colon = NULL;
            for (const char *p = s; *p; p++) if (*p == ':') colon = p;
            if (!colon) return -1;
            int v = 0;
            for (const char *p = colon + 1; *p >= '0' && *p <= '9'; p++) {
                v = v * 10 + (*p - '0');
            }
            return v;
        };

        char tmp[48];
        int x = 14;

        /* WiFi: shows our station IP — the one piece of network
           identity the user can't infer from build config. */
        snprintf(tmp, sizeof tmp, "%s",
                 st->wifi_ip && st->wifi_ip[0] ? st->wifi_ip : "—");
        x = chip(x, "WiFi", tmp,
                 st->wifi_ip && st->wifi_ip[0],
                 (rgb){0x44, 0xff, 0x88});
        x += 24;

        /* MIDI: just the port + live RTT. Peer IP is in menuconfig. */
        const int midi_port = port_of(st->midi_peer);
        if (st->midi_rtt_ms > 0 && midi_port > 0) {
            snprintf(tmp, sizeof tmp, "%d  %dms", midi_port, st->midi_rtt_ms);
        } else if (midi_port > 0) {
            snprintf(tmp, sizeof tmp, "%d", midi_port);
        } else {
            snprintf(tmp, sizeof tmp, "%s", st->midi_peer ? st->midi_peer : "—");
        }
        x = chip(x, "MIDI", tmp, st->midi_connected,
                 (rgb){0xff, 0xaa, 0x33});
        x += 24;

        /* OSC: just the port. */
        const int osc_port = port_of(st->osc_peer);
        if (osc_port > 0) {
            snprintf(tmp, sizeof tmp, "%d", osc_port);
        } else {
            snprintf(tmp, sizeof tmp, "%s",
                     st->osc_peer && st->osc_peer[0] ? st->osc_peer : "—");
        }
        x = chip(x, "OSC", tmp,
                 st->osc_peer && st->osc_peer[0],
                 (rgb){0x88, 0xaa, 0xff});

        /* Right-aligned: FPS · touches. */
        snprintf(tmp, sizeof tmp, "%d FPS  •  %d touch",
                 st->fps, st->active_fingers);
        int w = mpe_font_text_width_px(tmp, -1, small);
        mpe_font_draw_text(t->fb, t->width, t->height,
                           t->width - 14 - w, text_y, tmp, -1,
                           small, mp_rgb565(0xa0, 0xb0, 0xc0));
    }
}
