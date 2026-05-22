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

    /* 1. Activity halo: light up the touched key in channel colour. */
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        const mpe_finger *f = &c->fingers[i];
        if (f->is_ui) continue;
        const float recency = (f->ch >= 0) ? c->ch_busy_recency[f->ch] : 0.0f;
        if (recency < 0.02f) continue;
        if (f->key_idx < 0 || f->key_idx >= c->kb.n_keys) continue;
        const mpe_key *k = &c->kb.keys[f->key_idx];
        const rgb col = kChannelColor[f->ch & 0x0F];
        /* Heavier alpha on black keys (their base is darker so the
           tint is less visible at the same strength). Inset by 1 px
           so we don't paint over the key's edge bevel. */
        const uint8_t a = (uint8_t)(recency * (k->is_black ? 130.0f : 90.0f));
        mp_fill_rect_a(t, k->x + 1, k->y + 1,
                       k->w - 2, k->h - 2,
                       col.r, col.g, col.b, a);
    }

    /* 2. trails. */
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        const mpe_finger *f = &c->fingers[i];
        if (f->is_ui) continue;
        const rgb col = kChannelColor[(f->ch >= 0 ? f->ch : 0) & 0x0F];
        for (int k = 0; k < MPE_UI_TRAIL_LEN; k++) {
            const mpe_ui_trail_pt *p = &u->trails.pts[i][k];
            /* Tighter intensity floor + smaller radius cap = far less
               PSRAM bandwidth. The lost intensity (0.08..0.15) was
               sub-RGB565 quantization for typical channel colours
               anyway. */
            if (p->intensity < 0.15f) continue;
            int radius = 4 + (int)(14.0f * p->intensity);
            if (radius > 18) radius = 18;
            uint8_t r = (uint8_t)(col.r * p->intensity);
            uint8_t g = (uint8_t)(col.g * p->intensity);
            uint8_t b = (uint8_t)(col.b * p->intensity);
            mp_glow_add(t, p->x, p->y, radius, r, g, b);
        }
    }

    /* 4. current finger glows. Radii are tightly bounded so the
          dirty-rect margin (FINGER_GLOW_MARGIN_PX in main.cpp)
          provably covers every pixel we touch — otherwise the next
          frame's partial-restore leaves halo edges as artifacts.
          Two passes total (outer + inner) — visually rich enough,
          and ~3× cheaper than the four-pass version that drove the
          17 FPS regression. */
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

            /* Pressure (size) gently scales the outer halo, capped
               so the dirty-rect margin always wins. */
            const int z = p->strength;
            int outer = 38 + (z > 0 ? z / 8 : 0);
            if (outer > 58) outer = 58;

            mp_glow_add(t, p->x, p->y, outer,
                        col.r, col.g, col.b);
            mp_fill_circle_soft(t, p->x, p->y, 10,
                                col.r, col.g, col.b, 220);
            mp_fill_circle_soft(t, p->x, p->y, 4,
                                0xff, 0xff, 0xff, 200);
        }
    }

    /* 4. Floating per-finger info chip — the "more useful" win.
          Each active music finger gets a compact 2-line label
          near its touch point: note name big, "ch · bend" small. */
    if (latest) {
        static const char *kNN[12] = {
            "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
        };
        for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
            const mpe_finger *f = &c->fingers[i];
            if (!f->active || f->is_ui) continue;
            if (f->key_idx < 0 || f->key_idx >= c->kb.n_keys) continue;

            /* Resolve the finger's latest pixel from x/y_norm so we
               always label the *current* position, not the original
               touch anchor. */
            const int W = c->cfg.ui_w, H = c->cfg.ui_h;
            const int x0 = c->cfg.ui_x, y0 = c->cfg.ui_y;
            const int fx = x0 + (int)(f->x_norm * (float)W);
            const int fy = y0 + (int)((1.0f - f->y_norm) * (float)H);

            const int oct = (f->note / 12) - 1;
            char line1[12], line2[24];
            snprintf(line1, sizeof line1, "%s%d",
                     kNN[f->note % 12], oct);
            /* Compact "ch · z%" — the live pitch bend gets its own
               visual bar at the bottom of the chip (below), so the
               second text line can stay short. */
            const int z_pct = (int)(f->z * 100.0f + 0.5f);
            snprintf(line2, sizeof line2, "ch%d  z%d%%",
                     f->ch + 1, z_pct);

            const int sz1 = 18, sz2 = 11;
            const int bar_h = 4;
            const int w1 = mpe_font_text_width_px(line1, -1, sz1);
            const int w2 = mpe_font_text_width_px(line2, -1, sz2);
            const int inner_w = (w1 > w2 ? w1 : w2);
            const int chip_w = (inner_w < 56 ? 56 : inner_w) + 14;
            const int chip_h = sz1 + sz2 + bar_h + 10;

            int cy = fy - 32 - chip_h;
            if (cy < c->cfg.ui_y + 4) cy = fy + 40;
            int cx = fx - chip_w / 2;
            if (cx < 4) cx = 4;
            if (cx + chip_w > t->width - 4) cx = t->width - 4 - chip_w;

            const rgb col = kChannelColor[f->ch & 0x0F];

            /* Translucent backdrop tinted by channel — readable over
               busy backgrounds, ties the chip to the finger glow. */
            mp_fill_rect_a(t, cx, cy, chip_w, chip_h,
                           0x06, 0x09, 0x14, 220);
            mp_hline_a(t, cx, cy,              chip_w,
                       col.r, col.g, col.b, 120);
            mp_hline_a(t, cx, cy + chip_h - 1, chip_w,
                       0, 0, 0, 170);
            mp_vline_a(t, cx,              cy, chip_h,
                       col.r, col.g, col.b, 70);
            mp_vline_a(t, cx + chip_w - 1, cy, chip_h, 0, 0, 0, 70);

            mpe_font_draw_text(t->fb, t->width, t->height,
                               cx + (chip_w - w1) / 2, cy + 3,
                               line1, -1, sz1,
                               mp_rgb565(0xf4, 0xf6, 0xff));
            mpe_font_draw_text(t->fb, t->width, t->height,
                               cx + (chip_w - w2) / 2, cy + 3 + sz1,
                               line2, -1, sz2,
                               mp_rgb565(col.r, col.g, col.b));

            /* Pitch-bend bar at the bottom of the chip. A centred
               rail with a notch on the rest-position, and a bar
               extending left or right from centre by |bend|. This
               is what the synth sees, drawn the way a pitch-bend
               wheel reads. */
            const int bar_y = cy + chip_h - bar_h - 3;
            const int bar_inset = 6;
            const int bar_w = chip_w - 2 * bar_inset;
            const int bar_cx = cx + bar_inset + bar_w / 2;
            /* dim rail */
            mp_fill_rect_a(t, cx + bar_inset, bar_y, bar_w, bar_h,
                           0x18, 0x1c, 0x28, 200);
            /* rest-position notch */
            mp_vline_a(t, bar_cx, bar_y - 1, bar_h + 2,
                       0x40, 0x48, 0x58, 220);
            /* live bend bar */
            const float bend = ((float)f->last_pb - 8192.0f) / 8192.0f;
            int bend_px = (int)(bend * (float)(bar_w / 2));
            if (bend_px > 0) {
                mp_fill_rect_a(t, bar_cx, bar_y, bend_px, bar_h,
                               col.r, col.g, col.b, 230);
            } else if (bend_px < 0) {
                mp_fill_rect_a(t, bar_cx + bend_px, bar_y, -bend_px, bar_h,
                               col.r, col.g, col.b, 230);
            }
        }
    }

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
