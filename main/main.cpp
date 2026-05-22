/*
 * main.cpp — esp32-p4-nano-mpempe entry point.
 *
 * Two threads on the audio-critical path:
 *
 *  - touch_task (prio 7, ~250 Hz): polls the GT911, runs the
 *    controller update, dispatches MIDI + OSC immediately. This is
 *    the only place sends happen. Decoupling it from the renderer
 *    is what makes the instrument feel real-time — the previous
 *    iteration polled at the render rate (~13 Hz here) and
 *    produced audible stepping.
 *
 *  - render_task (the main task at prio ~5): copies the pre-baked
 *    static template into the back buffer via GDMA (esp_async_memcpy),
 *    overdraws only the dynamic elements (status bar text, halos,
 *    finger glows, trails) using CPU, then presents and waits for
 *    vsync. The GDMA runs in parallel with the CPU's dynamic-overlay
 *    work, so the per-frame cost is dominated by the smaller of the
 *    two operations rather than their sum.
 *
 * The static template is rendered once at startup: background
 * gradient, every grid cell with its tinted fill, every cell's
 * note label. None of that changes at run time, so paying for it
 * once buys back ~80 glyph rasterisations and 1024×600 pixel writes
 * per frame.
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <atomic>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"

#include "mpe_display.h"
#include "mpe_touch.h"
#include "mpe_wifi.h"
#include "mpe_paint.h"
#include "mpe_font.h"
#include "mpe_osc.h"
#include "mpe_applemidi.h"
#include "mpe_screenshot.h"
#include "nvs_flash.h"
}

#include "ui.h"
#include "controller.h"

static const char *TAG = "main";

extern const uint8_t font_ttf_start[] asm("_binary_font_ttf_start");
extern const uint8_t font_ttf_end[]   asm("_binary_font_ttf_end");

/* Exposed for controller.cpp — owned here. */
mpe_osc_client *g_osc = nullptr;

/* --- Shared state between touch + render tasks ---------------------- */
static mpe_controller         s_ctrl;
static std::atomic<int>       s_active_fingers{0};
/* When a finger releases, the touch task bumps this counter. The
   render task drains it by doing a FULL template→back-buffer
   memcpy on the next N frames instead of partial-restore. Brute-
   force backstop against any partial-restore corner case that
   could leave a "frozen mid-movement" ghost on the buffers (the
   user-reported "going back between 2 frames" symptom). */
static std::atomic<int>       s_force_full_restore{0};
static mpe_touch_frame        s_latest_tf;

/* --- Static template (pre-rendered background + grid + labels) ---- */
static uint16_t              *s_template = nullptr;
static const size_t           kFbBytes = (size_t)MPE_DISPLAY_WIDTH *
                                          MPE_DISPLAY_HEIGHT * 2;

/* --- Dirty-rect partial-redraw machinery -------------------------- *
 *
 * Full-screen memcpy every frame at 1.2 MB / ~150 MB/s PSRAM
 * bandwidth caps us at ~10 FPS before we even draw anything. The
 * dynamic surface area (5 fingers × ~200×200 glow boxes + the
 * status info area) is much smaller. We:
 *
 *   1. Initialize both display buffers with the static template
 *      at startup so the steady state is correct everywhere.
 *   2. Per buffer, track the bounding boxes we PAINTED LAST TIME
 *      we used that buffer (`s_prev_dirty[idx]`). Because we
 *      double-buffer, the back buffer we're about to paint still
 *      holds the dynamics from two frames ago.
 *   3. Each frame: restore template into the *union* of last-time
 *      dirty + this-time dirty rects, then draw the current
 *      dynamics on top.
 *
 * Net cost per frame: ~200-300 KB of PSRAM bandwidth instead of
 * 1.2 MB. */
typedef struct { int x, y, w, h; } rect_t;
#define MAX_DIRTY_RECTS 16
typedef struct { rect_t r[MAX_DIRTY_RECTS]; int n; } dirty_list_t;

static dirty_list_t s_prev_dirty[2] = {};
static int          s_back_local    = 0;   /* mirrors mpe_display's idx */

static inline void rect_clip_(rect_t *r)
{
    if (r->x < 0) { r->w += r->x; r->x = 0; }
    if (r->y < 0) { r->h += r->y; r->y = 0; }
    if (r->x + r->w > MPE_DISPLAY_WIDTH)
        r->w = MPE_DISPLAY_WIDTH - r->x;
    if (r->y + r->h > MPE_DISPLAY_HEIGHT)
        r->h = MPE_DISPLAY_HEIGHT - r->y;
    if (r->w < 0) r->w = 0;
    if (r->h < 0) r->h = 0;
}

static void dirty_push(dirty_list_t *d, int x, int y, int w, int h)
{
    if (d->n >= MAX_DIRTY_RECTS) return;
    rect_t r = { x, y, w, h };
    rect_clip_(&r);
    if (r.w == 0 || r.h == 0) return;
    d->r[d->n++] = r;
}

/* Per-rect restore from template→back. Defers to the display
   component's PPA-accelerated path, which transparently falls back
   to CPU memcpy for rects too small to amortise the PPA setup
   overhead. */
static void restore_rect_(uint16_t *back, const uint16_t *templ, rect_t r)
{
    mpe_display_rect_copy(back, templ, r.x, r.y, r.w, r.h);
}

/* --- Boot-screen animation -------------------------------------- *
 *
 * Drawn while WiFi is associating. Composed of a dark gradient
 * backdrop, the title + subtitle, an animated 5-bar equalizer
 * coloured by the same palette the player will see on their fingers
 * during play (ch2..ch6), and a status line that ticks the elapsed
 * seconds + the SSID being attempted. Each bar pulses on its own
 * frequency + phase so the cluster has a "living, listening" feel
 * instead of a synced-jump LED-strip look. */

static void draw_boot_screen(const mp_target *t, float t_s, const char *ssid)
{
    /* Background gradient — same palette as the runtime template
       so the transition into the keyboard is seamless. */
    mp_gradient_v(t, 0, 0, t->width, t->height,
                  0x0d, 0x10, 0x1f,
                  0x02, 0x03, 0x0a);

    /* Title block, vertically centred-ish above the equalizer. */
    mpe_font_draw_text_centered(t->fb, t->width, t->height,
                                t->width / 2, 140,
                                "MPE", 64,
                                mp_rgb565(0xea, 0xee, 0xff));
    mpe_font_draw_text_centered(t->fb, t->width, t->height,
                                t->width / 2, 214,
                                "controller", 22,
                                mp_rgb565(0x88, 0x98, 0xb0));

    /* Equalizer bars — 5, matching the MPE member-channel count.
       Heights drive off a sine of (t + per-bar phase) so each bar
       has its own rhythm. */
    static const struct { uint8_t r, g, b; float speed; float phase; }
        bar[5] = {
        { 0x00, 0xCC, 0xFF, 1.7f, 0.0f },   /* cyan   */
        { 0xFF, 0x33, 0x99, 1.4f, 1.2f },   /* pink   */
        { 0x66, 0xFF, 0x33, 1.9f, 2.4f },   /* lime   */
        { 0xFF, 0x99, 0x00, 1.3f, 3.6f },   /* amber  */
        { 0xBB, 0x44, 0xFF, 1.6f, 4.8f },   /* violet */
    };
    const int bar_w     = 36;
    const int bar_gap   = 14;
    const int bar_h_max = 130;
    const int bar_h_min = 16;
    const int total_w   = 5 * bar_w + 4 * bar_gap;
    const int x_start   = (t->width - total_w) / 2;
    const int y_base    = 380;

    for (int i = 0; i < 5; i++) {
        float h_norm = 0.5f + 0.5f * sinf(t_s * bar[i].speed + bar[i].phase);
        int h = bar_h_min + (int)(h_norm * (float)(bar_h_max - bar_h_min));
        int x = x_start + i * (bar_w + bar_gap);
        int y = y_base - h;

        /* Soft additive halo behind each bar — channel-coloured. */
        mp_glow_add(t, x + bar_w / 2, y + h / 2,
                    bar_w + 16,
                    (uint8_t)(bar[i].r * 5 / 10),
                    (uint8_t)(bar[i].g * 5 / 10),
                    (uint8_t)(bar[i].b * 5 / 10));
        /* Bar body with a vertical gradient (brighter at top). */
        mp_gradient_v(t, x, y, bar_w, h,
                      bar[i].r, bar[i].g, bar[i].b,
                      (uint8_t)(bar[i].r * 4 / 10),
                      (uint8_t)(bar[i].g * 4 / 10),
                      (uint8_t)(bar[i].b * 4 / 10));
        /* Bright cap at the top — like a peak indicator. */
        mp_fill_rect_a(t, x, y, bar_w, 3, 0xff, 0xff, 0xff, 200);
    }

    /* Status line: connection progress. */
    char status[96];
    const int secs = (int)t_s;
    /* Animated ellipsis: 0..3 dots based on time. */
    const char *dots[] = { "",  ".",  "..",  "..." };
    snprintf(status, sizeof status, "Connecting to %s%s (%ds)",
             (ssid && ssid[0]) ? ssid : "WiFi",
             dots[secs % 4], secs);
    mpe_font_draw_text_centered(t->fb, t->width, t->height,
                                t->width / 2, 500,
                                status, 18,
                                mp_rgb565(0xa0, 0xb0, 0xc8));

    /* Small caption below — what the device is going to be. */
    mpe_font_draw_text_centered(t->fb, t->width, t->height,
                                t->width / 2, 540,
                                "RTP-MIDI / OSC over WiFi", 14,
                                mp_rgb565(0x50, 0x60, 0x78));
}

/* --- Static template renderer ----------------------------------- */

static void draw_button_chip(const mp_target *t,
                             const mpe_button *b,
                             const char *label,
                             const char *value)
{
    /* Soft pill with a faint gradient: lighter at the top, darker
       at the bottom — makes the chip look slightly raised. */
    mp_gradient_v(t, b->x, b->y, b->w, b->h,
                  0x18, 0x1d, 0x2c,
                  0x0a, 0x0e, 0x18);
    mp_hline_a(t, b->x, b->y,           b->w, 0xff, 0xff, 0xff, 60);
    mp_hline_a(t, b->x, b->y + b->h - 1, b->w, 0x00, 0x00, 0x00, 160);
    mp_vline_a(t, b->x,          b->y, b->h, 0xff, 0xff, 0xff, 36);
    mp_vline_a(t, b->x + b->w - 1, b->y, b->h, 0x00, 0x00, 0x00, 80);

    /* Two-line text. The chip is 34 px tall (MPE_UI_BTN_H), so we
       size label + value to fit comfortably with 3 px padding. */
    const int label_sz = 10;
    const int val_sz   = 16;
    int lw = mpe_font_text_width_px(label, -1, label_sz);
    int vw = mpe_font_text_width_px(value, -1, val_sz);
    mpe_font_draw_text(t->fb, t->width, t->height,
                       b->x + (b->w - lw) / 2, b->y + 3,
                       label, -1, label_sz, mp_rgb565(0x7a, 0x88, 0xa0));
    mpe_font_draw_text(t->fb, t->width, t->height,
                       b->x + (b->w - vw) / 2, b->y + 3 + label_sz + 1,
                       value, -1, val_sz, mp_rgb565(0xf0, 0xf4, 0xff));
}

static void draw_button_step(const mp_target *t,
                             const mpe_button *b,
                             const char *glyph)
{
    mp_gradient_v(t, b->x, b->y, b->w, b->h,
                  0x16, 0x1b, 0x28,
                  0x08, 0x0c, 0x14);
    mp_hline_a(t, b->x, b->y,           b->w, 0xff, 0xff, 0xff, 50);
    mp_hline_a(t, b->x, b->y + b->h - 1, b->w, 0x00, 0x00, 0x00, 160);
    mp_vline_a(t, b->x,          b->y, b->h, 0xff, 0xff, 0xff, 28);
    mp_vline_a(t, b->x + b->w - 1, b->y, b->h, 0x00, 0x00, 0x00, 80);
    const int sz = 20;
    const int w = mpe_font_text_width_px(glyph, -1, sz);
    mpe_font_draw_text(t->fb, t->width, t->height,
                       b->x + (b->w - w) / 2,
                       b->y + (b->h - sz) / 2 - 1,
                       glyph, -1, sz, mp_rgb565(0xf0, 0xf4, 0xff));
}

static void render_template(uint16_t *fb,
                            const mpe_controller *c,
                            int top_bar_h)
{
    mp_target t = { .fb = fb,
                    .width  = MPE_DISPLAY_WIDTH,
                    .height = MPE_DISPLAY_HEIGHT };

    /* Background gradient (frozen). */
    mp_gradient_v(&t, 0, 0, t.width, t.height,
                  0x0d, 0x10, 0x1f,
                  0x03, 0x04, 0x0a);

    /* Status-bar panel. */
    mp_fill_rect_a(&t, 0, 0, t.width, top_bar_h, 0x06, 0x08, 0x14, 220);
    mp_hline_a(&t, 0, top_bar_h - 1, t.width, 0x33, 0x55, 0x88, 180);

    /* ----- Keyboard: white keys first, then black overlay. ------- */
    static const char *kNames[12] = {
        "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
    };

    for (int pass = 0; pass < 2; pass++) {
        const bool want_black = (pass == 1);
        for (int i = 0; i < c->kb.n_keys; i++) {
            const mpe_key *k = &c->kb.keys[i];
            if ((bool)k->is_black != want_black) continue;

            const int x = k->x, y = k->y, w = k->w, h = k->h;
            const bool in_scale = k->in_scale;

            if (k->is_black) {
                /* Black-key body: deep matte. Out-of-scale keys go
                   ever-so-slightly darker so the colour reads as
                   "muted" without disappearing. */
                uint8_t br = in_scale ? 0x10 : 0x06;
                uint8_t bg = in_scale ? 0x12 : 0x06;
                uint8_t bb = in_scale ? 0x18 : 0x08;
                uint8_t dr = in_scale ? 0x04 : 0x02;
                uint8_t dg = in_scale ? 0x05 : 0x02;
                uint8_t db = in_scale ? 0x0a : 0x04;
                /* Inset by 2 px to keep a tiny gap from neighbours. */
                mp_gradient_v(&t, x + 1, y, w - 2, h,
                              br, bg, bb, dr, dg, db);
                /* Top sheen highlight. */
                mp_hline_a(&t, x + 1, y,         w - 2, 0xff,0xff,0xff, 42);
                mp_hline_a(&t, x + 1, y + h - 1, w - 2, 0x00,0x00,0x00, 200);
                /* Faint side shadows give a "raised" look. */
                mp_vline_a(&t, x,          y, h, 0x00,0x00,0x00, 200);
                mp_vline_a(&t, x + w - 1,  y, h, 0x00,0x00,0x00, 200);
                /* Label: pitch class only on black keys (no octave). */
                const int label_sz = 14;
                const int lw = mpe_font_text_width_px(kNames[k->pc], -1, label_sz);
                const uint16_t fg = in_scale
                    ? mp_rgb565(0xc8, 0xd4, 0xe0)
                    : mp_rgb565(0x40, 0x46, 0x50);
                mpe_font_draw_text(t.fb, t.width, t.height,
                                   x + (w - lw) / 2,
                                   y + h - label_sz - 8,
                                   kNames[k->pc], -1, label_sz, fg);
            } else {
                /* White-key body: ivory-grey gradient, with the
                   bottom 30% subtly warmed toward amber for an
                   ivory feel. Out-of-scale keys are dim grey. */
                uint8_t tr, tg, tb, dr, dg, db;
                if (in_scale) {
                    if (k->pc == c->root_pc) {
                        tr=0xc8; tg=0xa8; tb=0x68;
                        dr=0x68; dg=0x50; db=0x28;
                    } else {
                        tr=0xc0; tg=0xc8; tb=0xd4;
                        dr=0x50; dg=0x56; db=0x60;
                    }
                } else {
                    tr=0x44; tg=0x48; tb=0x50;
                    dr=0x18; dg=0x1a; db=0x1e;
                }
                /* Inset by 1 px on each side so adjacent whites
                   show a hairline separator. */
                const int ix = x + 1, iy = y, iw = w - 2, ih = h;
                mp_gradient_v(&t, ix, iy, iw, ih, tr, tg, tb, dr, dg, db);
                /* Roli-style "ridge" stripe at ~25% from top. */
                {
                    const int ridge_y = iy + ih / 4;
                    mp_hline_a(&t, ix, ridge_y,     iw, 0xff,0xff,0xff, 50);
                    mp_hline_a(&t, ix, ridge_y + 1, iw, 0xff,0xff,0xff, 22);
                }
                /* Top + bottom bevels for tactility. */
                mp_hline_a(&t, ix, iy,            iw, 0xff,0xff,0xff, 90);
                mp_hline_a(&t, ix, iy + ih - 1,   iw, 0x00,0x00,0x00, 160);
                /* Side hairlines. */
                mp_vline_a(&t, x,          y, h, 0x00,0x00,0x00, 200);

                /* Octave labels go on C-naturals only; other whites
                   stay clean for a Seaboard-clean look. */
                if (k->pc == c->root_pc) {
                    char buf[16];
                    snprintf(buf, sizeof buf, "%s%d",
                             kNames[k->pc], (k->midi_note / 12) - 1);
                    const int sz = 14;
                    const int lw = mpe_font_text_width_px(buf, -1, sz);
                    mpe_font_draw_text(t.fb, t.width, t.height,
                                       x + (w - lw) / 2,
                                       y + h - sz - 8,
                                       buf, -1, sz,
                                       mp_rgb565(0x40, 0x2c, 0x10));
                }
            }
        }
    }

    /* ----- Top-bar title + chips ----- */
    /* Big "MPE" + small "controller" subtitle, left-aligned. */
    mpe_font_draw_text(t.fb, t.width, t.height, 14, 6,
                       "MPE",
                       -1, 28, mp_rgb565(0xea, 0xee, 0xff));
    mpe_font_draw_text(t.fb, t.width, t.height, 14 + 70, 18,
                       "controller",
                       -1, 14, mp_rgb565(0x70, 0x80, 0x98));

    /* Buttons + the "R1 +0 / R2 +0" caption between each stepper
       pair. The caption sits at the same y as the buttons and
       horizontally between the matching < and >. */
    const mpe_button *r1_dn = NULL, *r1_up = NULL;
    const mpe_button *r2_dn = NULL, *r2_up = NULL;
    for (int i = 0; i < c->n_buttons; i++) {
        const mpe_button *b = &c->buttons[i];
        switch (b->action) {
        case MPE_BTN_CYCLE_SCALE:
            draw_button_chip(&t, b, "Scale", mpe_controller_scale_name(c));
            break;
        case MPE_BTN_CYCLE_ROOT:
            draw_button_chip(&t, b, "Root", mpe_controller_root_name(c));
            break;
        case MPE_BTN_CYCLE_PB:
            draw_button_chip(&t, b, "Bend", mpe_controller_pb_label(c));
            break;
        case MPE_BTN_ROW0_OCT_DOWN: draw_button_step(&t, b, "<"); r1_dn = b; break;
        case MPE_BTN_ROW0_OCT_UP:   draw_button_step(&t, b, ">"); r1_up = b; break;
        case MPE_BTN_ROW1_OCT_DOWN: draw_button_step(&t, b, "<"); r2_dn = b; break;
        case MPE_BTN_ROW1_OCT_UP:   draw_button_step(&t, b, ">"); r2_up = b; break;
        default: break;
        }
    }
    /* Caption helper: dark inset between < and > showing "R1 +0". */
    auto draw_row_caption = [&](const mpe_button *dn, const mpe_button *up,
                                int row_idx){
        if (!dn || !up) return;
        const int cx = dn->x + dn->w;
        const int cw = up->x - cx;
        if (cw <= 0) return;
        /* Inset dark slot to visually pair with the steppers. */
        mp_fill_rect_a(&t, cx, dn->y, cw, dn->h, 0x08, 0x0c, 0x14, 235);
        mp_hline_a(&t, cx, dn->y,            cw, 0x00, 0x00, 0x00, 140);
        mp_hline_a(&t, cx, dn->y + dn->h - 1, cw, 0xff, 0xff, 0xff, 32);
        char buf[16];
        snprintf(buf, sizeof buf, "R%d", row_idx + 1);
        const int label_sz = 10;
        const int val_sz   = 16;
        int lw = mpe_font_text_width_px(buf, -1, label_sz);
        mpe_font_draw_text(t.fb, t.width, t.height,
                           cx + (cw - lw) / 2, dn->y + 3,
                           buf, -1, label_sz, mp_rgb565(0x7a, 0x88, 0xa0));
        snprintf(buf, sizeof buf, "%+d", c->row_oct_shift[row_idx]);
        int vw = mpe_font_text_width_px(buf, -1, val_sz);
        mpe_font_draw_text(t.fb, t.width, t.height,
                           cx + (cw - vw) / 2, dn->y + 3 + label_sz + 1,
                           buf, -1, val_sz, mp_rgb565(0xf0, 0xf4, 0xff));
    };
    draw_row_caption(r1_dn, r1_up, 0);
    draw_row_caption(r2_dn, r2_up, 1);

    /* Separator between button row and status row. */
    mp_hline_a(&t, 0, MPE_UI_STATUS_Y - 2, t.width,
               0x33, 0x55, 0x88, 80);
}

/* --- High-priority touch / MPE dispatch task -------------------- */

static void touch_task(void *arg)
{
    (void)arg;
    /* Target poll period: 4 ms (250 Hz). Well above the GT911's
       100 Hz sample rate so we never miss a fresh sample window,
       and well below the 35 ms latch hold inside mpe_touch. */
    const TickType_t period = pdMS_TO_TICKS(4);
    TickType_t next = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();
    int     prev_active = 0;
    while (true) {
        mpe_touch_frame tf = {};
        mpe_touch_poll(&tf);

        const int64_t now = esp_timer_get_time();
        const float dt = (float)(now - prev_us) / 1000000.0f;
        prev_us = now;

        mpe_controller_lock(&s_ctrl);
        const int active = mpe_controller_update(&s_ctrl, &tf);
        mpe_controller_tick_decay(&s_ctrl, dt);
        s_latest_tf = tf;
        mpe_controller_unlock(&s_ctrl);
        s_active_fingers.store(active, std::memory_order_relaxed);

        /* When ALL fingers come off — i.e. the last touch lifted —
           force the renderer through a multi-frame full clean so
           both buffers definitely look like the empty keyboard
           again. Mid-chord releases (5→4 finger drops) are NOT
           triggered: the residual queue handles per-finger areas,
           and the GT911 occasionally drops one of 5 sustained
           fingers and re-acquires it, which used to pin
           force_restore at >0 indefinitely and tank FPS to ~15. */
        if (active == 0 && prev_active > 0) {
            s_force_full_restore.store(4, std::memory_order_release);
        }
        prev_active = active;

        vTaskDelayUntil(&next, period);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "boot — heap free: %zu int / %zu psram",
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Initialise NVS early — the controller persists scale/root/PB/
       octave settings here, and we want them loaded before the
       splash so the keyboard template reflects the last session. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGW(TAG, "NVS init failed (%s) — settings won't persist",
                 esp_err_to_name(nvs_err));
    }

    /* 1. Display. */
    if (mpe_display_init() != ESP_OK) {
        ESP_LOGE(TAG, "display init failed; halting");
        return;
    }

    /* 2. Touch (depends on the I2C bus owned by mpe_display). */
    const bool touch_ok = (mpe_touch_init(mpe_display_i2c_bus()) == ESP_OK);
    if (!touch_ok) {
        ESP_LOGW(TAG, "touch unavailable — the UI will run but no MPE output");
    }

    /* 3. Font. */
    const size_t ttf_len = (size_t)(font_ttf_end - font_ttf_start);
    if (mpe_font_init(font_ttf_start, ttf_len) != ESP_OK) {
        ESP_LOGE(TAG, "font init failed — labels disabled");
    }

    /* 4. WiFi + animated boot screen.
         The MPE controller is fundamentally a networked instrument
         (RTP-MIDI + OSC), so we'd rather show the player something
         alive than freeze on a static splash for 30 seconds while
         the C6 negotiates association. We start WiFi async, then
         loop drawing an equalizer-bar animation each frame and
         polling the link state. */
    bool wifi_ok = false;
    if (strlen(CONFIG_WIFI_SSID) > 0) {
        if (mpe_wifi_start_async() != ESP_OK) {
            ESP_LOGW(TAG, "WiFi setup failed");
        } else {
            const int64_t boot_start_us = esp_timer_get_time();
            const int64_t boot_timeout_us = 30 * 1000 * 1000;
            for (;;) {
                esp_err_t r = mpe_wifi_wait(0);
                if (r == ESP_OK) { wifi_ok = true; break; }
                if (r == ESP_FAIL) break;
                if (esp_timer_get_time() - boot_start_us >= boot_timeout_us) break;

                uint16_t *fb = mpe_display_back_buffer();
                mp_target target = { fb, MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT };
                const float t_s = (float)(esp_timer_get_time() - boot_start_us)
                                  / 1000000.0f;
                draw_boot_screen(&target, t_s, CONFIG_WIFI_SSID);
                mpe_display_flush_writes(fb,
                    (size_t)MPE_DISPLAY_WIDTH * MPE_DISPLAY_HEIGHT * 2);
                mpe_display_present();
                /* Cap to ~50 FPS during boot and explicitly yield so
                   IDLE0 gets a slice — the task watchdog tripped
                   earlier on tight render loops. */
                vTaskDelay(pdMS_TO_TICKS(18));
            }
            if (!wifi_ok) ESP_LOGW(TAG, "WiFi failed — running offline");
        }
    } else {
        ESP_LOGW(TAG, "CONFIG_WIFI_SSID empty — network output disabled");
    }
    char wifi_ip[16] = "";
    if (wifi_ok) {
        mpe_wifi_get_ip_str(wifi_ip, sizeof wifi_ip);
        /* Tiny HTTP server that serves the live front buffer as
           BMP — pixel-perfect screenshots for docs. */
        if (mpe_screenshot_start() == ESP_OK) {
            ESP_LOGI(TAG, "screenshot: http://%s/screenshot.bmp", wifi_ip);
        }
    }

    /* 5. OSC. */
    static char osc_peer_str[48] = "";
    if (wifi_ok && strlen(CONFIG_MPE_OSC_HOST) > 0) {
        g_osc = mpe_osc_client_new(CONFIG_MPE_OSC_HOST, CONFIG_MPE_OSC_PORT);
        if (g_osc) {
            snprintf(osc_peer_str, sizeof osc_peer_str, "%s:%d",
                     CONFIG_MPE_OSC_HOST, CONFIG_MPE_OSC_PORT);
            ESP_LOGI(TAG, "OSC → %s", osc_peer_str);
        }
    }

    /* 6. AppleMIDI. */
    char midi_peer_str[48] = "";
    if (wifi_ok && strlen(CONFIG_MPE_MIDI_HOST) > 0) {
        snprintf(midi_peer_str, sizeof midi_peer_str, "%s:%d",
                 CONFIG_MPE_MIDI_HOST, CONFIG_MPE_MIDI_PORT);
        mpe_applemidi_start(CONFIG_MPE_MIDI_HOST,
                            (uint16_t)CONFIG_MPE_MIDI_PORT,
                            CONFIG_MPE_MIDI_SESSION_NAME);
    }

    /* 7. Controller config — dual keyboard. */
    mpe_controller_cfg cfg = {
        .rows               = CONFIG_MPE_KEYBOARD_ROWS,
        .octaves_per_row    = CONFIG_MPE_OCTAVES_PER_ROW,
        .lowest_octave      = CONFIG_MPE_LOWEST_OCTAVE,
        .row_octave_offset  = CONFIG_MPE_ROW_OCTAVE_OFFSET,
        .pb_range_semitones = CONFIG_MPE_MIDI_PB_RANGE_SEMITONES,
        .bundle_rate_hz     = CONFIG_MPE_OSC_BUNDLE_HZ,
        .fixed_velocity     = CONFIG_MPE_VELOCITY_FIXED,
        .member_lo          = 1,
        .member_hi          = 5,
        .ui_x = 0,
        .ui_y = MPE_UI_TOP_BAR_H,
        .ui_w = MPE_DISPLAY_WIDTH,
        .ui_h = MPE_DISPLAY_HEIGHT - MPE_UI_TOP_BAR_H,
    };
    mpe_controller_init(&s_ctrl, &cfg);

    /* On-screen chips. Top bar splits into two rows: button row
       (y=6..40) and status overlay (y=44..70). Buttons live in the
       static template; status overdraws dynamically. The layout
       below puts the title on the left, then scale + root chips,
       then two pairs of <  > steppers (one per row of the keyboard)
       with the row's current octave shift rendered as a static label
       between each pair.
       All buttons share the same row + height so the dirty-rect
       maintenance stays simple. */
    {
        const int16_t by = MPE_UI_BTN_Y;
        const int16_t bh = MPE_UI_BTN_H;
        const int16_t chip_w = 96;        /* chips slimmer to fit PB */
        const int16_t pb_chip_w = 80;
        const int16_t step_w = 28;
        const int16_t row_label_w = 46;
        const int16_t inter_chip = 6;
        const int16_t inter_step = 3;
        const int16_t inter_group = 14;
        const int16_t title_w = 210;

        int16_t x = title_w;
        const int16_t x_scale  = x;
        x += chip_w + inter_chip;
        const int16_t x_root   = x;
        x += chip_w + inter_chip;
        const int16_t x_pb     = x;
        x += pb_chip_w + inter_group;

        const int16_t x_r1_dn  = x;
        x += step_w + inter_step;
        x += row_label_w + inter_step;
        const int16_t x_r1_up  = x;
        x += step_w + inter_group;

        const int16_t x_r2_dn  = x;
        x += step_w + inter_step;
        x += row_label_w + inter_step;
        const int16_t x_r2_up  = x;

        const mpe_button btns[] = {
            { x_scale,  by, chip_w,    bh, MPE_BTN_CYCLE_SCALE   },
            { x_root,   by, chip_w,    bh, MPE_BTN_CYCLE_ROOT    },
            { x_pb,     by, pb_chip_w, bh, MPE_BTN_CYCLE_PB      },
            { x_r1_dn,  by, step_w,    bh, MPE_BTN_ROW0_OCT_DOWN },
            { x_r1_up,  by, step_w,    bh, MPE_BTN_ROW0_OCT_UP   },
            { x_r2_dn,  by, step_w,    bh, MPE_BTN_ROW1_OCT_DOWN },
            { x_r2_up,  by, step_w,    bh, MPE_BTN_ROW1_OCT_UP   },
        };
        mpe_controller_set_buttons(&s_ctrl, btns,
                                   sizeof btns / sizeof btns[0]);
    }

    mpe_ui ui;
    mpe_ui_init(&ui);

    /* 8. Pre-render the static UI template into PSRAM. Re-baked
       lazily inside the render loop whenever the controller's
       layout_version bumps (scale / root / row octave change). */
    s_template = (uint16_t *)heap_caps_aligned_alloc(
        64, kFbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_template) {
        ESP_LOGE(TAG, "template alloc failed (%zu B PSRAM)", kFbBytes);
        return;
    }
    render_template(s_template, &s_ctrl, ui.top_bar_h);
    mpe_display_flush_writes(s_template, kFbBytes);
    uint32_t baked_version = s_ctrl.layout_version;
    ESP_LOGI(TAG, "static template baked (%zu B, v%u)",
             kFbBytes, (unsigned)baked_version);

    /* 9. Paint the template into the scratch buffer once. The
       render loop overwrites it every frame anyway, but this gives
       a clean first frame before the loop starts. */
    {
        uint16_t *b = mpe_display_back_buffer();
        memcpy(b, s_template, kFbBytes);
        mpe_display_present();
    }

    /* 10. Kick the realtime touch + MPE dispatch task on CPU 1.
       Pinning matters: the render loop (this task) lives on CPU 0
       and is the one rendering frames. Without pinning, FreeRTOS
       may schedule the touch task on CPU 0 too, where both fight
       for cycles and FPS halves. CPU 1 sits mostly idle otherwise,
       so the touch task gets a full core to itself for GT911 polls
       + MIDI/OSC sends. */
    if (touch_ok) {
        xTaskCreatePinnedToCore(touch_task, "touch", 4096, NULL, 7,
                                NULL, 1);
        ESP_LOGI(TAG, "touch task @ 250 Hz (pinned to CPU 1)");
    }

    /* 11. Render loop. */
    bool mpe_config_sent = false;
    int frames = 0;
    int fps_disp = 0;
    int64_t last_fps_us = esp_timer_get_time();

    /* Local snapshot we render from so we don't hold the controller
       mutex across the (potentially 16 ms) render. */
    mpe_controller   snap_ctrl;
    mpe_touch_frame  snap_tf;

    ESP_LOGI(TAG, "entering render loop");

    while (true) {
        /* Has the layout changed (scale/root/octave button tap)?
           Re-bake the template, then mark BOTH buffers as fully
           dirty so the next two presents repaint everything. */
        const uint32_t live_ver = s_ctrl.layout_version;
        if (live_ver != baked_version) {
            render_template(s_template, &s_ctrl, ui.top_bar_h);
            mpe_display_flush_writes(s_template, kFbBytes);
            baked_version = live_ver;
            dirty_list_t full = {};
            dirty_push(&full, 0, 0, MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT);
            s_prev_dirty[0] = full;
            s_prev_dirty[1] = full;
        }

        /* Track the AppleMIDI session live state. We re-send the
           MPE configuration (Zone RPN + per-member PB-range RPN)
           on EVERY disconnected → connected transition. Without
           this, a host that drops the session (sleep, network
           hiccup) wakes up treating member channels as plain mono
           channels and never maps CC74 to per-note timbre — the
           expression would look "wired" on our side but do nothing
           on the host side. */
        const bool midi_up_now = mpe_applemidi_connected();
        if (midi_up_now && !mpe_config_sent) {
            const uint8_t members = (uint8_t)(cfg.member_hi - cfg.member_lo + 1);
            mpe_applemidi_send_mpe_config(members);
            for (int ch = cfg.member_lo; ch <= cfg.member_hi; ch++) {
                mpe_applemidi_send_pb_range((uint8_t)ch, cfg.pb_range_semitones);
            }
            mpe_config_sent = true;
            ESP_LOGI(TAG, "MPE configuration sent (%u members, PB ±%d semis)",
                     members, cfg.pb_range_semitones);
        } else if (!midi_up_now && mpe_config_sent) {
            /* Session went away. Clear the latch so we re-send when
               (or if) it comes back. */
            mpe_config_sent = false;
            ESP_LOGW(TAG, "MIDI session lost — will re-send MPE config on reconnect");
        }

        /* Snapshot controller + touch state under the lock — fast,
           ~hundreds of bytes copy, so we yield the lock back to the
           touch task immediately. */
        mpe_controller_lock(&s_ctrl);
        snap_ctrl = s_ctrl;       /* shallow copy ok — no pointers we own */
        snap_tf   = s_latest_tf;
        mpe_controller_unlock(&s_ctrl);
        snap_ctrl.lock = nullptr;
        (void)s_force_full_restore;
        (void)s_back_local;
        (void)ui;

        /* Single-FB architecture (post esp-bsp #581 finding).
           num_fbs=2 is broken in IDF v6.0; we now own a scratch
           buffer, paint a fresh full frame into it, and let
           draw_bitmap copy it into the driver's internal FB. All
           the partial-restore / dirty-rect / force-restore
           machinery for the old buffer-pair model is gone. */
        uint16_t *back = mpe_display_back_buffer();
        mp_target target = { back, MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT };

        /* Full template → scratch via PPA (DMA, ~3-5 ms at 1.2 MB
           on this stack — vs ~30 ms for CPU memcpy). The driver
           handles cache management on draw_bitmap; we don't need
           to track dirty rects ourselves. */
        mpe_display_rect_copy(back, s_template,
                              0, 0,
                              MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT);

        mpe_ui_status st = {};
        st.wifi_ip        = wifi_ip;
        st.midi_peer      = midi_peer_str;
        st.midi_rtt_ms    = mpe_applemidi_latency_ms();
        st.midi_connected = mpe_applemidi_connected();
        st.osc_peer       = (g_osc ? osc_peer_str : "");
        st.fps            = fps_disp;
        st.active_fingers = s_active_fingers.load(std::memory_order_relaxed);

        mpe_ui_render(&ui, &target, &snap_ctrl, &snap_tf, &st);

        mpe_display_present();

        frames++;
        const int64_t now = esp_timer_get_time();
        if ((now - last_fps_us) >= 1000000) {
            fps_disp = frames;
            frames = 0;
            last_fps_us = now;
            ESP_LOGI(TAG, "fps=%d heap=%zu int %zu psram",
                     fps_disp,
                     heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        }

        /* Yield 1 tick so IDLE0 actually runs (watchdog feeder). */
        vTaskDelay(1);
    }
}
