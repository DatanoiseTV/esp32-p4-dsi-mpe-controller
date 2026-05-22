/*
 * ui.h — animated MPE-controller UI renderer.
 *
 * Renders into the EK79007's 1024x600 RGB565 framebuffer at ~60 Hz.
 * Layout:
 *
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ TITLE  WiFi: ip   MIDI: peer (rtt ms)   OSC: peer   FPS: 60  │  ~60 px
 *   ├──────────────────────────────────────────────────────────────┤
 *   │                                                              │
 *   │            ISOMORPHIC NOTE GRID  (cols × rows)               │
 *   │       each cell labelled, root notes highlighted             │
 *   │              + animated finger glow trails                   │
 *   │                                                              │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * The renderer is stateful — it keeps trails of recent finger
 * positions for the "comet" effect and animates the background
 * gradient phase based on wall clock time. All state lives in a
 * mpe_ui struct supplied by the caller.
 */
#ifndef MPE_UI_H_
#define MPE_UI_H_

#include <stdint.h>
#include <stdbool.h>

#include "mpe_paint.h"
#include "controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPE_UI_TRAIL_LEN 4     /* trail comet length. Each trail point
                                  is a mp_glow_add call with per-pixel
                                  blend; the 5th+ points decayed below
                                  the drawing threshold most of the
                                  time anyway. 4 keeps the visible
                                  motion read but halves the work. */

/* The status bar is two stacked rows of 36 px each (button row +
   status row). 72 px total — the single source of truth for ui_y. */
#define MPE_UI_TOP_BAR_H     72
#define MPE_UI_BTN_Y         6
#define MPE_UI_BTN_H         34
#define MPE_UI_STATUS_Y      44
#define MPE_UI_STATUS_H      26

typedef struct {
    int   x, y;
    float intensity;   /* 0..1, fades with age */
} mpe_ui_trail_pt;

typedef struct {
    /* one ring buffer per finger slot */
    mpe_ui_trail_pt pts[MPE_CTRL_MAX_FINGERS][MPE_UI_TRAIL_LEN];
    int             head[MPE_CTRL_MAX_FINGERS];
} mpe_ui_trails;

typedef struct {
    /* Live status text supplied each frame; the renderer doesn't
       allocate. */
    const char *wifi_ip;
    const char *midi_peer;
    int         midi_rtt_ms;
    bool        midi_connected;
    const char *osc_peer;
    int         fps;
    int         active_fingers;
} mpe_ui_status;

typedef struct {
    mpe_ui_trails  trails;
    int64_t        anim_t0_us;     /* esp_timer_get_time() of init */
    int            top_bar_h;      /* px */
} mpe_ui;

void mpe_ui_init(mpe_ui *u);

/* Render one frame into target. Reads current touch + finger state
   from `controller`. */
void mpe_ui_render(mpe_ui *u,
                   const mp_target *target,
                   const mpe_controller *controller,
                   const mpe_touch_frame *latest_touch,
                   const mpe_ui_status *status);

/* Push the current finger position into the trail ring. Called
   between renders by the main loop. */
void mpe_ui_push_trails(mpe_ui *u, const mpe_controller *controller);

#ifdef __cplusplus
}
#endif

#endif
