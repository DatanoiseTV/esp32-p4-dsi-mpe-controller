/*
 * controller.h — touch-to-MPE/OSC controller, Seaboard-style.
 *
 * Surface model:
 *   - A dual-row piano keyboard with real white/black key layout.
 *     Each row spans cfg.octaves_per_row octaves; the top row is
 *     offset above the bottom row by cfg.row_octave_offset octaves
 *     so the player has two hand-sized ranges side by side.
 *   - Black keys overlay the upper portion of their row; hit-test
 *     priorities blacks first so a touch on the upper-middle of a
 *     C-position lands on C# (when present), not C.
 *   - Live state: scale + root + per-row octave shift. Changes
 *     re-tag every key with a new MIDI note and tint non-scale keys
 *     in the static template re-render.
 *
 * Per touch (MPE expression):
 *   STRIKE   velocity from contact size at NoteOn
 *   GLIDE    pitch-bend tracks Δx pixels from snap anchor;
 *            1 white-key of slide = 1 semitone (host-scaled by
 *            cfg.pb_range_semitones)
 *   SLIDE    CC74 from screen-Y normalised across the whole
 *            playable surface (0 at bottom, 127 at top)
 *   PRESS    channel pressure from ongoing contact size
 *   LIFT     NoteOff with release velocity 64 (GT911 can't measure
 *            actual lift force)
 *
 * On-screen controls:
 *   The status bar carries a scale selector, a root selector, and
 *   two octave-shift chips (one per row). Taps on these are caught
 *   by the controller before any MPE dispatch happens; each tap
 *   cycles the state and bumps a version counter so the renderer
 *   can rebuild the static template lazily.
 */
#ifndef MPE_CONTROLLER_H_
#define MPE_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "mpe_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPE_CTRL_MAX_FINGERS 5         /* GT911 hardware ceiling */
#define MPE_MAX_KEYS         128       /* 2 rows × 4 oct × 12 = 96 worst */
#define MPE_MAX_ROWS         3
#define MPE_NUM_SCALES       6
#define MPE_MAX_BUTTONS      8

/* Musical scale identifiers. The pitch-class mask for each is in
   controller.cpp's k_scale_pc_mask. */
typedef enum {
    MPE_SCALE_CHROMATIC = 0,
    MPE_SCALE_MAJOR,
    MPE_SCALE_MINOR,
    MPE_SCALE_PENTATONIC,
    MPE_SCALE_DORIAN,
    MPE_SCALE_BLUES,
} mpe_scale_id;

/* One key on the dual keyboard. midi_note is the value sent on
   NoteOn — it accounts for scale state, root, and per-row octave
   shift, so the touch path doesn't have to re-derive it. */
typedef struct {
    int16_t midi_note;
    int16_t row;
    int16_t x, y;
    int16_t w, h;
    uint8_t is_black;
    uint8_t pc;          /* pitch class (0..11) */
    uint8_t in_scale;    /* 1 if this key's pc is in the current scale */
} mpe_key;

typedef struct {
    int     n_keys;
    int     n_rows;
    int     row_top_y[MPE_MAX_ROWS];
    int     row_h    [MPE_MAX_ROWS];
    int     white_w  [MPE_MAX_ROWS];
    int     black_w  [MPE_MAX_ROWS];
    int     black_h  [MPE_MAX_ROWS];
    mpe_key keys[MPE_MAX_KEYS];
} mpe_keyboard;

/* On-screen button (scale/root/octave chips in the top bar). The
   action a tap takes is encoded by `action_id`. */
typedef enum {
    MPE_BTN_NONE = 0,
    MPE_BTN_CYCLE_SCALE,
    MPE_BTN_CYCLE_ROOT,
    MPE_BTN_CYCLE_PB,         /* pitch-bend range: 1 / 4 / 12 / 48 semis */
    MPE_BTN_TOGGLE_PB_MODE,   /* piecewise-linear vs uniform pitch bend */
    MPE_BTN_CYCLE_LAYOUT,     /* Piano / Grid / Slide */
    MPE_BTN_ROW0_OCT_DOWN,
    MPE_BTN_ROW0_OCT_UP,
    MPE_BTN_ROW1_OCT_DOWN,
    MPE_BTN_ROW1_OCT_UP,
} mpe_btn_action;

/* Available surface layouts. Piano is the dual-row keyboard we
   shipped with; Grid is an isomorphic cell layout (LinnStrument-
   style, each row tuned a perfect fourth above the one below);
   Slide is a continuous ribbon (Bebot-style) where any touch
   position picks a semitone, then horizontal motion bends + Y
   and pressure drive expression. */
typedef enum {
    MPE_LAYOUT_PIANO = 0,
    MPE_LAYOUT_GRID,
    MPE_LAYOUT_SLIDE,
    MPE_NUM_LAYOUTS,
} mpe_layout_id;

typedef struct {
    int16_t        x, y, w, h;
    mpe_btn_action action;
} mpe_button;

typedef struct {
    bool     active;
    int      tracking_id;
    int      ch;
    int      note;
    int      key_idx;
    int      init_x_px;
    int      init_y_px;
    float    x_norm;       /* 0..1 across the full touch surface */
    float    y_norm;       /* 0..1, 1 = top of surface */
    float    z;
    uint16_t last_pb;
    uint8_t  last_y_cc;
    uint8_t  last_z_cp;
    int64_t  press_us;
    bool     is_ui;        /* this contact was consumed by a UI button —
                              don't generate MIDI for it */
} mpe_finger;

typedef struct {
    int   rows;                 /* keyboard rows (default 2) */
    int   octaves_per_row;
    int   lowest_octave;        /* SPN; 2 = C2 on the bottom row */
    int   row_octave_offset;    /* top row C = bottom row C + this */
    int   pb_range_semitones;
    int   bundle_rate_hz;
    int   fixed_velocity;
    int   member_lo;
    int   member_hi;

    int   ui_x;
    int   ui_y;
    int   ui_w;
    int   ui_h;
} mpe_controller_cfg;

/* A short-lived "this finger just released; please keep restoring
   the screen at this rectangle for a few frames" entry. Without it,
   the buffer-pair dirty-rect bookkeeping is theoretically airtight
   but in practice misses certain release transitions and leaves
   stuck halos on the panel. The residual TTL gives the renderer
   multiple restore passes on both buffers — a robust backstop. */
#define MPE_RESIDUAL_CAP   8
typedef struct {
    int16_t x, y, w, h;
    int64_t expire_us;
} mpe_residual_dirty;

typedef struct {
    mpe_controller_cfg cfg;
    mpe_keyboard       kb;

    /* Live, mutable state from on-screen controls. */
    mpe_scale_id       scale;
    int                root_pc;            /* 0..11 */
    int                row_oct_shift[MPE_MAX_ROWS];
    int                pb_range_live;      /* current PB range in semis;
                                              cycled by the on-screen PB
                                              button. Initialised from
                                              cfg.pb_range_semitones. */
    int                pb_mode;            /* 0 = piecewise-linear
                                              (key-center anchored),
                                              1 = uniform "1 white-key
                                              width = 1 semitone" */
    mpe_layout_id      layout;             /* Piano / Grid / Slide */

    /* Bumped whenever any of the above changes so the renderer
       knows to rebake the static template. */
    uint32_t           layout_version;

    /* On-screen buttons (built at init with the rest of the UI). */
    int                n_buttons;
    mpe_button         buttons[MPE_MAX_BUTTONS];

    mpe_finger         fingers[MPE_CTRL_MAX_FINGERS];

    /* Stale-region cleanup queue — populated on finger release. */
    int                n_residual;
    mpe_residual_dirty residual[MPE_RESIDUAL_CAP];

    int64_t            last_osc_emit_us;
    float              ch_busy_recency[16];
    void              *lock;
} mpe_controller;

/* ----- lifecycle / state ------------------------------------------ */

void mpe_controller_init(mpe_controller *c, const mpe_controller_cfg *cfg);

void mpe_controller_set_buttons(mpe_controller *c,
                                const mpe_button *btns, int n);

/* Returns the human-readable name of the current scale. */
const char *mpe_controller_scale_name(const mpe_controller *c);
const char *mpe_controller_root_name (const mpe_controller *c);
const char *mpe_controller_pb_label  (const mpe_controller *c);
const char *mpe_controller_pb_mode_label(const mpe_controller *c);
const char *mpe_controller_layout_name(const mpe_controller *c);

/* Re-build the keyboard with current scale/root/octave shifts. Bumps
   layout_version. Cheap (only the keys[] array is touched). */
void mpe_controller_rebuild_keyboard(mpe_controller *c);

/* Hit-test a pixel coordinate against the keyboard. Returns key
   index, or -1 if none. Black keys win over whites where they
   overlap. */
int mpe_controller_hit_key(const mpe_controller *c, int px, int py);

/* Hit-test against the on-screen buttons. Returns action or
   MPE_BTN_NONE. */
mpe_btn_action mpe_controller_hit_button(const mpe_controller *c,
                                         int px, int py);

/* Apply the action (cycles scale / root / row octave). Returns true
   if the layout changed (caller should rebake the static template). */
bool mpe_controller_apply_button(mpe_controller *c,
                                 mpe_btn_action action);

/* ----- per-frame ---------------------------------------------------- */

int  mpe_controller_update(mpe_controller *c, const mpe_touch_frame *f);
void mpe_controller_tick_decay(mpe_controller *c, float dt_s);

void mpe_controller_lock(mpe_controller *c);
void mpe_controller_unlock(mpe_controller *c);

#ifdef __cplusplus
}
#endif

#endif
