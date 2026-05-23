/*
 * controller.cpp — Seaboard-style dual keyboard + MPE/OSC dispatch.
 *
 * The renderer does not run from here; this file owns the data
 * model (key map, scale state, finger state) and the touch -> MIDI
 * + OSC dispatch. main.cpp asks the controller to redraw the
 * static template whenever layout_version bumps.
 *
 * Hit-test rule: a touch first checks the on-screen buttons (top
 * bar). If it falls on one, the contact is "consumed by UI" and no
 * MPE traffic is generated for it. Otherwise we run keyboard hit-
 * test (blacks before whites) and either NoteOn a new finger or
 * continue tracking an existing one.
 */
#include "controller.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "mpe_applemidi.h"
#include "mpe_osc.h"

/* NVS namespace + keys for persisted on-screen settings. */
#define MPE_NVS_NS "mpe"

static void persist_state_(const mpe_controller *c)
{
    nvs_handle_t h;
    if (nvs_open(MPE_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "scale",    (uint8_t)c->scale);
    nvs_set_u8(h, "root_pc",  (uint8_t)c->root_pc);
    nvs_set_u8(h, "pb_range", (uint8_t)c->pb_range_live);
    nvs_set_u8(h, "pb_mode",  (uint8_t)c->pb_mode);
    nvs_set_i8(h, "r0_oct",   (int8_t)c->row_oct_shift[0]);
    nvs_set_i8(h, "r1_oct",   (int8_t)c->row_oct_shift[1]);
    nvs_commit(h);
    nvs_close(h);
}

static void load_state_(mpe_controller *c)
{
    nvs_handle_t h;
    if (nvs_open(MPE_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t u8;
    int8_t  i8;
    if (nvs_get_u8(h, "scale", &u8) == ESP_OK && u8 < MPE_NUM_SCALES) {
        c->scale = (mpe_scale_id)u8;
    }
    if (nvs_get_u8(h, "root_pc", &u8) == ESP_OK && u8 < 12) {
        c->root_pc = u8;
    }
    if (nvs_get_u8(h, "pb_range", &u8) == ESP_OK) {
        /* Accept only the values cycled by the on-screen button so we
           never end up with a value the UI can't represent. */
        if (u8 == 1 || u8 == 4 || u8 == 12 || u8 == 48) {
            c->pb_range_live = u8;
        }
    }
    if (nvs_get_u8(h, "pb_mode", &u8) == ESP_OK && u8 <= 2) {
        c->pb_mode = u8;
    }
    if (nvs_get_i8(h, "r0_oct", &i8) == ESP_OK && i8 >= -3 && i8 <= 3) {
        c->row_oct_shift[0] = i8;
    }
    if (nvs_get_i8(h, "r1_oct", &i8) == ESP_OK && i8 >= -3 && i8 <= 3) {
        c->row_oct_shift[1] = i8;
    }
    nvs_close(h);
}

extern mpe_osc_client *g_osc;
static const char *TAG = "ctrl";

static void init_velocity_curve_(void);

/* ---- helpers ------------------------------------------------------ */

static inline int clampi(int v, int lo, int hi)
{ return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

/* Bitmask of pitch classes for each scale, rotated so bit 0 = the
   scale's root (we rotate by `root_pc` at lookup time). */
static const uint16_t k_scale_pc_mask[MPE_NUM_SCALES] = {
    [MPE_SCALE_CHROMATIC] = 0x0FFF,                /* every pc */
    [MPE_SCALE_MAJOR]     = (1<<0)|(1<<2)|(1<<4)|(1<<5)|(1<<7)|(1<<9)|(1<<11),
    [MPE_SCALE_MINOR]     = (1<<0)|(1<<2)|(1<<3)|(1<<5)|(1<<7)|(1<<8)|(1<<10),
    [MPE_SCALE_PENTATONIC]= (1<<0)|(1<<2)|(1<<4)|(1<<7)|(1<<9),
    [MPE_SCALE_DORIAN]    = (1<<0)|(1<<2)|(1<<3)|(1<<5)|(1<<7)|(1<<9)|(1<<10),
    [MPE_SCALE_BLUES]     = (1<<0)|(1<<3)|(1<<5)|(1<<6)|(1<<7)|(1<<10),
};
static const char *k_scale_names[MPE_NUM_SCALES] = {
    "Chromatic", "Major", "Minor", "Pentatonic", "Dorian", "Blues",
};
static const char *k_note_names[12] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

static bool pc_in_scale(int pc, int root_pc, mpe_scale_id scale)
{
    int rel = ((pc - root_pc) % 12 + 12) % 12;
    return (k_scale_pc_mask[scale] >> rel) & 1u;
}

const char *mpe_controller_scale_name(const mpe_controller *c)
{ return k_scale_names[c->scale]; }
const char *mpe_controller_root_name (const mpe_controller *c)
{ return k_note_names[c->root_pc]; }

const char *mpe_controller_pb_label(const mpe_controller *c)
{
    switch (c->pb_range_live) {
    case 1:  return "1 st";
    case 4:  return "2 t";    /* two whole-tones = 4 semis */
    case 12: return "octv";
    case 48: return "48 st";
    default: return "?";
    }
}

const char *mpe_controller_pb_mode_label(const mpe_controller *c)
{
    switch (c->pb_mode) {
    case 1:  return "Lin";   /* 1 white_w = 1 semi (uniform) */
    case 2:  return "Wht";   /* 1 white_w = 2 semis (whole-step uniform) */
    default: return "Key";   /* piecewise key-center anchored */
    }
}

/* ---- keyboard build ---------------------------------------------- */

static int midi_for(int octave_spn, int pc)
{
    /* MIDI: middle C = 60 = "C4" in scientific pitch notation.
       So octave-N C = (N+1)*12. */
    return (octave_spn + 1) * 12 + pc;
}

void mpe_controller_rebuild_keyboard(mpe_controller *c)
{
    static const int kWhitePc[7]      = {0, 2, 4, 5, 7, 9, 11};
    static const int kBlackPc[5]      = {1, 3, 6, 8, 10};
    static const int kBlackAfter[5]   = {0, 1, 3, 4, 5}; /* whites the
                                                            black sits
                                                            right of */
    const mpe_controller_cfg *cfg = &c->cfg;
    mpe_keyboard *kb = &c->kb;
    kb->n_keys = 0;
    kb->n_rows = cfg->rows;

    const int row_h_total = cfg->ui_h / cfg->rows;
    const int whites_per_row = cfg->octaves_per_row * 7;
    const int white_w = cfg->ui_w / whites_per_row;
    const int black_w = (white_w * 6) / 10;
    const int black_h = (row_h_total * 60) / 100;

    for (int r = 0; r < cfg->rows; r++) {
        kb->row_top_y[r] = cfg->ui_y + r * row_h_total;
        kb->row_h    [r] = row_h_total;
        kb->white_w  [r] = white_w;
        kb->black_w  [r] = black_w;
        kb->black_h  [r] = black_h;

        /* Octave for THIS row. row 0 = top of screen = higher notes.
           Bottom row (last) starts at cfg.lowest_octave.
           Row above adds row_octave_offset per row of distance. */
        const int rows_from_bottom = (cfg->rows - 1) - r;
        const int row_base_octave  = cfg->lowest_octave
                                     + rows_from_bottom * cfg->row_octave_offset
                                     + c->row_oct_shift[r];

        /* Whites first, then blacks — the array order is also the
           render order (whites drawn first as the base layer). */
        for (int oct = 0; oct < cfg->octaves_per_row; oct++) {
            for (int wk = 0; wk < 7; wk++) {
                if (kb->n_keys >= MPE_MAX_KEYS) break;
                const int pc = kWhitePc[wk];
                const int wk_global = oct * 7 + wk;
                mpe_key *k = &kb->keys[kb->n_keys++];
                k->midi_note = midi_for(row_base_octave + oct, pc);
                k->row       = r;
                k->x         = cfg->ui_x + wk_global * white_w;
                k->y         = kb->row_top_y[r];
                k->w         = white_w;
                k->h         = row_h_total;
                k->is_black  = 0;
                k->pc        = (uint8_t)pc;
                k->in_scale  = pc_in_scale(pc, c->root_pc, c->scale) ? 1 : 0;
            }
        }
        for (int oct = 0; oct < cfg->octaves_per_row; oct++) {
            for (int bk = 0; bk < 5; bk++) {
                if (kb->n_keys >= MPE_MAX_KEYS) break;
                const int pc = kBlackPc[bk];
                const int wk_idx = oct * 7 + kBlackAfter[bk];
                mpe_key *k = &kb->keys[kb->n_keys++];
                k->midi_note = midi_for(row_base_octave + oct, pc);
                k->row       = r;
                k->x         = cfg->ui_x + (wk_idx + 1) * white_w - black_w / 2;
                k->y         = kb->row_top_y[r];
                k->w         = black_w;
                k->h         = black_h;
                k->is_black  = 1;
                k->pc        = (uint8_t)pc;
                k->in_scale  = pc_in_scale(pc, c->root_pc, c->scale) ? 1 : 0;
            }
        }
    }

    c->layout_version++;
}

void mpe_controller_init(mpe_controller *c, const mpe_controller_cfg *cfg)
{
    memset(c, 0, sizeof *c);
    c->cfg = *cfg;
    c->lock = (void *)xSemaphoreCreateMutex();
    init_velocity_curve_();
    c->scale          = MPE_SCALE_CHROMATIC;
    c->root_pc        = 0;
    c->pb_range_live  = cfg->pb_range_semitones;
    c->layout_version = 0;
    for (int i = 0; i < MPE_MAX_ROWS; i++) c->row_oct_shift[i] = 0;

    /* Pull persisted settings — scale, root, PB range, octave
       shifts — from NVS if they were saved previously. Falls back to
       the defaults above if no entry exists. */
    load_state_(c);
    ESP_LOGI(TAG, "loaded settings: scale=%d root=%d pb=%d r0=%d r1=%d",
             c->scale, c->root_pc, c->pb_range_live,
             c->row_oct_shift[0], c->row_oct_shift[1]);
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        c->fingers[i].active      = false;
        c->fingers[i].tracking_id = -1;
        c->fingers[i].ch          = -1;
        c->fingers[i].key_idx     = -1;
    }
    mpe_controller_rebuild_keyboard(c);
}

void mpe_controller_set_buttons(mpe_controller *c,
                                const mpe_button *btns, int n)
{
    if (n > MPE_MAX_BUTTONS) n = MPE_MAX_BUTTONS;
    c->n_buttons = n;
    for (int i = 0; i < n; i++) c->buttons[i] = btns[i];
}

void mpe_controller_lock(mpe_controller *c)
{ if (c->lock) xSemaphoreTake((SemaphoreHandle_t)c->lock, portMAX_DELAY); }
void mpe_controller_unlock(mpe_controller *c)
{ if (c->lock) xSemaphoreGive((SemaphoreHandle_t)c->lock); }

/* ---- hit test ---------------------------------------------------- */

int mpe_controller_hit_key(const mpe_controller *c, int px, int py)
{
    /* Black keys are stored after whites in the keys[] array, but
       they overlay the upper portion of their row, so we scan in
       reverse — first black-key suffix, then white-key prefix —
       which both honours visual stacking and avoids a second pass. */
    for (int i = c->kb.n_keys - 1; i >= 0; i--) {
        const mpe_key *k = &c->kb.keys[i];
        if (px >= k->x && px < k->x + k->w &&
            py >= k->y && py < k->y + k->h) {
            return i;
        }
    }
    return -1;
}

mpe_btn_action mpe_controller_hit_button(const mpe_controller *c,
                                         int px, int py)
{
    for (int i = 0; i < c->n_buttons; i++) {
        const mpe_button *b = &c->buttons[i];
        if (px >= b->x && px < b->x + b->w &&
            py >= b->y && py < b->y + b->h) {
            return b->action;
        }
    }
    return MPE_BTN_NONE;
}

bool mpe_controller_apply_button(mpe_controller *c, mpe_btn_action a)
{
    switch (a) {
    case MPE_BTN_CYCLE_SCALE:
        c->scale = (mpe_scale_id)((c->scale + 1) % MPE_NUM_SCALES);
        break;
    case MPE_BTN_CYCLE_ROOT:
        c->root_pc = (c->root_pc + 1) % 12;
        break;
    case MPE_BTN_CYCLE_PB: {
        /* 1 semitone → 4 (two tones) → 12 (octave) → 48 (wide MPE)
           and back. After cycling, re-publish the per-channel
           pitch-bend RPN to every member channel so the synth's
           scaling matches what we're now sending. */
        int next;
        switch (c->pb_range_live) {
        case 1:  next = 4;  break;
        case 4:  next = 12; break;
        case 12: next = 48; break;
        default: next = 1;  break;
        }
        c->pb_range_live = next;
        for (int ch = c->cfg.member_lo; ch <= c->cfg.member_hi; ch++) {
            mpe_applemidi_send_pb_range((uint8_t)ch, (uint8_t)next);
        }
        /* No keyboard rebuild needed for PB — but the chip label
           changed, so bump layout_version so the template re-bakes
           and the new value appears on screen. */
        c->layout_version++;
        persist_state_(c);
        return true;
    }
    case MPE_BTN_TOGGLE_PB_MODE:
        /* Cycle Key → Lin → Wht → Key. */
        c->pb_mode = (c->pb_mode + 1) % 3;
        c->layout_version++;
        persist_state_(c);
        return true;
    case MPE_BTN_ROW0_OCT_DOWN:
        if (c->row_oct_shift[0] > -3) c->row_oct_shift[0]--;
        break;
    case MPE_BTN_ROW0_OCT_UP:
        if (c->row_oct_shift[0] <  3) c->row_oct_shift[0]++;
        break;
    case MPE_BTN_ROW1_OCT_DOWN:
        if (c->cfg.rows > 1 && c->row_oct_shift[1] > -3) c->row_oct_shift[1]--;
        break;
    case MPE_BTN_ROW1_OCT_UP:
        if (c->cfg.rows > 1 && c->row_oct_shift[1] <  3) c->row_oct_shift[1]++;
        break;
    default:
        return false;
    }
    mpe_controller_rebuild_keyboard(c);
    persist_state_(c);
    return true;
}

/* ---- per-touch internals ---------------------------------------- */

static int find_finger_by_tid(const mpe_controller *c, int tid)
{
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        if (c->fingers[i].active && c->fingers[i].tracking_id == tid) return i;
    }
    return -1;
}

static int alloc_finger_slot(const mpe_controller *c)
{
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        if (!c->fingers[i].active) return i;
    }
    return -1;
}

static int alloc_member_channel(const mpe_controller *c)
{
    for (int ch = c->cfg.member_lo; ch <= c->cfg.member_hi; ch++) {
        bool used = false;
        for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
            if (c->fingers[i].active && c->fingers[i].ch == ch) {
                used = true; break;
            }
        }
        if (!used) return ch;
    }
    return -1;
}

/* Pixel position → semitone offset (piecewise-linear chromatic).
 *
 * On a piano-style layout the visual semitone spacing is NOT
 * uniform: white keys are 7 per octave (spaced octave_w/7 apart),
 * black keys sit between specific pairs, and the natural half-
 * steps E-F and B-C have NO black key between them. So a single
 * pixels-per-semitone constant can't make pitch match position
 * across all keys at once.
 *
 * Instead, model each octave as a piecewise-linear map between
 * the 12 key centers. Each key center is one tick on the half-
 * white-key grid (white_w / 2 pixels):
 *
 *   C   C#  D   D#  E       F   F#  G   G#  A   A#  B       C
 *   1   2   3   4   5       7   8   9   10  11  12  13      15
 *   semi 0..4 (slope 1 in half-units = slope 0.5 in white-units),
 *   GAP between E and F (no black key at position 6),
 *   semi 5..11 same pattern,
 *   GAP between B and next C.
 *
 * Linear interpolation between adjacent key-center positions makes
 * pitch glide smoothly. Sliding from C-center to D-center
 * (1 white_w = 2 half-units) produces 2 semitones. From E-center to
 * F-center (1 white_w but a natural half-step) produces 1 semitone.
 * Audibly matches the visual key spacing exactly. */
static float chromatic_semitone_at_(const mpe_controller *c, int row, int x_abs)
{
    const int white_w = c->kb.white_w[clampi(row, 0, MPE_MAX_ROWS - 1)];
    if (white_w <= 0) return 0.0f;
    const float half_w = (float)white_w / 2.0f;
    const float pos_units = (float)(x_abs - c->cfg.ui_x) / half_w;

    /* 14 half-units per octave (7 white_w). */
    const float OCT_UNITS = 14.0f;
    const float octave_idx = floorf(pos_units / OCT_UNITS);
    const float within_oct = pos_units - octave_idx * OCT_UNITS;

    /* Key-center positions (in half_w units within an octave). 13
       entries covering semitones 0..12 (C up to next C). */
    static const float pos_table[13] = {
        1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 15
    };

    float semis_in_oct;
    if (within_oct <= pos_table[0]) {
        /* Off the bottom (left of C center) — extrapolate. */
        semis_in_oct = within_oct - pos_table[0];
    } else if (within_oct >= pos_table[12]) {
        /* Off the top (right of next C center). */
        semis_in_oct = 12.0f + (within_oct - pos_table[12]);
    } else {
        /* Find bracketing centers and lerp. */
        int s = 0;
        while (s < 12 && within_oct >= pos_table[s + 1]) s++;
        const float lo = pos_table[s];
        const float hi = pos_table[s + 1];
        const float frac = (within_oct - lo) / (hi - lo);
        semis_in_oct = (float)s + frac;
    }
    return octave_idx * 12.0f + semis_in_oct;
}

/* Pixel displacement → 14-bit MPE pitch bend. Three modes cycled
   from the on-screen "Bend" chip:
     0 ("Key", default): piecewise-linear chromatic — the bend
        tracks actual key centers, so audible pitch matches the
        visual key spacing exactly even where natural half-steps
        live (E-F, B-C). Sliding between any two key centers
        produces their actual musical interval.
     1 ("Lin"): uniform "1 white-key width = 1 semitone".
        Original project default. Seaboard-strip feel — slides
        consistently regardless of which keys you cross, but
        doesn't match piano-key musical intervals.
     2 ("Wht"): uniform "1 white-key width = 2 semitones".
        Treats every white-to-adjacent-white distance as a whole
        step (the C-D ratio). Useful when you want the bend to
        match the visual white-key distance audibly — the natural
        half-step regions (E-F, B-C) become "wider" than musically
        accurate, but C-D and similar whole-step slides land
        right on the next white key. */
static uint16_t pb_from_displacement(const mpe_controller *c,
                                     int init_x, int dx_px, int row)
{
    float bend_semis;
    if (c->pb_mode == 1) {
        const int white_w = c->kb.white_w[clampi(row, 0, MPE_MAX_ROWS - 1)];
        const float key_w = white_w > 0 ? (float)white_w : 1.0f;
        bend_semis = (float)dx_px / key_w;
        (void)init_x;
    } else if (c->pb_mode == 2) {
        const int white_w = c->kb.white_w[clampi(row, 0, MPE_MAX_ROWS - 1)];
        const float key_w = white_w > 0 ? (float)white_w : 1.0f;
        bend_semis = (float)dx_px * 2.0f / key_w;
        (void)init_x;
    } else {
        const float init_semi = chromatic_semitone_at_(c, row, init_x);
        const float cur_semi  = chromatic_semitone_at_(c, row, init_x + dx_px);
        bend_semis = cur_semi - init_semi;
    }
    const float bend_norm = bend_semis / (float)c->pb_range_live;
    int v = 0x2000 + (int)(bend_norm * 8192.0f);
    return (uint16_t)clampi(v, 0, 0x3FFF);
}

/* Velocity / pressure curve.
 *
 * A linear map of contact-area to CC value feels wrong: GT911 reports
 * a normal fingertip at ~30-70 in its "size" units. A linear mapping
 * makes a soft tap already half-volume, while a forceful press only
 * adds a little. The musical feel we want:
 *
 *   - soft initial response (light touches → very low velocity), so
 *     the player has dynamic headroom from the bottom;
 *   - exponential ramp through the playable range so firm playing
 *     gets up into MIDI 90-127 without needing to mash;
 *   - saturation past a realistic upper bound (strength = 90) so the
 *     curve doesn't waste range on values the chip almost never
 *     reports.
 *
 * v = (clamp(strength, 0, 90) / 90)^1.7 × 127
 *
 * Pre-computed into a 128-entry LUT at init so the hot path is two
 * memory lookups (one for velocity, one for Z) with no FPU. */
/* Both knobs come from Kconfig so the player can match the curve to
   their panel + playing style. Defaults match a typical fingertip
   on the EK79007 glass with a moderately exponential ramp. */
#define VELOCITY_CURVE_MAX_IN  CONFIG_MPE_PRESSURE_SATURATION
#define VELOCITY_CURVE_EXP     ((float)CONFIG_MPE_PRESSURE_EXPONENT_X100 / 100.0f)
static uint8_t s_velocity_lut[128];

static void init_velocity_curve_(void)
{
    const int   max_in = VELOCITY_CURVE_MAX_IN;
    const float exp_v  = VELOCITY_CURVE_EXP;
    for (int s = 0; s < 128; s++) {
        int sat = s > max_in ? max_in : s;
        float n = (float)sat / (float)max_in;
        float v = powf(n, exp_v) * 127.0f;
        if (v < 0) v = 0;
        if (v > 127) v = 127;
        s_velocity_lut[s] = (uint8_t)(v + 0.5f);
    }
}

static uint8_t midi_from_strength(int strength)
{
    if (strength <= 0) return 0;
    if (strength > 127) strength = 127;
    return s_velocity_lut[strength];
}

static void compute_norms(const mpe_controller *c, int px, int py,
                          float *out_x_norm, float *out_y_norm)
{
    const float xn = ((float)(px - c->cfg.ui_x)) / (float)(c->cfg.ui_w);
    const float yn = 1.0f - ((float)(py - c->cfg.ui_y)) / (float)(c->cfg.ui_h);
    if (out_x_norm) *out_x_norm = clampf(xn, 0.0f, 1.0f);
    if (out_y_norm) *out_y_norm = clampf(yn, 0.0f, 1.0f);
}

/* ---- main update -------------------------------------------------- */

int mpe_controller_update(mpe_controller *c, const mpe_touch_frame *f)
{
    const int64_t now = esp_timer_get_time();
    bool seen[MPE_CTRL_MAX_FINGERS] = { false, false, false, false, false };

    const int n = (f->count > MPE_TOUCH_MAX_POINTS) ? MPE_TOUCH_MAX_POINTS : f->count;
    for (int i = 0; i < n; i++) {
        const mpe_touch_point *p = &f->points[i];

        int slot = find_finger_by_tid(c, p->tracking_id);

        if (slot < 0) {
            /* New contact. First check the UI buttons — a tap there
               is consumed (no MIDI). */
            const mpe_btn_action act = mpe_controller_hit_button(c, p->x, p->y);
            if (act != MPE_BTN_NONE) {
                slot = alloc_finger_slot(c);
                if (slot < 0) continue;
                mpe_finger *fg = &c->fingers[slot];
                fg->active      = true;
                fg->tracking_id = p->tracking_id;
                fg->is_ui       = true;
                fg->ch          = -1;
                fg->key_idx     = -1;
                /* Apply the action exactly once per touch. */
                mpe_controller_apply_button(c, act);
                seen[slot] = true;
                continue;
            }

            const int kidx = mpe_controller_hit_key(c, p->x, p->y);
            if (kidx < 0) continue;
            const mpe_key *k = &c->kb.keys[kidx];

            slot = alloc_finger_slot(c);
            if (slot < 0) continue;
            int ch = alloc_member_channel(c);
            if (ch < 0) continue;

            mpe_finger *fg = &c->fingers[slot];
            fg->active      = true;
            fg->tracking_id = p->tracking_id;
            fg->is_ui       = false;
            fg->ch          = ch;
            fg->key_idx     = kidx;
            fg->note        = k->midi_note;
            fg->init_x_px   = p->x;
            fg->init_y_px   = p->y;
            fg->press_us    = now;

            /* Strict MPE expression-reset → NoteOn order. Every
               expression dimension is brought to a known state on
               the member channel *before* the NoteOn so the host's
               sound (filter, timbre, amp env) starts from a clean
               slate even if the channel was previously occupied by
               a finger that lifted weirdly. The MMA MPE spec
               recommends this exact order. */
            float xn, yn;
            compute_norms(c, p->x, p->y, &xn, &yn);

            /* (1) Pitch bend → centre. Subsequent glide updates ride
                   from this anchor. */
            mpe_applemidi_pitch_bend((uint8_t)ch, 0x2000);
            fg->last_pb = 0x2000;

            /* (2) CC74 (Y / timbre / "slide") → screen-Y absolute.
                   Most MPE synths map this onto filter cutoff or a
                   dedicated "timbre" macro. */
            uint8_t yv = (uint8_t)(yn * 127.0f);
            mpe_applemidi_cc((uint8_t)ch, 74, yv);
            fg->last_y_cc = yv;

            /* (3) Channel pressure (Z / "press") → 0. Ongoing
                   samples update from here. Some hosts route Z to
                   amp envelope or volume; explicit zero avoids the
                   filter snapping open from a stale value on
                   channel re-use. */
            mpe_applemidi_channel_pressure((uint8_t)ch, 0);
            fg->last_z_cp = 0;

            fg->x_norm = xn;
            fg->y_norm = yn;

            /* (4) NoteOn — strike velocity from contact area (or
                   the Kconfig fixed value). */
            uint8_t vel = (c->cfg.fixed_velocity > 0)
                              ? (uint8_t)c->cfg.fixed_velocity
                              : midi_from_strength(p->strength);
            if (vel == 0) vel = 1;
            mpe_applemidi_note_on((uint8_t)ch, (uint8_t)fg->note, vel);
            fg->z = clampf((float)p->strength / 96.0f, 0.0f, 1.0f);

            ESP_LOGD(TAG, "down tid=%d slot=%d ch=%d key=%d note=%d vel=%d",
                     p->tracking_id, slot, ch, kidx, fg->note, vel);

            seen[slot] = true;
            c->ch_busy_recency[ch] = 1.0f;
            continue;
        }

        /* Ongoing contact. */
        mpe_finger *fg = &c->fingers[slot];
        seen[slot] = true;
        if (fg->is_ui) continue;   /* UI taps don't generate motion CCs */

        float xn = 0, yn = 0;
        compute_norms(c, p->x, p->y, &xn, &yn);
        fg->x_norm = xn;
        fg->y_norm = yn;

        /* GLIDE — pitch bend from snap anchor displacement; deadband
           filters sub-pixel jitter. */
        const int dx_px = p->x - fg->init_x_px;
        uint16_t pb = pb_from_displacement(c, fg->init_x_px, dx_px,
                                           c->kb.keys[fg->key_idx].row);
        const int pb_delta = (int)pb - (int)fg->last_pb;
        if (pb_delta > 12 || pb_delta < -12 || pb == 0x2000) {
            mpe_applemidi_pitch_bend((uint8_t)fg->ch, pb);
            fg->last_pb = pb;
        }

        /* SLIDE — CC74 continuous from screen-Y. */
        uint8_t yv = (uint8_t)clampi((int)(yn * 127.0f), 0, 127);
        if (yv != fg->last_y_cc) {
            mpe_applemidi_cc((uint8_t)fg->ch, 74, yv);
            fg->last_y_cc = yv;
        }

        /* PRESS — channel pressure from ongoing contact size. */
        uint8_t zv = midi_from_strength(p->strength);
        if (zv != fg->last_z_cp) {
            mpe_applemidi_channel_pressure((uint8_t)fg->ch, zv);
            fg->last_z_cp = zv;
        }
        fg->z = clampf((float)p->strength / 96.0f, 0.0f, 1.0f);
        c->ch_busy_recency[fg->ch] = 1.0f;
    }

    /* Release vanished fingers. */
    int active = 0;
    for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
        mpe_finger *fg = &c->fingers[i];
        if (!fg->active) continue;
        if (seen[i]) { active++; continue; }

        /* Stamp the finger's last visible area into the residual
           queue so the renderer keeps restoring it for the next
           ~250 ms across both buffers.
           CRITICAL: this rect must cover BOTH the key (where the
           activity halo lived) AND the finger dot's drawn pixel
           area. The dot is centred on the touch position, which
           can be 30+ px above or below the key boundary (top/
           bottom of the playable surface) — leaving the dot's
           overflow out of the residual was exactly why we saw
           "half circle stuck" after-release ghosts in the screenshot.
           The bounding box of (key_rect) ∪ (touch_pos ± 50) covers
           the worst-case dot drawing for any finger. */
        if (!fg->is_ui &&
            fg->key_idx >= 0 && fg->key_idx < c->kb.n_keys &&
            c->n_residual < MPE_RESIDUAL_CAP) {
            const mpe_key *k = &c->kb.keys[fg->key_idx];
            const int fpx = c->cfg.ui_x +
                            (int)(fg->x_norm * (float)c->cfg.ui_w);
            const int fpy = c->cfg.ui_y +
                            (int)((1.0f - fg->y_norm) * (float)c->cfg.ui_h);
            const int dot_margin = 50;   /* > max dot radius (32) + safety */

            int rx = k->x;
            int ry = k->y;
            int r_right  = k->x + k->w;
            int r_bottom = k->y + k->h;
            if (fpx - dot_margin < rx) rx = fpx - dot_margin;
            if (fpy - dot_margin < ry) ry = fpy - dot_margin;
            if (fpx + dot_margin > r_right)  r_right  = fpx + dot_margin;
            if (fpy + dot_margin > r_bottom) r_bottom = fpy + dot_margin;

            mpe_residual_dirty *r = &c->residual[c->n_residual++];
            r->x = (int16_t)rx;
            r->y = (int16_t)ry;
            r->w = (int16_t)(r_right - rx);
            r->h = (int16_t)(r_bottom - ry);
            r->expire_us = now + 250000;
        }

        if (!fg->is_ui) {
            mpe_applemidi_channel_pressure((uint8_t)fg->ch, 0);
            mpe_applemidi_note_off((uint8_t)fg->ch, (uint8_t)fg->note, 64);
        }
        fg->active      = false;
        fg->tracking_id = -1;
        fg->ch          = -1;
        fg->key_idx     = -1;
        fg->is_ui       = false;
    }

    /* Garbage-collect expired residuals. Swap-and-pop to keep the
       array dense without holes. */
    for (int i = 0; i < c->n_residual; ) {
        if (c->residual[i].expire_us <= now) {
            c->residual[i] = c->residual[c->n_residual - 1];
            c->n_residual--;
        } else {
            i++;
        }
    }

    /* OSC bundle. */
    const int period_us = 1000000 / c->cfg.bundle_rate_hz;
    if (g_osc && (now - c->last_osc_emit_us) >= period_us) {
        c->last_osc_emit_us = now;
        uint8_t buf[512];
        mpe_osc_writer w;
        mpe_osc_writer_init(&w, buf, sizeof buf);
        size_t bundle = mpe_osc_bundle_begin(&w);
        (void)bundle;

        for (int i = 0; i < MPE_CTRL_MAX_FINGERS; i++) {
            const mpe_finger *fg = &c->fingers[i];
            if (!fg->active || fg->is_ui) continue;
            /* /mpe/touch ,iiifffff
                 i slot, i ch (1..16), i note,
                 f x_norm, f y_norm, f pb_norm, f z,
                 f x_cell (white-key index across full screen),
                 f row (0..rows-1) */
            const mpe_key *k = &c->kb.keys[fg->key_idx];
            const float x_cell = (float)fg->x_norm *
                                 (float)(c->cfg.octaves_per_row * 7);
            size_t m = mpe_osc_msg_begin(&w, "/mpe/touch", "iiiffffff");
            mpe_osc_arg_i(&w, i);
            mpe_osc_arg_i(&w, fg->ch + 1);
            mpe_osc_arg_i(&w, fg->note);
            mpe_osc_arg_f(&w, fg->x_norm);
            mpe_osc_arg_f(&w, fg->y_norm);
            mpe_osc_arg_f(&w, ((float)fg->last_pb - 8192.0f) / 8192.0f);
            mpe_osc_arg_f(&w, fg->z);
            mpe_osc_arg_f(&w, x_cell);
            mpe_osc_arg_f(&w, (float)k->row);
            mpe_osc_msg_end(&w, m);
        }
        if (active == 0) {
            size_t m = mpe_osc_msg_begin(&w, "/mpe/clear", "");
            mpe_osc_msg_end(&w, m);
        }
        if (!w.overflow) {
            mpe_osc_client_send(g_osc, w.buf, w.pos);
        }
    }

    return active;
}

void mpe_controller_tick_decay(mpe_controller *c, float dt_s)
{
    const float decay = expf(-dt_s * 3.5f);
    for (int i = 0; i < 16; i++) {
        c->ch_busy_recency[i] *= decay;
        if (c->ch_busy_recency[i] < 0.001f) c->ch_busy_recency[i] = 0.0f;
    }
}
