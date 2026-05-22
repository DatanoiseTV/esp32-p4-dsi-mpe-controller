/*
 * mpe_paint — RGB565 framebuffer primitives sized for the EK79007's
 * 1024x600 panel scanned continuously by the ESP32-P4's DSI engine.
 *
 * Single-buffer model: we paint straight into the driver-owned
 * framebuffer; the DSI link DMAs it out at ~60 Hz. The primitives
 * here are written for that pattern — clipping is per-primitive, no
 * dirty-rect bookkeeping, no shadow buffer.
 *
 * Colors are packed RGB565 (5 R, 6 G, 5 B, little-endian) the
 * EK79007 expects. mp_rgb565() builds one from 8-bit components.
 * Alpha is passed as a separate 0..255 value; the implementation
 * expands the RGB565 destination to RGB888-equivalent, blends, and
 * repacks. For glow effects we also expose an additive-blend variant
 * that saturates per-channel.
 *
 * Performance:
 *   - All primitives are word-wise where it matters.
 *   - Filled circle uses an integer span scan; the alpha variant
 *     fades the edge over a one-pixel radius so finger blobs don't
 *     look pixelated.
 *   - mp_fill_radial() draws an additive radial gradient — the
 *     building block for "touch glow" sprites.
 *
 * Coordinates are in pixel space, origin top-left, clipped to the
 * panel bounds inside each primitive (no out-of-bounds writes — the
 * primitives shrink the work area instead of asserting).
 */
#ifndef MPE_PAINT_H_
#define MPE_PAINT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t *fb;
    int       width;
    int       height;
} mp_target;

/* Pack 8-bit RGB into the panel's RGB565. */
static inline uint16_t mp_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/* Solid fills. */
void mp_clear(const mp_target *t, uint16_t color);
void mp_fill_rect(const mp_target *t,
                  int x, int y, int w, int h, uint16_t color);

/* Alpha-blended rect. alpha in 0..255; 0 is no-op, 255 == mp_fill_rect. */
void mp_fill_rect_a(const mp_target *t,
                    int x, int y, int w, int h,
                    uint8_t r, uint8_t g, uint8_t b, uint8_t alpha);

/* Vertical gradient — interpolates linearly from `top` at y to
   `bottom` at y+h-1. The endpoints are RGB888 components. */
void mp_gradient_v(const mp_target *t, int x, int y, int w, int h,
                   uint8_t r0, uint8_t g0, uint8_t b0,
                   uint8_t r1, uint8_t g1, uint8_t b1);

/* 1-pixel rectangle outline. */
void mp_stroke_rect(const mp_target *t, int x, int y, int w, int h,
                    uint16_t color);

/* Filled circle with soft (1-px AA) edge, alpha blended. */
void mp_fill_circle_soft(const mp_target *t, int cx, int cy, int radius,
                         uint8_t r, uint8_t g, uint8_t b, uint8_t alpha);

/* Additive radial gradient: at the center we add (r,g,b) at full
   intensity, falling off to zero at `radius`. Channels saturate at
   255 — this is the right model for stacked finger glows. The
   falloff is quadratic which looks closer to a real light source
   than linear. */
void mp_glow_add(const mp_target *t, int cx, int cy, int radius,
                 uint8_t r, uint8_t g, uint8_t b);

/* Horizontal line, alpha blended. */
void mp_hline_a(const mp_target *t, int x, int y, int w,
                uint8_t r, uint8_t g, uint8_t b, uint8_t alpha);

/* Vertical line, alpha blended. */
void mp_vline_a(const mp_target *t, int x, int y, int h,
                uint8_t r, uint8_t g, uint8_t b, uint8_t alpha);

/* Text rendering lives in mpe_font (TTF via stb_truetype) — pulled
   in by the caller, not by this component. */

#ifdef __cplusplus
}
#endif

#endif
