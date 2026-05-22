/*
 * mpe_font — TrueType rendering via stb_truetype, alpha-blended into
 * the RGB565 framebuffer.
 *
 * Same engine the sibling esp32-p4-nano-browser uses; that one runs at
 * full body-text scale across hundreds of glyphs per repaint and is
 * the fastest known-working text path on this hardware (no Slint
 * dependency, no FreeType).
 *
 * The instrument UI needs much less than a browser does — half a
 * dozen labels at large sizes for the title + status bar, plus
 * ~80 cell labels (note names like "C4", "F#3") at a smaller size.
 * The built-in LRU glyph cache is sized for that case.
 *
 * Init is one-shot from the embedded TTF blob exposed by EMBED_FILES
 * in main/CMakeLists.txt.
 */
#ifndef MPE_FONT_H_
#define MPE_FONT_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mpe_font_init(const uint8_t *ttf_data, size_t ttf_len);
bool      mpe_font_ready(void);

/* Pixel ascent (baseline → top) at the given size. */
int       mpe_font_ascent_px(int size_px);

/* Pixel line height at the given size. */
int       mpe_font_line_height_px(int size_px);

/* Pixel advance for a single codepoint. */
int       mpe_font_advance_px(uint32_t codepoint, int size_px);

/* Pixel width of an entire UTF-8 string at the given size. */
int       mpe_font_text_width_px(const char *utf8, int byte_len,
                                 int size_px);

/* Render one codepoint into the framebuffer. (x, y) is the
   TOP-LEFT of the glyph's nominal cell — internally the renderer
   adds ascent + glyph y_off. Color is alpha-blended against the
   existing pixel so anti-aliasing reads correctly over varied
   backgrounds (e.g. soft glow circles). */
void      mpe_font_draw_glyph(uint16_t *fb, int fb_w, int fb_h,
                              int x, int y, uint32_t codepoint,
                              int size_px, uint16_t fg_rgb565);

/* Render a UTF-8 string. Returns the right edge of the last glyph,
   so the caller can chain segments. byte_len == -1 means strlen. */
int       mpe_font_draw_text(uint16_t *fb, int fb_w, int fb_h,
                             int x, int y,
                             const char *utf8, int byte_len,
                             int size_px, uint16_t fg_rgb565);

/* Convenience: center-aligned draw. (cx, y) is the desired centerline
   x at y. The renderer measures the string first, then offsets. */
int       mpe_font_draw_text_centered(uint16_t *fb, int fb_w, int fb_h,
                                      int cx, int y,
                                      const char *utf8, int size_px,
                                      uint16_t fg_rgb565);

#ifdef __cplusplus
}
#endif

#endif
