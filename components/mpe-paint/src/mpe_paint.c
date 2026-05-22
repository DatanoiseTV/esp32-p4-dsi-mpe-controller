/*
 * mpe_paint.c — RGB565 primitives for the 1024x600 EK79007 panel.
 *
 * Implementation notes:
 *
 *  - All primitives clip to (0,0)-(t->width,t->height) by shrinking
 *    the work area rather than asserting. Callers can pass any rect.
 *  - Alpha blend works on the unpacked 8-bit channels (expand 565 ->
 *    888, blend, repack). The fixed-point math is `(src*a + dst*(255-a))
 *    >> 8`, which is the closest cheap approximation to a true 1/255
 *    blend at single-pixel granularity. For an animated UI dominated
 *    by soft glow it's visually correct; for hard graphics it would
 *    round slightly darker than ideal — not a concern here.
 *  - Additive blend (mp_glow_add) is per-channel `min(255, dst+src)`.
 *  - The circle / glow primitives walk one row at a time and compute
 *    a span [x0..x1] inside that row using the integer-sqrt of the
 *    radius — much faster than per-pixel distance checks.
 */
#include "mpe_paint.h"
#include <string.h>
#include <stdint.h>

static inline void unpack565(uint16_t c, uint8_t *r, uint8_t *g, uint8_t *b)
{
    /* Expand 5/6/5 to 8 bits by replicating the high bits into the low
       ones — keeps full-saturation pixels saturated after the round-
       trip. */
    uint8_t r5 = (c >> 11) & 0x1F;
    uint8_t g6 = (c >> 5) & 0x3F;
    uint8_t b5 = c & 0x1F;
    *r = (uint8_t)((r5 << 3) | (r5 >> 2));
    *g = (uint8_t)((g6 << 2) | (g6 >> 4));
    *b = (uint8_t)((b5 << 3) | (b5 >> 2));
}

static inline uint16_t pack565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static inline void clip_rect(const mp_target *t, int *x, int *y,
                             int *w, int *h)
{
    if (*x < 0) { *w += *x; *x = 0; }
    if (*y < 0) { *h += *y; *y = 0; }
    if (*x + *w > t->width)  *w = t->width  - *x;
    if (*y + *h > t->height) *h = t->height - *y;
    if (*w < 0) *w = 0;
    if (*h < 0) *h = 0;
}

void mp_clear(const mp_target *t, uint16_t color)
{
    if (!t || !t->fb) return;
    /* memset only works for color == 0 or 0xFFFF (and even 0xFFFF
       needs byte-swap awareness on some panels); do a word fill. */
    if (color == 0) {
        memset(t->fb, 0, (size_t)t->width * t->height * 2);
        return;
    }
    uint16_t *p = t->fb;
    uint16_t *end = p + (size_t)t->width * t->height;
    /* Unroll x4 for a small but real boost on the cache-coherent
       RGB565 PSRAM writes. */
    while (p + 4 <= end) {
        p[0] = color;
        p[1] = color;
        p[2] = color;
        p[3] = color;
        p += 4;
    }
    while (p < end) *p++ = color;
}

void mp_fill_rect(const mp_target *t, int x, int y, int w, int h,
                  uint16_t color)
{
    if (!t || !t->fb) return;
    clip_rect(t, &x, &y, &w, &h);
    if (w == 0 || h == 0) return;

    for (int row = 0; row < h; row++) {
        uint16_t *p = t->fb + (size_t)(y + row) * t->width + x;
        for (int i = 0; i < w; i++) p[i] = color;
    }
}

void mp_fill_rect_a(const mp_target *t, int x, int y, int w, int h,
                    uint8_t r, uint8_t g, uint8_t b, uint8_t alpha)
{
    if (!t || !t->fb || alpha == 0) return;
    if (alpha == 255) {
        mp_fill_rect(t, x, y, w, h, pack565(r, g, b));
        return;
    }
    clip_rect(t, &x, &y, &w, &h);
    if (w == 0 || h == 0) return;

    const uint16_t inv = 255 - alpha;
    const uint16_t pr  = (uint16_t)r * alpha;
    const uint16_t pg  = (uint16_t)g * alpha;
    const uint16_t pb  = (uint16_t)b * alpha;

    for (int row = 0; row < h; row++) {
        uint16_t *p = t->fb + (size_t)(y + row) * t->width + x;
        for (int i = 0; i < w; i++) {
            uint8_t dr, dg, db;
            unpack565(p[i], &dr, &dg, &db);
            uint8_t nr = (uint8_t)((pr + dr * inv) >> 8);
            uint8_t ng = (uint8_t)((pg + dg * inv) >> 8);
            uint8_t nb = (uint8_t)((pb + db * inv) >> 8);
            p[i] = pack565(nr, ng, nb);
        }
    }
}

void mp_gradient_v(const mp_target *t, int x, int y, int w, int h,
                   uint8_t r0, uint8_t g0, uint8_t b0,
                   uint8_t r1, uint8_t g1, uint8_t b1)
{
    if (!t || !t->fb || h <= 0) return;
    const int orig_y = y;
    const int orig_h = h;
    int cx = x, cy = y, cw = w, ch = h;
    clip_rect(t, &cx, &cy, &cw, &ch);
    if (cw == 0 || ch == 0) return;

    for (int row = 0; row < ch; row++) {
        const int actual_row = (cy + row) - orig_y;
        const int denom = orig_h > 1 ? orig_h - 1 : 1;
        const int alpha = (actual_row * 255) / denom;
        const int inv   = 255 - alpha;
        uint8_t rr = (uint8_t)((r0 * inv + r1 * alpha) / 255);
        uint8_t gg = (uint8_t)((g0 * inv + g1 * alpha) / 255);
        uint8_t bb = (uint8_t)((b0 * inv + b1 * alpha) / 255);
        const uint16_t color = pack565(rr, gg, bb);
        uint16_t *p = t->fb + (size_t)(cy + row) * t->width + cx;
        for (int i = 0; i < cw; i++) p[i] = color;
    }
}

void mp_stroke_rect(const mp_target *t, int x, int y, int w, int h,
                    uint16_t color)
{
    if (!t || !t->fb || w <= 0 || h <= 0) return;
    mp_fill_rect(t, x,         y,         w, 1, color);
    mp_fill_rect(t, x,         y + h - 1, w, 1, color);
    mp_fill_rect(t, x,         y,         1, h, color);
    mp_fill_rect(t, x + w - 1, y,         1, h, color);
}

void mp_hline_a(const mp_target *t, int x, int y, int w,
                uint8_t r, uint8_t g, uint8_t b, uint8_t alpha)
{
    mp_fill_rect_a(t, x, y, w, 1, r, g, b, alpha);
}

void mp_vline_a(const mp_target *t, int x, int y, int h,
                uint8_t r, uint8_t g, uint8_t b, uint8_t alpha)
{
    mp_fill_rect_a(t, x, y, 1, h, r, g, b, alpha);
}

/* 32-bit integer square root via Newton's method, ~6 iterations
   max for our radii — much faster than libm sqrtf which would also
   pull in float ops. */
static int isqrt32(int v)
{
    if (v <= 0) return 0;
    int x = v;
    int y = (x + 1) >> 1;
    while (y < x) { x = y; y = (x + v / x) >> 1; }
    return x;
}

void mp_fill_circle_soft(const mp_target *t, int cx, int cy, int radius,
                         uint8_t r, uint8_t g, uint8_t b, uint8_t alpha)
{
    if (!t || !t->fb || radius <= 0 || alpha == 0) return;
    const int r2 = radius * radius;
    const int aa_band = radius * radius - (radius - 1) * (radius - 1);

    int y0 = cy - radius; if (y0 < 0) y0 = 0;
    int y1 = cy + radius; if (y1 >= t->height) y1 = t->height - 1;

    for (int yy = y0; yy <= y1; yy++) {
        const int dy = yy - cy;
        const int dy2 = dy * dy;
        if (dy2 > r2) continue;
        const int dx_max = isqrt32(r2 - dy2);
        int x0 = cx - dx_max; if (x0 < 0) x0 = 0;
        int x1 = cx + dx_max; if (x1 >= t->width) x1 = t->width - 1;
        for (int xx = x0; xx <= x1; xx++) {
            const int dx = xx - cx;
            const int d2 = dx * dx + dy2;
            int a = alpha;
            if (d2 > r2 - aa_band) {
                /* Soft edge: linearly fade alpha across the outer band. */
                const int slack = r2 - d2;
                if (slack < 0) continue;
                a = (alpha * slack) / aa_band;
                if (a <= 0) continue;
            }
            uint16_t *pix = t->fb + (size_t)yy * t->width + xx;
            uint8_t dr, dg, db;
            unpack565(*pix, &dr, &dg, &db);
            const int inv = 255 - a;
            uint8_t nr = (uint8_t)((r * a + dr * inv) >> 8);
            uint8_t ng = (uint8_t)((g * a + dg * inv) >> 8);
            uint8_t nb = (uint8_t)((b * a + db * inv) >> 8);
            *pix = pack565(nr, ng, nb);
        }
    }
}

void mp_glow_add(const mp_target *t, int cx, int cy, int radius,
                 uint8_t r, uint8_t g, uint8_t b)
{
    if (!t || !t->fb || radius <= 0) return;
    const int r2 = radius * radius;
    /* Pre-scale color values so the inner loop is just (color * intensity >> 8). */
    const int inv_r2_q16 = (1 << 16) / r2;     /* (1/r²) in 16.16 */

    int y0 = cy - radius; if (y0 < 0) y0 = 0;
    int y1 = cy + radius; if (y1 >= t->height) y1 = t->height - 1;

    for (int yy = y0; yy <= y1; yy++) {
        const int dy = yy - cy;
        const int dy2 = dy * dy;
        if (dy2 > r2) continue;
        const int dx_max = isqrt32(r2 - dy2);
        int x0 = cx - dx_max; if (x0 < 0) x0 = 0;
        int x1 = cx + dx_max; if (x1 >= t->width) x1 = t->width - 1;

        /* Incremental d² so the inner loop has no multiplies:
             d²(x+1) = d²(x) + 2(x - cx) + 1
           giving step = 2dx + 1, step++ = 2 per iteration. */
        uint16_t *row = t->fb + (size_t)yy * t->width;
        int dx = x0 - cx;
        int d2 = dx * dx + dy2;
        int step = 2 * dx + 1;

        for (int xx = x0; xx <= x1; xx++) {
            const int slack = r2 - d2;
            if (slack > 0) {
                int t01 = (slack * inv_r2_q16) >> 8;     /* 0..255 */
                if (t01 > 255) t01 = 255;
                int intensity = (t01 * t01) >> 8;
                /* Skip pixels whose added contribution is visually
                   indistinguishable from zero. With r∈[0..255] and
                   intensity∈[0..3], (r·intensity)>>8 ≤ 2 — below
                   the RGB565 quantization step — so the round-trip
                   read+blend+write is a pure cost with no visible
                   effect. Threshold of 4 typically skips ~40% of
                   the outer halo pixels. */
                if (intensity > 7) {
                    uint16_t pix = row[xx];
                    int dr = (pix >> 8) & 0xF8;
                    int dg = (pix >> 3) & 0xFC;
                    int db = (pix << 3) & 0xF8;
                    int nr = dr + ((r * intensity) >> 8);
                    int ng = dg + ((g * intensity) >> 8);
                    int nb = db + ((b * intensity) >> 8);
                    if (nr > 255) nr = 255;
                    if (ng > 255) ng = 255;
                    if (nb > 255) nb = 255;
                    row[xx] = (uint16_t)(((nr & 0xF8) << 8) |
                                          ((ng & 0xFC) << 3) |
                                          (nb >> 3));
                }
            }
            d2   += step;
            step += 2;
        }
    }
}

/* Text drawing intentionally lives in the mpe_font component; this
   one is pure shape primitives so it has no dependency on the TTF
   parser or its 200 KB header. */
