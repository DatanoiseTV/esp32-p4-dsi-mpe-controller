/*
 * mpe_font.c — stb_truetype-backed renderer with LRU glyph cache.
 *
 * Lifted from the proven esp32-p4-nano-browser pipeline. The cache
 * sits in PSRAM; each glyph is rasterised once at a (codepoint,
 * size_px) pair and cached as an 8-bit alpha bitmap. Hits are pure
 * memory copies + RGB565 blend.
 */
#include "mpe_font.h"

#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_heap_caps.h"

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#define STBTT_malloc(x, u)  ((void)(u), heap_caps_malloc((x), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT))
#define STBTT_free(x, u)    ((void)(u), free(x))
#define STBTT_assert(x)     do { (void)(x); } while (0)
#include "stb_truetype.h"

static const char *TAG = "font";

static stbtt_fontinfo s_font;
static bool           s_ready    = false;
static int            s_ascent_raw   = 0;
static int            s_descent_raw  = 0;
static int            s_linegap_raw  = 0;

#define FONT_CACHE_CAP 512  /* instrument UI has ~100 distinct glyphs */

typedef struct {
    uint32_t key;        /* (codepoint << 8) | size_px; 0 = empty */
    uint64_t last_used;
    int      w, h;
    int      x_off, y_off;
    int      advance;
    uint8_t *bm;
} glyph_entry_t;

static glyph_entry_t s_cache[FONT_CACHE_CAP];
static uint64_t      s_cache_clock = 0;

static inline uint32_t glyph_key_(uint32_t cp, int size_px)
{
    return (cp << 8) | (uint32_t)(size_px & 0xFF);
}

static glyph_entry_t *cache_lookup_or_alloc_(uint32_t cp, int size_px)
{
    uint32_t key = glyph_key_(cp, size_px);
    uint32_t idx = (key * 2654435761u) % FONT_CACHE_CAP;
    glyph_entry_t *victim = NULL;
    uint64_t victim_age = UINT64_MAX;
    for (uint32_t probe = 0; probe < 64; probe++) {
        glyph_entry_t *e = &s_cache[(idx + probe) % FONT_CACHE_CAP];
        if (e->key == key) {
            e->last_used = ++s_cache_clock;
            return e;
        }
        if (e->key == 0) {
            e->last_used = ++s_cache_clock;
            return e;
        }
        if (e->last_used < victim_age) {
            victim_age = e->last_used;
            victim     = e;
        }
    }
    if (victim) {
        if (victim->bm) free(victim->bm);
        memset(victim, 0, sizeof *victim);
        victim->last_used = ++s_cache_clock;
    }
    return victim;
}

static glyph_entry_t *get_glyph_(uint32_t cp, int size_px)
{
    if (!s_ready || size_px <= 0) return NULL;
    glyph_entry_t *e = cache_lookup_or_alloc_(cp, size_px);
    if (!e) return NULL;
    if (e->key == glyph_key_(cp, size_px) && (e->bm || (e->w == 0 && e->h == 0))) {
        return e;
    }

    float scale = stbtt_ScaleForPixelHeight(&s_font, (float)size_px);
    int glyph = stbtt_FindGlyphIndex(&s_font, (int)cp);

    int adv_raw = 0, lsb = 0;
    stbtt_GetCodepointHMetrics(&s_font, (int)cp, &adv_raw, &lsb);
    e->advance = (int)(adv_raw * scale + 0.5f);

    if (glyph == 0) {
        e->key      = glyph_key_(cp, size_px);
        e->w = e->h = 0;
        e->bm       = NULL;
        return e;
    }

    int x0, y0, x1, y1;
    stbtt_GetGlyphBitmapBox(&s_font, glyph, scale, scale, &x0, &y0, &x1, &y1);
    int gw = x1 - x0;
    int gh = y1 - y0;

    e->key     = glyph_key_(cp, size_px);
    e->w       = gw;
    e->h       = gh;
    e->x_off   = x0;
    e->y_off   = y0;

    if (gw > 0 && gh > 0) {
        e->bm = heap_caps_malloc((size_t)gw * (size_t)gh,
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (e->bm) {
            stbtt_MakeGlyphBitmap(&s_font, e->bm, gw, gh, gw,
                                  scale, scale, glyph);
        }
    }
    return e;
}

esp_err_t mpe_font_init(const uint8_t *ttf_data, size_t ttf_len)
{
    if (!ttf_data || ttf_len == 0) {
        ESP_LOGE(TAG, "no TTF supplied");
        return ESP_ERR_INVALID_ARG;
    }
    int offset = stbtt_GetFontOffsetForIndex(ttf_data, 0);
    if (offset < 0) {
        ESP_LOGE(TAG, "stbtt_GetFontOffsetForIndex failed");
        return ESP_FAIL;
    }
    if (!stbtt_InitFont(&s_font, ttf_data, offset)) {
        ESP_LOGE(TAG, "stbtt_InitFont failed");
        return ESP_FAIL;
    }
    stbtt_GetFontVMetrics(&s_font, &s_ascent_raw, &s_descent_raw, &s_linegap_raw);
    s_ready = true;
    ESP_LOGI(TAG, "TTF up: %zu bytes, ascent=%d descent=%d linegap=%d",
             ttf_len, s_ascent_raw, s_descent_raw, s_linegap_raw);
    return ESP_OK;
}

bool mpe_font_ready(void) { return s_ready; }

int mpe_font_line_height_px(int size_px)
{
    if (!s_ready || size_px <= 0) return size_px > 0 ? size_px : 12;
    float scale = stbtt_ScaleForPixelHeight(&s_font, (float)size_px);
    return (int)((s_ascent_raw - s_descent_raw + s_linegap_raw) * scale + 0.5f);
}

int mpe_font_ascent_px(int size_px)
{
    if (!s_ready || size_px <= 0) return size_px;
    float scale = stbtt_ScaleForPixelHeight(&s_font, (float)size_px);
    return (int)(s_ascent_raw * scale + 0.5f);
}

int mpe_font_advance_px(uint32_t cp, int size_px)
{
    if (!s_ready || size_px <= 0) {
        return size_px > 0 ? (size_px * 6 / 10) : 6;
    }
    glyph_entry_t *g = get_glyph_(cp, size_px);
    return g ? g->advance : (size_px * 6 / 10);
}

/* Minimal UTF-8 decoder; advances p. Returns codepoint or 0xFFFD. */
static uint32_t utf8_next_(const char **p, const char *end)
{
    if (*p >= end) return 0;
    unsigned char c = (unsigned char)**p;
    if (c < 0x80) { (*p)++; return c; }
    int extra;
    uint32_t cp;
    if      ((c & 0xE0) == 0xC0) { extra = 1; cp = c & 0x1F; }
    else if ((c & 0xF0) == 0xE0) { extra = 2; cp = c & 0x0F; }
    else if ((c & 0xF8) == 0xF0) { extra = 3; cp = c & 0x07; }
    else                         { (*p)++;    return 0xFFFD; }
    (*p)++;
    for (int i = 0; i < extra; i++) {
        if (*p >= end) return 0xFFFD;
        unsigned char n = (unsigned char)**p;
        if ((n & 0xC0) != 0x80) return 0xFFFD;
        cp = (cp << 6) | (n & 0x3F);
        (*p)++;
    }
    return cp;
}

int mpe_font_text_width_px(const char *utf8, int byte_len, int size_px)
{
    if (!utf8) return 0;
    int len = (byte_len < 0) ? (int)strlen(utf8) : byte_len;
    const char *p   = utf8;
    const char *end = utf8 + len;
    int w = 0;
    while (p < end) {
        uint32_t cp = utf8_next_(&p, end);
        if (cp == 0) break;
        w += mpe_font_advance_px(cp, size_px);
    }
    return w;
}

static inline uint16_t blend_(uint16_t bg, uint16_t fg, uint8_t a)
{
    if (a == 0)   return bg;
    if (a == 255) return fg;
    uint8_t br = (bg >> 11) & 0x1F, bgn = (bg >> 5) & 0x3F, bb = bg & 0x1F;
    uint8_t fr = (fg >> 11) & 0x1F, fgn = (fg >> 5) & 0x3F, fb = fg & 0x1F;
    uint16_t r = (uint16_t)((fr * a + br * (255 - a)) / 255);
    uint16_t g = (uint16_t)((fgn * a + bgn * (255 - a)) / 255);
    uint16_t b = (uint16_t)((fb * a + bb * (255 - a)) / 255);
    return (uint16_t)((r << 11) | (g << 5) | b);
}

void mpe_font_draw_glyph(uint16_t *fb, int fb_w, int fb_h,
                         int x, int y, uint32_t cp,
                         int size_px, uint16_t fg)
{
    if (!s_ready || !fb || size_px <= 0) return;
    glyph_entry_t *g = get_glyph_(cp, size_px);
    if (!g || !g->bm || g->w <= 0 || g->h <= 0) return;

    const int dst_x = x + g->x_off;
    const int dst_y = y + mpe_font_ascent_px(size_px) + g->y_off;
    for (int gy = 0; gy < g->h; gy++) {
        int py = dst_y + gy;
        if (py < 0 || py >= fb_h) continue;
        uint16_t *line = fb + (size_t)py * (size_t)fb_w;
        const uint8_t *src = g->bm + (size_t)gy * (size_t)g->w;
        for (int gx = 0; gx < g->w; gx++) {
            int px = dst_x + gx;
            if (px < 0 || px >= fb_w) continue;
            uint8_t a = src[gx];
            if (a) line[px] = blend_(line[px], fg, a);
        }
    }
}

int mpe_font_draw_text(uint16_t *fb, int fb_w, int fb_h,
                       int x, int y,
                       const char *utf8, int byte_len,
                       int size_px, uint16_t fg)
{
    if (!utf8 || !s_ready) return x;
    int len = (byte_len < 0) ? (int)strlen(utf8) : byte_len;
    const char *p   = utf8;
    const char *end = utf8 + len;
    int cur_x = x;
    while (p < end) {
        uint32_t cp = utf8_next_(&p, end);
        if (cp == 0) break;
        glyph_entry_t *g = get_glyph_(cp, size_px);
        if (!g) break;
        if (g->bm && g->w > 0 && g->h > 0 && cur_x < fb_w) {
            const int dst_x = cur_x + g->x_off;
            const int dst_y = y + mpe_font_ascent_px(size_px) + g->y_off;
            for (int gy = 0; gy < g->h; gy++) {
                int py = dst_y + gy;
                if (py < 0 || py >= fb_h) continue;
                uint16_t *line = fb + (size_t)py * (size_t)fb_w;
                const uint8_t *src = g->bm + (size_t)gy * (size_t)g->w;
                for (int gx = 0; gx < g->w; gx++) {
                    int px = dst_x + gx;
                    if (px < 0 || px >= fb_w) continue;
                    uint8_t a = src[gx];
                    if (a) line[px] = blend_(line[px], fg, a);
                }
            }
        }
        cur_x += g->advance;
        if (cur_x >= fb_w) break;
    }
    return cur_x;
}

int mpe_font_draw_text_centered(uint16_t *fb, int fb_w, int fb_h,
                                int cx, int y,
                                const char *utf8, int size_px,
                                uint16_t fg)
{
    int w = mpe_font_text_width_px(utf8, -1, size_px);
    return mpe_font_draw_text(fb, fb_w, fb_h, cx - w / 2, y,
                              utf8, -1, size_px, fg);
}
