/*
 * mpe_screenshot.c — /screenshot.bmp endpoint via esp_http_server.
 *
 * The BMP wire format we emit is BITMAPV4HEADER + raw RGB565 with
 * BI_BITFIELDS. Total header = 14 (BMP file header) + 108
 * (BITMAPV4) = 122 bytes. Negative height = top-down scan order,
 * which matches our framebuffer layout (row 0 = top of screen).
 *
 * Channel masks for RGB565 (host-byte-order uint16_t):
 *   R = 0xF800, G = 0x07E0, B = 0x001F
 *
 * Cache: the front buffer was last CPU-written when it was the
 * back buffer (one or more frames ago). Subsequent PPA writes /
 * partial restores went through DMA, bypassing the CPU cache, so
 * our cache lines for this region may be stale. Invalidate before
 * reading.
 */
#include "mpe_screenshot.h"

#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_cache.h"

#include "mpe_display.h"

static const char *TAG = "shot";
static httpd_handle_t s_http = NULL;

#define BMP_HEADER_SIZE 122
#define V4_HEADER_SIZE  108
#define FB_BYTES        ((size_t)MPE_DISPLAY_WIDTH * MPE_DISPLAY_HEIGHT * 2)

static void write_le_u16(uint8_t *p, uint16_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void write_le_u32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static void write_le_i32(uint8_t *p, int32_t v)
{ write_le_u32(p, (uint32_t)v); }

static void build_bmp_header(uint8_t *hdr, int w, int h)
{
    memset(hdr, 0, BMP_HEADER_SIZE);

    /* BMP file header (14 B). */
    hdr[0] = 'B'; hdr[1] = 'M';
    write_le_u32(hdr + 2,  (uint32_t)(BMP_HEADER_SIZE + FB_BYTES));
    /* reserved = 0 */
    write_le_u32(hdr + 10, BMP_HEADER_SIZE);    /* pixel data offset */

    /* BITMAPV4HEADER (108 B, starts at offset 14). */
    write_le_u32(hdr + 14, V4_HEADER_SIZE);     /* header size */
    write_le_i32(hdr + 18, (int32_t)w);
    write_le_i32(hdr + 22, -(int32_t)h);        /* negative = top-down */
    write_le_u16(hdr + 26, 1);                  /* planes */
    write_le_u16(hdr + 28, 16);                 /* bpp */
    write_le_u32(hdr + 30, 3);                  /* BI_BITFIELDS */
    write_le_u32(hdr + 34, (uint32_t)FB_BYTES); /* pixel data size */
    /* X/Y pixels per metre, palette colors, important colors: leave 0 */

    /* Channel masks for RGB565 little-endian uint16_t. */
    write_le_u32(hdr + 54, 0xF800);             /* R */
    write_le_u32(hdr + 58, 0x07E0);             /* G */
    write_le_u32(hdr + 62, 0x001F);             /* B */
    write_le_u32(hdr + 66, 0x0000);             /* A */
    /* CSType = "Win " (LE-stored ASCII 'W','i','n',' '). */
    hdr[70] = 0x20; hdr[71] = 0x6E; hdr[72] = 0x69; hdr[73] = 0x57;
}

static esp_err_t screenshot_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"screenshot.bmp\"");
    /* No caching — every fetch is the live framebuffer. */
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    uint8_t hdr[BMP_HEADER_SIZE];
    build_bmp_header(hdr, MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT);
    esp_err_t err = httpd_resp_send_chunk(req, (const char *)hdr,
                                          BMP_HEADER_SIZE);
    if (err != ESP_OK) return err;

    /* Read whichever buffer the panel is currently scanning out.
       Invalidate cache first so we see pixels actually in PSRAM (the
       front buffer was last CPU-written one or more frames ago and
       DMA writes since have bypassed cache). */
    uint16_t *fb = mpe_display_front_buffer();
    if (!fb) {
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_FAIL;
    }
    /* FB pointer is at the start of a panel-driver allocation
       (64-byte aligned) and FB_BYTES = 1228800 = multiple of 64,
       so no UNALIGNED flag needed (IDF rejects it for M2C anyway). */
    esp_cache_msync(fb, FB_BYTES, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

    /* Chunked send so we don't need a full FB_BYTES buffer in RAM.
       64 KiB chunks: comfortably under any LWIP send queue limit, and
       still few enough chunks (~19 for a 1024×600 RGB565 FB) to make
       the per-chunk TCP overhead negligible. */
    const size_t CHUNK = 64 * 1024;
    size_t left = FB_BYTES;
    const uint8_t *p = (const uint8_t *)fb;
    while (left > 0) {
        size_t n = left > CHUNK ? CHUNK : left;
        esp_err_t e = httpd_resp_send_chunk(req, (const char *)p, n);
        if (e != ESP_OK) {
            httpd_resp_send_chunk(req, NULL, 0);
            return e;
        }
        p    += n;
        left -= n;
    }
    httpd_resp_send_chunk(req, NULL, 0);   /* end chunked response */
    ESP_LOGI(TAG, "screenshot served (%u bytes)",
             (unsigned)(BMP_HEADER_SIZE + FB_BYTES));
    return ESP_OK;
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    static const char html[] =
        "<!doctype html><html><head><meta charset=utf-8>"
        "<title>MPE controller</title>"
        "<style>body{font-family:system-ui;background:#0a0c14;color:#e8eaf0;"
        "padding:24px;}img{max-width:100%;border:1px solid #223;"
        "border-radius:8px;display:block;margin:16px 0;}a{color:#88aaff;}"
        "</style></head><body>"
        "<h1>esp32-p4-dsi-mpe-controller</h1>"
        "<p>Live framebuffer:</p>"
        "<img src=\"/screenshot.bmp\" alt=\"screenshot\">"
        "<p><a href=\"/screenshot.bmp\">Direct download (.bmp)</a></p>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, sizeof html - 1);
}

esp_err_t mpe_screenshot_start(void)
{
    if (s_http) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.lru_purge_enable = true;
    cfg.stack_size       = 6144;     /* httpd worker stack */
    cfg.max_uri_handlers = 4;
    cfg.core_id          = 1;        /* serve HTTP off the render CPU
                                        so a screenshot fetch can't
                                        steal frame budget */

    esp_err_t err = httpd_start(&s_http, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t shot = {
        .uri      = "/screenshot.bmp",
        .method   = HTTP_GET,
        .handler  = screenshot_get_handler,
    };
    httpd_uri_t idx = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = index_get_handler,
    };
    httpd_register_uri_handler(s_http, &shot);
    httpd_register_uri_handler(s_http, &idx);

    ESP_LOGI(TAG, "screenshot server listening on :80");
    return ESP_OK;
}
