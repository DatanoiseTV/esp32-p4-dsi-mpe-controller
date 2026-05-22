/*
 * mpe_display.c — DSI panel bring-up + double-buffered presentation.
 *
 * Init sequence (same proven path as the sibling -browser/-espdisp):
 *   1. esp_ldo_acquire_channel  — power the DSI PHY (LDO ch 3, 2.5 V).
 *      *MANDATORY* on ESP32-P4. Without it the PHY stays off and the
 *      backlight cuts out alongside.
 *   2. backlight LEDC at 0 %    — clean fade-in.
 *   3. DSI bus + DBI IO         — vendor-default lane bit rate.
 *   4. EK79007 vendor config    — driver macro provides h/vsync porches.
 *   5. panel reset + init       — vendor init sequence.
 *   6. esp_lcd_dpi_panel_get_frame_buffer(panel, 2, ...) — claims
 *      pointers to BOTH driver-owned framebuffers (num_fbs=2).
 *   7. esp_lcd_dpi_panel_register_event_callbacks — installs the
 *      on_refresh_done callback for vsync semaphore signalling.
 *   8. I2C bus for the GT911 touch controller.
 *   9. backlight to configured default.
 *
 * Presentation: esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, back_fb)
 * tells the driver to switch the scanout to back_fb at the next
 * vsync. We then wait on the refresh-done semaphore so the caller
 * knows the swap has happened (and the previous back-buffer — now
 * "front" — is free to re-paint).
 */
#include "mpe_display.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ek79007.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/ppa.h"
#include "esp_cache.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "display";

#define DPHY_LDO_CHAN        3
#define DPHY_LDO_VOLTAGE_MV  2500
#define DSI_LANES            2
#define LCD_NUM_FBS          2

static esp_ldo_channel_handle_t  s_phy_pwr   = NULL;
static esp_lcd_dsi_bus_handle_t  s_bus       = NULL;
static esp_lcd_panel_io_handle_t s_io        = NULL;
static esp_lcd_panel_handle_t    s_panel     = NULL;
static uint16_t                 *s_fbs[LCD_NUM_FBS] = { NULL, NULL };
static int                       s_back_idx  = 0;
static SemaphoreHandle_t         s_vsync_sem = NULL;
static bool                      s_bl_inited = false;
static i2c_master_bus_handle_t   s_i2c_bus   = NULL;
static ppa_client_handle_t       s_ppa_srm   = NULL;

#define BL_LEDC_TIMER    LEDC_TIMER_0
#define BL_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define BL_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define BL_LEDC_FREQ_HZ  5000
#define BL_LEDC_DUTY_MAX ((1U << 10) - 1U)

static esp_err_t dphy_power_on_(void)
{
    esp_ldo_channel_config_t cfg = {};
    cfg.chan_id    = DPHY_LDO_CHAN;
    cfg.voltage_mv = DPHY_LDO_VOLTAGE_MV;
    return esp_ldo_acquire_channel(&cfg, &s_phy_pwr);
}

static esp_err_t backlight_init_(void)
{
    ledc_timer_config_t t = {};
    t.speed_mode      = BL_LEDC_MODE;
    t.duty_resolution = BL_LEDC_DUTY_RES;
    t.timer_num       = BL_LEDC_TIMER;
    t.freq_hz         = BL_LEDC_FREQ_HZ;
    t.clk_cfg         = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&t), TAG, "ledc timer");
    ledc_channel_config_t c = {};
    c.gpio_num   = CONFIG_DISP_PIN_LCD_BL;
    c.speed_mode = BL_LEDC_MODE;
    c.channel    = BL_LEDC_CHANNEL;
    c.timer_sel  = BL_LEDC_TIMER;
    c.duty       = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&c), TAG, "ledc channel");
    s_bl_inited = true;
    return ESP_OK;
}

esp_err_t mpe_display_set_backlight(int pct)
{
    if (!s_bl_inited) return ESP_ERR_INVALID_STATE;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    uint32_t duty = (BL_LEDC_DUTY_MAX * (uint32_t)pct) / 100;
    ESP_RETURN_ON_ERROR(ledc_set_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL, duty),
                        TAG, "duty");
    return ledc_update_duty(BL_LEDC_MODE, BL_LEDC_CHANNEL);
}

static esp_err_t i2c_bus_init_(void)
{
    if (s_i2c_bus) return ESP_OK;
    i2c_master_bus_config_t cfg = {
        .i2c_port          = -1,
        .sda_io_num        = CONFIG_DISP_PIN_TOUCH_SDA,
        .scl_io_num        = CONFIG_DISP_PIN_TOUCH_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags             = { .enable_internal_pullup = 1 },
    };
    return i2c_new_master_bus(&cfg, &s_i2c_bus);
}

void *mpe_display_i2c_bus(void)
{
    if (!s_i2c_bus) i2c_bus_init_();
    return (void *)s_i2c_bus;
}

/* Fires when a draw_bitmap-queued buffer swap actually finishes
   scanning out. In IDF v6.0 this is the dependable per-present
   notification on the EK79007 / DPI path; on_refresh_done (which
   was meant to fire every refresh, not just on swap) doesn't
   actually fire on this stack — apps that waited on it deadlocked
   on the very first present. */
static bool IRAM_ATTR swap_done_cb_(esp_lcd_panel_handle_t panel,
                                    esp_lcd_dpi_panel_event_data_t *edata,
                                    void *user_ctx)
{
    (void)panel; (void)edata; (void)user_ctx;
    BaseType_t hp = pdFALSE;
    if (s_vsync_sem) xSemaphoreGiveFromISR(s_vsync_sem, &hp);
    return hp == pdTRUE;
}

esp_err_t mpe_display_init(void)
{
    if (s_panel) return ESP_OK;

    ESP_RETURN_ON_ERROR(dphy_power_on_(), TAG, "dphy LDO");

    ESP_RETURN_ON_ERROR(backlight_init_(), TAG, "bl init");
    ESP_RETURN_ON_ERROR(mpe_display_set_backlight(0), TAG, "bl off");

    esp_lcd_dsi_bus_config_t bus_cfg = {};
    bus_cfg.bus_id             = 0;
    bus_cfg.num_data_lanes     = DSI_LANES;
    bus_cfg.phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = CONFIG_DISP_DSI_LANE_BITRATE_MBPS;
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_cfg, &s_bus),
                        TAG, "new_dsi_bus");

    esp_lcd_dbi_io_config_t dbi_cfg = {};
    dbi_cfg.virtual_channel = 0;
    dbi_cfg.lcd_cmd_bits    = 8;
    dbi_cfg.lcd_param_bits  = 8;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(s_bus, &dbi_cfg, &s_io),
                        TAG, "new_panel_io_dbi");

    esp_lcd_dpi_panel_config_t dpi_cfg =
        EK79007_1024_600_PANEL_60HZ_CONFIG_CF(LCD_COLOR_FMT_RGB565);
    dpi_cfg.num_fbs            = LCD_NUM_FBS;
    dpi_cfg.dpi_clock_freq_mhz = CONFIG_DISP_DPI_CLOCK_FREQ_MHZ;

    ek79007_vendor_config_t vendor = {};
    vendor.mipi_config.dsi_bus    = s_bus;
    vendor.mipi_config.dpi_config = &dpi_cfg;
    vendor.mipi_config.lane_num   = DSI_LANES;

    esp_lcd_panel_dev_config_t dev = {};
    dev.reset_gpio_num = CONFIG_DISP_PIN_LCD_RST;
    dev.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB;
    dev.bits_per_pixel = 16;
    dev.vendor_config  = &vendor;

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ek79007(s_io, &dev, &s_panel),
                        TAG, "new_panel_ek79007");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel),  TAG, "init");

    void *fb0 = NULL, *fb1 = NULL;
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(
                            s_panel, LCD_NUM_FBS, &fb0, &fb1),
                        TAG, "get_frame_buffer");
    s_fbs[0] = (uint16_t *)fb0;
    s_fbs[1] = (uint16_t *)fb1;
    if (!s_fbs[0] || !s_fbs[1]) {
        ESP_LOGE(TAG, "panel returned NULL framebuffer(s)");
        return ESP_ERR_INVALID_STATE;
    }

    const size_t fb_bytes =
        (size_t)MPE_DISPLAY_WIDTH * MPE_DISPLAY_HEIGHT * 2;
    memset(s_fbs[0], 0, fb_bytes);
    memset(s_fbs[1], 0, fb_bytes);

    /* The CPU just wrote both framebuffers via cache; the DSI engine
       reads them through DMA. Flush so memory matches what we wrote. */
    esp_cache_msync(s_fbs[0], fb_bytes,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    esp_cache_msync(s_fbs[1], fb_bytes,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    s_back_idx = 0;

    s_vsync_sem = xSemaphoreCreateBinary();
    if (!s_vsync_sem) return ESP_ERR_NO_MEM;

    esp_lcd_dpi_panel_event_callbacks_t cbs = {};
    cbs.on_color_trans_done = swap_done_cb_;
    ESP_RETURN_ON_ERROR(
        esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL),
        TAG, "register_event_callbacks");

    /* Kick off the scanout. WITHOUT this initial draw_bitmap the
       panel never starts refreshing and the on_color_trans_done /
       on_refresh_done callbacks never fire — which is what pinned
       us at 10 FPS for several rounds of debugging. The sibling
       espdisp project does this exact same call after registering
       its vsync ISR; matches the IDF dpi-panel reference pattern. */
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0,
                                  MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT,
                                  s_fbs[0]),
        TAG, "initial draw_bitmap");
    s_back_idx = 1;   /* fb[0] is now front; we paint fb[1] next. */

    /* Register a PPA SRM client so the app can DMA per-rect template
       restores instead of doing them on the CPU. Failure here is
       non-fatal — the app falls back to memcpy. */
    {
        ppa_client_config_t ppa_cfg = {};
        ppa_cfg.oper_type             = PPA_OPERATION_SRM;
        ppa_cfg.max_pending_trans_num = 2;
        if (ppa_register_client(&ppa_cfg, &s_ppa_srm) != ESP_OK) {
            ESP_LOGW(TAG, "PPA SRM register failed — CPU memcpy fallback");
            s_ppa_srm = NULL;
        } else {
            ESP_LOGI(TAG, "PPA SRM client registered");
        }
    }

    if (i2c_bus_init_() != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init failed; touch will be unavailable");
    }

    ESP_RETURN_ON_ERROR(
        mpe_display_set_backlight(CONFIG_DISP_BL_DEFAULT_PCT),
        TAG, "bl on");

    ESP_LOGI(TAG, "LCD up: %dx%d RGB565, %d fbs (back=%p, front=%p)",
             MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT, LCD_NUM_FBS,
             (void *)s_fbs[0], (void *)s_fbs[1]);
    return ESP_OK;
}

uint16_t *mpe_display_back_buffer(void)
{
    return s_fbs[s_back_idx];
}

uint16_t *mpe_display_front_buffer(void)
{
    return s_fbs[s_back_idx ^ 1];
}

void mpe_display_rect_copy(uint16_t *dst, const uint16_t *src,
                           int x, int y, int w, int h)
{
    if (!dst || !src || w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > MPE_DISPLAY_WIDTH)  w = MPE_DISPLAY_WIDTH  - x;
    if (y + h > MPE_DISPLAY_HEIGHT) h = MPE_DISPLAY_HEIGHT - y;
    if (w <= 0 || h <= 0) return;

    /* CPU memcpy path. We default to this for everything below the
       full-screen threshold because PPA DMA bypasses the CPU cache:
       after PPA writes the rect, the CPU still holds stale cache
       lines for that region. The follow-up M2C invalidate is meant
       to drop those lines, but it isn't reliably effective on this
       stack — the visible symptom is "stuck halos" on keys, where
       the next frame's alpha-blend reads the cached old halo,
       blends new content against it, and writes back, leaving a
       ghost the dirty-rect restore can't clear.
       memcpy goes through the cache so subsequent CPU reads /
       writes are always coherent. The PPA path is kept only for
       the (rare) full-screen rebake, where setup overhead is
       negligible against ~1.2 MB of data. */
    if (s_ppa_srm == NULL ||
        (size_t)w * (size_t)h <
            (size_t)MPE_DISPLAY_WIDTH * MPE_DISPLAY_HEIGHT / 2) {
        const int stride = MPE_DISPLAY_WIDTH;
        for (int ry = 0; ry < h; ry++) {
            memcpy(dst + (size_t)(y + ry) * stride + x,
                   src + (size_t)(y + ry) * stride + x,
                   (size_t)w * 2);
        }
        return;
    }

    /* SRM at 1:1 = pure DMA copy of a sub-rectangle. Cache contract:
         - The CALLER is responsible for flushing writes to `src`
           before it goes "live" (we expose mpe_display_flush_writes
           for one-shot post-bake). The src here is a static template
           that doesn't change frame-to-frame, so we don't pay the
           full-FB flush per copy.
         - AFTER the PPA writes `dst`, we invalidate the dst rect's
           cache lines so subsequent CPU reads (in alpha-blend
           overlays) see PPA's fresh data, not stale lines from when
           this buffer was painted two frames ago. */
    const size_t fb_bytes = (size_t)MPE_DISPLAY_WIDTH *
                             MPE_DISPLAY_HEIGHT * 2;

    ppa_srm_oper_config_t op = {};
    op.in.buffer            = (void *)src;
    op.in.pic_w             = MPE_DISPLAY_WIDTH;
    op.in.pic_h             = MPE_DISPLAY_HEIGHT;
    op.in.block_w           = w;
    op.in.block_h           = h;
    op.in.block_offset_x    = x;
    op.in.block_offset_y    = y;
    op.in.srm_cm            = PPA_SRM_COLOR_MODE_RGB565;
    op.out.buffer           = dst;
    op.out.buffer_size      = fb_bytes;
    op.out.pic_w            = MPE_DISPLAY_WIDTH;
    op.out.pic_h            = MPE_DISPLAY_HEIGHT;
    op.out.block_offset_x   = x;
    op.out.block_offset_y   = y;
    op.out.srm_cm           = PPA_SRM_COLOR_MODE_RGB565;
    op.rotation_angle       = PPA_SRM_ROTATION_ANGLE_0;
    op.scale_x              = 1.0f;
    op.scale_y              = 1.0f;
    op.rgb_swap             = 0;
    op.byte_swap            = 0;
    op.mode                 = PPA_TRANS_MODE_BLOCKING;

    if (ppa_do_scale_rotate_mirror(s_ppa_srm, &op) != ESP_OK) {
        /* Fallback to CPU copy on PPA error. */
        const int stride = MPE_DISPLAY_WIDTH;
        for (int ry = 0; ry < h; ry++) {
            memcpy(dst + (size_t)(y + ry) * stride + x,
                   src + (size_t)(y + ry) * stride + x,
                   (size_t)w * 2);
        }
        return;
    }

    /* Invalidate cache lines for the rect rows so subsequent CPU
       reads of `dst` see what PPA just wrote (and not stale lines
       from 2 frames ago, which is what produced the "fill bug"
       smearing in alpha-blend overlays).
       The range is naturally cache-line aligned — each row is
       MPE_DISPLAY_WIDTH * 2 = 2048 B = exactly 32 cache lines on the
       P4 — so we can call M2C without the UNALIGNED flag, which the
       IDF doesn't permit for M2C anyway. */
    {
        const size_t row_bytes = (size_t)MPE_DISPLAY_WIDTH * 2;
        uint8_t *p = (uint8_t *)dst + (size_t)y * row_bytes;
        const size_t span = (size_t)h * row_bytes;
        (void)fb_bytes;
        esp_cache_msync(p, span, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    }
}

void mpe_display_flush_writes(void *buf, size_t bytes)
{
    if (!buf || bytes == 0) return;
    esp_cache_msync(buf, bytes,
                    ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                    ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

esp_err_t mpe_display_present_y(int y0, int h)
{
    if (!s_panel) return ESP_ERR_INVALID_STATE;

    if (y0 < 0) { h += y0; y0 = 0; }
    if (y0 + h > MPE_DISPLAY_HEIGHT) h = MPE_DISPLAY_HEIGHT - y0;
    if (h <= 0) {
        /* Nothing dirty in this frame — still need to draw_bitmap
           to keep the swap chain ticking, but no flush needed. */
    } else {
        const size_t row_bytes = (size_t)MPE_DISPLAY_WIDTH * 2;
        uint8_t *start = (uint8_t *)s_fbs[s_back_idx] +
                         (size_t)y0 * row_bytes;
        size_t   span  = (size_t)h * row_bytes;
        esp_cache_msync(start, span,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                        ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(
        s_panel, 0, 0,
        MPE_DISPLAY_WIDTH, MPE_DISPLAY_HEIGHT,
        s_fbs[s_back_idx]);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
        return err;
    }
    s_back_idx ^= 1;
    return ESP_OK;
}

esp_err_t mpe_display_present(void)
{
    /* Full-FB flush — for callers that don't track dirty Y. */
    return mpe_display_present_y(0, MPE_DISPLAY_HEIGHT);
}
