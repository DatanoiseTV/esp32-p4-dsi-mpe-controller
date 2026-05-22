/*
 * mpe_touch.c — GT911 5-point read, BSP-compatible NC/NC init.
 *
 * The GT911 has two operating I2C addresses (0x14 and 0x5D) selected
 * during reset by the state of INT + RST. The "addr-select reset"
 * sequence (drive INT, pulse RST, release) is what bricked a device
 * on this hardware earlier (see project memory feedback_gt911_brick).
 * We mirror the BSP example exactly: leave RST and INT as
 * GPIO_NUM_NC, let the chip latch its default address (0x14 on the
 * EK79007 sub-board), and probe for it on the bus.
 *
 * For polyphony we want the controller's tracking IDs — they tell us
 * which finger is which across frames even when contacts cross. The
 * esp_lcd_touch_gt911 driver caches them per read; we use the
 * modern esp_lcd_touch_get_data() API to copy them out alongside
 * (x, y, strength).
 */
#include "mpe_touch.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"
#include "sdkconfig.h"

static const char *TAG = "touch";

static esp_lcd_panel_io_handle_t s_touch_io = NULL;
static esp_lcd_touch_handle_t    s_touch    = NULL;

/* The GT911 + esp_lcd_touch driver have a known gotcha: when the
   chip's "buffer ready" status bit is 0 (no fresh sample yet), the
   driver clears the cached point count to zero. A poll between
   GT911 samples therefore looks identical to "all fingers lifted",
   which made every poll produce a NoteOff/NoteOn boundary in the
   controller — audible as stepping on slide gestures. The fix: latch
   the last frame that had contacts, and report it on subsequent
   "empty" polls until enough time has elapsed that the finger really
   has lifted. 35 ms is comfortably longer than the GT911's worst-
   case inter-sample gap (~12 ms at the default 100 Hz scan rate). */
#define LATCH_HOLD_US    35000
static mpe_touch_frame s_last_active;
static int64_t         s_last_active_us;

esp_err_t mpe_touch_init(void *i2c_bus)
{
    if (!i2c_bus) return ESP_ERR_INVALID_ARG;

    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)i2c_bus;
    uint8_t addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS;
    if (i2c_master_probe(bus, addr, 100) != ESP_OK) {
        addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
        if (i2c_master_probe(bus, addr, 100) != ESP_OK) {
            ESP_LOGW(TAG, "GT911 not on the bus at 0x14 or 0x5D");
            return ESP_ERR_NOT_FOUND;
        }
    }

    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    io_cfg.dev_addr = addr;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_i2c(bus, &io_cfg, &s_touch_io),
        TAG, "new_panel_io_i2c");

    esp_lcd_touch_config_t cfg = {
        .x_max = 1024,
        .y_max = 600,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags  = { .swap_xy = 0, .mirror_x = 1, .mirror_y = 1 },
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_touch_new_i2c_gt911(s_touch_io, &cfg, &s_touch),
        TAG, "gt911 new");

    ESP_LOGI(TAG, "GT911 5-pt up at 0x%02X", addr);
    return ESP_OK;
}

esp_err_t mpe_touch_poll(mpe_touch_frame *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;
    out->count = 0;
    if (!s_touch) return ESP_ERR_INVALID_STATE;

    if (esp_lcd_touch_read_data(s_touch) != ESP_OK) {
        /* Transient I2C error — fall through to the latch so we
           don't drop a held finger on a single missed I2C cycle. */
    }

    esp_lcd_touch_point_data_t data[MPE_TOUCH_MAX_POINTS] = {};
    uint8_t cnt = 0;
    (void)esp_lcd_touch_get_data(s_touch, data, &cnt, MPE_TOUCH_MAX_POINTS);
    if (cnt > MPE_TOUCH_MAX_POINTS) cnt = MPE_TOUCH_MAX_POINTS;

    const int64_t now = esp_timer_get_time();

    if (cnt > 0) {
        out->count = cnt;
        for (uint8_t i = 0; i < cnt; i++) {
            out->points[i].tracking_id = data[i].track_id;
            out->points[i].x           = data[i].x;
            out->points[i].y           = data[i].y;
            out->points[i].strength    = data[i].strength;
        }
        s_last_active     = *out;
        s_last_active_us  = now;
        return ESP_OK;
    }

    /* No fresh sample. Two indistinguishable cases:
       (a) finger really lifted; or
       (b) GT911 just hasn't produced the next sample yet (the driver
           clears its point cache on every poll that finds bit-7=0).
       If (b), reporting empty means the controller fires NoteOff and
       the next non-empty poll re-triggers NoteOn — audible stepping.
       We assume (b) until LATCH_HOLD_US elapses, then declare (a). */
    if (s_last_active.count > 0 &&
        (now - s_last_active_us) < LATCH_HOLD_US) {
        *out = s_last_active;
        return ESP_OK;
    }

    /* Real release: clear latch + return empty. */
    s_last_active.count = 0;
    s_last_active_us    = 0;
    out->count          = 0;
    return ESP_OK;
}
