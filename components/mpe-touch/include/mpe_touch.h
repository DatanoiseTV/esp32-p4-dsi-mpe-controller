/*
 * mpe_touch — GT911 5-point capacitive touch on the EK79007 display
 * sub-board, brick-safe NC/NC init pattern.
 *
 * The GT911 has two operating I2C addresses (0x14 and 0x5D) selected
 * during reset by the INT/RST line state. The "addr-select reset"
 * sequence (drive INT, pulse RST, release) is what bricked an earlier
 * device on this hardware (see project memory feedback_gt911_brick_risk).
 * We follow the BSP example exactly: leave both RST and INT as
 * GPIO_NUM_NC, let the chip latch its default address (0x14 on the
 * EK79007 sub-board), and probe for it on the bus.
 *
 * Unlike the sibling browser project (which polls a single point for
 * scroll), this one reads all 5 contacts so the app can drive MPE
 * polyphony — one MIDI member channel per finger.
 *
 * Coordinate system after mirror_x/y = pixel-correct against the
 * 1024x600 framebuffer (origin top-left).
 */
#ifndef MPE_TOUCH_H_
#define MPE_TOUCH_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPE_TOUCH_MAX_POINTS 5

/* One contact, in pixel coordinates.
 *
 *  - tracking_id: a stable id the GT911 keeps for the lifetime of one
 *    finger-on-glass event. Identical id across frames means "same
 *    finger" — the host uses it to track gestures across samples.
 *  - strength: 0..255-ish "size" of the contact (GT911 reports this as
 *    an approximation of pressure; small/sharp finger = lower number).
 */
typedef struct {
    int     tracking_id;
    int     x;
    int     y;
    int     strength;
} mpe_touch_point;

typedef struct {
    int             count;
    mpe_touch_point points[MPE_TOUCH_MAX_POINTS];
} mpe_touch_frame;

esp_err_t mpe_touch_init(void *i2c_bus);

/* Read the latest touch frame from the GT911. `out->count` is the
   number of active contacts (0..MPE_TOUCH_MAX_POINTS). Always returns
   ESP_OK once init succeeded — even an empty frame is a valid result. */
esp_err_t mpe_touch_poll(mpe_touch_frame *out);

#ifdef __cplusplus
}
#endif

#endif
