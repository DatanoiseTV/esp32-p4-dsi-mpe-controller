/*
 * mpe_display — EK79007 1024x600 DSI panel bring-up for the
 * ESP32-P4-Nano. Double-buffered RGB565 framebuffer in PSRAM,
 * managed by the panel driver: we paint into the back buffer, call
 * mpe_display_present() to queue a swap, and the call blocks until
 * the DSI engine has scanned out the now-shown buffer's first
 * complete frame.
 *
 * Why double-buffer for this app: at single-buffer the DSI scanout
 * races our paint and produces visible tearing on any frame that
 * takes longer than one display refresh (~16.6 ms). The instrument
 * UI's animated glow + per-frame gradient comfortably exceeds that
 * even on the P4, so we let the driver swap two buffers and we
 * synchronise to refresh.
 */
#ifndef MPE_DISPLAY_H_
#define MPE_DISPLAY_H_

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPE_DISPLAY_WIDTH  1024
#define MPE_DISPLAY_HEIGHT 600

esp_err_t   mpe_display_init(void);

/* Returns the buffer the application should paint into for the NEXT
   frame. After painting, call mpe_display_present(); the returned
   pointer flips between the two driver-owned buffers each present. */
uint16_t   *mpe_display_back_buffer(void);

/* Returns the buffer currently being scanned out by the DSI engine
   — useful for screenshotting (read the pixels the user is actually
   seeing). The caller is expected to invalidate its CPU cache view
   of the buffer before reading if it has previously cached lines
   from when this same buffer was the back buffer. */
uint16_t   *mpe_display_front_buffer(void);

/* Queue a swap so the back buffer becomes the front, then block
   until the panel has actually swapped (one refresh worth of wait).
   On return the back-buffer pointer (next call to back_buffer()) has
   flipped to the other buffer. */
esp_err_t   mpe_display_present(void);

/* Variant: instead of flushing the entire 1.2 MB framebuffer cache
   to PSRAM before the swap, flush only the Y range [y0, y0+h)
   that the caller actually modified. The full FB flush was the
   single biggest per-frame fixed cost; restricting to dirty rows
   typically cuts it by 30-60%. (y0, h) must be sane row indices in
   the back buffer; clipping is done internally. */
esp_err_t   mpe_display_present_y(int y0, int h);

esp_err_t   mpe_display_set_backlight(int pct);

/* Returns the I2C master bus that the touch controller shares with
   anything else on the panel side. Lazily initialised; safe to call
   any time after mpe_display_init(). */
void       *mpe_display_i2c_bus(void);

/* PPA-accelerated rect copy from `src` into `dst` at (x,y,w,h).
   Both buffers must be full-screen RGB565 (stride = MPE_DISPLAY_WIDTH).
   Blocking. Falls back to CPU memcpy if the PPA hasn't been set up
   yet (init failure or pre-init call). */
void        mpe_display_rect_copy(uint16_t *dst, const uint16_t *src,
                                  int x, int y, int w, int h);

/* Flush CPU-side cache writes to PSRAM. Call after a producer (e.g.
   the static-template rebake) finishes writing a buffer that a DMA
   engine (PPA / DSI) will subsequently read. Without this, DMA can
   see stale memory while the CPU cache still holds the latest
   writes. */
void        mpe_display_flush_writes(void *buf, size_t bytes);

#ifdef __cplusplus
}
#endif

#endif
