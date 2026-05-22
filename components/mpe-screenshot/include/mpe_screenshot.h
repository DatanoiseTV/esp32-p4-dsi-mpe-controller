/*
 * mpe_screenshot — tiny HTTP endpoint that serves the currently-
 * displayed framebuffer as a BMP file, for grabbing pixel-perfect
 * UI captures into the repo's docs/.
 *
 * Usage:
 *   1. Device boots, WiFi associates, screenshot server starts.
 *   2. On the host:
 *        curl http://<device-ip>/screenshot.bmp -o screenshot.bmp
 *      The BMP is a BITMAPV4HEADER + RGB565 raw pixels (BI_BITFIELDS
 *      with the standard 5-6-5 channel masks), so it opens in any
 *      image viewer / browser / converter without further decoding.
 *
 * This is debug-only — bind to all interfaces, no auth. Don't ship
 * it on a hostile network.
 */
#ifndef MPE_SCREENSHOT_H_
#define MPE_SCREENSHOT_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mpe_screenshot_start(void);

#ifdef __cplusplus
}
#endif

#endif
