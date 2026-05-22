/*
 * mpe_wifi — station-mode bring-up via esp_hosted on ESP32-P4.
 *
 * The P4 has no native radio; esp_wifi_remote routes the WiFi API to
 * the on-board ESP32-C6 over SDIO. From the app's perspective it's
 * identical to esp_wifi on a chip that has a radio.
 *
 * Single-shot blocking init: returns once an IP has been assigned or
 * the retry budget is exhausted. Caller deals with later reconnects
 * via the auto-reconnect handler we register internally.
 */
#ifndef MPE_WIFI_H_
#define MPE_WIFI_H_

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Blocks for up to ~30 s. ESP_OK once associated + IP-up; ESP_ERR_TIMEOUT
   if the retry budget was exhausted. Safe to call once from app_main. */
esp_err_t mpe_wifi_init_blocking(void);

bool      mpe_wifi_is_up(void);

/* On success, fills `out` with a null-terminated dotted-quad of the
   station's current IPv4 address. Returns ESP_ERR_INVALID_STATE if
   the link isn't up yet. */
esp_err_t mpe_wifi_get_ip_str(char *out, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif
