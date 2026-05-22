/*
 * mpe_wifi.c — esp_hosted (P4 → C6 over SDIO) station bring-up.
 * Mirrors the proven browser-tree wifi implementation; adds an IP-
 * string accessor for the on-screen status overlay.
 */
#include "mpe_wifi.h"

#include <string.h>
#include <stdatomic.h>

#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t  s_evt;
static _Atomic int         s_retry = 0;
static _Atomic bool        s_up    = false;
static esp_netif_t        *s_netif = NULL;

static void evt_handler(void *arg, esp_event_base_t base,
                        int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        atomic_store(&s_up, false);
        const int n = atomic_fetch_add(&s_retry, 1) + 1;
        if (n <= CONFIG_WIFI_MAX_RETRY) {
            ESP_LOGW(TAG, "disconnected, retry %d/%d",
                     n, CONFIG_WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_evt, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        atomic_store(&s_retry, 0);
        atomic_store(&s_up, true);
        xEventGroupSetBits(s_evt, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "got IP");
    }
}

esp_err_t mpe_wifi_init_blocking(void)
{
    if (strlen(CONFIG_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "CONFIG_WIFI_SSID is empty — set it via menuconfig");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, evt_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, evt_handler, NULL));

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid, CONFIG_WIFI_SSID,
            sizeof wcfg.sta.ssid - 1);
    strncpy((char *)wcfg.sta.password, CONFIG_WIFI_PASSWORD,
            sizeof wcfg.sta.password - 1);
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_evt, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) return ESP_OK;
    if (bits & WIFI_FAIL_BIT)      return ESP_ERR_TIMEOUT;
    return ESP_ERR_TIMEOUT;
}

bool mpe_wifi_is_up(void) { return atomic_load(&s_up); }

esp_err_t mpe_wifi_get_ip_str(char *out, size_t out_len)
{
    if (!out || out_len < 8) return ESP_ERR_INVALID_ARG;
    if (!s_netif || !atomic_load(&s_up)) {
        out[0] = '\0';
        return ESP_ERR_INVALID_STATE;
    }
    esp_netif_ip_info_t info;
    esp_err_t err = esp_netif_get_ip_info(s_netif, &info);
    if (err != ESP_OK) {
        out[0] = '\0';
        return err;
    }
    esp_ip4addr_ntoa(&info.ip, out, out_len);
    return ESP_OK;
}
