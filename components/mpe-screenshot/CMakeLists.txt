idf_component_register(
    SRCS "src/mpe_screenshot.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_http_server
        esp_hw_support
        mpe-display
        log)
