idf_component_register(
    SRCS "src/mpe_applemidi.c"
    INCLUDE_DIRS "include"
    REQUIRES
        lwip
        esp_timer
        esp_hw_support
        log)
