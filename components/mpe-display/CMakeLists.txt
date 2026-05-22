idf_component_register(
    SRCS "src/mpe_display.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_lcd
        esp_psram
        esp_mm
        esp_hw_support
        esp_driver_ledc
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ppa
        espressif__esp_lcd_ek79007
        log)
