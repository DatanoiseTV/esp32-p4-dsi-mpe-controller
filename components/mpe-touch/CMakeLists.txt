idf_component_register(
    SRCS "src/mpe_touch.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_lcd
        esp_timer
        espressif__esp_lcd_touch
        espressif__esp_lcd_touch_gt911
        log)
