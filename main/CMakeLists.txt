idf_component_register(
    SRCS
        "main.cpp"
        "ui.cpp"
        "controller.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_psram
        esp_timer
        esp_netif
        nvs_flash
        lwip
        mpe-display
        mpe-touch
        mpe-wifi
        mpe-paint
        mpe-font
        mpe-osc
        mpe-applemidi
        mpe-screenshot
    EMBED_FILES
        # Inter Variable (OFL 1.1). Replace with any other TrueType
        # by overwriting this file — stb_truetype handles whatever
        # outline format the TTF contains.
        "font.ttf"
)
