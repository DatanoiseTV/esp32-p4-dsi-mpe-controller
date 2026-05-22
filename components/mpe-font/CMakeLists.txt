# mpe-font — stb_truetype TTF renderer for the instrument UI.
#
# stb_truetype is single-header public-domain; vendored at ../../deps.
# The TTF itself is embedded by the application via EMBED_FILES
# (see main/CMakeLists.txt).

set(STB_PATH "${CMAKE_CURRENT_LIST_DIR}/../../deps/stb_truetype.h")
if (NOT EXISTS "${STB_PATH}")
    message(FATAL_ERROR
        "stb_truetype.h not found at ${STB_PATH}. Vendor it once with:\n"
        "  curl -L https://raw.githubusercontent.com/nothings/stb/master/stb_truetype.h \\\n"
        "    -o deps/stb_truetype.h")
endif()

idf_component_register(
    SRCS "src/mpe_font.c"
    INCLUDE_DIRS "include" "${CMAKE_CURRENT_LIST_DIR}/../../deps"
    REQUIRES esp_psram log)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -O3
    -funroll-loops
    -Wno-error=missing-field-initializers
    -Wno-error=unused-function)
