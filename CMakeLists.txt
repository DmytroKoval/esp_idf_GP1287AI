idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    return() # This component is not supported by the POSIX/Linux simulator
endif()

set(srcs "src/esp_lcd_panel_gp1287.c")
set(includes "include")
set(priv_include "priv_include")
set(priv_requires "esp_mm" "esp_psram" "esp_pm" )
set(public_requires "driver" "esp_driver_gpio" "esp_lcd" "esp_driver_spi")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       PRIV_INCLUDE_DIRS ${priv_include}
                       PRIV_REQUIRES ${priv_requires}
                       REQUIRES ${public_requires})
project(gp1287)