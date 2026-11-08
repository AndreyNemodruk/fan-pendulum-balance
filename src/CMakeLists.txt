idf_component_register(
    SRCS "main.c" "bmi160.c" "drv8833.c"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include"
    REQUIRES esp_driver_spi esp_timer esp_driver_ledc esp_driver_gpio
)
