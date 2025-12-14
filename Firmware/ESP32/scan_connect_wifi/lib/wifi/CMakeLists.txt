idf_component_register(
    SRCS "wifi.c"
    REQUIRES nvs_flash esp_wifi esp_netif esp_event
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio  # Thêm esp_driver_gpio để sử dụng GPIO
)



