# CMakeLists.txt cho component "main"
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_netif nvs_flash mqtt driver protocol_examples_common
    PRIV_REQUIRES mbedtls esp-tls
)



