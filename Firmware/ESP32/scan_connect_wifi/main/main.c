#include "wifi.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    // Init NVS + WiFi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Quét trước 
    wifi_scan_and_print();

    // Kết nối
    wifi_init_sta();
}
