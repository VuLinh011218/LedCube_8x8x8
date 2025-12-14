#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "esp_crt_bundle.h"

#define TAG "led_mqtt_demo"

// ==== Hardware ====
#define LED_GPIO        GPIO_NUM_2
#define LED_ACTIVE_HIGH 1

// ==== MQTT ====
#define TOPIC_CMD   "linh/led/cmd"
#define TOPIC_STATE "linh/led/state"
#define BROKER_URI  "mqtts://1fc92a68f1ab453896be588db856104e.s1.eu.hivemq.cloud:8883"

static bool s_led_on = false;

// ---- helpers ----
static inline void led_apply(bool on) {
    int level = LED_ACTIVE_HIGH ? (on ? 1 : 0) : (on ? 0 : 1);
    gpio_set_level(LED_GPIO, level);
    s_led_on = on;
}
static inline void led_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    led_apply(false);
}
static void publish_state(esp_mqtt_client_handle_t client) {
    const char *payload = s_led_on ? "ON" : "OFF";
    esp_mqtt_client_publish(client, TOPIC_STATE, payload, 0, 1, 1);
    ESP_LOGI(TAG, "State -> %s (retained)", payload);
}
static bool payload_is_on(const char *data, int len) {
    return (len >= 2 &&
            (data[0] == 'O' || data[0] == 'o') &&
            (data[1] == 'N' || data[1] == 'n'));
}

// ---- MQTT events ----
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(client, TOPIC_CMD, 1);
        publish_state(client);
        break;

    case MQTT_EVENT_DATA:
        if (event->topic_len == strlen(TOPIC_CMD) &&
            strncmp(event->topic, TOPIC_CMD, event->topic_len) == 0) {
            bool turn_on = payload_is_on(event->data, event->data_len);
            led_apply(turn_on);
            publish_state(client);
            ESP_LOGI(TAG, "CMD %.*s -> LED %s",
                     event->data_len, event->data, turn_on ? "ON" : "OFF");
        } else {
            ESP_LOGI(TAG, "DATA %.*s = %.*s",
                     event->topic_len, event->topic,
                     event->data_len, event->data);
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

// ---- MQTT start ----
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials = {
            .username = "hivemq.webclient.1757612029764",
            .authentication = { .password = "29j,6OW@?Nt0uSBJsf#k" },
        },
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// ---- app_main ----
void app_main(void)
{
    ESP_LOGI(TAG, "Startup");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    led_init();
    ESP_ERROR_CHECK(example_connect());  // Wi-Fi/Ethernet
    mqtt_app_start();
}
