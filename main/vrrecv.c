#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define TRACKER_ESPNOW_CHANNEL     1
static const char* TAG= "recv";

typedef struct {
    float q[4];  /* W, X, Y, Z */
} orientation_packet_t;

static QueueHandle_t orientation_queue;

static void espnow_receive_cb(const esp_now_recv_info_t *info,
                              const uint8_t *data, int data_len)
{
    float *values = (float *)data;

    ESP_LOGI(TAG, "W: %f, X: %f, Y: %f, Z: %f",values[0], values[1], values[2], values[3] );
}

static void espnow_init(void)
{

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));
    ESP_ERROR_CHECK(esp_wifi_set_channel(TRACKER_ESPNOW_CHANNEL,
                                         WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());

    ESP_LOGI(TAG, "ESP-NOW initialized in station mode on channel %d",
             TRACKER_ESPNOW_CHANNEL);

    uint8_t mac[6] = {};
ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));

ESP_LOGI(TAG, "Receiver STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    espnow_init();

    /* Run these after starting Wi-Fi and calling esp_now_init(). */
    orientation_queue = xQueueCreate(1, sizeof(orientation_packet_t));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));

}
