#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "receiver_protocol.h"

#define TRACKER_ESPNOW_CHANNEL 1U
#define RECEIVER_QUEUE_LENGTH 16U
#define COMMAND_QUEUE_LENGTH 8U
#define RECEIVER_TASK_STACK_SIZE 4096U
#define USB_TASK_STACK_SIZE 3072U
#define RECEIVER_TASK_PRIORITY 5U
#define USB_TASK_PRIORITY 4U
#define USB_TX_BUFFER_SIZE 512U
#define USB_RX_BUFFER_SIZE 256U
#define DIAGNOSTIC_DRIVE_GPIO GPIO_NUM_0
#define DIAGNOSTIC_SENSE_GPIO GPIO_NUM_1
#define DIAGNOSTIC_INTERVAL_MS 100U

typedef struct {
    uint8_t sender_mac[RECEIVER_MAC_SIZE];
    uint8_t payload[FUSION_PAYLOAD_CAPACITY];
    size_t payload_size;
} receiver_event_t;

typedef struct {
    uint8_t slot;
    uint8_t type;
    tracker_calibration_command_t command;
    mag_command_t mag_command;
} host_command_t;

static QueueHandle_t receiver_queue;
static QueueHandle_t command_queue;
static bool diagnostic_mode;

static int16_t get_i16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static bool diagnostic_jumper_present(void)
{
    gpio_config_t sense_config = {
        .pin_bit_mask = 1ULL << DIAGNOSTIC_SENSE_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sense_config));
    gpio_config_t drive_config = {
        .pin_bit_mask = 1ULL << DIAGNOSTIC_DRIVE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&drive_config));
    ESP_ERROR_CHECK(gpio_set_level(DIAGNOSTIC_DRIVE_GPIO, 0));
    vTaskDelay(pdMS_TO_TICKS(1));
    bool present = gpio_get_level(DIAGNOSTIC_SENSE_GPIO) == 0;
    ESP_ERROR_CHECK(gpio_reset_pin(DIAGNOSTIC_DRIVE_GPIO));
    ESP_ERROR_CHECK(gpio_reset_pin(DIAGNOSTIC_SENSE_GPIO));
    return present;
}

static void write_diagnostics(const receiver_session_t *session)
{
    const uint8_t *right =
        session->state.slots[RECEIVER_RIGHT_SLOT].controller_data;
    const uint8_t *left =
        session->state.slots[RECEIVER_LEFT_SLOT].controller_data;
    char message[256];
    int length = snprintf(
        message, sizeof(message),
        "R[%s] q=%d,%d,%d,%d a=%d,%d,%d | "
        "L[%s] q=%d,%d,%d,%d a=%d,%d,%d\r\n",
        session->sender_assigned[RECEIVER_RIGHT_SLOT] ? "paired" : "waiting",
        get_i16(&right[0]), get_i16(&right[2]), get_i16(&right[4]),
        get_i16(&right[6]), get_i16(&right[8]), get_i16(&right[10]),
        get_i16(&right[12]),
        session->sender_assigned[RECEIVER_LEFT_SLOT] ? "paired" : "waiting",
        get_i16(&left[0]), get_i16(&left[2]), get_i16(&left[4]),
        get_i16(&left[6]), get_i16(&left[8]), get_i16(&left[10]),
        get_i16(&left[12]));
    if (length <= 0) return;
    size_t write_length = (size_t)length;
    if (write_length >= sizeof(message)) write_length = sizeof(message) - 1U;
    (void)usb_serial_jtag_write_bytes(message, write_length,
                                      pdMS_TO_TICKS(20));
}

static void write_usb_frame(uint8_t type, const uint8_t *payload,
                            size_t payload_size)
{
    uint8_t frame[FUSION_FRAME_SIZE];
    receiver_build_usb_frame(type, payload, payload_size, frame);
    (void)usb_serial_jtag_write_bytes(frame, sizeof(frame),
                                      pdMS_TO_TICKS(20));
}

static void espnow_receive_cb(const esp_now_recv_info_t *info,
                              const uint8_t *data, int data_len)
{
    if (info == NULL || info->src_addr == NULL || data == NULL ||
        data_len <= 0 || data_len > (int)FUSION_PAYLOAD_CAPACITY) {
        return;
    }
    bool recognized =
        (data_len == (int)TRACKER_PACKET_SIZE &&
         data[0] == TRACKER_PACKET_ID_QUATERNION) ||
        (data_len == (int)TRACKER_CALIBRATION_STATUS_SIZE &&
         data[0] == TRACKER_PACKET_ID_CALIBRATION_STATUS) ||
        (data_len == (int)MAG_SAMPLE_SIZE && data[0] == MAG_PACKET_SAMPLE);
    if (!recognized) return;

    receiver_event_t event = {.payload_size = (size_t)data_len};
    memcpy(event.sender_mac, info->src_addr, sizeof(event.sender_mac));
    memcpy(event.payload, data, event.payload_size);
    (void)xQueueSend(receiver_queue, &event, 0);
}

static bool ensure_peer(const uint8_t mac[RECEIVER_MAC_SIZE])
{
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, RECEIVER_MAC_SIZE);
    peer.channel = TRACKER_ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    return err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST;
}

static void send_synthetic_status(uint8_t slot, uint16_t request_id,
                                  tracker_calibration_result_t result)
{
    tracker_calibration_status_t status = {
        .stage = TRACKER_CAL_STAGE_FAILED,
        .result = result,
        .request_id = request_id,
    };
    uint8_t status_packet[TRACKER_CALIBRATION_STATUS_SIZE];
    uint8_t payload[1 + TRACKER_CALIBRATION_STATUS_SIZE];
    tracker_calibration_status_encode(status_packet, &status);
    payload[0] = slot;
    memcpy(&payload[1], status_packet, sizeof(status_packet));
    write_usb_frame(FUSION_CALIBRATION_STATUS, payload, sizeof(payload));
}

static void process_host_command(receiver_session_t *session,
                                 const host_command_t *host)
{
    if (host->type == FUSION_MAG_COMMAND) {
        if (host->slot < RECEIVER_SLOT_COUNT && session->sender_assigned[host->slot]) {
            const uint8_t *mac = session->sender_macs[host->slot];
            uint8_t packet[MAG_COMMAND_SIZE];
            mag_command_encode(packet, &host->mag_command);
            if (ensure_peer(mac)) (void)esp_now_send(mac, packet, sizeof(packet));
        }
        return;
    }
    if (host->slot >= RECEIVER_SLOT_COUNT ||
        !session->sender_assigned[host->slot]) {
        send_synthetic_status(host->slot, host->command.request_id,
                              TRACKER_CAL_RESULT_UNAVAILABLE);
        return;
    }
    const uint8_t *mac = session->sender_macs[host->slot];
    if (!ensure_peer(mac)) {
        send_synthetic_status(host->slot, host->command.request_id,
                              TRACKER_CAL_RESULT_TRANSPORT);
        return;
    }
    uint8_t packet[TRACKER_CALIBRATION_COMMAND_SIZE];
    tracker_calibration_command_encode(packet, &host->command);
    if (esp_now_send(mac, packet, sizeof(packet)) != ESP_OK) {
        send_synthetic_status(host->slot, host->command.request_id,
                              TRACKER_CAL_RESULT_TRANSPORT);
    }
}

static void process_receiver_event(receiver_session_t *session,
                                   const receiver_event_t *event,
                                   TickType_t *last_diagnostic_tick,
                                   bool *diagnostic_sample_written)
{
    if (event->payload_size == TRACKER_PACKET_SIZE) {
        receiver_ingest_result_t result = receiver_session_ingest(
            session, event->sender_mac, event->payload, event->payload_size);
        if (result != RECEIVER_INGEST_APPLIED_RIGHT &&
            result != RECEIVER_INGEST_APPLIED_LEFT) {
            return;
        }
        size_t slot = result == RECEIVER_INGEST_APPLIED_RIGHT
            ? RECEIVER_RIGHT_SLOT
            : RECEIVER_LEFT_SLOT;
        (void)ensure_peer(event->sender_mac);
        if (diagnostic_mode) {
            TickType_t now = xTaskGetTickCount();
            if (!*diagnostic_sample_written ||
                now - *last_diagnostic_tick >=
                    pdMS_TO_TICKS(DIAGNOSTIC_INTERVAL_MS)) {
                write_diagnostics(session);
                *last_diagnostic_tick = now;
                *diagnostic_sample_written = true;
            }
        } else {
            write_usb_frame((uint8_t)slot, event->payload,
                            event->payload_size);
        }
        return;
    }

    if (event->payload_size == MAG_SAMPLE_SIZE) {
        mag_sample_t sample;
        if (!mag_sample_decode(event->payload, event->payload_size, &sample)) return;
        size_t slot;
        if (!receiver_session_assign_sender(session, event->sender_mac, &slot)) return;
        (void)ensure_peer(event->sender_mac);
        if (!diagnostic_mode) {
            uint8_t payload[1 + MAG_SAMPLE_SIZE];
            payload[0] = (uint8_t)slot;
            memcpy(&payload[1], event->payload, MAG_SAMPLE_SIZE);
            write_usb_frame(FUSION_MAG_SAMPLE, payload, sizeof(payload));
        }
        return;
    }

    tracker_calibration_status_t status;
    if (!tracker_calibration_status_decode(event->payload,
                                            event->payload_size, &status)) {
        return;
    }
    size_t slot;
    if (!receiver_session_assign_sender(session, event->sender_mac, &slot)) {
        return;
    }
    (void)ensure_peer(event->sender_mac);
    if (!diagnostic_mode) {
        uint8_t payload[1 + TRACKER_CALIBRATION_STATUS_SIZE];
        payload[0] = (uint8_t)slot;
        memcpy(&payload[1], event->payload, event->payload_size);
        write_usb_frame(FUSION_CALIBRATION_STATUS, payload, sizeof(payload));
    }
}

static void receiver_task(void *arg)
{
    (void)arg;
    receiver_session_t session;
    receiver_session_init(&session);
    TickType_t last_diagnostic_tick = 0;
    bool diagnostic_sample_written = false;

    for (;;) {
        host_command_t host;
        while (xQueueReceive(command_queue, &host, 0) == pdTRUE) {
            process_host_command(&session, &host);
        }
        receiver_event_t event;
        if (xQueueReceive(receiver_queue, &event, pdMS_TO_TICKS(20)) ==
            pdTRUE) {
            process_receiver_event(&session, &event, &last_diagnostic_tick,
                                   &diagnostic_sample_written);
        }
    }
}

static void usb_command_task(void *arg)
{
    (void)arg;
    uint8_t frame[FUSION_FRAME_SIZE];
    size_t used = 0;
    for (;;) {
        uint8_t byte;
        if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(100)) != 1) {
            continue;
        }
        if (used == 0 && byte != 0xa5U) continue;
        if (used == 1 && byte != 0x5aU) {
            used = byte == 0xa5U ? 1U : 0U;
            if (used == 1U) frame[0] = byte;
            continue;
        }
        frame[used++] = byte;
        if (used != FUSION_FRAME_SIZE) continue;
        used = 0;

        uint8_t type;
        const uint8_t *payload;
        size_t payload_size;
        if (!receiver_decode_usb_frame(frame, &type, &payload, &payload_size) ||
            payload_size != 1U + TRACKER_CALIBRATION_COMMAND_SIZE ||
            (type != FUSION_CALIBRATION_COMMAND && type != FUSION_MAG_COMMAND)) {
            continue;
        }
        host_command_t host = {.slot = payload[0], .type = type};
        bool decoded = type == FUSION_MAG_COMMAND
            ? mag_command_decode(&payload[1], MAG_COMMAND_SIZE, &host.mag_command)
            : tracker_calibration_command_decode(
                &payload[1], TRACKER_CALIBRATION_COMMAND_SIZE, &host.command);
        if (host.slot < RECEIVER_SLOT_COUNT && decoded) {
            if (!diagnostic_mode) {
                (void)xQueueSend(command_queue, &host, pdMS_TO_TICKS(20));
            }
        }
    }
}

static void usb_transport_init(void)
{
    usb_serial_jtag_driver_config_t config = {
        .tx_buffer_size = USB_TX_BUFFER_SIZE,
        .rx_buffer_size = USB_RX_BUFFER_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));
    ESP_ERROR_CHECK(esp_wifi_set_channel(TRACKER_ESPNOW_CHANNEL,
                                         WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_receive_cb));
}

void app_main(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
        result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);

    receiver_queue = xQueueCreate(RECEIVER_QUEUE_LENGTH,
                                  sizeof(receiver_event_t));
    command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(host_command_t));
    assert(receiver_queue != NULL && command_queue != NULL);

    diagnostic_mode = diagnostic_jumper_present();
    usb_transport_init();
    espnow_init();
    if (diagnostic_mode) {
        static const char message[] =
            "vrrecv native-USB diagnostics enabled; calibration commands "
            "disabled until the jumper is removed\r\n";
        (void)usb_serial_jtag_write_bytes(message, sizeof(message) - 1U,
                                          pdMS_TO_TICKS(20));
    }

    assert(xTaskCreate(receiver_task, "receiver", RECEIVER_TASK_STACK_SIZE,
                       NULL, RECEIVER_TASK_PRIORITY, NULL) == pdPASS);
    assert(xTaskCreate(usb_command_task, "usb_command", USB_TASK_STACK_SIZE,
                       NULL, USB_TASK_PRIORITY, NULL) == pdPASS);
}
