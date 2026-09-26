#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RECEIVER_SLOT_COUNT 2U
#define RECEIVER_RIGHT_SLOT 0U
#define RECEIVER_LEFT_SLOT 1U

#define TRACKER_PACKET_SIZE 24U
#define TRACKER_PACKET_ID_QUATERNION 1U
#define TRACKER_PACKET_VERSION 1U

#define HADES_CONTROLLER_STATE_SIZE 29U
#define HADES_CONTROLLER_PAYLOAD_SIZE 63U
#define HADES_UART_REPORT_SIZE 64U
#define HADES_UART_FRAME_SIZE 67U

typedef struct {
    uint16_t sequence;
    uint32_t timestamp_us;
    int16_t quaternion[4];
    int16_t acceleration[3];
    uint16_t status;
} receiver_sample_t;

typedef struct {
    uint8_t controller_data[HADES_CONTROLLER_STATE_SIZE];
} receiver_slot_t;

typedef struct {
    receiver_slot_t slots[RECEIVER_SLOT_COUNT];
} receiver_state_t;

bool receiver_packet_decode(const uint8_t *data, size_t size,
                            receiver_sample_t *sample);

void receiver_state_init(receiver_state_t *state);
void receiver_state_apply_sample(receiver_state_t *state, size_t slot,
                                 const receiver_sample_t *sample);

void receiver_build_hades_frame(const receiver_state_t *state,
                                uint8_t frame_sequence,
                                uint8_t output[HADES_UART_FRAME_SIZE]);
