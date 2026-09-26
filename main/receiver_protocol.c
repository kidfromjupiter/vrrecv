#include "receiver_protocol.h"

#include <string.h>

#define HADES_PACKET_ID_CONTROLLERS 2U
#define HADES_REPORT_ID 1U
#define HADES_UART_MAGIC 0xaaU
#define HADES_CAPABILITY_ACCELEROMETER 0x0010U

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void init_controller_data(uint8_t data[HADES_CONTROLLER_STATE_SIZE])
{
    memset(data, 0, HADES_CONTROLLER_STATE_SIZE);
    put_u16(&data[0], 32767U);
    put_u16(&data[27], HADES_CAPABILITY_ACCELEROMETER);
}

bool receiver_packet_decode(const uint8_t *data, size_t size,
                            receiver_sample_t *sample)
{
    if (data == NULL || sample == NULL || size != TRACKER_PACKET_SIZE ||
        data[0] != TRACKER_PACKET_ID_QUATERNION ||
        data[1] != TRACKER_PACKET_VERSION) {
        return false;
    }

    sample->sequence = get_u16(&data[2]);
    sample->timestamp_us = get_u32(&data[4]);
    for (size_t i = 0; i < 4; ++i) {
        sample->quaternion[i] = (int16_t)get_u16(&data[8 + i * 2]);
    }
    for (size_t i = 0; i < 3; ++i) {
        sample->acceleration[i] = (int16_t)get_u16(&data[16 + i * 2]);
    }
    sample->status = get_u16(&data[22]);
    return true;
}

void receiver_state_init(receiver_state_t *state)
{
    memset(state, 0, sizeof(*state));
    for (size_t i = 0; i < RECEIVER_SLOT_COUNT; ++i) {
        init_controller_data(state->slots[i].controller_data);
    }
}

void receiver_state_apply_sample(receiver_state_t *state, size_t slot,
                                 const receiver_sample_t *sample)
{
    if (state == NULL || sample == NULL || slot >= RECEIVER_SLOT_COUNT) {
        return;
    }
    uint8_t *data = state->slots[slot].controller_data;
    for (size_t i = 0; i < 4; ++i) {
        put_u16(&data[i * 2], (uint16_t)sample->quaternion[i]);
    }
    for (size_t i = 0; i < 3; ++i) {
        put_u16(&data[8 + i * 2], (uint16_t)sample->acceleration[i]);
    }
}

void receiver_build_hades_frame(const receiver_state_t *state,
                                uint8_t frame_sequence,
                                uint8_t output[HADES_UART_FRAME_SIZE])
{
    memset(output, 0, HADES_UART_FRAME_SIZE);
    output[0] = HADES_UART_MAGIC;
    output[1] = frame_sequence;
    output[2] = HADES_UART_REPORT_SIZE;
    output[3] = HADES_REPORT_ID;
    output[4] = HADES_PACKET_ID_CONTROLLERS;
    memcpy(&output[5], state->slots[RECEIVER_RIGHT_SLOT].controller_data,
           HADES_CONTROLLER_STATE_SIZE);
    memcpy(&output[34], state->slots[RECEIVER_LEFT_SLOT].controller_data,
           HADES_CONTROLLER_STATE_SIZE);
}
