#include "receiver_protocol.h"

#include <string.h>

#define HADES_CAPABILITY_ACCELEROMETER 0x0010U
#define FUSION_MAGIC_0 0xa5U
#define FUSION_MAGIC_1 0x5aU

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

static uint16_t crc16_ccitt(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xffffU;
    for (size_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
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

void receiver_session_init(receiver_session_t *session)
{
    if (session == NULL) {
        return;
    }

    memset(session, 0, sizeof(*session));
    receiver_state_init(&session->state);
}

bool receiver_session_assign_sender(
    receiver_session_t *session,
    const uint8_t sender_mac[RECEIVER_MAC_SIZE], size_t *assigned_slot)
{
    if (session == NULL || sender_mac == NULL) return false;
    size_t slot = RECEIVER_SLOT_COUNT;
    for (size_t i = 0; i < RECEIVER_SLOT_COUNT; ++i) {
        if (session->sender_assigned[i] &&
            memcmp(session->sender_macs[i], sender_mac,
                   RECEIVER_MAC_SIZE) == 0) {
            slot = i;
            break;
        }
    }
    if (slot == RECEIVER_SLOT_COUNT) {
        for (size_t i = 0; i < RECEIVER_SLOT_COUNT; ++i) {
            if (!session->sender_assigned[i]) {
                memcpy(session->sender_macs[i], sender_mac,
                       RECEIVER_MAC_SIZE);
                session->sender_assigned[i] = true;
                slot = i;
                break;
            }
        }
    }
    if (slot == RECEIVER_SLOT_COUNT) return false;
    if (assigned_slot != NULL) *assigned_slot = slot;
    return true;
}

receiver_ingest_result_t receiver_session_ingest(
    receiver_session_t *session,
    const uint8_t sender_mac[RECEIVER_MAC_SIZE],
    const uint8_t *data, size_t size)
{
    receiver_sample_t sample;
    if (session == NULL || sender_mac == NULL ||
        !receiver_packet_decode(data, size, &sample)) {
        return RECEIVER_INGEST_INVALID;
    }

    size_t slot;
    if (!receiver_session_assign_sender(session, sender_mac, &slot)) {
        return RECEIVER_INGEST_UNKNOWN_SENDER;
    }

    receiver_state_apply_sample(&session->state, slot, &sample);
    return slot == RECEIVER_RIGHT_SLOT ? RECEIVER_INGEST_APPLIED_RIGHT
                                       : RECEIVER_INGEST_APPLIED_LEFT;
}

void receiver_build_fusion_frame(uint8_t slot,
                                 const uint8_t tracker_packet[TRACKER_PACKET_SIZE],
                                 uint8_t output[FUSION_FRAME_SIZE])
{
    receiver_build_usb_frame(slot, tracker_packet, TRACKER_PACKET_SIZE, output);
}

void receiver_build_usb_frame(uint8_t type, const uint8_t *payload,
                              size_t payload_size,
                              uint8_t output[FUSION_FRAME_SIZE])
{
    memset(output, 0, FUSION_FRAME_SIZE);
    output[0] = FUSION_MAGIC_0;
    output[1] = FUSION_MAGIC_1;
    output[2] = FUSION_PROTOCOL_VERSION;
    output[3] = type;
    if (payload_size > FUSION_PAYLOAD_CAPACITY) payload_size = 0;
    output[4] = (uint8_t)payload_size;
    if (payload_size != 0 && payload != NULL) {
        memcpy(&output[5], payload, payload_size);
    }
    put_u16(&output[29], crc16_ccitt(&output[2], 27U));
}

bool receiver_decode_usb_frame(const uint8_t frame[FUSION_FRAME_SIZE],
                               uint8_t *type, const uint8_t **payload,
                               size_t *payload_size)
{
    if (frame == NULL || type == NULL || payload == NULL ||
        payload_size == NULL || frame[0] != FUSION_MAGIC_0 ||
        frame[1] != FUSION_MAGIC_1 ||
        frame[2] != FUSION_PROTOCOL_VERSION ||
        frame[4] > FUSION_PAYLOAD_CAPACITY ||
        get_u16(&frame[29]) != crc16_ccitt(&frame[2], 27U)) {
        return false;
    }
    *type = frame[3];
    *payload_size = frame[4];
    *payload = &frame[5];
    return true;
}

void tracker_calibration_command_encode(
    uint8_t output[TRACKER_CALIBRATION_COMMAND_SIZE],
    const tracker_calibration_command_t *command)
{
    memset(output, 0, TRACKER_CALIBRATION_COMMAND_SIZE);
    output[0] = TRACKER_PACKET_ID_CALIBRATION_COMMAND;
    output[1] = TRACKER_CALIBRATION_PACKET_VERSION;
    output[2] = (uint8_t)command->action;
    output[3] = (uint8_t)command->expected_stage;
    put_u16(&output[4], command->request_id);
}

bool tracker_calibration_command_decode(
    const uint8_t *data, size_t size, tracker_calibration_command_t *command)
{
    if (data == NULL || command == NULL ||
        size != TRACKER_CALIBRATION_COMMAND_SIZE ||
        data[0] != TRACKER_PACKET_ID_CALIBRATION_COMMAND ||
        data[1] != TRACKER_CALIBRATION_PACKET_VERSION ||
        data[2] < TRACKER_CAL_ACTION_QUERY ||
        data[2] > TRACKER_CAL_ACTION_CANCEL ||
        data[3] > TRACKER_CAL_STAGE_LAST || data[6] != 0 || data[7] != 0) {
        return false;
    }
    command->action = (tracker_calibration_action_t)data[2];
    command->expected_stage = (tracker_calibration_stage_t)data[3];
    command->request_id = get_u16(&data[4]);
    return true;
}

void tracker_calibration_status_encode(
    uint8_t output[TRACKER_CALIBRATION_STATUS_SIZE],
    const tracker_calibration_status_t *status)
{
    memset(output, 0, TRACKER_CALIBRATION_STATUS_SIZE);
    output[0] = TRACKER_PACKET_ID_CALIBRATION_STATUS;
    output[1] = TRACKER_CALIBRATION_PACKET_VERSION;
    output[2] = (uint8_t)status->stage;
    output[3] = (uint8_t)status->result;
    output[4] = status->flags;
    output[5] = status->elapsed_seconds;
    put_u16(&output[6], status->request_id);
    put_u16(&output[8], status->sample_count);
}

bool tracker_calibration_status_decode(
    const uint8_t *data, size_t size, tracker_calibration_status_t *status)
{
    if (data == NULL || status == NULL ||
        size != TRACKER_CALIBRATION_STATUS_SIZE ||
        data[0] != TRACKER_PACKET_ID_CALIBRATION_STATUS ||
        data[1] != TRACKER_CALIBRATION_PACKET_VERSION ||
        data[2] > TRACKER_CAL_STAGE_LAST ||
        data[3] > TRACKER_CAL_RESULT_MAP_ALIGNMENT ||
        (data[4] & ~0x07U) || data[10] != 0 || data[11] != 0) {
        return false;
    }
    status->stage = (tracker_calibration_stage_t)data[2];
    status->result = (tracker_calibration_result_t)data[3];
    status->flags = data[4];
    status->elapsed_seconds = data[5];
    status->request_id = get_u16(&data[6]);
    status->sample_count = get_u16(&data[8]);
    return true;
}

bool tracker_axis_map_valid(const int8_t map[3])
{
    if (map == NULL) return false;
    unsigned used = 0;
    int axes[3], determinant = 1;
    for (int i = 0; i < 3; ++i) {
        axes[i] = map[i] < 0 ? -map[i] : map[i];
        if (axes[i] < 1 || axes[i] > 3 || (used & (1U << axes[i]))) return false;
        used |= 1U << axes[i];
        if (map[i] < 0) determinant = -determinant;
    }
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (axes[i] > axes[j]) determinant = -determinant;
    return determinant == 1;
}

void tracker_calibration_map_report_encode(
    uint8_t output[TRACKER_CALIBRATION_MAP_REPORT_SIZE],
    const tracker_calibration_map_report_t *report)
{
    memset(output, 0, TRACKER_CALIBRATION_MAP_REPORT_SIZE);
    output[0] = TRACKER_PACKET_ID_CALIBRATION_MAP_REPORT;
    output[1] = TRACKER_CALIBRATION_PACKET_VERSION;
    put_u16(&output[2], report->request_id);
    for (int i = 0; i < 3; ++i) {
        output[4 + i] = (uint8_t)report->imu_axis_map[i];
        output[7 + i] = (uint8_t)report->mag_axis_map[i];
    }
    output[10] = report->flags;
}

bool tracker_calibration_map_report_decode(
    const uint8_t *data, size_t size, tracker_calibration_map_report_t *report)
{
    if (data == NULL || report == NULL || size != TRACKER_CALIBRATION_MAP_REPORT_SIZE ||
        data[0] != TRACKER_PACKET_ID_CALIBRATION_MAP_REPORT ||
        data[1] != TRACKER_CALIBRATION_PACKET_VERSION || data[11] != 0 ||
        (data[10] & ~(TRACKER_CAL_STATUS_HAS_CALIBRATION | TRACKER_CAL_STATUS_HAS_MAPS)) ||
        !(data[10] & TRACKER_CAL_STATUS_HAS_CALIBRATION)) return false;
    tracker_calibration_map_report_t decoded = {.request_id = get_u16(&data[2]), .flags = data[10]};
    for (int i = 0; i < 3; ++i) {
        decoded.imu_axis_map[i] = (int8_t)data[4 + i];
        decoded.mag_axis_map[i] = (int8_t)data[7 + i];
    }
    if (!tracker_axis_map_valid(decoded.imu_axis_map) ||
        !tracker_axis_map_valid(decoded.mag_axis_map)) return false;
    *report = decoded;
    return true;
}
