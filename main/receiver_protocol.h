#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "magnetometer_protocol.h"

#define RECEIVER_SLOT_COUNT 2U
#define RECEIVER_RIGHT_SLOT 0U
#define RECEIVER_LEFT_SLOT 1U
#define RECEIVER_MAC_SIZE 6U

#define TRACKER_PACKET_SIZE 24U
#define TRACKER_PACKET_ID_QUATERNION 1U
#define TRACKER_PACKET_ID_CALIBRATION_COMMAND 2U
#define TRACKER_PACKET_ID_CALIBRATION_STATUS 3U
#define TRACKER_PACKET_VERSION 1U
#define TRACKER_CALIBRATION_COMMAND_SIZE 8U
#define TRACKER_CALIBRATION_STATUS_SIZE 12U

#define HADES_CONTROLLER_STATE_SIZE 29U
#define FUSION_FRAME_SIZE 31U
#define FUSION_PROTOCOL_VERSION 1U
#define FUSION_RIGHT_SLOT 0U
#define FUSION_LEFT_SLOT 1U
#define FUSION_CALIBRATION_COMMAND 0x80U
#define FUSION_CALIBRATION_STATUS 0x81U
#define FUSION_MAG_COMMAND 0x82U
#define FUSION_MAG_SAMPLE 0x83U
#define FUSION_PAYLOAD_CAPACITY 24U

typedef enum {
    TRACKER_CAL_ACTION_QUERY = 1,
    TRACKER_CAL_ACTION_START_FULL,
    TRACKER_CAL_ACTION_CONFIRM,
    TRACKER_CAL_ACTION_FINISH_MAG,
    TRACKER_CAL_ACTION_CANCEL,
} tracker_calibration_action_t;

typedef enum {
    TRACKER_CAL_STAGE_IDLE = 0,
    TRACKER_CAL_STAGE_WAIT_STILL,
    TRACKER_CAL_STAGE_CAPTURE_STILL,
    TRACKER_CAL_STAGE_WAIT_POS_X,
    TRACKER_CAL_STAGE_CAPTURE_POS_X,
    TRACKER_CAL_STAGE_WAIT_NEG_X,
    TRACKER_CAL_STAGE_CAPTURE_NEG_X,
    TRACKER_CAL_STAGE_WAIT_POS_Y,
    TRACKER_CAL_STAGE_CAPTURE_POS_Y,
    TRACKER_CAL_STAGE_WAIT_NEG_Y,
    TRACKER_CAL_STAGE_CAPTURE_NEG_Y,
    TRACKER_CAL_STAGE_WAIT_POS_Z,
    TRACKER_CAL_STAGE_CAPTURE_POS_Z,
    TRACKER_CAL_STAGE_WAIT_NEG_Z,
    TRACKER_CAL_STAGE_CAPTURE_NEG_Z,
    TRACKER_CAL_STAGE_WAIT_MAG,
    TRACKER_CAL_STAGE_CAPTURE_MAG,
    TRACKER_CAL_STAGE_SAVING,
    TRACKER_CAL_STAGE_COMPLETE,
    TRACKER_CAL_STAGE_FAILED,
    TRACKER_CAL_STAGE_CANCELLED,
} tracker_calibration_stage_t;

typedef enum {
    TRACKER_CAL_RESULT_NONE = 0,
    TRACKER_CAL_RESULT_OK,
    TRACKER_CAL_RESULT_BUSY,
    TRACKER_CAL_RESULT_INVALID_STAGE,
    TRACKER_CAL_RESULT_SENSOR,
    TRACKER_CAL_RESULT_MOTION,
    TRACKER_CAL_RESULT_WRONG_FACE,
    TRACKER_CAL_RESULT_MAG_SAMPLES,
    TRACKER_CAL_RESULT_MAG_RANGE,
    TRACKER_CAL_RESULT_SAVE,
    TRACKER_CAL_RESULT_CANCELLED,
    TRACKER_CAL_RESULT_UNAVAILABLE,
    TRACKER_CAL_RESULT_TIMEOUT,
    TRACKER_CAL_RESULT_TRANSPORT,
} tracker_calibration_result_t;

#define TRACKER_CAL_STATUS_HAS_CALIBRATION 0x01U
#define TRACKER_CAL_STATUS_CAN_FINISH       0x02U

typedef struct {
    tracker_calibration_action_t action;
    tracker_calibration_stage_t expected_stage;
    uint16_t request_id;
} tracker_calibration_command_t;

typedef struct {
    tracker_calibration_stage_t stage;
    tracker_calibration_result_t result;
    uint8_t flags;
    uint8_t elapsed_seconds;
    uint16_t request_id;
    uint16_t sample_count;
} tracker_calibration_status_t;

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

typedef struct {
    receiver_state_t state;
    uint8_t sender_macs[RECEIVER_SLOT_COUNT][RECEIVER_MAC_SIZE];
    bool sender_assigned[RECEIVER_SLOT_COUNT];
} receiver_session_t;

typedef enum {
    RECEIVER_INGEST_INVALID = 0,
    RECEIVER_INGEST_APPLIED_RIGHT,
    RECEIVER_INGEST_APPLIED_LEFT,
    RECEIVER_INGEST_UNKNOWN_SENDER,
} receiver_ingest_result_t;

bool receiver_packet_decode(const uint8_t *data, size_t size,
                            receiver_sample_t *sample);

void receiver_state_init(receiver_state_t *state);
void receiver_state_apply_sample(receiver_state_t *state, size_t slot,
                                 const receiver_sample_t *sample);

void receiver_session_init(receiver_session_t *session);
receiver_ingest_result_t receiver_session_ingest(
    receiver_session_t *session,
    const uint8_t sender_mac[RECEIVER_MAC_SIZE],
    const uint8_t *data, size_t size);

bool receiver_session_assign_sender(
    receiver_session_t *session,
    const uint8_t sender_mac[RECEIVER_MAC_SIZE], size_t *slot);

void tracker_calibration_command_encode(
    uint8_t output[TRACKER_CALIBRATION_COMMAND_SIZE],
    const tracker_calibration_command_t *command);
bool tracker_calibration_command_decode(
    const uint8_t *data, size_t size, tracker_calibration_command_t *command);
void tracker_calibration_status_encode(
    uint8_t output[TRACKER_CALIBRATION_STATUS_SIZE],
    const tracker_calibration_status_t *status);
bool tracker_calibration_status_decode(
    const uint8_t *data, size_t size, tracker_calibration_status_t *status);

void receiver_build_usb_frame(uint8_t type, const uint8_t *payload,
                              size_t payload_size,
                              uint8_t output[FUSION_FRAME_SIZE]);
bool receiver_decode_usb_frame(const uint8_t frame[FUSION_FRAME_SIZE],
                               uint8_t *type, const uint8_t **payload,
                               size_t *payload_size);

void receiver_build_fusion_frame(uint8_t slot,
                                 const uint8_t tracker_packet[TRACKER_PACKET_SIZE],
                                 uint8_t output[FUSION_FRAME_SIZE]);
