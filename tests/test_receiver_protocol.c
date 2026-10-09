#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "receiver_protocol.h"

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
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

static void make_packet(uint8_t packet[TRACKER_PACKET_SIZE], uint16_t sequence,
                        int16_t q0, int16_t accel0)
{
    memset(packet, 0, TRACKER_PACKET_SIZE);
    packet[0] = TRACKER_PACKET_ID_QUATERNION;
    packet[1] = TRACKER_PACKET_VERSION;
    put_u16(&packet[2], sequence);
    put_u32(&packet[4], 0x89abcdefU);
    put_u16(&packet[8], (uint16_t)q0);
    put_u16(&packet[10], 1000U);
    put_u16(&packet[12], (uint16_t)-2000);
    put_u16(&packet[14], 3000U);
    put_u16(&packet[16], (uint16_t)accel0);
    put_u16(&packet[18], (uint16_t)-512);
    put_u16(&packet[20], 2048U);
    put_u16(&packet[22], 0x0013U);
}

static void test_packet_decode(void)
{
    uint8_t packet[TRACKER_PACKET_SIZE];
    make_packet(packet, 0x1234U, 32767, -1024);
    receiver_sample_t sample;
    assert(receiver_packet_decode(packet, sizeof(packet), &sample));
    assert(sample.sequence == 0x1234U);
    assert(sample.timestamp_us == 0x89abcdefU);
    assert(sample.quaternion[0] == 32767);
    assert(sample.quaternion[2] == -2000);
    assert(sample.acceleration[0] == -1024);
    assert(sample.acceleration[1] == -512);
    assert(sample.status == 0x0013U);

    assert(!receiver_packet_decode(packet, sizeof(packet) - 1, &sample));
    packet[0] = 99;
    assert(!receiver_packet_decode(packet, sizeof(packet), &sample));
    packet[0] = TRACKER_PACKET_ID_QUATERNION;
    packet[1] = 2;
    assert(!receiver_packet_decode(packet, sizeof(packet), &sample));
}

static void test_fusion_frame(void)
{
    assert(crc16_ccitt((const uint8_t *)"123456789", 9U) == 0x29b1U);
    uint8_t packet[TRACKER_PACKET_SIZE];
    make_packet(packet, 7, 12345, -2345);
    uint8_t frame[FUSION_FRAME_SIZE];
    receiver_build_fusion_frame(FUSION_RIGHT_SLOT, packet, frame);
    assert(frame[0] == 0xa5U && frame[1] == 0x5aU);
    assert(frame[2] == FUSION_PROTOCOL_VERSION);
    assert(frame[3] == FUSION_RIGHT_SLOT);
    assert(frame[4] == TRACKER_PACKET_SIZE);
    assert(memcmp(&frame[5], packet, TRACKER_PACKET_SIZE) == 0);
    assert(get_u16(&frame[29]) == crc16_ccitt(&frame[2], 27U));
}

static void test_usb_and_calibration_protocol(void)
{
    tracker_calibration_command_t command = {
        .action = TRACKER_CAL_ACTION_CONFIRM,
        .expected_stage = TRACKER_CAL_STAGE_WAIT_NEG_Z,
        .request_id = 0x4321,
    };
    uint8_t command_packet[TRACKER_CALIBRATION_COMMAND_SIZE];
    tracker_calibration_command_encode(command_packet, &command);
    tracker_calibration_command_t decoded_command;
    assert(tracker_calibration_command_decode(
        command_packet, sizeof(command_packet), &decoded_command));
    assert(decoded_command.action == command.action);
    assert(decoded_command.expected_stage == command.expected_stage);
    assert(decoded_command.request_id == command.request_id);

    uint8_t payload[1 + TRACKER_CALIBRATION_COMMAND_SIZE];
    payload[0] = RECEIVER_LEFT_SLOT;
    memcpy(&payload[1], command_packet, sizeof(command_packet));
    uint8_t frame[FUSION_FRAME_SIZE];
    receiver_build_usb_frame(FUSION_CALIBRATION_COMMAND, payload,
                             sizeof(payload), frame);
    uint8_t frame_type;
    const uint8_t *decoded_payload;
    size_t decoded_size;
    assert(receiver_decode_usb_frame(frame, &frame_type, &decoded_payload,
                                     &decoded_size));
    assert(frame_type == FUSION_CALIBRATION_COMMAND);
    assert(decoded_size == sizeof(payload));
    assert(memcmp(decoded_payload, payload, sizeof(payload)) == 0);
    frame[29] ^= 1U;
    assert(!receiver_decode_usb_frame(frame, &frame_type, &decoded_payload,
                                      &decoded_size));

    tracker_calibration_status_t status = {
        .stage = TRACKER_CAL_STAGE_CAPTURE_MAG,
        .flags = TRACKER_CAL_STATUS_CAN_FINISH,
        .elapsed_seconds = 11,
        .request_id = 99,
        .sample_count = 321,
    };
    uint8_t status_packet[TRACKER_CALIBRATION_STATUS_SIZE];
    tracker_calibration_status_encode(status_packet, &status);
    tracker_calibration_status_t decoded_status;
    assert(tracker_calibration_status_decode(
        status_packet, sizeof(status_packet), &decoded_status));
    assert(decoded_status.stage == TRACKER_CAL_STAGE_CAPTURE_MAG);
    assert(decoded_status.elapsed_seconds == 11);
    assert(decoded_status.sample_count == 321);
}

static void test_session_pairing(void)
{
    static const uint8_t right_mac[RECEIVER_MAC_SIZE] =
        {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    static const uint8_t left_mac[RECEIVER_MAC_SIZE] =
        {0x20, 0x21, 0x22, 0x23, 0x24, 0x25};
    static const uint8_t unknown_mac[RECEIVER_MAC_SIZE] =
        {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
    uint8_t packet[TRACKER_PACKET_SIZE];
    receiver_session_t session;

    receiver_session_init(&session);
    assert(!session.sender_assigned[RECEIVER_RIGHT_SLOT]);
    assert(!session.sender_assigned[RECEIVER_LEFT_SLOT]);

    make_packet(packet, 1, 1111, 2222);
    packet[0] = 99;
    assert(receiver_session_ingest(&session, right_mac, packet,
                                   sizeof(packet)) ==
           RECEIVER_INGEST_INVALID);
    assert(!session.sender_assigned[RECEIVER_RIGHT_SLOT]);

    size_t beacon_slot;
    assert(receiver_session_assign_sender(&session, right_mac, &beacon_slot));
    assert(beacon_slot == RECEIVER_RIGHT_SLOT);

    packet[0] = TRACKER_PACKET_ID_QUATERNION;
    assert(receiver_session_ingest(&session, right_mac, packet,
                                   sizeof(packet)) ==
           RECEIVER_INGEST_APPLIED_RIGHT);
    assert(session.sender_assigned[RECEIVER_RIGHT_SLOT]);
    assert(memcmp(session.sender_macs[RECEIVER_RIGHT_SLOT], right_mac,
                  RECEIVER_MAC_SIZE) == 0);
    assert((int16_t)get_u16(
               &session.state.slots[RECEIVER_RIGHT_SLOT].controller_data[0]) ==
           1111);

    make_packet(packet, 2, 3333, 4444);
    assert(receiver_session_ingest(&session, left_mac, packet,
                                   sizeof(packet)) ==
           RECEIVER_INGEST_APPLIED_LEFT);
    assert(session.sender_assigned[RECEIVER_LEFT_SLOT]);
    assert((int16_t)get_u16(
               &session.state.slots[RECEIVER_LEFT_SLOT].controller_data[0]) ==
           3333);

    make_packet(packet, 3, 5555, 6666);
    assert(receiver_session_ingest(&session, right_mac, packet,
                                   sizeof(packet)) ==
           RECEIVER_INGEST_APPLIED_RIGHT);
    assert((int16_t)get_u16(
               &session.state.slots[RECEIVER_RIGHT_SLOT].controller_data[0]) ==
           5555);

    make_packet(packet, 4, 7777, 8888);
    assert(receiver_session_ingest(&session, unknown_mac, packet,
                                   sizeof(packet)) ==
           RECEIVER_INGEST_UNKNOWN_SENDER);
    assert((int16_t)get_u16(
               &session.state.slots[RECEIVER_RIGHT_SLOT].controller_data[0]) ==
           5555);
    assert((int16_t)get_u16(
               &session.state.slots[RECEIVER_LEFT_SLOT].controller_data[0]) ==
           3333);
}

int main(void)
{
    test_packet_decode();
    test_fusion_frame();
    test_usb_and_calibration_protocol();
    test_session_pairing();
    puts("receiver protocol tests passed");
    return 0;
}
