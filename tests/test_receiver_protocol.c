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

static void test_hades_frame(void)
{
    receiver_state_t state;
    receiver_state_init(&state);

    uint8_t frame[HADES_UART_FRAME_SIZE];
    receiver_build_hades_frame(&state, 0xffU, frame);
    assert(frame[0] == 0xaaU);
    assert(frame[1] == 0xffU);
    assert(frame[2] == HADES_UART_REPORT_SIZE);
    assert(frame[3] == 1U);
    assert(frame[4] == 2U);
    assert(get_u16(&frame[5]) == 32767U);
    assert(get_u16(&frame[34]) == 32767U);
    assert(get_u16(&frame[32]) == 0x0010U);
    assert(get_u16(&frame[61]) == 0x0010U);
    for (size_t i = 63; i < HADES_UART_FRAME_SIZE; ++i) {
        assert(frame[i] == 0);
    }

    uint8_t packet[TRACKER_PACKET_SIZE];
    make_packet(packet, 7, 12345, -2345);
    receiver_sample_t sample;
    assert(receiver_packet_decode(packet, sizeof(packet), &sample));
    receiver_state_apply_sample(&state, RECEIVER_RIGHT_SLOT, &sample);
    receiver_build_hades_frame(&state, 0U, frame);
    assert(frame[1] == 0U);
    assert((int16_t)get_u16(&frame[5]) == 12345);
    assert((int16_t)get_u16(&frame[13]) == -2345);
    assert(get_u16(&frame[34]) == 32767U);

    make_packet(packet, 8, -1234, 678);
    assert(receiver_packet_decode(packet, sizeof(packet), &sample));
    receiver_state_apply_sample(&state, RECEIVER_LEFT_SLOT, &sample);
    receiver_build_hades_frame(&state, 1U, frame);
    assert((int16_t)get_u16(&frame[5]) == 12345);
    assert((int16_t)get_u16(&frame[34]) == -1234);
    assert((int16_t)get_u16(&frame[42]) == 678);
}

int main(void)
{
    test_packet_decode();
    test_hades_frame();
    puts("receiver protocol tests passed");
    return 0;
}
