#pragma once
#include <stdint.h>

/* =========================================================================
 * protocol.h
 *
 * Wire format for the UDP control packet sent from the ground-station
 * Python script (reading the RadioMaster Zorro over USB) to the drone.
 *
 * This layout is mirrored exactly in python_controller/protocol.py -
 * if you change one, you MUST change the other, and both structs must
 * stay packed / byte-for-byte identical.
 *
 * All multi-byte fields are little-endian (native on both ESP32-C6 and
 * x86/ARM ground stations, so no byte-swapping is done).
 * ========================================================================= */

#define DRONE_PROTOCOL_MAGIC   0x4E4F5244u  // "DRON" little-endian

typedef struct __attribute__((packed)) {
    uint32_t magic;       // must equal DRONE_PROTOCOL_MAGIC
    uint16_t seq;         // incrementing sequence number
    uint16_t throttle;    // 0..1000
    int16_t  roll;        // -500..+500
    int16_t  pitch;       // -500..+500
    int16_t  yaw;         // -500..+500
    uint8_t  arm_switch;  // 0 = disarm request, 1 = arm request
    uint8_t  aux1;        // reserved for future use (flight modes etc.)
    uint16_t checksum;    // sum of all preceding bytes, 16-bit wraparound
} control_packet_t;

#define CONTROL_PACKET_SIZE  (sizeof(control_packet_t))

/* Simple, fast, "good enough over a trusted local link" checksum.
 * This is NOT cryptographic - it exists only to reject garbled/torn
 * UDP payloads, not to authenticate the sender. If you need this
 * control link to be tamper-resistant, add real auth on top. */
static inline uint16_t protocol_checksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

/* Computes and returns the checksum over every field except the
 * checksum field itself. */
static inline uint16_t protocol_compute_packet_checksum(const control_packet_t *pkt)
{
    return protocol_checksum((const uint8_t *)pkt,
                              CONTROL_PACKET_SIZE - sizeof(pkt->checksum));
}
