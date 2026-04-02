#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_HEADER 0xAA
#define MAX_PAYLOAD_SIZE 64

typedef enum {
    CMD_SET_JOINT_ANGLES = 0x10,
    CMD_SET_JOINT_ANGLE_SINGLE = 0x11,
    CMD_GET_TELEMETRY = 0x20,
    CMD_SYSTEM_RESET = 0x30,
    CMD_CALIBRATE_IMU = 0x31,
    CMD_SET_PID_GAINS = 0x40,
    CMD_SET_MODE = 0x50
} CommandType_t;

typedef enum {
    TEL_FULL = 0x01,
    TEL_ANGLES_ONLY = 0x02,
    TEL_IMU_ONLY = 0x03,
    TEL_ERROR = 0xF0,
    TEL_ACK = 0xF1
} TelemetryType_t;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t data[MAX_PAYLOAD_SIZE];
} Packet_t;

uint8_t Protocol_CRC8(const uint8_t *data, uint16_t len);
bool Protocol_DecodePacket(const uint8_t *raw_buffer, uint16_t len, Packet_t *packet);
uint16_t Protocol_EncodePacket(const Packet_t *packet, uint8_t *output_buffer);

#endif // PROTOCOL_H
