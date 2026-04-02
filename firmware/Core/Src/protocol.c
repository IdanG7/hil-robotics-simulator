#include "protocol.h"
#include <string.h>

uint8_t Protocol_CRC8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool Protocol_DecodePacket(const uint8_t *raw_buffer, uint16_t len, Packet_t *packet) {
    if (len < 4) {
        return false;
    }
    if (raw_buffer[0] != PACKET_HEADER) {
        return false;
    }
    uint8_t type = raw_buffer[1];
    uint8_t length = raw_buffer[2];
    if (length > MAX_PAYLOAD_SIZE || len != (4 + length)) {
        return false;
    }
    uint8_t crc_expected = Protocol_CRC8(&raw_buffer[1], 2 + length);
    uint8_t crc_received = raw_buffer[3 + length];
    if (crc_expected != crc_received) {
        return false;
    }
    packet->type = type;
    packet->length = length;
    if (length > 0) {
        memcpy(packet->data, &raw_buffer[3], length);
    }
    return true;
}

uint16_t Protocol_EncodePacket(const Packet_t *packet, uint8_t *output_buffer) {
    if (packet->length > MAX_PAYLOAD_SIZE) {
        return 0;
    }
    output_buffer[0] = PACKET_HEADER;
    output_buffer[1] = packet->type;
    output_buffer[2] = packet->length;
    if (packet->length > 0) {
        memcpy(&output_buffer[3], packet->data, packet->length);
    }
    uint8_t crc = Protocol_CRC8(&output_buffer[1], 2 + packet->length);
    output_buffer[3 + packet->length] = crc;
    return 4 + packet->length;
}
