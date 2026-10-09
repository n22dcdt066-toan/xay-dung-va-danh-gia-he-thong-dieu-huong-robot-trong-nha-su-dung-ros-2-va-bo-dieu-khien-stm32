#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <Arduino.h>

#define TX_PACKET_SIZE 14
#define RX_PACKET_SIZE 46

// Cấu trúc gói tin gửi từ ESP32 xuống STM32 (14 bytes)
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];      // [0xAA, 0x55]
    uint8_t seq_id;         // Sequence ID
    float v_ref;            // Vận tốc thẳng (m/s)
    float w_ref;            // Vận tốc góc (rad/s)
    uint8_t control_flag;   // Bit 0: Cho phép chạy, Bit 1: E-Stop, Bit 2: Reset Odom
    uint16_t crc16;         // CRC16 Modbus
} Master_Cmd_Packet_t;

// Cấu trúc gói tin nhận từ STM32 lên ESP32 (46 bytes)
typedef struct {
    uint8_t header[2];      // [0x55, 0xAA]
    uint8_t seq_id;
    uint32_t timestamp_ms;
    float x;
    float y;
    float theta;
    float linear_v;
    float angular_w;
    int32_t enc_FL;
    int32_t enc_FR;
    int32_t enc_RL;
    int32_t enc_RR;
    uint8_t state;
    uint16_t crc16;
} Robot_Feedback_Packet_t;
#pragma pack(pop)

// Hàm tính CRC16 Modbus
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

#endif
