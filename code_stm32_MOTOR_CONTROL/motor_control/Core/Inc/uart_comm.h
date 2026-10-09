#ifndef UART_COMM_H_
#define UART_COMM_H_

#include <stdint.h>
#include <stdbool.h>

/* Định dạng Header */
#define PACKET_HEADER_1         0xAA
#define PACKET_HEADER_2         0x55
#define PACKET_RSP_HEADER_1     0x55
#define PACKET_RSP_HEADER_2     0xAA

/* 
 * Kích thước gói nhận (RX) từ Master:
 * [0xAA, 0x55] [SEQ_ID(1)] [v_ref (4)] [w_ref (4)] [CONTROL_FLAG(1)] [CRC16(2)]
 * Tổng: 2 + 1 + 4 + 4 + 1 + 2 = 14 bytes
 */
#define RX_PACKET_SIZE          14

/* 
 * Kích thước gói phản hồi (TX) lên Master:
 * [0x55, 0xAA] [SEQ_ID(1)] [TIMESTAMP_MS (4)] [x(4)] [y(4)] [theta(4)] [v(4)] [w(4)] 
 * [enc_FL(4)] [enc_FR(4)] [enc_RL(4)] [enc_RR(4)] [STATE(1)] [CRC16(2)]
 * Tổng: 2 + 1 + 4 + 12 + 8 + 16 + 1 + 2 = 46 bytes
 */
#define TX_PACKET_SIZE          46

/* Cấu trúc dữ liệu nhận (đã giải mã) */
typedef struct {
    uint8_t seq_id;
    float v_ref;
    float w_ref;
    uint8_t control_flag;
    bool is_valid; // Cờ báo hiệu gói tin nhận được là hợp lệ (Header và CRC đúng)
} Master_Cmd_Packet_t;

/* Cấu trúc dữ liệu gửi (trước khi đóng gói) */
typedef struct {
    uint8_t seq_id;
    uint32_t timestamp_ms; // Thời gian hệ thống (HAL_GetTick)
    float x;
    float y;
    float theta;
    float linear_v;
    float angular_w;
    int32_t enc_FL; // Tổng xung tích lũy bánh trước trái
    int32_t enc_FR; // Tổng xung tích lũy bánh trước phải
    int32_t enc_RL; // Tổng xung tích lũy bánh sau trái
    int32_t enc_RR; // Tổng xung tích lũy bánh sau phải
    uint8_t state;  // Trạng thái FSM hiện tại
} Robot_Feedback_Packet_t;

/* Tính toán CRC16 (Modbus) */
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length);

/* Giải mã mảng bytes nhận được từ DMA sang Cấu trúc Lệnh */
void Unpack_Master_Command(const uint8_t *rx_buf, Master_Cmd_Packet_t *cmd);

/* Đóng gói Cấu trúc phản hồi thành mảng bytes để truyền DMA */
void Pack_Robot_Feedback(const Robot_Feedback_Packet_t *fb, uint8_t *tx_buf);

#endif /* UART_COMM_H_ */
