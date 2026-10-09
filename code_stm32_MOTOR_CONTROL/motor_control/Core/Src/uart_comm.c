#include "uart_comm.h"
#include <string.h>

/* -------------------------------------------------------------
 * CÁC HÀM TIỆN ÍCH COPY BỘ NHỚ (Đảm bảo Little-Endian)
 * Cấu trúc ARM trên STM32 mặc định là Little-Endian nên việc dùng 
 * memcpy ở đây hoàn toàn an toàn và nhanh gọn.
 * ------------------------------------------------------------- */
static void float_to_bytes(float val, uint8_t *buf) {
    memcpy(buf, &val, 4);
}

static float bytes_to_float(const uint8_t *buf) {
    float val;
    memcpy(&val, buf, 4);
    return val;
}

static void uint32_to_bytes(uint32_t val, uint8_t *buf) {
    memcpy(buf, &val, 4);
}

static void int32_to_bytes(int32_t val, uint8_t *buf) {
    memcpy(buf, &val, 4);
}

/* -------------------------------------------------------------
 * HÀM TÍNH TOÁN CRC16 (Chuẩn Modbus)
 * Đa thức: 0xA001
 * ------------------------------------------------------------- */
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* -------------------------------------------------------------
 * GIẢI MÃ GÓI NHẬN TỪ MASTER
 * ------------------------------------------------------------- */
void Unpack_Master_Command(const uint8_t *rx_buf, Master_Cmd_Packet_t *cmd) {
    cmd->is_valid = false;
    
    // 1. Kiểm tra 2 bytes Header
    if (rx_buf[0] != PACKET_HEADER_1 || rx_buf[1] != PACKET_HEADER_2) {
        return; 
    }
    
    // 2. Tính và đối chiếu CRC16 (Tính cho toàn bộ payload ngoại trừ 2 bytes CRC cuối)
    uint16_t received_crc = rx_buf[RX_PACKET_SIZE - 2] | (rx_buf[RX_PACKET_SIZE - 1] << 8); // Dịch Little-Endian
    uint16_t calculated_crc = Calculate_CRC16(rx_buf, RX_PACKET_SIZE - 2);
    
    if (received_crc != calculated_crc) {
        return; // CRC sai, bỏ qua gói tin này
    }
    
    // 3. Trích xuất dữ liệu nếu CRC đúng
    cmd->seq_id = rx_buf[2];
    cmd->v_ref = bytes_to_float(&rx_buf[3]);
    cmd->w_ref = bytes_to_float(&rx_buf[7]);
    cmd->control_flag = rx_buf[11];
    cmd->is_valid = true;
}

/* -------------------------------------------------------------
 * ĐÓNG GÓI DỮ LIỆU ĐỂ PHẢN HỒI LÊN MASTER
 * ------------------------------------------------------------- */
void Pack_Robot_Feedback(const Robot_Feedback_Packet_t *fb, uint8_t *tx_buf) {
    // 1. Chèn Header
    tx_buf[0] = PACKET_RSP_HEADER_1;
    tx_buf[1] = PACKET_RSP_HEADER_2;
    
    // 2. Chèn Payload
    tx_buf[2] = fb->seq_id;
    uint32_to_bytes(fb->timestamp_ms, &tx_buf[3]);
    
    float_to_bytes(fb->x, &tx_buf[7]);
    float_to_bytes(fb->y, &tx_buf[11]);
    float_to_bytes(fb->theta, &tx_buf[15]);
    
    float_to_bytes(fb->linear_v, &tx_buf[19]);
    float_to_bytes(fb->angular_w, &tx_buf[23]);
    
    int32_to_bytes(fb->enc_FL, &tx_buf[27]);
    int32_to_bytes(fb->enc_FR, &tx_buf[31]);
    int32_to_bytes(fb->enc_RL, &tx_buf[35]);
    int32_to_bytes(fb->enc_RR, &tx_buf[39]);
    
    tx_buf[43] = fb->state;
    
    // 3. Tính và chèn CRC16
    uint16_t crc = Calculate_CRC16(tx_buf, TX_PACKET_SIZE - 2);
    tx_buf[44] = (uint8_t)(crc & 0xFF);         // LSB (Little Endian)
    tx_buf[45] = (uint8_t)((crc >> 8) & 0xFF);  // MSB
}
