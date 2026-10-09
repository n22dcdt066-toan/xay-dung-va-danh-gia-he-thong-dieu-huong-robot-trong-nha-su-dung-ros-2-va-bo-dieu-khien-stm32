#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "uart_protocol.h"
#include "web_page.h"

// Cấu hình Wi-Fi (Chế độ phát Access Point)
const char* ssid = "test123";
const char* password = "012761tan";

WebServer server(80);

// Biến trạng thái mô phỏng & quỹ đạo
enum TestMode { MODE_IDLE, MODE_P1, MODE_P2, MODE_P3, MODE_P4 };
TestMode current_mode = MODE_IDLE;
uint32_t mode_start_time = 0;

bool fault_timeout = false;
bool fault_crc = false;
bool fault_jitter = false;
bool e_stop = false;
bool send_stop_reset = false; // Cờ gửi Reset 1 lần khi bấm Tạm dừng để thoát FAULT_STOP

// Dữ liệu truyền/nhận
Master_Cmd_Packet_t cmd_pkt;
Robot_Feedback_Packet_t fb_pkt;
uint8_t seq_counter = 0;
String telemetry_json = "{}";

// Khai báo HardwareSerial (UART1 cho STM32)
HardwareSerial STM32_Serial(1);
#define RXD1 18
#define TXD1 17

// --- Hàm xử lý HTTP ---
void handleRoot() {
    server.send(200, "text/html", index_html);
}

void handleCommand() {
    if (server.hasArg("cmd")) {
        String message = server.arg("cmd");
        Serial.println("Nhận lệnh từ Web: " + message);

        if (message == "CMD_RUN_P1") { current_mode = MODE_P1; mode_start_time = millis(); fault_timeout = fault_jitter = fault_crc = false; e_stop = false; }
        else if (message == "CMD_RUN_P2") { current_mode = MODE_P2; mode_start_time = millis(); fault_timeout = fault_jitter = fault_crc = false; e_stop = false; }
        else if (message == "CMD_RUN_P3") { current_mode = MODE_P3; mode_start_time = millis(); fault_timeout = fault_jitter = fault_crc = false; e_stop = false; }
        else if (message == "CMD_RUN_P4") { current_mode = MODE_P4; mode_start_time = millis(); fault_timeout = fault_jitter = fault_crc = false; e_stop = false; }
        else if (message == "CMD_STOP") {
            current_mode = MODE_IDLE;
            e_stop = false;
            /* FIX: Xóa fault_timeout để task tiếp tục gửi gói v=0 xuống STM32.
             * Đồng thời bật cờ gửi Reset 1 lần để thoát FAULT_STOP nếu đang bị ở đó. */
            fault_timeout = false;
            send_stop_reset = true;
        }
        else if (message == "CMD_RESET") { current_mode = MODE_IDLE; e_stop = false; fault_timeout = false; fault_crc = false; fault_jitter = false; }
        else if (message == "FAULT_TIMEOUT") { fault_timeout = true; Serial.println("MÔ PHỎNG: Lỗi Mất Kết Nối!"); }
        else if (message == "FAULT_CRC") { fault_crc = true; Serial.println("MÔ PHỎNG: Lỗi sai CRC16!"); }
        else if (message == "FAULT_JITTER") { fault_jitter = true; Serial.println("MÔ PHỎNG: Lỗi Jitter (Độ trễ)!"); }
        else if (message == "FAULT_ESTOP") { e_stop = true; Serial.println("MÔ PHỎNG: Khẩn cấp E-STOP!"); }
        
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleTelemetry() {
    server.send(200, "application/json", telemetry_json);
}

// --- Luồng xử lý quỹ đạo (20Hz = 50ms) ---
void TrajectoryTask(void *pvParameters) {
    for (;;) {
        uint32_t t = millis() - mode_start_time;
        float v_req = 0.0f;
        float w_req = 0.0f;
        uint8_t ctrl_flag = 0x00; // Mặc định: không có flag đặc biệt

        if (e_stop) {
            ctrl_flag = 0x02; // Bit 1: E-STOP khẩn cấp
        } else if (send_stop_reset) {
            /* FIX: Gửi Bit 0 (Reset Fault) 1 lần sau khi CMD_STOP được bấm,
             * giúp STM32 thoát FAULT_STOP về READY với v=0,w=0 */
            ctrl_flag = 0x01;
            send_stop_reset = false; // Chỉ gửi 1 lần
        } else if (current_mode != MODE_IDLE) {
            ctrl_flag = 0x01; // Đang chạy: Bit 0 = Allow run
        }
        // MODE_IDLE bình thường: ctrl_flag = 0x00

        // Tính v_req / w_req theo kịch bản (chỉ khi không E-STOP)
        if (!e_stop) {
            switch(current_mode) {
                case MODE_P1: // Chạy thẳng 2m (Khoảng 10s tại 0.2m/s)
                    if (t < 2000) { v_req = 0.1f * (t / 2000.0f); } // Tăng tốc mượt
                    else if (t < 10000) { v_req = 0.2f; }
                    else if (t < 12000) { v_req = 0.2f * (1.0f - (t - 10000)/2000.0f); }
                    else { v_req = 0; current_mode = MODE_IDLE; }
                    break;
                case MODE_P2: // Quay tại chỗ
                    if (t < 10000) w_req = 0.5f; // Quay phải 10s
                    else if (t < 20000) w_req = -0.5f; // Quay trái 10s
                    else { w_req = 0; current_mode = MODE_IDLE; }
                    break;
                case MODE_P3: // Step response
                    v_req = 0.5f; // Nhảy ngay lập tức lên 0.5m/s
                    break;
                case MODE_P4: // S-Curve Sinusoidal
                    v_req = 0.3f * sin(t * 0.001f);
                    w_req = 0.2f * cos(t * 0.001f);
                    break;
                default: // MODE_IDLE
                    v_req = 0; w_req = 0;
            }
        }

        // Đóng gói lệnh
        cmd_pkt.header[0] = 0xAA;
        cmd_pkt.header[1] = 0x55;
        cmd_pkt.seq_id = seq_counter++;
        cmd_pkt.v_ref = v_req;
        cmd_pkt.w_ref = w_req;
        cmd_pkt.control_flag = ctrl_flag;
        
        // Mô phỏng lỗi CRC
        if (fault_crc) {
            cmd_pkt.crc16 = 0x0000; // Cố tình ghi sai CRC
        } else {
            cmd_pkt.crc16 = Modbus_CRC16((uint8_t*)&cmd_pkt, TX_PACKET_SIZE - 2);
        }

        // Truyền UART (Có thể bị ngắt bởi Jitter hoặc Mất kết nối)
        if (!fault_timeout) {
            if (fault_jitter) {
                vTaskDelay(pdMS_TO_TICKS(400)); // Cố tình tạo độ trễ cực cao để xem STM32 xử lý sao
                fault_jitter = false; // Chỉ làm 1 lần
            }
            STM32_Serial.write((uint8_t*)&cmd_pkt, TX_PACKET_SIZE);
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Chu kỳ 50ms (20Hz)
    }
}

// --- Luồng nhận dữ liệu phản hồi (Polling / Queue) ---
void UartReceiveTask(void *pvParameters) {
    uint8_t rx_buf[RX_PACKET_SIZE];
    uint8_t rx_index = 0;

    for (;;) {
        while (STM32_Serial.available() > 0) {
            uint8_t c = STM32_Serial.read();
            if (rx_index == 0 && c != 0x55) continue;
            if (rx_index == 1 && c != 0xAA) { rx_index = 0; continue; }
            
            rx_buf[rx_index++] = c;
            
            if (rx_index == RX_PACKET_SIZE) {
                // Nhận đủ gói, kiểm tra CRC
                uint16_t cal_crc = Modbus_CRC16(rx_buf, RX_PACKET_SIZE - 2);
                uint16_t rx_crc = rx_buf[RX_PACKET_SIZE - 2] | (rx_buf[RX_PACKET_SIZE - 1] << 8);
                
                if (cal_crc == rx_crc) {
                    memcpy(&fb_pkt, rx_buf, RX_PACKET_SIZE);
                    
                    // Tạo chuỗi JSON
                    char json[256];
                    snprintf(json, sizeof(json), 
                        "{\"seq\":%d, \"time\":%lu, \"x\":%.3f, \"y\":%.3f, \"theta\":%.3f, "
                        "\"v\":%.3f, \"w\":%.3f, \"efl\":%d, \"efr\":%d, \"erl\":%d, \"err\":%d, \"state\":%d}",
                        fb_pkt.seq_id, fb_pkt.timestamp_ms, fb_pkt.x, fb_pkt.y, fb_pkt.theta,
                        fb_pkt.linear_v, fb_pkt.angular_w, fb_pkt.enc_FL, fb_pkt.enc_FR, fb_pkt.enc_RL, fb_pkt.enc_RR, fb_pkt.state);
                    
                    telemetry_json = String(json);

                    // Xuất Serial Monitor (Định dạng CSV cho dễ plot)
                    Serial.printf("%lu, %d, %.3f, %.3f, %.3f, %.3f, %.3f, %d, %d, %d, %d, %d\n",
                        fb_pkt.timestamp_ms, fb_pkt.seq_id, fb_pkt.x, fb_pkt.y, fb_pkt.theta, 
                        fb_pkt.linear_v, fb_pkt.angular_w, fb_pkt.enc_FL, fb_pkt.enc_FR, fb_pkt.enc_RL, fb_pkt.enc_RR, fb_pkt.state);
                }
                rx_index = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Chờ 5ms để nhường CPU
    }
}

void setup() {
    Serial.begin(115200);
    STM32_Serial.begin(115200, SERIAL_8N1, RXD1, TXD1);

    // Phát Wi-Fi
    WiFi.softAP(ssid, password);
    Serial.println("Wi-Fi AP Started: " + String(ssid));
    Serial.println("IP Address: " + WiFi.softAPIP().toString());

    // Cài đặt Web Server
    server.on("/", HTTP_GET, handleRoot);
    server.on("/command", HTTP_GET, handleCommand);
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.begin();
    Serial.println("Web Server Started.");

    // Tạo các Task trên Core 1 (Core 0 dành cho WiFi)
    xTaskCreatePinnedToCore(TrajectoryTask, "TrajTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(UartReceiveTask, "UartRxTask", 4096, NULL, 3, NULL, 1);

    Serial.println("ESP32-S3 Master Test Harness is READY!");
    Serial.println("Timestamp_ms, Seq, X, Y, Theta, V_real, W_real, Enc_FL, Enc_FR, Enc_RL, Enc_RR, State");
}

void loop() {
    server.handleClient();
    delay(10); // delay() của Arduino gọi vTaskDelay dưới nền
}
