#ifndef SAFETY_FSM_H_
#define SAFETY_FSM_H_

#include <stdint.h>
#include <stdbool.h>

/* Thời gian tối đa (ms) không nhận được lệnh từ Master trước khi dừng xe */
#define TIMEOUT_CMD_MS   (300) 

typedef enum {
    STATE_INIT = 0,     // Đang khởi tạo hệ thống
    STATE_READY,        // Đã khởi tạo xong, chờ lệnh điều khiển
    STATE_RUNNING,      // Đang chạy bình thường
    STATE_FAULT_STOP    // Lỗi (mất kết nối, E-Stop...), đã ngắt PWM
} Robot_State_t;

typedef struct {
    Robot_State_t state;
    uint32_t last_cmd_tick; // Dấu thời gian (HAL_GetTick) nhận lệnh cuối
} Safety_FSM_t;

/* Khởi tạo FSM */
void FSM_Init(Safety_FSM_t *fsm);

/* Được gọi liên tục trong ControlTask để giám sát timeout */
void FSM_Update(Safety_FSM_t *fsm, uint32_t current_tick_ms);

/* Được gọi khi nhận được gói tin UART hợp lệ (cập nhật watchdog) */
void FSM_Feed_Watchdog(Safety_FSM_t *fsm, uint32_t current_tick_ms);

/* Chuyển ngay sang trạng thái lỗi (ví dụ nhận lệnh E-Stop hoặc lỗi phần cứng) */
void FSM_Trigger_Fault(Safety_FSM_t *fsm);

/* Master gửi lệnh Reset/Acknowledge để khôi phục hoạt động */
void FSM_Reset_Fault(Safety_FSM_t *fsm);

/* Kiểm tra xem hệ thống có được phép xuất PWM ra động cơ hay không */
bool FSM_Is_Running(Safety_FSM_t *fsm);

#endif /* SAFETY_FSM_H_ */
