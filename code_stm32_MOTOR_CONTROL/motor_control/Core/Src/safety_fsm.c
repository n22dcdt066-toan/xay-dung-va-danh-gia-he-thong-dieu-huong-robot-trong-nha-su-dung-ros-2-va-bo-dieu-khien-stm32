#include "safety_fsm.h"

void FSM_Init(Safety_FSM_t *fsm) {
    fsm->state = STATE_INIT;
    fsm->last_cmd_tick = 0;
}

void FSM_Update(Safety_FSM_t *fsm, uint32_t current_tick_ms) {
    switch (fsm->state) {
        case STATE_INIT:
            // Hệ thống tự động chuyển sang READY sau khi Setup hoàn tất
            fsm->state = STATE_READY;
            break;
            
        case STATE_READY:
            // Đang chờ lệnh đầu tiên để chạy. Watchdog không tác động ở đây.
            break;
            
        case STATE_RUNNING:
            // Kiểm tra timeout
            if ((current_tick_ms - fsm->last_cmd_tick) > TIMEOUT_CMD_MS) {
                // Quá hạn không nhận được lệnh từ Master
                fsm->state = STATE_FAULT_STOP;
            }
            break;
            
        case STATE_FAULT_STOP:
            // Trạng thái chốt chặn (Latch). Chỉ thoát khi Master gửi cờ Reset.
            break;
    }
}

void FSM_Feed_Watchdog(Safety_FSM_t *fsm, uint32_t current_tick_ms) {
    fsm->last_cmd_tick = current_tick_ms;
    
    // Nếu đang ở READY và nhận được lệnh -> Cho phép chạy
    if (fsm->state == STATE_READY) {
        fsm->state = STATE_RUNNING;
    }
}

void FSM_Trigger_Fault(Safety_FSM_t *fsm) {
    fsm->state = STATE_FAULT_STOP;
}

void FSM_Reset_Fault(Safety_FSM_t *fsm) {
    // Chỉ cho phép khôi phục về READY, không chạy ngay lập tức
    if (fsm->state == STATE_FAULT_STOP) {
        fsm->state = STATE_READY;
    }
}

bool FSM_Is_Running(Safety_FSM_t *fsm) {
    return (fsm->state == STATE_RUNNING);
}
