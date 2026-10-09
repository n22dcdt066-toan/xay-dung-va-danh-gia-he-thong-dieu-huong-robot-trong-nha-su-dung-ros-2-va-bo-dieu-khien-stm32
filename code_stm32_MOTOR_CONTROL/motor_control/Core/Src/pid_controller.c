#include "pid_controller.h"

void PID_Init(PID_Controller_t *pid, float kp, float ki, float dt, float u_max, float u_min) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->dt = dt;
    pid->u_max = u_max;
    pid->u_min = u_min;
    pid->integral = 0.0f;
}

float PID_Compute(PID_Controller_t *pid, float ref_val, float actual_val) {
    float error = ref_val - actual_val;
    
    // Ước lượng giá trị tích phân tiếp theo (Hình chữ nhật)
    float integral_candidate = pid->integral + pid->Ki * pid->dt * error;
    
    // Tín hiệu điều khiển ngõ ra (chưa bão hòa)
    float u0 = pid->Kp * error + integral_candidate;
    
    float u_out;
    
    // Xử lý Bão hòa (Saturation) & Chống bão hòa tích phân (Anti-windup)
    if (u0 > pid->u_max) {
        u_out = pid->u_max;
        // Chỉ tiếp tục tích lũy I nếu sai lệch làm giảm sự bão hòa
        if (error < 0.0f) {
            pid->integral = integral_candidate;
        }
    } else if (u0 < pid->u_min) {
        u_out = pid->u_min;
        // Chỉ tiếp tục tích lũy I nếu sai lệch làm giảm sự bão hòa
        if (error > 0.0f) {
            pid->integral = integral_candidate;
        }
    } else {
        // Trong vùng tuyến tính
        u_out = u0;
        pid->integral = integral_candidate;
    }
    
    return u_out;
}

void PID_Reset(PID_Controller_t *pid) {
    pid->integral = 0.0f;
}
