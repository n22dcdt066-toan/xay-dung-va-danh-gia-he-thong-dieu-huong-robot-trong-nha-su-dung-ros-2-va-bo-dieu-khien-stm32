#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float dt;           // Thời gian trích mẫu Ts (s)
    
    float u_min;        // Giới hạn dưới của ngõ ra (ví dụ: -100.0%)
    float u_max;        // Giới hạn trên của ngõ ra (ví dụ: 100.0%)
    
    float integral;     // Trạng thái bộ tích phân (I)
} PID_Controller_t;

/* Khởi tạo thông số PID */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float dt, float u_max, float u_min);

/* Tính toán PID với Anti-windup 
 * Trả về tín hiệu điều khiển u[k] (ngõ ra PWM/Velocity)
 */
float PID_Compute(PID_Controller_t *pid, float ref_val, float actual_val);

/* Xóa bộ nhớ tích phân khi dừng/lỗi để tránh giật cục khi chạy lại */
void PID_Reset(PID_Controller_t *pid);

#endif /* PID_CONTROLLER_H_ */
