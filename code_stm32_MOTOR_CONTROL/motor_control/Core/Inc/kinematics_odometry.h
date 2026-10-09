#ifndef KINEMATICS_ODOMETRY_H_
#define KINEMATICS_ODOMETRY_H_

#include <stdint.h>
#include <math.h>

/* ========================================================
 * THÔNG SỐ CƠ KHÍ CỦA ROBOT
 * (Điều chỉnh theo kích thước thực tế)
 * ======================================================== */
#define ROBOT_WHEEL_RADIUS      (0.0325f)  // Bán kính bánh xe ri (m) - ví dụ 65mm đường kính
#define ROBOT_TRACK_WIDTH       (0.150f)   // Khoảng cách 2 bánh hiệu dụng be (m)

/* ========================================================
 * THÔNG SỐ ENCODER & ĐỘNG CƠ (L-Type 520, 1:40)
 * ======================================================== */
#define MOTOR_PPR               (11)       // Xung cơ bản
#define ENCODER_MULTI           (4)        // Chế độ Quadrature 4x của Timer
#define GEAR_RATIO              (40)       // Tỷ số truyền 1:40
#define ENCODER_RESOLUTION      (MOTOR_PPR * ENCODER_MULTI * GEAR_RATIO) // 1760 xung/vòng bánh

/* ========================================================
 * THÔNG SỐ ĐIỀU KHIỂN
 * ======================================================== */
#define CONTROL_DT              (0.01f)    // Chu kỳ lấy mẫu Ts = 10ms (100Hz)
#define MATH_PI                 (3.1415926535f)

typedef struct {
    float x;        // Vị trí trục X toàn cục (m)
    float y;        // Vị trí trục Y toàn cục (m)
    float theta;    // Góc định hướng toàn cục (rad), phạm vi [-PI, PI)
    float linear_v; // Vận tốc tịnh tiến thực tế của thân xe (m/s)
    float angular_w;// Vận tốc góc thực tế của thân xe (rad/s)
} Robot_Odometry_t;

/* Khởi tạo bộ nhớ Odometry */
void Odometry_Init(Robot_Odometry_t *odom);

/* Giải thuật Động học nghịch (Inverse Kinematics)
 * Trả về vận tốc mục tiêu (v_ref_*) cho từng bánh (m/s)
 */
void Kinematics_Inverse(float v_ref, float w_ref, float *v_ref_FL, float *v_ref_FR, float *v_ref_RL, float *v_ref_RR);

/* Giải thuật Ước lượng vị trí (Forward Kinematics / Odometry) 
 * Nhận vào sự thay đổi xung (delta_ticks) của 4 bánh trong chu kỳ Ts
 */
void Odometry_Update(Robot_Odometry_t *odom, int16_t delta_FL, int16_t delta_FR, int16_t delta_RL, int16_t delta_RR);

/* Hàm hỗ trợ tính delta ticks và tích lũy tổng số ticks từ thanh ghi CNT của Timer
 * Gọi hàm này trước Odometry_Update cho từng bánh xe.
 */
/* BUG FIX: Dùng uint32_t/int32_t để hỗ trợ đúng cả TIM2/TIM5 (32-bit) và TIM3/TIM4 (16-bit) */
void Encoder_Update(int32_t *total_ticks, int32_t *delta_ticks, uint32_t current_tim_count, uint32_t *last_tim_count);

#endif /* KINEMATICS_ODOMETRY_H_ */
