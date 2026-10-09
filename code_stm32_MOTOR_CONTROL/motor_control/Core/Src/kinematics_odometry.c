#include "kinematics_odometry.h"

void Odometry_Init(Robot_Odometry_t *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->linear_v = 0.0f;
    odom->angular_w = 0.0f;
}

void Kinematics_Inverse(float v_ref, float w_ref, float *v_ref_FL, float *v_ref_FR, float *v_ref_RL, float *v_ref_RR) {
    float v_L = v_ref - (ROBOT_TRACK_WIDTH / 2.0f) * w_ref;
    float v_R = v_ref + (ROBOT_TRACK_WIDTH / 2.0f) * w_ref;

    // Phân phối vận tốc cho 4 bánh (4WD Skid-steer)
    *v_ref_FL = v_L;
    *v_ref_RL = v_L;
    *v_ref_FR = v_R;
    *v_ref_RR = v_R;
}

static float Wrap_Angle(float angle) {
    while (angle > MATH_PI) angle -= 2.0f * MATH_PI;
    while (angle <= -MATH_PI) angle += 2.0f * MATH_PI;
    return angle;
}

void Odometry_Update(Robot_Odometry_t *odom, int16_t delta_FL, int16_t delta_FR, int16_t delta_RL, int16_t delta_RR) {
    // 1. Tính quãng đường lăn của từng bánh xe (m)
    // s = (2*pi*R / N) * delta_ticks
    float k_s = (2.0f * MATH_PI * ROBOT_WHEEL_RADIUS) / (float)ENCODER_RESOLUTION;
    
    float s_FL = k_s * (float)delta_FL;
    float s_FR = k_s * (float)delta_FR;
    float s_RL = k_s * (float)delta_RL;
    float s_RR = k_s * (float)delta_RR;

    // 2. Tổng hợp phản hồi hai phía (Trung bình cộng)
    float s_L = (s_FL + s_RL) / 2.0f;
    float s_R = (s_FR + s_RR) / 2.0f;

    // 3. Ước lượng dịch chuyển trung bình của thân xe
    float delta_s = (s_R + s_L) / 2.0f;
    float delta_theta = (s_R - s_L) / ROBOT_TRACK_WIDTH;

    // 4. Tính vận tốc thực tế của thân xe
    odom->linear_v = delta_s / CONTROL_DT;
    odom->angular_w = delta_theta / CONTROL_DT;

    // 5. Cập nhật vị trí toàn cục theo xấp xỉ điểm giữa (Midpoint Approximation)
    odom->x += delta_s * cosf(odom->theta + delta_theta / 2.0f);
    odom->y += delta_s * sinf(odom->theta + delta_theta / 2.0f);
    odom->theta = Wrap_Angle(odom->theta + delta_theta);
}

void Encoder_Update(int32_t *total_ticks, int32_t *delta_ticks, uint32_t current_tim_count, uint32_t *last_tim_count) {
    // Ép kiểu sang int32_t để xử lý đúng vấn đề tràn (overflow/underflow) của thanh ghi Timer 16/32-bit
    *delta_ticks = (int32_t)(current_tim_count - *last_tim_count);
    *last_tim_count = current_tim_count;
    *total_ticks += *delta_ticks; // Cập nhật tổng xung liên tục
}
