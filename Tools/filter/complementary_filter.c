#include "complementary_filter.h"

static float alpha = 0.98f;      // 互补滤波权重（陀螺仪信任度）
static float pitch_angle = 0.0f; // 滤波后的俯仰角（度）
static float roll_angle = 0.0f;  // 滤波后的横滚角（度）
static float yaw_angle = 0.0f;   // 滤波后的偏航角（度）

/**
 * @brief 从加速度计数据计算俯仰角和横滚角
 * @param ax, ay, az 加速度计原始值（已转换为 g 单位）
 * @param pitch_acc 输出俯仰角（度）
 * @param roll_acc 输出横滚角（度）
 */
static void accel_to_angle(float ax, float ay, float az, float *pitch_acc, float *roll_acc)
{
    // 避免分母为零
    if (az == 0.0f) az = 1e-6f;

    // 横滚角：绕 X 轴旋转，根据 Y 和 Z 分量
    *roll_acc = atan2f(ay, az) * (180.0f / PI);

    // 俯仰角：绕 Y 轴旋转，根据 X 和 水平分量
    float horiz = sqrtf(ay * ay + az * az);
    if (horiz == 0.0f) horiz = 1e-6f;
    *pitch_acc = atan2f(-ax, horiz) * (180.0f / PI);
}

/**
 * @brief 互补滤波器更新，计算欧拉角
 * @param ax, ay, az 加速度计原始值（已转换为 g 单位）
 * @param gx_deg, gy_deg, gz_deg 陀螺仪原始值（度/秒）
 * @param dt 采样时间间隔（秒）
 * @param pitch 输出俯仰角（度）
 * @param roll  输出横滚角（度）
 * @param yaw   输出偏航角（度）
 */
void Complementary_Update(float ax, float ay, float az,
                          float gx_deg, float gy_deg, float gz_deg,
                          float dt, float *pitch, float *roll, float *yaw)
{
    float pitch_acc, roll_acc;
    accel_to_angle(ax, ay, az, &pitch_acc, &roll_acc);

    float pitch_gyro = pitch_angle + gy_deg * dt;
    float roll_gyro  = roll_angle  + gx_deg * dt;
    float yaw_gyro   = yaw_angle   + gz_deg * dt;

    pitch_angle = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    roll_angle  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    yaw_angle   = yaw_gyro;

    *pitch = pitch_angle;
    *roll  = roll_angle;
    *yaw   = yaw_angle;
}
