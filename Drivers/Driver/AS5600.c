#include "AS5600.h"
#include <math.h>

static const float TWO_PI = 6.283185307179586f;
static const float PI = 3.141592653589793f;
static const float RAD_PER_COUNT = 6.283185307179586f / 4096.0f;
static const float ANGLE_FILTER_TAU_S = 0.00025f;

int Sensor_DIR = 1;
int Motor_PP = 7;
extern float electrical_offset;

/* Integer increments retain every encoder count, regardless of run time. */
static int64_t total_counts;
static uint16_t last_raw;
static uint8_t first_read = 1U;
static float wrapped_angle;
static float mechanical_acceleration;    /* 机械角加速度（rad/s²） */

static float WrapAngle(float angle)
{
    angle = fmodf(angle, TWO_PI);
    if (angle < 0.0f) angle += TWO_PI;
    return angle;
}

static float AngleDifference(float a, float b)
{
    float difference = a - b;
    if (difference > PI) difference -= TWO_PI;
    if (difference < -PI) difference += TWO_PI;
    return difference;
}

float AS5600_GetWrappedAngle(void)
{
    return wrapped_angle;
}

/********************************************
	读取机械角加速度
	功能：返回最近一次成功采样计算的角加速度
	输入：无
	输出：角加速度（rad/s²），由现有滤波速度差分得到
*******************************************/
float AS5600_GetAcceleration(void)
{
    return mechanical_acceleration;
}

/* Input must be the single-turn angle, never the growing position value. */
float FOC_GetElectricalAngle(float mechanical_angle)
{
    return WrapAngle(Sensor_DIR * Motor_PP * mechanical_angle - electrical_offset);
}

uint8_t as5600_read_byte(float *angle, float *mechanical_speed, float dt)
{
    uint8_t data[2];
    uint16_t raw;
    int32_t delta;
    float measurement;
    float angle_alpha;
    float speed_alpha;
    float raw_speed;
    float previous_speed;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c2, AS5600_ADDR, AS5600_ANGLE_REG, I2C_MEMADD_SIZE_8BIT,
        data, 2, 10);

    if (status != HAL_OK) return (uint8_t)status;
    if (dt <= 0.0f) dt = 0.001f;
    raw = (((uint16_t)data[0] << 8) | data[1]) & 0x0FFFU;
    measurement = (float)raw * RAD_PER_COUNT;

    if (first_read)
    {
        last_raw = raw;
        wrapped_angle = measurement;
        total_counts = 0;
        first_read = 0U;
        *angle = 0.0f;
        *mechanical_speed = 0.0f;
        mechanical_acceleration = 0.0f;
        return HAL_OK;
    }

    delta = (int32_t)raw - (int32_t)last_raw;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    total_counts += delta;
    last_raw = raw;

    /* Apply the same small LPF on a bounded circular coordinate.
     * The shortest angular difference handles the 0/2pi boundary.
     */
    angle_alpha = dt / (ANGLE_FILTER_TAU_S + dt);
    wrapped_angle = WrapAngle(wrapped_angle + angle_alpha *
                              AngleDifference(measurement, wrapped_angle));

    /* Convert the exact count sum afresh. Float position may lose resolution
     * at very large travel, but its rounding cannot affect commutation.
     * Keep the same small filter lag in position feedback.
     */
    *angle = (float)((double)total_counts * (6.283185307179586 / 4096.0)) +
             AngleDifference(wrapped_angle, measurement);
    raw_speed = (float)delta * RAD_PER_COUNT / dt;
    speed_alpha = dt / (0.02f + dt);
    /* 在原有速度更新前后取差值，不改变角度和速度计算。 */
    previous_speed = *mechanical_speed;
    *mechanical_speed += speed_alpha * (raw_speed - *mechanical_speed);
    mechanical_acceleration = (*mechanical_speed - previous_speed) / dt;
    return HAL_OK;
}
