#include "imu_filter.h"
#include "drone_config.h"
#include <math.h>

static float s_roll = 0.0f;
static float s_pitch = 0.0f;

void imu_filter_reset(void)
{
    s_roll = 0.0f;
    s_pitch = 0.0f;
}

attitude_t imu_filter_update(const mpu6050_data_t *s, float dt_s)
{
    /* Angle estimate purely from accelerometer (noisy, but doesn't
     * drift). Assumes accel_z is roughly "up" when level. */
    float accel_roll_deg  = atan2f(s->accel_y_g, s->accel_z_g) * (180.0f / (float)M_PI);
    float accel_pitch_deg = atan2f(-s->accel_x_g,
                                    sqrtf(s->accel_y_g * s->accel_y_g +
                                          s->accel_z_g * s->accel_z_g)) * (180.0f / (float)M_PI);

    /* Integrate gyro rates (clean short-term, drifts long-term) and
     * blend with the accelerometer estimate to cancel drift. */
    float gyro_roll_deg  = s_roll  + s->gyro_x_dps * dt_s;
    float gyro_pitch_deg = s_pitch + s->gyro_y_dps * dt_s;

    s_roll  = COMP_FILTER_ALPHA * gyro_roll_deg  + (1.0f - COMP_FILTER_ALPHA) * accel_roll_deg;
    s_pitch = COMP_FILTER_ALPHA * gyro_pitch_deg + (1.0f - COMP_FILTER_ALPHA) * accel_pitch_deg;

    attitude_t out = { .roll_deg = s_roll, .pitch_deg = s_pitch };
    return out;
}
