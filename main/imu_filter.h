#pragma once
#include "mpu6050.h"

typedef struct {
    float roll_deg;
    float pitch_deg;
} attitude_t;

/* Resets the filter's internal angle state to 0. Call once at boot
 * (ideally right after the drone has been sitting flat and still). */
void imu_filter_reset(void);

/* Feed one new IMU sample in, get the updated attitude estimate out.
 * dt_s is the time in seconds since the previous call. */
attitude_t imu_filter_update(const mpu6050_data_t *sample, float dt_s);
