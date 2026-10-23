#pragma once
#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float accel_x_g, accel_y_g, accel_z_g;   // in units of g
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; // in degrees/sec
    float temp_c;
} mpu6050_data_t;

/* Initializes I2C bus + wakes/configures the MPU6050. */
esp_err_t mpu6050_init(void);

/* Reads accel+gyro+temp, converts to physical units, applies the
 * gyro bias computed by mpu6050_calibrate_gyro(). */
esp_err_t mpu6050_read(mpu6050_data_t *out);

/* Averages `num_samples` gyro readings while the drone must be held
 * completely still, and stores the result as the zero-rate bias that
 * mpu6050_read() will subtract from every future reading. Call this
 * once at boot before arming is allowed. */
esp_err_t mpu6050_calibrate_gyro(int num_samples);
