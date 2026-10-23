#pragma once
#include "esp_err.h"

/* Starts the IMU-read task and the stabilization/control task.
 * Call after mpu6050_init(), motors_init(), leds_init(), comm_init()
 * have all already succeeded. */
esp_err_t flight_control_start(void);
