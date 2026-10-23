#pragma once
#include "esp_err.h"

esp_err_t motors_init(void);

/* Sets one motor's speed as a 0.0 - 1.0 fraction of max duty.
 * Values are clamped internally, so out-of-range input is safe. */
void motors_set(int motor_index /* 0..3 */, float duty_fraction);

/* Convenience: set all four at once (index order 0=M1,1=M2,2=M3,3=M4) */
void motors_set_all(float m1, float m2, float m3, float m4);

/* Immediately cuts all motors to zero. Called by the failsafe/disarm
 * path - safe to call from any task. */
void motors_stop_all(void);
