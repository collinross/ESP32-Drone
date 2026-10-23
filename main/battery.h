#pragma once
#include "esp_err.h"
#include <stdbool.h>

esp_err_t battery_init(void);

/* Reads and averages the ADC, returns pack voltage in volts
 * (already scaled by BATTERY_DIVIDER_RATIO). */
float battery_read_voltage(void);

/* True once voltage has dropped at/below BATTERY_LOW_VOLTAGE_TOTAL */
bool battery_is_low(void);

/* True once voltage has dropped at/below BATTERY_CRITICAL_VOLTAGE -
 * flight_control.c uses this to force a disarm. */
bool battery_is_critical(void);

/* Runs one sample+LED-update cycle. Intended to be called
 * periodically from a low-priority task. */
void battery_task_step(void);
