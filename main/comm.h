#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t throttle;  // 0..1000
    int16_t  roll;      // -500..500
    int16_t  pitch;     // -500..500
    int16_t  yaw;       // -500..500
    bool     arm_request;
} control_input_t;

/* Brings up the WiFi SoftAP and starts the background UDP receive
 * task. Safe to call once at boot. */
esp_err_t comm_init(void);

/* Thread-safe: copies out the most recently received (and validated)
 * control input. Returns false if we've never received a valid
 * packet yet. */
bool comm_get_latest_input(control_input_t *out);

/* True if a valid packet has arrived within FAILSAFE_TIMEOUT_MS.
 * flight_control.c must treat false here as "disarm immediately". */
bool comm_link_is_alive(void);
