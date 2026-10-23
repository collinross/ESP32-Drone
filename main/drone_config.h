#pragma once

/* =========================================================================
 * drone_config.h
 *
 * Single place for every pin assignment and tunable constant. Change
 * hardware wiring or tuning values here rather than hunting through
 * the rest of the code.
 *
 * BOARD: Seeed Studio XIAO ESP32-C6
 *
 * IMPORTANT HARDWARE NOTE:
 * The ESP32-C6 only has ADC channels on GPIO0-GPIO6. GPIO19 (as
 * originally requested for the battery sense input) is NOT ADC
 * capable on this chip. Current pin arrangement:
 *
 *   - Low battery LED   -> GPIO0   (matches original request)
 *   - Battery ADC input -> GPIO6   (ADC1_CH6, since GPIO19 can't do
 *                                   analog input on this chip)
 *
 * GPIO19 is unused in this build.
 * ========================================================================= */

/* ---------------- I2C / MPU6050 ---------------- */
#define I2C_PORT                I2C_NUM_0
#define I2C_SDA_GPIO            22
#define I2C_SCL_GPIO            23
#define I2C_CLK_SPEED_HZ        400000      // 400kHz fast mode
#define MPU6050_I2C_ADDR        0x68        // AD0 pin tied low (default)

/* ---------------- IMU axis remap (mounting orientation) ----------------
 * The rest of the firmware assumes:
 *   output X axis points toward the FRONT (nose) of the drone
 *   output Y axis points toward the RIGHT
 *   output Z axis points UP
 *
 * If the MPU6050 isn't mounted flat with its silkscreen-printed X/Y/Z
 * axes matching that, use this block to correct it in software rather
 * than re-soldering anything. Every OUTPUT axis is built from one RAW
 * sensor axis (0=raw X, 1=raw Y, 2=raw Z, as the chip actually reports
 * them) times a sign:
 *
 *     output[X] = X_SIGN * raw[X_SRC]
 *     output[Y] = Y_SIGN * raw[Y_SRC]
 *     output[Z] = Z_SIGN * raw[Z_SRC]
 *
 * Defaults below are IDENTITY (no remap) - this restores the exact
 * original, pre-remap sensor behavior. Once your MPU6050 is
 * connecting reliably again, change these if/when you're ready to
 * correct for its actual mounting orientation - see "Correcting IMU
 * mounting orientation" in the README for the bring-up test to use
 * when you do. Getting this wrong (once you do change it) can turn
 * self-leveling into positive feedback that flips the drone the
 * instant it arms - always verify with props OFF first.
 */
#define IMU_REMAP_X_SRC   0
#define IMU_REMAP_X_SIGN  (-1)
#define IMU_REMAP_Y_SRC   1
#define IMU_REMAP_Y_SIGN  (+1)
#define IMU_REMAP_Z_SRC   2
#define IMU_REMAP_Z_SIGN  (-1)

/* Sanity-check aid for the bring-up test below: set to 1 to have the
 * firmware log live roll/pitch and (once armed) motor duty percentages
 * over serial so you can watch the sign/remap behavior directly.
 * Leave enabled during bring-up, then set back to 0 (or leave on,
 * it's harmless / just log spam) once everything checks out. */
#define DEBUG_LOG_ORIENTATION  1

/* ---------------- Motors (PWM via low-side N-MOSFET) ---------------- */
/* Motor layout assumption, X-frame, viewed from above, nose = "front":
 *
 *        FRONT
 *     M2       M1
 *        \   /
 *         \ /
 *          X
 *         / \
 *        /   \
 *     M3       M4
 *        REAR
 *
 * M1 = front-right, M2 = front-left, M3 = rear-left, M4 = rear-right.
 * If your physical wiring doesn't match this, either re-plug the
 * motors to match, or edit the mixer in flight_control.c to match
 * your wiring instead. Also verify prop rotation direction (M1/M3
 * spin one way, M2/M4 the other way for a standard quad X) - this
 * matters for actual flight, the mixer signs below assume that.
 */
#define MOTOR1_GPIO             18
#define MOTOR2_GPIO             2
#define MOTOR3_GPIO             21
#define MOTOR4_GPIO             20

#define MOTOR_PWM_FREQ_HZ       20000       // 20kHz - above audible range
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_10_BIT   // 0-1023 duty steps
#define MOTOR_PWM_MAX_DUTY      1023
#define MOTOR_PWM_MIN_DUTY      0

/* These MOSFETs pull the motor's negative terminal down to switch
 * current through it, so HIGHER duty cycle = MORE current = MORE
 * speed (non-inverted logic). If your motors behave backwards,
 * flip MOTOR_PWM_INVERTED to 1 below. */
#define MOTOR_PWM_INVERTED      0

/* Minimum duty sent to a motor any time it should be "spinning idle"
 * once armed - many small brushed motors stall or judder at very low
 * duty, so we floor it here rather than at true 0 while armed. */
#define MOTOR_IDLE_DUTY_PERCENT 5           // percent of max duty

/* ---------------- LEDs ---------------- */
#define ARMED_LED_GPIO          1
#define LOWBAT_LED_GPIO         0

/* ---------------- Battery sense ---------------- */
#define BATTERY_ADC_GPIO        6           // GPIO19 is not ADC-capable on C6
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_6  // GPIO6 == ADC1_CH6 on C6
#define BATTERY_ADC_UNIT        ADC_UNIT_1

/* Your board halves the battery voltage before it reaches the ADC
 * (a 1:1 resistor divider), so actual pack voltage = 2 * measured.
 * Adjust DIVIDER_RATIO if your resistors give a different ratio. */
#define BATTERY_DIVIDER_RATIO   2.0f

/* Tune these to your battery chemistry/cell count. Defaults assume
 * a 1S Li-ion/LiPo pack. */
#define BATTERY_CELL_COUNT          1
#define BATTERY_LOW_VOLTAGE_TOTAL   3.4f   // ~3.4V/cell -> warn
#define BATTERY_CRITICAL_VOLTAGE    3.2f   // ~3.2V/cell -> force disarm
#define BATTERY_SAMPLE_PERIOD_MS    500
#define BATTERY_ADC_AVG_SAMPLES     32

/* ---------------- WiFi / control link ---------------- */
#define WIFI_AP_SSID            "ESP32DRONE"
#define WIFI_AP_PASS            "drone1234"   // 8+ chars required for WPA2
#define WIFI_AP_CHANNEL         6
#define WIFI_AP_MAX_CONN        1

#define CONTROL_UDP_PORT        4210

/* If no valid control packet arrives within this window, the drone
 * disarms and cuts motors. This is the single most important safety
 * number in this file - do not set it too high. */
#define FAILSAFE_TIMEOUT_MS     500

/* ---------------- Control loop timing ---------------- */
#define IMU_TASK_PERIOD_MS      5      // ~200 Hz sensor + attitude update
#define CONTROL_TASK_PERIOD_MS  5      // ~200 Hz stabilization / mixing

/* ---------------- Stick scaling (must match python_controller) -------- */
#define STICK_THROTTLE_MAX      1000   // 0..1000
#define STICK_AXIS_RANGE        500    // roll/pitch/yaw: -500..+500

/* Max commanded angle (degrees) at full roll/pitch stick deflection */
#define MAX_ANGLE_DEG           30.0f

/* Max commanded yaw rotation rate (deg/s) at full yaw stick deflection */
#define MAX_YAW_RATE_DPS        180.0f

/* ---------------- PID gains (starting points only - MUST be tuned
 * on your actual airframe; these will NOT be correct out of the box) */
#define PID_ROLL_KP     1.8f
#define PID_ROLL_KI     0.02f
#define PID_ROLL_KD     0.7f

#define PID_PITCH_KP    1.8f
#define PID_PITCH_KI    0.02f
#define PID_PITCH_KD    0.7f

#define PID_YAW_KP      2.0f
#define PID_YAW_KI      0.01f
#define PID_YAW_KD      0.0f

#define PID_INTEGRAL_LIMIT   200.0f
#define PID_OUTPUT_LIMIT     400.0f   // clamps PID contribution into mixer

/* Complementary filter weight for gyro vs accel (0..1, closer to 1
 * trusts the gyro integration more and the accelerometer less) */
#define COMP_FILTER_ALPHA    0.98f
