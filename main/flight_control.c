#include "flight_control.h"
#include "drone_config.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include "motors.h"
#include "leds.h"
#include "comm.h"
#include "battery.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "flight_ctrl";

/* ---------------- shared attitude state (written by imu task, read
 * by control task) ---------------- */
static SemaphoreHandle_t s_attitude_lock;
static attitude_t s_attitude = { 0 };
static float s_gyro_x_dps = 0, s_gyro_y_dps = 0, s_gyro_z_dps = 0;

/* ---------------- simple PID ---------------- */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_measurement; // used for "derivative on measurement"
    float i_limit;
    float out_limit;
} pid_t;

static pid_t s_pid_roll  = { PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  0, 0, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT };
static pid_t s_pid_pitch = { PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 0, 0, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT };
static pid_t s_pid_yaw   = { PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   0, 0, PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT };

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Angle-error PID that uses gyro RATE as the derivative term instead
 * of differentiating the (noisy) error signal directly - this is the
 * standard "derivative on measurement" trick and avoids derivative
 * kick when the setpoint changes suddenly (e.g. a fast stick flick). */
static float pid_update_angle(pid_t *p, float setpoint_deg, float measured_deg,
                               float measured_rate_dps, float dt_s)
{
    float error = setpoint_deg - measured_deg;

    p->integral += error * dt_s;
    p->integral = clampf(p->integral, -p->i_limit, p->i_limit);

    float d_term = -measured_rate_dps; // rate of the measurement, negated as "derivative of error"

    float out = p->kp * error + p->ki * p->integral + p->kd * d_term;
    return clampf(out, -p->out_limit, p->out_limit);
}

/* Pure rate-mode PID (used for yaw, since we have no absolute
 * heading reference without a magnetometer). */
static float pid_update_rate(pid_t *p, float setpoint_dps, float measured_dps, float dt_s)
{
    float error = setpoint_dps - measured_dps;

    p->integral += error * dt_s;
    p->integral = clampf(p->integral, -p->i_limit, p->i_limit);

    float d_term = (error - p->prev_measurement) / (dt_s > 0.0001f ? dt_s : 0.0001f);
    p->prev_measurement = error;

    float out = p->kp * error + p->ki * p->integral + p->kd * d_term;
    return clampf(out, -p->out_limit, p->out_limit);
}

static void pid_reset(pid_t *p)
{
    p->integral = 0.0f;
    p->prev_measurement = 0.0f;
}

/* ---------------- IMU task: reads sensor + updates attitude ---------------- */
static void imu_task(void *arg)
{
    int64_t last_us = esp_timer_get_time();

    while (1) {
        mpu6050_data_t sample;
        if (mpu6050_read(&sample) == ESP_OK) {
            int64_t now = esp_timer_get_time();
            float dt = (now - last_us) / 1e6f;
            last_us = now;
            if (dt <= 0.0f || dt > 0.5f) {
                dt = IMU_TASK_PERIOD_MS / 1000.0f; // sanity fallback
            }

            attitude_t att = imu_filter_update(&sample, dt);

            xSemaphoreTake(s_attitude_lock, portMAX_DELAY);
            s_attitude = att;
            s_gyro_x_dps = sample.gyro_x_dps;
            s_gyro_y_dps = sample.gyro_y_dps;
            s_gyro_z_dps = sample.gyro_z_dps;
            xSemaphoreGive(s_attitude_lock);

#if DEBUG_LOG_ORIENTATION
            static uint32_t dbg_div = 0;
            if ((dbg_div++ % 40) == 0) { // ~every 200ms at 200Hz
                ESP_LOGI(TAG, "[ORIENTATION] roll=%+6.1f pitch=%+6.1f  "
                              "gx=%+6.1f gy=%+6.1f gz=%+6.1f  ax=%+.2f ay=%+.2f az=%+.2f",
                         att.roll_deg, att.pitch_deg,
                         sample.gyro_x_dps, sample.gyro_y_dps, sample.gyro_z_dps,
                         sample.accel_x_g, sample.accel_y_g, sample.accel_z_g);
            }
#endif
        } else {
            ESP_LOGW(TAG, "MPU6050 read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

/* ---------------- control task: stabilization + mixing + arming ---------------- */
static void control_task(void *arg)
{
    bool armed = false;
    int64_t last_us = esp_timer_get_time();

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_us) / 1e6f;
        last_us = now;
        if (dt <= 0.0f || dt > 0.5f) {
            dt = CONTROL_TASK_PERIOD_MS / 1000.0f;
        }

        control_input_t input;
        bool have_input = comm_get_latest_input(&input);
        bool link_ok = comm_link_is_alive();
        bool battery_ok = !battery_is_critical();

        attitude_t att;
        float gx, gy, gz;
        xSemaphoreTake(s_attitude_lock, portMAX_DELAY);
        att = s_attitude;
        gx = s_gyro_x_dps; gy = s_gyro_y_dps; gz = s_gyro_z_dps;
        xSemaphoreGive(s_attitude_lock);

        /* ---- Arming state machine ----
         * Arm requires: valid+fresh link, battery not critical, an
         * explicit arm request from the transmitter, AND throttle
         * low at the moment of arming (classic safety interlock so
         * the drone can't leap off the ground on connect). */
        bool safe_to_arm_now = have_input && link_ok && battery_ok &&
                                input.arm_request && input.throttle < 50;

        if (!armed) {
            if (safe_to_arm_now) {
                armed = true;
                pid_reset(&s_pid_roll);
                pid_reset(&s_pid_pitch);
                pid_reset(&s_pid_yaw);
                imu_filter_reset();
                ESP_LOGI(TAG, "ARMED");
            }
        } else {
            /* Disarm immediately on: link loss, critical battery,
             * or the pilot releasing the arm switch. This is the
             * failsafe path - keep it simple and fast. */
            if (!link_ok || !battery_ok || !input.arm_request) {
                armed = false;
                motors_stop_all();
                ESP_LOGW(TAG, "DISARMED (link_ok=%d battery_ok=%d arm_req=%d)",
                         link_ok, battery_ok, have_input ? input.arm_request : 0);
            }
        }

        led_armed_set(armed);

        if (!armed) {
            motors_stop_all();
            vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        /* ---- Convert sticks to setpoints ---- */
        float throttle_frac = input.throttle / (float)STICK_THROTTLE_MAX; // 0..1
        float setpoint_roll_deg  =  (input.roll  / (float)STICK_AXIS_RANGE) * MAX_ANGLE_DEG;
        float setpoint_pitch_deg =  (input.pitch / (float)STICK_AXIS_RANGE) * MAX_ANGLE_DEG;
        float setpoint_yaw_dps   =  (input.yaw   / (float)STICK_AXIS_RANGE) * MAX_YAW_RATE_DPS;

        /* ---- Stabilization PIDs ---- */
        float roll_out  = pid_update_angle(&s_pid_roll,  setpoint_roll_deg,  att.roll_deg,  gx, dt);
        float pitch_out = pid_update_angle(&s_pid_pitch, setpoint_pitch_deg, att.pitch_deg, gy, dt);
        float yaw_out   = pid_update_rate(&s_pid_yaw,    setpoint_yaw_dps,   gz, dt);

        /* Normalize PID contributions (which are in the +-PID_OUTPUT_LIMIT
         * range) down to a small +-fraction that gets added on top of
         * throttle, so stabilization can't fully overwhelm throttle. */
        float roll_n  = roll_out  / PID_OUTPUT_LIMIT * 0.5f;
        float pitch_n = pitch_out / PID_OUTPUT_LIMIT * 0.5f;
        float yaw_n   = yaw_out   / PID_OUTPUT_LIMIT * 0.5f;

        /* ---- X-frame mixer ----
         * See the pin/motor-layout comment in drone_config.h for the
         * assumed physical position of M1..M4 and prop spin
         * direction - re-check that against your actual build. */
        float m1 = throttle_frac - pitch_n - roll_n + yaw_n; // front-right
        float m2 = throttle_frac - pitch_n + roll_n - yaw_n; // front-left
        float m3 = throttle_frac + pitch_n + roll_n + yaw_n; // rear-left
        float m4 = throttle_frac + pitch_n - roll_n - yaw_n; // rear-right

        float idle = MOTOR_IDLE_DUTY_PERCENT / 100.0f;
        m1 = clampf(m1, idle, 1.0f);
        m2 = clampf(m2, idle, 1.0f);
        m3 = clampf(m3, idle, 1.0f);
        m4 = clampf(m4, idle, 1.0f);

        /* If throttle is essentially zero, don't apply the idle floor -
         * let the pilot hold armed-but-stationary without the props
         * spinning at idle speed. */
        if (throttle_frac < 0.02f) {
            m1 = m2 = m3 = m4 = 0.0f;
        }

        motors_set_all(m1, m2, m3, m4);

#if DEBUG_LOG_ORIENTATION
        static uint32_t dbg_div2 = 0;
        if ((dbg_div2++ % 40) == 0) { // ~every 200ms at 200Hz
            ESP_LOGI(TAG, "[MOTORS] roll=%+6.1f pitch=%+6.1f  m1(FR)=%.0f%% m2(FL)=%.0f%% "
                          "m3(RL)=%.0f%% m4(RR)=%.0f%%",
                     att.roll_deg, att.pitch_deg, m1 * 100, m2 * 100, m3 * 100, m4 * 100);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

esp_err_t flight_control_start(void)
{
    s_attitude_lock = xSemaphoreCreateMutex();
    if (!s_attitude_lock) {
        return ESP_ERR_NO_MEM;
    }

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 15, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 14, NULL);

    ESP_LOGI(TAG, "Flight control tasks started");
    return ESP_OK;
}
