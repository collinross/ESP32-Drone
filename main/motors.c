#include "motors.h"
#include "drone_config.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motors";

/* One LEDC channel per motor. All share one timer since they all
 * run at the same frequency/resolution. */
static const int s_motor_gpio[4] = { MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO };
static const ledc_channel_t s_motor_channel[4] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
};

#define MOTOR_LEDC_TIMER   LEDC_TIMER_0
#define MOTOR_LEDC_MODE    LEDC_LOW_SPEED_MODE

esp_err_t motors_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = MOTOR_LEDC_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = s_motor_gpio[i],
            .speed_mode = MOTOR_LEDC_MODE,
            .channel    = s_motor_channel[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = MOTOR_LEDC_TIMER,
            .duty       = MOTOR_PWM_INVERTED ? MOTOR_PWM_MAX_DUTY : 0,
            .hpoint     = 0,
        };
        err = ledc_channel_config(&ch_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed for motor %d (GPIO%d): %s",
                     i + 1, s_motor_gpio[i], esp_err_to_name(err));
            return err;
        }
    }

    motors_stop_all();
    ESP_LOGI(TAG, "Motors initialized (M1=GPIO%d M2=GPIO%d M3=GPIO%d M4=GPIO%d)",
              MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO);
    return ESP_OK;
}

void motors_set(int motor_index, float duty_fraction)
{
    if (motor_index < 0 || motor_index > 3) {
        return;
    }
    if (duty_fraction < 0.0f) duty_fraction = 0.0f;
    if (duty_fraction > 1.0f) duty_fraction = 1.0f;

    uint32_t duty = (uint32_t)(duty_fraction * MOTOR_PWM_MAX_DUTY);
    if (MOTOR_PWM_INVERTED) {
        duty = MOTOR_PWM_MAX_DUTY - duty;
    }

    ledc_set_duty(MOTOR_LEDC_MODE, s_motor_channel[motor_index], duty);
    ledc_update_duty(MOTOR_LEDC_MODE, s_motor_channel[motor_index]);
}

void motors_set_all(float m1, float m2, float m3, float m4)
{
    motors_set(0, m1);
    motors_set(1, m2);
    motors_set(2, m3);
    motors_set(3, m4);
}

void motors_stop_all(void)
{
    motors_set_all(0.0f, 0.0f, 0.0f, 0.0f);
}
