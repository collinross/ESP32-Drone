#include "drone_config.h"
#include "mpu6050.h"
#include "imu_filter.h"
#include "motors.h"
#include "leds.h"
#include "battery.h"
#include "comm.h"
#include "flight_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

/* Low-priority background task: just samples the battery and drives
 * the low-battery LED. Deliberately kept separate from the tight
 * control loop so a slow ADC read can never stall stabilization. */
static void battery_task(void *arg)
{
    while (1) {
        battery_task_step();
        vTaskDelay(pdMS_TO_TICKS(BATTERY_SAMPLE_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32-C6 Drone Flight Controller booting ===");

    leds_init();
    led_armed_set(false);
    led_lowbat_set(false);

    ESP_ERROR_CHECK(motors_init());

    esp_err_t err = mpu6050_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed (%s) - halting, check I2C wiring "
                      "(SDA=GPIO%d SCL=GPIO%d)", esp_err_to_name(err),
                 I2C_SDA_GPIO, I2C_SCL_GPIO);
        /* Blink both LEDs forever as a visible "hardware fault" signal
         * rather than silently doing nothing - flying without a
         * working IMU would be dangerous even if we let it continue. */
        while (1) {
            led_armed_set(true);
            led_lowbat_set(false);
            vTaskDelay(pdMS_TO_TICKS(150));
            led_armed_set(false);
            led_lowbat_set(true);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
    }

    /* Board must be held still during this - it sets the gyro's
     * zero-rate reference that every future reading is corrected by. */
    mpu6050_calibrate_gyro(500);
    imu_filter_reset();

    ESP_ERROR_CHECK(battery_init());
    ESP_ERROR_CHECK(comm_init());
    ESP_ERROR_CHECK(flight_control_start());

    xTaskCreate(battery_task, "battery_task", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "Init complete. Connect to WiFi '%s' and run the ground "
                  "station script to arm.", WIFI_AP_SSID);
}
