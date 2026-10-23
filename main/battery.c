#include "battery.h"
#include "drone_config.h"
#include "leds.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_ok = false;

static volatile float s_last_voltage = 0.0f;
static volatile bool s_low = false;
static volatile bool s_critical = false;

esp_err_t battery_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,     // full ~0-3.3V input range
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Calibration makes the raw counts -> millivolts conversion much
     * more accurate than a hand-rolled linear formula. Falls back to
     * an approximate formula below if the chip doesn't support it. */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
        s_cali_ok = true;
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable, using approximate scaling");
    }

    ESP_LOGI(TAG, "Battery ADC initialized on GPIO%d", BATTERY_ADC_GPIO);
    return ESP_OK;
}

float battery_read_voltage(void)
{
    int64_t sum_mv = 0;
    int good = 0;

    for (int i = 0; i < BATTERY_ADC_AVG_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw) != ESP_OK) {
            continue;
        }
        int mv = 0;
        if (s_cali_ok) {
            adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        } else {
            mv = (raw * 3300) / 4095; // rough fallback for 12-bit ADC @ ~3.3V range
        }
        sum_mv += mv;
        good++;
    }

    if (good == 0) {
        return s_last_voltage; // keep last known value rather than reporting 0
    }

    float measured_v = (sum_mv / (float)good) / 1000.0f;
    float pack_v = measured_v * BATTERY_DIVIDER_RATIO;
    s_last_voltage = pack_v;
    return pack_v;
}

bool battery_is_low(void)      { return s_low; }
bool battery_is_critical(void) { return s_critical; }

void battery_task_step(void)
{
    float v = battery_read_voltage();

    s_low = (v <= BATTERY_LOW_VOLTAGE_TOTAL);
    s_critical = (v <= BATTERY_CRITICAL_VOLTAGE);

    led_lowbat_set(s_low);

    static uint32_t log_div = 0;
    if ((log_div++ % 10) == 0) { // don't spam the log every 500ms
        ESP_LOGI(TAG, "Battery: %.2fV%s%s", v,
                 s_low ? " [LOW]" : "", s_critical ? " [CRITICAL]" : "");
    }
}
