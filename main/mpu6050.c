#include "mpu6050.h"
#include "drone_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "mpu6050";

/* Registers */
#define REG_PWR_MGMT_1     0x6B
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_WHO_AM_I       0x75

/* Full-scale ranges we configure: +-500 dps gyro, +-4g accel.
 * (Reasonable headroom for a small quad; adjust if you expect
 * harder maneuvers / higher vibration levels.) */
#define GYRO_LSB_PER_DPS   65.5f     // for +-500dps range
#define ACCEL_LSB_PER_G    8192.0f  // for +-4g range

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

static float s_gyro_bias_x = 0, s_gyro_bias_y = 0, s_gyro_bias_z = 0;

/* Applies the IMU_REMAP_* configuration from drone_config.h to one
 * raw 3-axis sample. `raw` is indexed [0]=X,[1]=Y,[2]=Z exactly as
 * the sensor reports it; `out` is [0]=X,[1]=Y,[2]=Z in the frame the
 * rest of the firmware expects (X=front, Y=right, Z=up). Used for
 * both accel and gyro so calibration and live reads stay consistent. */
static void apply_axis_remap(const float raw[3], float out[3])
{
    out[0] = IMU_REMAP_X_SIGN * raw[IMU_REMAP_X_SRC];
    out[1] = IMU_REMAP_Y_SIGN * raw[IMU_REMAP_Y_SRC];
    out[2] = IMU_REMAP_Z_SIGN * raw[IMU_REMAP_Z_SRC];
}

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &start_reg, 1, data, len, 100);
}

/* Probes every valid 7-bit I2C address and logs which ones ACK.
 * Called automatically when the configured MPU6050_I2C_ADDR doesn't
 * respond, so you can see what's actually on the bus - useful for
 * clone/fake MPU6050 boards that sometimes sit at a different
 * address than the genuine part (0x68 with AD0 low, 0x69 with AD0
 * high are the two common ones, but clones can vary). */
static void scan_i2c_bus(void)
{
    ESP_LOGW(TAG, "Scanning I2C bus for any responding devices...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
            ESP_LOGW(TAG, "  -> device found at address 0x%02X", addr);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGE(TAG, "  -> nothing responded at all. This points to wiring/power, "
                      "not just a wrong address: double check SDA=GPIO%d, SCL=GPIO%d, "
                      "plus VCC and GND to the sensor board.",
                 I2C_SDA_GPIO, I2C_SCL_GPIO);
    } else {
        ESP_LOGW(TAG, "If none of the above is 0x%02X (the address this firmware is "
                      "configured for), update MPU6050_I2C_ADDR in drone_config.h to "
                      "match whatever address it found.", MPU6050_I2C_ADDR);
    }
}

esp_err_t mpu6050_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Read WHO_AM_I purely for logging/diagnostics - genuine MPU6050s
     * report 0x68, but plenty of clone/fake boards report something
     * else entirely (0x69, 0x70, 0x98, whatever the copied die
     * actually is) while still speaking the same register protocol
     * perfectly fine. So a mismatch here is informational only, never
     * a reason to fail init. A READ FAILURE (below) is the real
     * problem - that means nothing acknowledged at this address at
     * all, which is what triggers the bus scan. */
    uint8_t who = 0;
    err = read_regs(REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No response from MPU6050 at address 0x%02X (SDA=GPIO%d SCL=GPIO%d)",
                 MPU6050_I2C_ADDR, I2C_SDA_GPIO, I2C_SCL_GPIO);
        scan_i2c_bus();
        return err;
    }
    ESP_LOGI(TAG, "Sensor responded, WHO_AM_I = 0x%02X (0x68 = genuine MPU6050, "
                  "anything else is a common clone-board ID and is fine)", who);

    /* Wake the device up (default is asleep on power-up) and pick
     * the gyro clock reference for better stability than internal RC. */
    esp_err_t r1 = write_reg(REG_PWR_MGMT_1, 0x01); // clock = PLL w/ X gyro ref
    esp_err_t r2 = write_reg(REG_SMPLRT_DIV, 0x00); // sample rate = gyro rate
    esp_err_t r3 = write_reg(REG_CONFIG, 0x03);     // DLPF ~44Hz, reduces noise
    esp_err_t r4 = write_reg(REG_GYRO_CONFIG, 0x08);  // +-500 dps
    esp_err_t r5 = write_reg(REG_ACCEL_CONFIG, 0x08); // +-4g

    if (r1 || r2 || r3 || r4 || r5) {
        ESP_LOGE(TAG, "One or more MPU6050 config writes failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 initialized OK");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *out)
{
    uint8_t raw[14];
    esp_err_t err = read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];
    int16_t temp_raw = (raw[6] << 8) | raw[7];
    int16_t gx = (raw[8] << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    float raw_accel[3] = { ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G };
    float raw_gyro[3]  = { gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS };
    float accel[3], gyro[3];
    apply_axis_remap(raw_accel, accel);
    apply_axis_remap(raw_gyro, gyro);

    out->accel_x_g = accel[0];
    out->accel_y_g = accel[1];
    out->accel_z_g = accel[2];

    out->gyro_x_dps = gyro[0] - s_gyro_bias_x;
    out->gyro_y_dps = gyro[1] - s_gyro_bias_y;
    out->gyro_z_dps = gyro[2] - s_gyro_bias_z;

    out->temp_c = (temp_raw / 340.0f) + 36.53f;

    return ESP_OK;
}

esp_err_t mpu6050_calibrate_gyro(int num_samples)
{
    if (num_samples <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Calibrating gyro bias - keep the drone perfectly still...");

    double sum_x = 0, sum_y = 0, sum_z = 0;
    int good = 0;

    for (int i = 0; i < num_samples; i++) {
        uint8_t raw[14];
        if (read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw)) == ESP_OK) {
            int16_t gx = (raw[8] << 8) | raw[9];
            int16_t gy = (raw[10] << 8) | raw[11];
            int16_t gz = (raw[12] << 8) | raw[13];
            float raw_gyro[3] = { gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS };
            float gyro[3];
            apply_axis_remap(raw_gyro, gyro);
            sum_x += gyro[0];
            sum_y += gyro[1];
            sum_z += gyro[2];
            good++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (good < num_samples / 2) {
        ESP_LOGE(TAG, "Gyro calibration failed - too many bad I2C reads");
        return ESP_FAIL;
    }

    s_gyro_bias_x = (float)(sum_x / good);
    s_gyro_bias_y = (float)(sum_y / good);
    s_gyro_bias_z = (float)(sum_z / good);

    ESP_LOGI(TAG, "Gyro bias: x=%.3f y=%.3f z=%.3f (from %d samples)",
             s_gyro_bias_x, s_gyro_bias_y, s_gyro_bias_z, good);

    return ESP_OK;
}
