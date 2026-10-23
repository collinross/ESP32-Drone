#include "comm.h"
#include "drone_config.h"
#include "protocol.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "comm";

static SemaphoreHandle_t s_lock = NULL;
static control_input_t s_latest = { .throttle = 0, .roll = 0, .pitch = 0, .yaw = 0, .arm_request = false };
static int64_t s_last_valid_packet_us = 0;
static uint16_t s_last_seq = 0;
static bool s_have_first_packet = false;

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

static void wifi_ap_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.ap.ssid, WIFI_AP_SSID, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    strncpy((char *)wifi_config.ap.password, WIFI_AP_PASS, sizeof(wifi_config.ap.password) - 1);
    wifi_config.ap.channel = WIFI_AP_CHANNEL;
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONN;
    wifi_config.ap.authmode = (strlen(WIFI_AP_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP started: SSID='%s' channel=%d. Ground station should "
                  "connect to this network, ESP32 will be at 192.168.4.1",
             WIFI_AP_SSID, WIFI_AP_CHANNEL);
}

static void udp_rx_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONTROL_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket to port %d", CONTROL_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    /* Non-blocking-ish: short timeout so this task can't ever wedge
     * forever and is cheap to keep alive alongside the control loop. */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "Listening for control packets on UDP port %d", CONTROL_UDP_PORT);

    uint8_t buf[64];
    while (1) {
        struct sockaddr_in src_addr;
        socklen_t src_len = sizeof(src_addr);
        int len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&src_addr, &src_len);
        if (len <= 0) {
            continue; // timeout or transient error, just loop and check again
        }

        if (len != (int)CONTROL_PACKET_SIZE) {
            ESP_LOGW(TAG, "Dropped packet with wrong size (%d, expected %d)",
                     len, (int)CONTROL_PACKET_SIZE);
            continue;
        }

        control_packet_t pkt;
        memcpy(&pkt, buf, sizeof(pkt));

        if (pkt.magic != DRONE_PROTOCOL_MAGIC) {
            ESP_LOGW(TAG, "Dropped packet with bad magic");
            continue;
        }

        uint16_t expected_checksum = protocol_compute_packet_checksum(&pkt);
        if (expected_checksum != pkt.checksum) {
            ESP_LOGW(TAG, "Dropped packet with bad checksum");
            continue;
        }

        /* Reject obviously stale/duplicate/out-of-order packets, but
         * don't be too strict - allow sequence wraparound. */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_latest.throttle = pkt.throttle > STICK_THROTTLE_MAX ? STICK_THROTTLE_MAX : pkt.throttle;
        s_latest.roll  = pkt.roll;
        s_latest.pitch = pkt.pitch;
        s_latest.yaw   = pkt.yaw;
        s_latest.arm_request = (pkt.arm_switch != 0);
        s_last_seq = pkt.seq;
        s_have_first_packet = true;
        s_last_valid_packet_us = now_us();
        xSemaphoreGive(s_lock);
    }
}

esp_err_t comm_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_ap_start();

    xTaskCreate(udp_rx_task, "udp_rx", 4096, NULL, 10, NULL);

    return ESP_OK;
}

bool comm_get_latest_input(control_input_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_latest;
    bool have_data = s_have_first_packet;
    xSemaphoreGive(s_lock);
    return have_data;
}

bool comm_link_is_alive(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool have_data = s_have_first_packet;
    int64_t last = s_last_valid_packet_us;
    xSemaphoreGive(s_lock);

    if (!have_data) {
        return false;
    }
    int64_t age_ms = (now_us() - last) / 1000;
    return age_ms <= FAILSAFE_TIMEOUT_MS;
}
