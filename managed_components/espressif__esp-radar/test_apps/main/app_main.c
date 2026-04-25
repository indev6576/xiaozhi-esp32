/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_radar.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_heap_trace.h"

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
#if CONFIG_ESP_RADAR_TEST_MULTI_PEER
static const uint8_t CONFIG_CSI_SEND_MAC_2[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x01};
static esp_radar_handle_t s_peer2 = NULL;
static volatile uint32_t s_peer2_csi_frames = 0;
#endif
static const char *TAG = "esp_radar_test_app";
#define CSI_PRINT_STEP 5
#define ESP_RADAR_WIFI_CHANNEL 12

static void radar_rx_cb(void *ctx, const wifi_radar_info_t *info)
{
    ESP_LOGI(TAG, "waveform_jitter %lf, waveform_wander %lf", info->waveform_jitter, info->waveform_wander);
}

#if CONFIG_ESP_RADAR_TEST_MULTI_PEER
static void radar_rx_cb_ex(void *ctx, const uint8_t mac[6], const wifi_radar_info_t *info)
{
    ESP_LOGI(TAG, "peer=" MACSTR ", waveform_jitter %lf, waveform_wander %lf",
             MAC2STR(mac), info->waveform_jitter, info->waveform_wander);
}
#endif
static void wifi_csi_rx_cb_1(void *ctx, const wifi_csi_filtered_info_t *filtered_info)
{
    // static int s_count = 0;
    // if (!s_count) {
    //     ets_printf("================ CSI RECV ================\n");
    // }
    // if (filtered_info->data_type == WIFI_CSI_DATA_TYPE_INT16) {
    //     const int16_t *csi = filtered_info->valid_data_i16;
    //     int sample_cnt = filtered_info->valid_len / sizeof(int16_t);
    //     ets_printf("CSI_DATA," MACSTR ",%d,%d,\"[", MAC2STR(filtered_info->mac), sample_cnt, s_count);
    //     const char *sep = "";
    //     for (int k = 0; k < sample_cnt; k += 2 * CSI_PRINT_STEP) {
    //         ets_printf("%s%d,%d", sep, csi[k], csi[k + 1]);
    //         sep = ",";
    //     }
    //     ets_printf("]\"\n");

    // } else {
    //     const int8_t *csi = filtered_info->valid_data;
    //     int sample_cnt = filtered_info->valid_len;
    //     ets_printf("CSI_DATA," MACSTR ",%d,%d,\"[", MAC2STR(filtered_info->mac), sample_cnt, s_count);
    //     const char *sep = "";
    //     for (int k = 0; k < sample_cnt; k += 2 * CSI_PRINT_STEP) {
    //         ets_printf("%s%d,%d", sep, csi[k], csi[k + 1]);
    //         sep = ",";
    //     }
    //     ets_printf("]\"\n");
    // }
    // s_count++;

#if CONFIG_ESP_RADAR_TEST_MULTI_PEER
    if (filtered_info && memcmp(filtered_info->mac, CONFIG_CSI_SEND_MAC_2, 6) == 0) {
        s_peer2_csi_frames++;
    }
#endif
}

void app_main()
{
    /**
     * @brief Initialize NVS
     */

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    memcpy(csi_config.filter_mac, CONFIG_CSI_SEND_MAC, 6);
    csi_config.csi_filtered_cb = wifi_csi_rx_cb_1;
    csi_config.csi_compensate_en = true;
    csi_config.csi_recv_interval = 20;
    esp_radar_wifi_config_t wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT();
    wifi_config.channel = ESP_RADAR_WIFI_CHANNEL;
    esp_radar_espnow_config_t espnow_config = ESP_RADAR_ESPNOW_CONFIG_DEFAULT();
    esp_radar_dec_config_t dec_config = ESP_RADAR_DEC_CONFIG_DEFAULT();
#if CONFIG_ESP_RADAR_TEST_MULTI_PEER
    dec_config.wifi_radar_cb_ex = radar_rx_cb_ex;
#else
    dec_config.wifi_radar_cb = radar_rx_cb;
#endif
    dec_config.ltf_type = RADAR_LTF_TYPE_LLTF;
    esp_radar_config_t radar_config = {
        .wifi_config = wifi_config,
        .csi_config = csi_config,
        .espnow_config = espnow_config,
        .dec_config = dec_config,
    };
    ESP_ERROR_CHECK(esp_radar_init(&radar_config));

#if CONFIG_ESP_RADAR_TEST_MULTI_PEER
    /* Create additional peer(s) for multi-peer test */
    ESP_ERROR_CHECK(esp_radar_new_peer(CONFIG_CSI_SEND_MAC_2, &s_peer2));
#endif

    esp_radar_start();
    float wander_threshold = 0;
    float jitter_threshold = 0;
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "esp_radar_train_start");
    esp_radar_train_start();

    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "esp_radar_train_stop");
    esp_err_t err0 = esp_radar_train_stop(&wander_threshold, &jitter_threshold);
    if (err0 == ESP_OK) {
        ESP_LOGI(TAG, "wander_threshold: %f, jitter_threshold: %f", wander_threshold, jitter_threshold);
    } else if (err0 == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "train_stop failed (no training data?), err=%s", esp_err_to_name(err0));
    } else {
        ESP_LOGW(TAG, "train_stop failed, err=%s", esp_err_to_name(err0));
    }

#if CONFIG_ESP_RADAR_TEST_MULTI_PEER
    if (s_peer2) {
        float wander2 = 0;
        float jitter2 = 0;
        ESP_LOGI(TAG, "esp_radar_train_start_ex(peer2=" MACSTR ")", MAC2STR(CONFIG_CSI_SEND_MAC_2));
        ESP_ERROR_CHECK(esp_radar_train_start_ex(s_peer2));

        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "esp_radar_train_stop_ex(peer2=" MACSTR ")", MAC2STR(CONFIG_CSI_SEND_MAC_2));
        esp_err_t err = esp_radar_train_stop_ex(s_peer2, &wander2, &jitter2);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "peer2 thresholds: wander=%f, jitter=%f", wander2, jitter2);
        } else if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "peer2 train_stop_ex failed (no training data?), err=%s, peer2_csi_frames=%u",
                     esp_err_to_name(err), (unsigned)s_peer2_csi_frames);
        } else {
            ESP_LOGW(TAG, "peer2 train_stop_ex failed, err=%s, peer2_csi_frames=%u",
                     esp_err_to_name(err), (unsigned)s_peer2_csi_frames);
        }
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "esp_radar_stop");
    esp_radar_stop();
    ESP_LOGI(TAG, "esp_radar_deinit");
    esp_radar_deinit();

}
