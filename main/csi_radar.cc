#include "csi_radar.h"
#include "sdkconfig.h"

#if CONFIG_USE_CSI_RADAR
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_event_base.h>
#include <string.h>
#include <math.h>

#define TAG "CsiRadar"

#define RADAR_BUFF_MAX_LEN 25
#define BREATH_WINDOW_SIZE 60

static float s_sort_buffer_[BREATH_WINDOW_SIZE];

CsiRadar* CsiRadar::instance_ = nullptr;
bool CsiRadar::s_initialized_ = false;
bool CsiRadar::s_train_start_ = false;
bool CsiRadar::s_last_room_status_ = false;
bool CsiRadar::s_last_human_status_ = false;

float CsiRadar::s_buff_wander_[RADAR_BUFF_MAX_LEN] = {0};
float CsiRadar::s_buff_jitter_[RADAR_BUFF_MAX_LEN] = {0};
uint32_t CsiRadar::s_buff_count_ = 0;

float CsiRadar::predict_someone_threshold_ = 0.001f;
float CsiRadar::predict_someone_sensitivity_ = 0.15f;
float CsiRadar::predict_move_threshold_ = 0.0003f;
float CsiRadar::predict_move_sensitivity_ = 0.20f;
uint32_t CsiRadar::predict_buff_size_ = 5;
uint32_t CsiRadar::predict_outliers_number_ = 2;

float CsiRadar::s_breath_jitter_buf_[BREATH_WINDOW_SIZE] = {0};
uint32_t CsiRadar::s_breath_buf_idx_ = 0;
uint32_t CsiRadar::s_breath_sample_count_ = 0;
float CsiRadar::s_noise_floor_ = 0.0f;
float CsiRadar::s_breath_threshold_ = 0.0f;
float CsiRadar::s_breath_rate_smooth_ = 0;

uint32_t CsiRadar::s_last_update_time_ = 0;
uint32_t CsiRadar::s_last_move_time_ = 0;
uint32_t CsiRadar::s_last_someone_time_ = 0;
uint32_t CsiRadar::s_last_radar_data_time_ = 0;
uint32_t CsiRadar::s_count_ = 0;

CsiRadar::CsiRadar() {
    instance_ = this;
}

CsiRadar::~CsiRadar() {
    Stop();
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

void CsiRadar::StartIfNeeded() {
    if (instance_ && !s_initialized_) {
        ESP_LOGI(TAG, "Starting CSI Radar via global function...");
        instance_->Start();
    }
}

void CsiRadar::RadarCallbackImpl(void* ctx, const wifi_radar_info_t* info) {
    if (instance_) {
        static uint32_t last_log = 0;
        uint32_t now = esp_log_timestamp();
        if (now - last_log > 5000) {
            ESP_LOGI(TAG, "RadarCallbackImpl called, waveform_wander=%.6f, waveform_jitter=%.6f",
                     info->waveform_wander, info->waveform_jitter);
            last_log = now;
        }
        instance_->ProcessRadarData(info);
    } else {
        ESP_LOGW(TAG, "RadarCallbackImpl called but instance_ is null!");
    }
}

bool CsiRadar::Start() {
    if (s_initialized_) {
        ESP_LOGW(TAG, "CSI Radar already initialized");
        return true;
    }

    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGE(TAG, "Failed to get WiFi mode: %d", err);
        return false;
    }

    if (err == ESP_ERR_WIFI_NOT_INIT || mode != WIFI_MODE_STA) {
        if (mode != WIFI_MODE_STA) {
            err = esp_wifi_set_mode(WIFI_MODE_STA);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set WiFi mode to STA: %d", err);
                return false;
            }
        }
        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WiFi: %d", err);
            return false;
        }
        ESP_LOGI(TAG, "WiFi initialized in STA mode");
    } else {
        ESP_LOGI(TAG, "WiFi already in STA mode");
    }

    wifi_ap_record_t ap_info;
    int retry = 0;
    const int max_retries = 20;
    while (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK && retry < max_retries) {
        retry++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (retry >= max_retries) {
        ESP_LOGW(TAG, "WiFi not connected, skipping CSI Radar");
        return true;
    }
    ESP_LOGI(TAG, "WiFi connected to %s (channel %d)", ap_info.ssid, ap_info.primary);

    esp_err_t loop_err = esp_event_loop_create_default();
    if (loop_err != ESP_OK && loop_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %d", loop_err);
        return false;
    }
    ESP_LOGI(TAG, "Event loop ready");

    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    esp_radar_dec_config_t dec_config = ESP_RADAR_DEC_CONFIG_DEFAULT();

    csi_config.csi_recv_interval = 50;
    csi_config.csi_filtered_cb = NULL;
    csi_config.dump_ack_en = true;
    memset(csi_config.filter_mac, 0, 6);

    dec_config.wifi_radar_cb = RadarCallbackImpl;
    dec_config.ltf_type = RADAR_LTF_TYPE_LLTF;
    dec_config.outliers_threshold = 0;
    dec_config.csi_handle_time = 200;
    dec_config.dec_window_size = 4;

    ESP_LOGI(TAG, "Initializing CSI and decoder...");
    err = esp_radar_csi_init(&csi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init CSI: %d", err);
    } else {
        ESP_LOGI(TAG, "CSI init success");
    }

    err = esp_radar_dec_init(&dec_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init decoder: %d", err);
    } else {
        ESP_LOGI(TAG, "Decoder init success");
    }

    err = esp_radar_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start radar: %d", err);
    } else {
        ESP_LOGI(TAG, "Radar started successfully");
    }

    xTaskCreate([](void* arg) {
        wifi_ap_record_t ap_info;
        uint8_t sta_mac[6];
        esp_wifi_sta_get_ap_info(&ap_info);
        esp_wifi_get_mac(WIFI_IF_STA, sta_mac);

        typedef struct {
            uint8_t frame_control[2];
            uint16_t duration;
            uint8_t destination_address[6];
            uint8_t source_address[6];
            uint8_t broadcast_address[6];
            uint16_t sequence_control;
        } __attribute__((packed)) wifi_null_data_t;

        wifi_null_data_t null_data = {
            .frame_control = {0x48, 0x01},
            .duration = 0x0000,
            .sequence_control = 0x0000,
        };

        memcpy(null_data.destination_address, ap_info.bssid, 6);
        memcpy(null_data.broadcast_address, ap_info.bssid, 6);
        memcpy(null_data.source_address, sta_mac, 6);

        esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);
        
        ESP_LOGI(TAG, "Starting NULL data sender");
        
        while (true) {
            esp_wifi_80211_tx(WIFI_IF_STA, &null_data, sizeof(wifi_null_data_t), true);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }, "null_data_sender", 2048, NULL, 3, NULL);

    static bool event_handler_registered = false;
    if (!event_handler_registered) {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, [](void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
            ESP_LOGW(TAG, "WiFi disconnected");
        }, NULL));
        
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, [](void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
            ESP_LOGI(TAG, "WiFi reconnected");
        }, NULL));
        
        event_handler_registered = true;
    }

    s_initialized_ = true;
    s_count_ = 0;
    s_train_start_ = false;
    s_last_room_status_ = false;
    s_last_human_status_ = false;
    s_last_update_time_ = 0;
    s_last_move_time_ = 0;
    s_last_someone_time_ = 0;
    s_last_radar_data_time_ = 0;
    s_buff_count_ = 0;
    s_breath_buf_idx_ = 0;
    s_breath_sample_count_ = 0;
    s_noise_floor_ = 0.0f;
    s_breath_threshold_ = 0.0f;
    s_breath_rate_smooth_ = 0;

    ESP_LOGI(TAG, "CSI Radar started successfully");
    return true;
}

void CsiRadar::Stop() {
    if (!s_initialized_) {
        return;
    }

    s_initialized_ = false;
    ESP_LOGI(TAG, "CSI Radar stopped");
}

void CsiRadar::ProcessRadarData(const wifi_radar_info_t* info) {
    uint32_t buff_max_size = predict_buff_size_;
    uint32_t someone_count = 0;
    uint32_t move_count = 0;
    bool room_status = false;
    bool human_status = false;

    s_buff_wander_[s_buff_count_ % RADAR_BUFF_MAX_LEN] = info->waveform_wander;
    s_buff_jitter_[s_buff_count_ % RADAR_BUFF_MAX_LEN] = info->waveform_jitter;
    s_buff_count_++;

    static uint32_t last_count_log = 0;
    if (s_buff_count_ % 100 == 0 && s_buff_count_ != last_count_log) {
        ESP_LOGI(TAG, "CSI data buffer count: %u/%u", s_buff_count_, buff_max_size);
        last_count_log = s_buff_count_;
    }

    if (s_buff_count_ < buff_max_size) {
        return;
    }

    float wander_average = trimmean(s_buff_wander_, RADAR_BUFF_MAX_LEN, 0.5f);
    float jitter_midean = median(s_buff_jitter_, RADAR_BUFF_MAX_LEN);

    for (int i = 0; i < buff_max_size; i++) {
        uint32_t index = (s_buff_count_ - 1 - i) % RADAR_BUFF_MAX_LEN;
        if (s_buff_wander_[index] * predict_someone_sensitivity_ > predict_someone_threshold_) {
            someone_count++;
        }

        float move_detection_value = jitter_midean / predict_move_sensitivity_;
        if (move_detection_value > 0.1f) {
            move_count++;
        } else {
            move_count = 0;
        }
    }

    if (someone_count >= 1 || move_count >= 1) {
        room_status = true;
    }

    float move_detection_value = jitter_midean / predict_move_sensitivity_;
    if (move_detection_value > 0.1f) {
        human_status = true;
    } else {
        human_status = false;
    }

    if (!s_count_) {
        ESP_LOGI(TAG, "================ RADAR RECV ================");
        ESP_LOGI(TAG, "type,sequence,timestamp,waveform_wander,someone_threshold,someone_status,waveform_jitter,move_threshold,move_status");
    }

    uint32_t current_time = esp_log_timestamp();

    if (s_train_start_) {
        s_last_move_time_ = current_time;
        s_last_someone_time_ = current_time;
        return;
    }

    if (current_time - s_last_radar_data_time_ >= 333) {
        printf("RADAR_DADA,%u,%u,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%d\n",
               (unsigned int)(s_count_++), (unsigned int)current_time,
               info->waveform_wander, wander_average, predict_someone_threshold_ / predict_someone_sensitivity_, room_status,
               info->waveform_jitter, jitter_midean, jitter_midean / predict_move_sensitivity_, human_status);
        s_last_radar_data_time_ = current_time;
    }

    s_breath_jitter_buf_[s_breath_buf_idx_] = info->waveform_jitter;
    s_breath_buf_idx_ = (s_breath_buf_idx_ + 1) % BREATH_WINDOW_SIZE;
    if (s_breath_sample_count_ < BREATH_WINDOW_SIZE) {
        s_breath_sample_count_++;
    }

    static uint32_t s_breath_update_time = 0;
    float breath_rate = 0.0f;

    if (current_time - s_breath_update_time >= 1000) {
        s_breath_update_time = current_time;

        if (s_breath_sample_count_ >= 20) {
            float sum = 0, sumsq = 0, max_val = 0, min_val = 1e9f;
            for (uint32_t i = 0; i < s_breath_sample_count_; i++) {
                sum += s_breath_jitter_buf_[i];
                sumsq += s_breath_jitter_buf_[i] * s_breath_jitter_buf_[i];
                if (s_breath_jitter_buf_[i] > max_val) max_val = s_breath_jitter_buf_[i];
                if (s_breath_jitter_buf_[i] < min_val) min_val = s_breath_jitter_buf_[i];
            }
            float mean = sum / s_breath_sample_count_;
            float variance = (sumsq / s_breath_sample_count_) - (mean * mean);
            float std_dev = (variance > 0) ? sqrtf(variance) : 0;

            if (!room_status || !human_status) {
                s_noise_floor_ = s_noise_floor_ * 0.95f + mean * 0.05f;
            }

            s_breath_threshold_ = s_noise_floor_ + 2.0f * std_dev + 0.01f;

            bool breath_detected = false;
            if (room_status && (mean > s_breath_threshold_)) {
                float max_corr = 0;
                int best_lag = 0;

                for (int lag = 8; lag < 20 && lag < (int)s_breath_sample_count_ / 2; lag++) {
                    float corr = 0;
                    for (uint32_t i = 0; i < s_breath_sample_count_ - lag; i++) {
                        float dx = s_breath_jitter_buf_[i] - mean;
                        float dy = s_breath_jitter_buf_[i + lag] - mean;
                        corr += dx * dy;
                    }
                    corr /= (s_breath_sample_count_ - lag);
                    if (corr > max_corr) {
                        max_corr = corr;
                        best_lag = lag;
                    }
                }

                if (max_corr > std_dev * std_dev * 0.5f) {
                    breath_detected = true;
                    float breath_rate_raw = 1000.0f / best_lag;
                    s_breath_rate_smooth_ = s_breath_rate_smooth_ * 0.7f + breath_rate_raw * 0.3f;
                    breath_rate = s_breath_rate_smooth_;
                }
            }

            if (!breath_detected) {
                s_breath_rate_smooth_ = s_breath_rate_smooth_ * 0.95f;
                breath_rate = s_breath_rate_smooth_;
            }
        }

        int people_count = (room_status && human_status) ? 1 : 0;
        bool final_room_status = s_last_room_status_ || room_status;
        bool final_human_status = human_status;

        if (current_time - s_last_update_time_ >= 3000) {
            ESP_LOGI(TAG, "雷达检测结果: 有人=%s, 移动=%s, 呼吸率=%.2f, 人数=%d",
                     final_room_status ? "是" : "否",
                     final_human_status ? "是" : "否",
                     breath_rate,
                     people_count);

            if (callback_) {
                callback_(final_room_status, final_human_status, breath_rate, people_count);
            }

            s_last_update_time_ = current_time;
            s_last_room_status_ = room_status;
            s_last_human_status_ = human_status;
        } else {
            if (room_status) s_last_room_status_ = true;
            s_last_human_status_ = human_status;
        }
    }
}

float CsiRadar::trimmean(const float* array, size_t len, float percent) {
    if (len == 0) return 0;
    if (len > RADAR_BUFF_MAX_LEN) len = RADAR_BUFF_MAX_LEN;

    memcpy(s_sort_buffer_, array, len * sizeof(float));

    for (int i = 0; i < (int)len - 1; i++) {
        for (int j = 0; j < (int)len - i - 1; j++) {
            if (s_sort_buffer_[j] > s_sort_buffer_[j + 1]) {
                float temp = s_sort_buffer_[j];
                s_sort_buffer_[j] = s_sort_buffer_[j + 1];
                s_sort_buffer_[j + 1] = temp;
            }
        }
    }

    size_t trim_count = (size_t)(len * percent);
    float sum = 0;
    size_t count = 0;

    for (size_t i = trim_count; i < len - trim_count; i++) {
        sum += s_sort_buffer_[i];
        count++;
    }

    return count > 0 ? sum / count : 0;
}

float CsiRadar::median(const float* a, size_t len) {
    if (len == 0) return 0;
    if (len > RADAR_BUFF_MAX_LEN) len = RADAR_BUFF_MAX_LEN;

    memcpy(s_sort_buffer_, a, len * sizeof(float));

    for (int i = 0; i < (int)len - 1; i++) {
        for (int j = 0; j < (int)len - i - 1; j++) {
            if (s_sort_buffer_[j] > s_sort_buffer_[j + 1]) {
                float temp = s_sort_buffer_[j];
                s_sort_buffer_[j] = s_sort_buffer_[j + 1];
                s_sort_buffer_[j + 1] = temp;
            }
        }
    }

    return len % 2 ? s_sort_buffer_[len / 2] : (s_sort_buffer_[len / 2 - 1] + s_sort_buffer_[len / 2]) / 2;
}

#endif
