#include "csi_radar.h"
#include "sdkconfig.h"

#if CONFIG_USE_CSI_RADAR
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <string.h>
#include <math.h>

#define TAG "CsiRadar"

#define RADAR_BUFF_MAX_LEN 25
#define BREATH_WINDOW_SIZE 60

CsiRadar* CsiRadar::instance_ = nullptr;
bool CsiRadar::s_initialized_ = false;
bool CsiRadar::s_train_start_ = false;
bool CsiRadar::s_last_room_status_ = false;
bool CsiRadar::s_last_human_status_ = false;

float CsiRadar::s_buff_wander_[RADAR_BUFF_MAX_LEN] = {0};
float CsiRadar::s_buff_jitter_[RADAR_BUFF_MAX_LEN] = {0};
uint32_t CsiRadar::s_buff_count_ = 0;

float CsiRadar::predict_someone_threshold_ = 0;
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

bool CsiRadar::Start() {
    if (s_initialized_) {
        ESP_LOGW(TAG, "CSI Radar already initialized");
        return true;
    }

    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get WiFi mode: %d", err);
        return false;
    }

    if (mode != WIFI_MODE_STA) {
        esp_err_t err2 = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set WiFi mode to STA: %d", err2);
            return false;
        }

        err2 = esp_wifi_start();
        if (err2 != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start WiFi: %d", err2);
            return false;
        }
    }

    esp_radar_csi_config_t csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT();
    esp_radar_wifi_config_t wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT();
    esp_radar_dec_config_t dec_config = ESP_RADAR_DEC_CONFIG_DEFAULT();

    csi_config.csi_recv_interval = 50;
    csi_config.csi_filtered_cb = NULL;
    memcpy(csi_config.filter_mac, "\x1a\x00\x00\x00\x00\x00", 6);

    dec_config.wifi_radar_cb = RadarCallbackImpl;

    err = esp_radar_wifi_init(&wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init wifi radar: %d", err);
        return false;
    }

    err = esp_radar_csi_init(&csi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init CSI: %d", err);
        return false;
    }

    err = esp_radar_dec_init(&dec_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init decoder: %d", err);
        return false;
    }

    err = esp_radar_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start radar: %d", err);
        return false;
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

    esp_radar_stop();
    s_initialized_ = false;
    ESP_LOGI(TAG, "CSI Radar stopped");
}

void CsiRadar::RadarCallbackImpl(void* ctx, const wifi_radar_info_t* info) {
    if (instance_) {
        instance_->ProcessRadarData(info);
    }
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
            float range = max_val - min_val;

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

                if (max_corr > 0.3f * variance && best_lag > 0) {
                    breath_detected = true;
                    float period_sec = best_lag / 3.0f;
                    breath_rate = 60.0f / period_sec;
                    breath_rate = s_breath_rate_smooth_ * 0.7f + breath_rate * 0.3f;
                    s_breath_rate_smooth_ = breath_rate;

                    if (breath_rate < 8) breath_rate = 8;
                    if (breath_rate > 30) breath_rate = 30;
                }
            }

            if (room_status && !breath_detected && range > 0.03f) {
                breath_rate = mean * 500;
                if (breath_rate < 8) breath_rate = 8;
                if (breath_rate > 30) breath_rate = 30;
            }

            ESP_LOGD(TAG, "呼吸检测: mean=%.4f, std=%.4f, range=%.4f, noise=%.4f, thresh=%.4f, rate=%.1f",
                     mean, std_dev, range, s_noise_floor_, s_breath_threshold_, breath_rate);
        }
    }

    if (current_time - s_last_update_time_ >= 1000) {
        bool final_room_status = s_last_room_status_ || room_status;
        bool final_human_status = human_status;
        int people_count = final_room_status ? 1 : 0;

        if (final_human_status) {
            ESP_LOGW(TAG, "雷达检测结果: 有人=%s, 移动=%s, 呼吸率=%.2f, 人数=%d",
                     final_room_status ? "是" : "否",
                     final_human_status ? "是" : "否",
                     breath_rate,
                     people_count);
        } else {
            ESP_LOGI(TAG, "雷达检测结果: 有人=%s, 移动=%s, 呼吸率=%.2f, 人数=%d",
                     final_room_status ? "是" : "否",
                     final_human_status ? "是" : "否",
                     breath_rate,
                     people_count);
        }

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

float CsiRadar::trimmean(const float* array, size_t len, float percent) {
    if (len == 0) return 0;

    float* sorted = new float[len];
    memcpy(sorted, array, len * sizeof(float));

    for (int i = 0; i < (int)len - 1; i++) {
        for (int j = 0; j < (int)len - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    size_t trim_count = (size_t)(len * percent);
    float sum = 0;
    size_t count = 0;

    for (size_t i = trim_count; i < len - trim_count; i++) {
        sum += sorted[i];
        count++;
    }

    delete[] sorted;
    return count > 0 ? sum / count : 0;
}

float CsiRadar::median(const float* a, size_t len) {
    if (len == 0) return 0;

    float* sorted = new float[len];
    memcpy(sorted, a, len * sizeof(float));

    for (int i = 0; i < (int)len - 1; i++) {
        for (int j = 0; j < (int)len - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    float result = sorted[len / 2];
    delete[] sorted;
    return result;
}

#endif
