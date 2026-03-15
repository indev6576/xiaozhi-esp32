#ifndef CSI_RADAR_H
#define CSI_RADAR_H

#include <functional>
#include <string>

#if CONFIG_USE_CSI_RADAR

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_radar.h"

class CsiRadar {
public:
    using RadarCallback = std::function<void(bool someone, bool moving, float breath_rate, int people_count)>;

    CsiRadar();
    ~CsiRadar();

    bool Start();
    void Stop();

    void SetCallback(RadarCallback callback) { callback_ = callback; }

    static CsiRadar* GetInstance() { return instance_; }

    /**
     * Global function to start CSI Radar if needed
     * This can be called from other components (e.g., Ml307Board)
     */
    static void StartIfNeeded();

private:
    static void RadarCallbackImpl(void* ctx, const wifi_radar_info_t* info);

    void ProcessRadarData(const wifi_radar_info_t* info);

    static CsiRadar* instance_;
    RadarCallback callback_;

    static bool s_initialized_;

    static float s_buff_wander_[25];
    static float s_buff_jitter_[25];
    static uint32_t s_buff_count_;

    static float predict_someone_threshold_;
    static float predict_someone_sensitivity_;
    static float predict_move_threshold_;
    static float predict_move_sensitivity_;
    static uint32_t predict_buff_size_;
    static uint32_t predict_outliers_number_;

    static float s_breath_jitter_buf_[60];
    static uint32_t s_breath_buf_idx_;
    static uint32_t s_breath_sample_count_;
    static float s_noise_floor_;
    static float s_breath_threshold_;
    static float s_breath_rate_smooth_;

    static uint32_t s_last_update_time_;
    static uint32_t s_last_move_time_;
    static uint32_t s_last_someone_time_;
    static uint32_t s_last_radar_data_time_;
    static uint32_t s_count_;

    static bool s_last_room_status_;
    static bool s_last_human_status_;
    static bool s_train_start_;

    static float trimmean(const float* array, size_t len, float percent);
    static float median(const float* a, size_t len);
};

#endif

#endif
