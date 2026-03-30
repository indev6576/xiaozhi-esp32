#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "boards/x1m0-oled-dule/config.h"
#include "led/single_led.h"
#include "led/gpio_led.h"

#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <wifi_station.h>

#define TAG "X1M0_OLED_DULE"

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);
bool is_sensor_heated_ = false;
class X1M0_OLED_DULE : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    void init_spk_led()
    {
        // 增加延迟，等待上电稳定
        vTaskDelay(pdMS_TO_TICKS(10));
    
        gpio_reset_pin(AUDIO_I2S_SPK_GPIO_MODE);
        /* Set the GPIO as a push/pull output */
        gpio_set_direction(AUDIO_I2S_SPK_GPIO_MODE, GPIO_MODE_OUTPUT);
        gpio_set_level(AUDIO_I2S_SPK_GPIO_MODE, 1);
    
        // 增加延迟，等待上电稳定
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        boot_button_.OnDoubleClick([this]() {
            GetAudioCodec()->SetOutputVolume(100);
        });

        touch_button_.OnClick([this]() {
            // Application::GetInstance().StartListening();
        ESP_LOGW(TAG, "OnClick");
        });
        touch_button_.OnDoubleClick([this]() {
            // Application::GetInstance().StopListening();
        ESP_LOGW(TAG, "OnDoubleClick");
        });
        touch_button_.OnLongPress([this]() {
            // Application::GetInstance().StopListening();
        ESP_LOGW(TAG, "OnLongPress");
        });
        touch_button_.OnMultipleClick([this]() {
            // Application::GetInstance().StopListening();
        ESP_LOGW(TAG, "OnMultipleClick");
        });
    }


public:
    X1M0_OLED_DULE() : 
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(FUNCTION_BUTTON_GPIO) {
            // 设置全局日志级别为ESP_LOG_NONE，即不输出任何日志
            // esp_log_level_set("*", ESP_LOG_NONE);
            
        init_spk_led();
        InitializeButtons();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplexPdm audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    #ifdef CONFIG_IDF_TARGET_M0
    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }
    #endif


};

DECLARE_BOARD(X1M0_OLED_DULE);
