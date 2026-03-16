#include "dual_network_board.h"
#include "codecs/es8312_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#if CONFIG_USE_CSI_RADAR
#include "wifi_board.h"
#endif

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#define TAG "X1ML307Board"

class X1ML307Board : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    Button sensor_button_;

#if CONFIG_USE_CSI_RADAR
    WifiBoard csi_wifi_board_;
#endif

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeGPIO() {
        gpio_config_t io_conf_1 = {.pin_bit_mask = (1ULL << MCU_VCC_CTL),
                                   .mode = GPIO_MODE_OUTPUT,
                                   .pull_up_en = GPIO_PULLUP_DISABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};
        gpio_config(&io_conf_1);

        SwithOnOff(true);
    }

    void SwithOnOff(bool onOff) {
        gpio_set_level(MCU_VCC_CTL, onOff);
        // gpio_hold_en(MCU_VCC_CTL);
        ESP_LOGI(TAG, "SwithOnOff: %d", onOff);
        ESP_LOGI(TAG, "MCU_VCC_CTL: %d", gpio_get_level(MCU_VCC_CTL));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            ESP_LOGI(TAG, "Boot button clicked");
            if (app.GetInterComStatus())
            {
                app.SendMessage("{\"type\":\"intercom_dev2serv\",\"action\":\"stop\",\"target\":\"1658\"}");
                app.SetInterCom(false);
            }
            else {
                Application::GetInstance().ToggleChatState();
            }
            
        });
        boot_button_.OnLongPress([this]() {
            SwithOnOff(false);
            ESP_LOGI(TAG, "Boot button long pressed");
        });
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            auto& board = Board::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting || app.GetDeviceState() == kDeviceStateWifiConfiguring) {
                this->SwitchNetworkType(); 
            }
            else { 
                app.SendMessage("{\"type\":\"intercom_dev2serv\",\"action\":\"start\",\"target\":\"1658\"}");
                app.SetInterCom(true);
            }
        });

        sensor_button_.OnPressDown([this]() {
            auto& app = Application::GetInstance();
            app.SendMessage("{\"type\":\"sensor_dev2serv\",\"people_in\":\"yes\"}");
            ESP_LOGI(TAG, "Sensor detect people in");
        });
        sensor_button_.OnPressUp([this]() {
            auto& app = Application::GetInstance();
            app.SendMessage("{\"type\":\"sensor_dev2serv\",\"people_in\":\"no\"}");
            ESP_LOGI(TAG, "Sensor detect people out");
        });
    }


public:
    X1ML307Board() : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, GPIO_NUM_NC), boot_button_(BOOT_BUTTON_GPIO, true), sensor_button_(SENSOR_BUTTON_GPIO, true){

        InitializeCodecI2c();
        InitializeGPIO();
        InitializeButtons();

#if CONFIG_USE_CSI_RADAR
        ESP_LOGI(TAG, "Starting CSI WiFi board for radar...");
        csi_wifi_board_.StartNetwork();
#endif

        ESP_LOGI(TAG, "X1ML307Board initialized");
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
         static Es8312AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }
};


DECLARE_BOARD(X1ML307Board);
