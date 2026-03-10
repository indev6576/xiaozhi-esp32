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

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#define TAG "X1ML307Board"

class X1ML307Board : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;

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
            Application::GetInstance().ToggleChatState();
            ESP_LOGI(TAG, "Boot button clicked");
        });
        boot_button_.OnLongPress([this]() {
            SwithOnOff(false);
            ESP_LOGI(TAG, "Boot button long pressed");
        });
    }


public:
    X1ML307Board() : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, GPIO_NUM_NC), boot_button_(BOOT_BUTTON_GPIO, true){

        InitializeCodecI2c();
        InitializeGPIO();
        InitializeButtons();
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
