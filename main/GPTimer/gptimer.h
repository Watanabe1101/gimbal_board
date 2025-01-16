#pragma once

#include "driver/gptimer.h"
//APBクロック80Mhzを使用
class GPTimer {
private:
    gptimer_handle_t timer;
    gptimer_alarm_config_t alarm_config;
    gptimer_alarm_cb_t userCallback;

public:
    GPTimer() : timer(nullptr), userCallback(nullptr) {}

    bool init(uint32_t resolution_hz, uint64_t alarm_count) {
        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = resolution_hz,
        };
        if (gptimer_new_timer(&timer_config, &timer) != ESP_OK) {
            return false;
        }

        alarm_config = {
            .alarm_count = alarm_count,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = true
            }
        };
        if (gptimer_set_alarm_action(timer, &alarm_config) != ESP_OK) {
            return false;
        }

        return true;
    }

    bool registerCallback(gptimer_alarm_cb_t callback) {
        userCallback = callback;
        gptimer_event_callbacks_t cbs = {
            .on_alarm = internalCallback
        };
        return (gptimer_register_event_callbacks(timer, &cbs, this) == ESP_OK);
    }

    bool start() {
        return (gptimer_enable(timer) == ESP_OK && gptimer_start(timer) == ESP_OK);
    }

    bool stop() {
        // タイマーを停止し、無効化する
        bool stopped = (gptimer_stop(timer) == ESP_OK);
        return (stopped && gptimer_disable(timer) == ESP_OK);
    }

private:
    static bool IRAM_ATTR internalCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
        GPTimer* gpTimer = static_cast<GPTimer*>(user_ctx);
        if (gpTimer->userCallback) {
            return gpTimer->userCallback(timer, edata, user_ctx);
        }
        return false;
    }
};