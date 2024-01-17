#include "buzzer.h"

static const char *TAG = "Buzzer";

void buzzer_init() {
    ledc_timer_config_t ledc_timer_buzz = {
        .speed_mode = BUZZ_MODE,
        .duty_resolution = BUZZ_DUTY_RES,
        .timer_num = BUZZ_TIMER,
        .freq_hz = BUZZ_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_buzz));

    ledc_channel_config_t ledc_channel_buzz = {
        .speed_mode = BUZZ_MODE,
        .channel = BUZZ_CHANNEL,
        .timer_sel = BUZZ_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZ_OUTPUT_IO,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_buzz));

    ESP_LOGI(TAG, "Buzzer initialized");

       // Set duty
    ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 50 * 4095 / 100)); // 50% duty //Can change 4096 to different sound qualities like "3*4096/4" which gives 75%
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));
}

void buzzer_play_tone(float frequency, int duration_ms) {
    ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, frequency);
    ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, BUZZ_DUTY);
    ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

void buzzer_stop() {
    ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 0);
    ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL);
}
