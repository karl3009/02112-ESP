#ifndef BUZZER_H
#define BUZZER_H

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Buzzer configuration macros
#define BUZZ_TIMER LEDC_TIMER_1
#define BUZZ_MODE LEDC_LOW_SPEED_MODE
#define BUZZ_OUTPUT_IO (0)
#define BUZZ_CHANNEL LEDC_CHANNEL_4
#define BUZZ_DUTY_RES LEDC_TIMER_13_BIT
#define BUZZ_DUTY (4096)
#define BUZZ_FREQUENCY (1000)

void buzzer_init();
void buzzer_play_tone(float frequency, int duration_ms);
void buzzer_stop();

#endif // BUZZER_H
