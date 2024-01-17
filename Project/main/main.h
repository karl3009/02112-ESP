#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>

// FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ESP includes
#include "esp_system.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_spiffs.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "esp_timer.h"

// Driver includes
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/adc.h"

// Display libraries
#include "ssd1306.h"
#include "font8x8_basic.h"

// Sensor libraries
#include <am2320.h>
#include "Adafruit_Stemma_soil_sensor.h"

#define tag "EXAMPLE_ALL"

#define RED_LED_GPIO 9
#define BUTTON_1_GPIO_PIN 18
#define BUTTON_2_GPIO_PIN 7

#define I2C_MASTER_FREQ_HZ 50000 // Reduce it to 50000 if the temperature/umidity sensor fails
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_SDA_GPIO 2
#define I2C_MASTER_SCL_GPIO 3
#define I2C_NUM 0

// PWM library to control LED intensity and/or play tone on buzzer
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_RED (4)   // Define the output GPIO for red
#define LEDC_OUTPUT_IO_GREEN (5) // Define the output GPIO for green
#define LEDC_OUTPUT_IO_BLUE (6)  // Define the output GPIO for blue
#define LEDC_CHANNEL_RED LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE LEDC_CHANNEL_2
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (1000)           // Frequency in Hertz. Set frequency at 1 kHz

#define BUZZ_TIMER LEDC_TIMER_1
#define BUZZ_MODE LEDC_LOW_SPEED_MODE
#define BUZZ_OUTPUT_IO (0) // Define the output GPIO for red
#define BUZZ_CHANNEL LEDC_CHANNEL_4
#define BUZZ_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define BUZZ_DUTY (4096)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define BUZZ_FREQUENCY (1000)           // Frequency in Hertz. Set frequency at 1 kHz

//
static const char *TAG = "FileSystem";


int btn1;
int btn2;

// Global variables
int soil_moisture_value;
float soil_temperature_value;
float air_humidity_value;
float air_temperature_value;
int light_value;
char light_quality[32];
char air_humidity_quality[32];
char air_temperature_quality[32];
char soil_moisture_quality[32];
char soil_temperature_quality[32];
int soil_m_bad = 0;
int soil_t_bad = 0;
int air_h_bad = 0;
int air_t_bad = 0;
int logging = 0;
int good_condition = 0;
int64_t baseTime;
int64_t currentTime;

#endif