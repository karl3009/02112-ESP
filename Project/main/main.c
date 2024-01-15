/*
 * A components demo for course 02112
 */
#include <unistd.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include <string.h>
#include "esp_log.h"

// Driver libraries
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_spiffs.h"
#include "esp_log.h"

// Display libraies
#include "ssd1306.h"
#include "font8x8_basic.h"

// Temperature/humidity sensor library
#include <am2320.h>

// Stemma soil sensor library
#include "Adafruit_Stemma_soil_sensor.h"

// Interupt
#include "freertos/queue.h"

// Buzzer

#define tag "EXAMPLE_ALL"

#define RED_LED_GPIO 8
#define BUTTON_1_GPIO_PIN 18
#define BUTTON_2_GPIO_PIN 19

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

// Custom:
QueueHandle_t interputQueue;

static const char *TAG = "FileSystem";

volatile bool buttonPressed = false;

void print_info()
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        printf("Get flash size failed");
        return;
    }

    printf("%luMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size());
}

void display_menu(SSD1306_t *dev, const char *message)
{
    ESP_LOGI(tag, "Displaying menu on OLED.");
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
    ssd1306_display_text(dev, 0, message, strlen(message), false);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void display_demo()
{
    SSD1306_t dev;
    int center, top; //, bottom;
    // char lineChar[20];

    // Initialize the display (shared i2c) only once after boot.
    i2c_master_shared_i2c_init(&dev);

    // Uncomment this if you want to flip the display
    // dev._flip = true;

    ssd1306_init(&dev, 128, 64);

    ESP_LOGI(tag, "Writing a large Hello (notice the 2 different colours).");
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text_x3(&dev, 0, "Hello", 5, false);

    // delay 2 seconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(tag, "Writing some text line by line (notice the 2 different colours)");
    top = 2;
    center = 3;
    // bottom = 8;
    ssd1306_display_text(&dev, 0, "SSD1306 128x64", 14, false);
    ssd1306_display_text(&dev, 1, "ABCDEFGHIJKLMNOP", 16, false);
    ssd1306_display_text(&dev, 2, "abcdefghijklmnop", 16, false);
    ssd1306_display_text(&dev, 3, "Hello World!!", 13, false);
    // You can clear lines as follows (uncomment to see the effect)
    // ssd1306_clear_line(&dev, 4, true);
    // ssd1306_clear_line(&dev, 5, true);
    // ssd1306_clear_line(&dev, 6, true);
    // ssd1306_clear_line(&dev, 7, true);
    ssd1306_display_text(&dev, 4, "SSD1306 128x64", 14, true);
    ssd1306_display_text(&dev, 5, "ABCDEFGHIJKLMNOP", 16, true);
    ssd1306_display_text(&dev, 6, "abcdefghijklmnop", 16, true);
    ssd1306_display_text(&dev, 7, "Hello World!!", 13, true);

    // delay 2 seconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Display Count Down
    ESP_LOGI(tag, "Displaying a count down on top of a black square.");
    uint8_t image[24];
    memset(image, 0, sizeof(image));
    ssd1306_display_image(&dev, top, (6 * 8 - 1), image, sizeof(image));
    ssd1306_display_image(&dev, top + 1, (6 * 8 - 1), image, sizeof(image));
    ssd1306_display_image(&dev, top + 2, (6 * 8 - 1), image, sizeof(image));
    for (int font = 0x35; font > 0x30; font--)
    {
        memset(image, 0, sizeof(image));
        ssd1306_display_image(&dev, top + 1, (7 * 8 - 1), image, 8);
        memcpy(image, font8x8_basic_tr[font], 8);
        if (dev._flip)
            ssd1306_flip(image, 8);
        ssd1306_display_image(&dev, top + 1, (7 * 8 - 1), image, 8);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Horizontal Scroll
    ESP_LOGI(tag, "Horizontal scrolling.");
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, center, "Horizontal", 10, false);
    ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_STOP);

    // Vertical Scroll
    ESP_LOGI(tag, "Vertical scrolling.");
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, center, "Vertical", 8, false);
    ssd1306_hardware_scroll(&dev, SCROLL_DOWN);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_UP);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_STOP);

    // Invert
    ESP_LOGI(tag, "Invert colours.");
    ssd1306_clear_screen(&dev, true);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, center, "  Good Bye!!", 12, true);
}

void temperaure_humidity(float *temp, int *hum)
{
    i2c_dev_t dev = {0};

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    float temperature, humidity;

    esp_err_t res = am2320_get_rht(&dev, &temperature, &humidity);
    // 500 ms delay
    vTaskDelay((500) / portTICK_PERIOD_MS);
    *temp = temperature;
    *hum = humidity;
}

void stemma_soil(int *moisture_result, float *temperature_result)
{
    int ret = ESP_OK;
    uint16_t moisture_value = 0;
    float temperature_value = 0;

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(adafruit_stemma_soil_sensor_shared_i2c_init());

    ret = adafruit_stemma_soil_sensor_read_moisture(I2C_NUM, &moisture_value);

    if (ret == ESP_OK)
    {
        ESP_LOGI(tag, "Moisture value: \t%u", moisture_value - 650);
    }

    ret = adafruit_stemma_soil_sensor_read_temperature(I2C_NUM, &temperature_value);

    if (ret == ESP_OK)
    {
        ESP_LOGI(tag, "Temperature value: \t%.1f", temperature_value);
    }

    // 500 ms delay
    vTaskDelay((500) / portTICK_PERIOD_MS);
    *moisture_result = moisture_value - 650;
    *temperature_result = temperature_value;
}
void light_adc(int *light_result)
{
    // Configuring the ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11); // ADC1_CHANNEL_0 is on GPIO0 (GPIOzero)
    int val = adc1_get_raw(ADC1_CHANNEL_1);
    ESP_LOGI(tag, "Light sensor ADC value: %d", val);
    // 500 ms delay
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1 secon
    *light_result = val;
}

void sensors_task(void *params)
{
    // Temperature_humidity
    i2c_dev_t dev = {0};

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    float temperature, humidity;

    // Stemma_soil
    int ret = ESP_OK;
    uint16_t moisture_value = 0;
    float temperature_value = 0;

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(adafruit_stemma_soil_sensor_shared_i2c_init());

    while (1)
    {
        // Stemma
        ret = adafruit_stemma_soil_sensor_read_moisture(I2C_NUM, &moisture_value);

        if (ret == ESP_OK)
        {
            ESP_LOGI(tag, "Moisture value: \t%u", moisture_value - 650);
        }

        ret = adafruit_stemma_soil_sensor_read_temperature(I2C_NUM, &temperature_value);

        if (ret == ESP_OK)
        {
            ESP_LOGI(tag, "Temperature value: \t%.1f", temperature_value);
        }

        // Humidity
        esp_err_t res = am2320_get_rht(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(tag, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
        else
            ESP_LOGE(tag, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void init_i2c()
{
    // Initialize common I2C port for display, soil sensor, and temperature/umidity sensor
    // Initialized it as follows only once here in the main, then use the shared_init
    // functions for the different components as shown in this demo (see _demo functions).

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_GPIO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));
}

void temperaure_humidity_demo()
{
    i2c_dev_t dev = {0};

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    float temperature, humidity;

    for (int i = 0; i < 20; i++)
    {
        esp_err_t res = am2320_get_rht(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(tag, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
        else
            ESP_LOGE(tag, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        // 500 ms delay
        vTaskDelay((500) / portTICK_PERIOD_MS);
    }
}

void stemma_soil_demo()
{
    int ret = ESP_OK;
    uint16_t moisture_value = 0;
    float temperature_value = 0;

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(adafruit_stemma_soil_sensor_shared_i2c_init());

    for (int i = 0; i < 10; i++)
    {
        ret = adafruit_stemma_soil_sensor_read_moisture(I2C_NUM, &moisture_value);

        if (ret == ESP_OK)
        {
            ESP_LOGI(tag, "Moisture value: \t%u", moisture_value - 650);
        }

        ret = adafruit_stemma_soil_sensor_read_temperature(I2C_NUM, &temperature_value);

        if (ret == ESP_OK)
        {
            ESP_LOGI(tag, "Temperature value: \t%.1f", temperature_value);
        }

        // 500 ms delay
        vTaskDelay((500) / portTICK_PERIOD_MS);
    }
}

void led_fade_demo()
{
    // Prepare and then apply the LEDC PWM timer configuration (one time can drive multiple channels)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 1 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_red = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RED,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_RED,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));

    ledc_channel_config_t ledc_channel_green = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_GREEN,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_GREEN,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));

    ledc_channel_config_t ledc_channel_blue = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_BLUE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_BLUE,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue));

    // Now the initialization is done
    ESP_LOGI(tag, "Initialization complete. Now fading the 3 colours.");

    // Ramp down duty cycle
    for (int duty = 8192 - 1; duty >= 8192 - 1 - 2048; duty = duty - 2)
    {
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, duty));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, (8192 - 1) + (8192 - 1 - 2048) - duty));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
        // 1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);
        // printf("%d\n", duty);
    }

    // Ramp up duty cycle
    for (int duty = 8192 - 1 - 2048; duty < 8192 - 1; duty = duty + 2)
    {
        // Set duty
        // Set duty
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, duty));
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, (8192-1) + (8192-1-2048) - duty)); //Leaving green off
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty)); //Leaving blue on
        //  Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED));
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN)); //Leaving green off
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE)); //Leaving blue on
        // 1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);
        // printf("%d\n", duty);
    }
}

void buzzer_demo()
{
    // Prepare and then apply the LEDC PWM timer configuration (we use it for the buzzer)
    ledc_timer_config_t ledc_timer_buzz = {
        .speed_mode = BUZZ_MODE,
        .duty_resolution = BUZZ_DUTY_RES,
        .timer_num = BUZZ_TIMER,
        .freq_hz = BUZZ_FREQUENCY, // Set output frequency at 1 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_buzz));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_buzz = {
        .speed_mode = BUZZ_MODE,
        .channel = BUZZ_CHANNEL,
        .timer_sel = BUZZ_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZ_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_buzz));

    // Now the initialization is done
    ESP_LOGI(tag, "Initialization complete. Playing 3 tones.");

    // Set duty
    ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 3 * 4096 / 4)); // 75% duty

    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));

    ESP_LOGI(tag, "Rick Astley - Never Gonna Give You Up");

    // Define note frequencies and durations for the chorus of "Never Gonna Give You Up"
    double notes[] = {659, 831, 988, 1319, 988, 1319};
    int durations[] = {300, 300, 300, 500, 300, 1000};

    for (int i = 0; i < 7; i++)
    {
        ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, notes[i] * 2));
        vTaskDelay((durations[i]) / portTICK_PERIOD_MS);
    }

    // Set duty
    ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 0)); // 0% duty
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));

    ESP_LOGI(tag, "Rick Astley - Never Gonna Give You Up - Finished");
}
void gpio_demo()
{
    gpio_config_t io_conf;

    // Configure RED LED GPIO
    io_conf.pin_bit_mask = (1ULL << RED_LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << BUTTON_1_GPIO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << BUTTON_2_GPIO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    ESP_LOGI(tag, "GPIO configuration done (1 output and 2 inputs)");

    ESP_LOGI(tag, "Blinking LED 5 times.");
    for (int i = 0; i < 5; i++)
    {
        gpio_set_level(RED_LED_GPIO, 1);
        vTaskDelay((500) / portTICK_PERIOD_MS);
        gpio_set_level(RED_LED_GPIO, 0);
        vTaskDelay((500) / portTICK_PERIOD_MS);
    }

    ESP_LOGI(tag, "Testing buttons for the next 10 seconds (push the buttons!).");
    int button_1_status = gpio_get_level(BUTTON_1_GPIO_PIN);
    int button_1_now;
    int button_2_status = gpio_get_level(BUTTON_2_GPIO_PIN);
    int button_2_now;
    for (int i = 0; i < 10000 / 50; i++)
    {
        button_1_now = gpio_get_level(BUTTON_1_GPIO_PIN);
        button_2_now = gpio_get_level(BUTTON_2_GPIO_PIN);
        if (button_1_status != button_1_now)
        {
            if (button_1_now == 0)
            {
                ESP_LOGI(tag, "BUTTON 1 PRESSED (reading 0).");
            }
            else
            {
                ESP_LOGI(tag, "BUTTON 1 RELEASED (reading 1).");
            }
            button_1_status = button_1_now;
        }

        if (button_2_status != button_2_now)
        {
            if (button_2_now == 0)
            {
                ESP_LOGI(tag, "BUTTON 2 PRESSED (reading 0).");
            }
            else
            {
                ESP_LOGI(tag, "BUTTON 2 RELEASED (reading 1).");
            }
            button_2_status = button_2_now;
        }

        vTaskDelay((50) / portTICK_PERIOD_MS);
    }
}

void light_adc_demo()
{

    const char *intensity;

    // Configuring the ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11); // ADC1_CHANNEL_0 is on GPIO0 (GPIOzero)

    for (int i = 0; i < 20; i++)
    {
        int val = adc1_get_raw(ADC1_CHANNEL_1);

        if (val < 100)
        {
            intensity = "Dark";
        }
        else if (val < 250)
        {
            intensity = "Dim";
        }
        else if (val < 600)
        {
            intensity = "Light";
        }
        else if (val < 900)
        {
            intensity = "Bright";
        }
        else
        {
            intensity = "Very bright";
        }

        ESP_LOGI(tag, "Light sensor ADC value: %d: %s", val, intensity);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1 second
    }
}

void setup()
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL << BUTTON_1_GPIO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << BUTTON_2_GPIO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

void initDisplay(SSD1306_t *dev)
{
    i2c_master_shared_i2c_init(dev);

    ssd1306_init(dev, 128, 64);

    ssd1306_clear_screen(dev, false);

    ssd1306_contrast(dev, 0xff);
}

char *display_all(SSD1306_t *dev)
{
#include <string.h>

    const char light_quality[32];
    const char air_humidity_quality[32];
    const char air_temperature_quality[32];
    const char soil_moisture_quality[32];
    const char soil_temperature_quality[32];
    int moisture_result;
    float temperature_result;
    stemma_soil(&moisture_result, &temperature_result);

    float temp;
    int hum;
    temperaure_humidity(&temp, &hum);

    int light_result;

    light_adc(&light_result);

    int badCondition = 0;

    if (moisture_result < 50)
    {
        badCondition = 1;
        strcpy(soil_moisture_quality, "Too dry");
    }
    else if (moisture_result > 200)
    {
        badCondition = 1;
        strcpy(soil_moisture_quality, "Too wet");
    }
    else
    {
        strcpy(soil_moisture_quality, "Good");
    }

    if (temperature_result < 12)
    {
        badCondition = 1;
        strcpy(soil_temperature_quality, "Too cold");
    }
    else if (temperature_result > 27)
    {
        badCondition = 1;
        strcpy(soil_temperature_quality, "Too hot");
    }
    else
    {
        strcpy(soil_temperature_quality, "Good");
    }

    if (hum < 10)
    {
        badCondition = 1;
        strcpy(air_humidity_quality, "Too dry");
    }
    else if (hum > 30)
    {
        badCondition = 1;
        strcpy(air_humidity_quality, "Too wet");
    }
    else
    {
        strcpy(air_humidity_quality, "Good");
    }

    if (light_result < 100)
    {
        strcpy(light_quality, "Dark");
    }
    else if (light_result < 250)
    {
        strcpy(light_quality, "Dim");
    }
    else if (light_result < 600)
    {
        strcpy(light_quality, "Light");
    }
    else if (light_result < 900)
    {
        strcpy(light_quality, "Bright");
    }
    else
    {
        strcpy(light_quality, "Very bright");
    }

    if (badCondition == 1)
    {
        gpio_set_level(RED_LED_GPIO, 1);
    }
    else
    {
        gpio_set_level(RED_LED_GPIO, 0);
    }

    // int center, top; //, bottom;

    char soil_m_result[32];
    char soil_t_result[32];
    sprintf(soil_m_result, "Gnd Mst: %d", moisture_result);
    sprintf(soil_t_result, "Gnd Tmp: %.1fC", temperature_result);

    char air_m_result[32];
    char air_t_result[32];
    sprintf(air_m_result, "Air Hum: %d%%  ", hum);
    sprintf(air_t_result, "Air Tmp: %.1fC", temp);

    char light_display[32];
    sprintf(light_display, "LGT lvl: %d", light_result);

    // ssd1306_clear_line(&dev, 0, true);
    ssd1306_display_text(dev, 2, soil_m_result, strlen(soil_m_result), false);
    ssd1306_display_text(dev, 3, soil_t_result, strlen(soil_t_result), false);
    ssd1306_display_text(dev, 4, air_m_result, strlen(air_t_result), false);
    ssd1306_display_text(dev, 5, air_t_result, strlen(air_m_result), false);
    ssd1306_display_text(dev, 6, light_display, strlen(light_display), false);

    char *str = malloc(40 * sizeof(char)); // Allocate memory
    if (str == NULL)
    {
        // Handle allocation failure
        exit(1);
    }

    sprintf(str, "\n%d, %.1f, %d, %.1f, %d\n", moisture_result, temperature_result, hum, temp, light_result);
    return str;
}

void fileread(char *str)
{

    esp_vfs_spiffs_conf_t config = {
        .base_path = "/storage",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};
    esp_err_t result = esp_vfs_spiffs_register(&config);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result));
        return;
    }

    size_t total = 0, used = 0;
    result = esp_spiffs_info(config.partition_label, &total, &used);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(result));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    FILE *f = fopen("/storage/myfile.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "%s", str);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    f = fopen("/storage/myfile.txt", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    char line[64];
    printf("Read from file: '");
    while (fgets(line, sizeof(line), f) != NULL)
    {
        printf("%s", line);
    }
    printf("'\n");

    fclose(f);
}
void button_switch(SSD1306_t *dev)
{

    int switchState = 3;
    const char *programRunning[] = {"Display values", "air", "light", "soil"}; // Array of strings
    const char currentProgram[32];
    int state = 0;
    int pinNumber, count = 0;

    printf("\nInitializing\n");
    for (int i = 0; i < 3; i++)
    {
        vTaskDelay(50);
        printf("%d/3...\n", i + 1);
    }

    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            count++;
            if (count % 2 == 0)
            {
                if (pinNumber == 19)
                {
                    switchState = (switchState + 1) % 4; // Cycles through 0, 1, 2, 3

                    sprintf(currentProgram, "%d. %s", switchState + 1, programRunning[switchState]);

                    display_menu(dev, currentProgram);
                    printf("Program : %s \t|", currentProgram);
                }

                if (pinNumber == 18)
                {
                    switch (switchState)
                    {
                    case 0:
                        int totalLength = 0;
                        char *x = NULL;
                        char *tempStr;

                        for (int i = 0; i < 10; i++)
                        {
                            tempStr = display_all(dev);
                            if (tempStr != NULL)
                            {
                                totalLength += strlen(tempStr) + 1; // +1 for null-terminator

                                // Reallocate x with the new size
                                char *new_x = realloc(x, totalLength * sizeof(char));
                                if (new_x == NULL)
                                {
                                    // Handle allocation failure
                                    ESP_LOGE(TAG, "Memory reallocation failed for x");
                                    free(tempStr);
                                    free(x);
                                    break;
                                }
                                x = new_x;

                                // Concatenate tempStr to x
                                if (i == 0)
                                {
                                    strcpy(x, tempStr); // Copy first string
                                }
                                else
                                {
                                    strcat(x, tempStr); // Concatenate subsequent strings
                                }

                                free(tempStr); // Free tempStr after using it
                            }
                        }

                        if (x != NULL)
                        {
                            fileread(x);
                            free(x); // Free the concatenated string memory
                        }

                        break;
                    case 1:
                        buzzer_demo();
                        break;
                    case 2:

                        break;
                    case 3:
                        stemma_soil_demo();
                        break;
                    }
                }

                printf("GPIO %d was pressed %d times. The state is %d\n", pinNumber, count / 2, state);
            }
        }
    }
}

void button(gpio_num_t GPIO)
{

    gpio_config_t io_conf;

    // Button GPIO
    gpio_reset_pin(GPIO);
    gpio_set_direction(GPIO, GPIO_MODE_INPUT);
    gpio_pulldown_en(GPIO);
    gpio_pullup_dis(GPIO);
    gpio_set_intr_type(GPIO, GPIO_INTR_ANYEDGE);
    gpio_config(&io_conf);

    interputQueue = xQueueCreate(1, sizeof(int));
    if (interputQueue == NULL)
    {
        // Handle error: Queue creation failed
        ESP_LOGE("Queue Create", "Failed to create queue");
        return;
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO, gpio_interrupt_handler, (void *)GPIO);
}

void app_main(void)
{

    printf("\nPrinting device information:\n");
    print_info();

    init_i2c();
    SSD1306_t dev;

    initDisplay(&dev);

    // Buttons
    button(BUTTON_1_GPIO_PIN);
    button(BUTTON_2_GPIO_PIN);

    button_switch(&dev);
    // esp_restart();
}