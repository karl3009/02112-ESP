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

//
static const char *TAG = "FileSystem";

// button variabels
int btn1;
int btn2;
// Global variables
int moisture_result;
float temperature_result;
float hum;
float temp;
int light_result;
char light_quality[32];
char air_humidity_quality[32];
char air_temperature_quality[32];
char soil_moisture_quality[32];
char soil_temperature_quality[32];
int soil_m_bad = 0;
int soil_t_bad = 0;
int air_h_bad = 0;
int air_t_bad = 0;

void display_menu(SSD1306_t *dev, const char *message)
{
    ESP_LOGI(tag, "Displaying menu on OLED.");
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
    ssd1306_display_text(dev, 0, message, strlen(message), false);
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void air_sensor()
{
    i2c_dev_t dev = {0};

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    esp_err_t res = am2320_get_rht(&dev, &temp, &hum);
    // 500 ms delay
    vTaskDelay((500) / portTICK_PERIOD_MS);
}

void soil_sensor(int *moisture_result, float *temperature_result)
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

void light_sensor(int *light_result)
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

void rgb(int soil_m_bad, int soil_t_bad, int air_h_bad, int air_t_bad, int good_condition)
{
    int red_duty = 0;
    int green_duty = 0;
    int blue_duty = 0;
    int scale = 8100 / 255;

    if (soil_m_bad == 1 || air_h_bad == 1)
    {
        blue_duty = scale * 255;
    }
    if (soil_t_bad == 1 || air_t_bad == 1)
    {
        red_duty = scale * 255;
    }
    if (good_condition == 1)
    {
        green_duty = scale * 255;
    }

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
        .duty = red_duty, // Set duty to 0%
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
    ESP_LOGI(tag, "start color.");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, red_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, green_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, blue_duty));

    // 1000 ms delay
}

void buzzer()
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
    ESP_LOGI(tag, "Initialization complete. Playing 7 * 2 tones. First part slow, second part fast");

    // Set duty
    ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 50 * 4095 / 100)); // 50% duty //Can change 4096 to different sound qualities like "3*4096/4" which gives 75%
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));
    // 1000 ms delay
    ESP_LOGI(tag, "Delaying for 1000ms.");
    vTaskDelay((1000) / portTICK_PERIOD_MS);
    // Playing frequency:

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 523.25)); // 50% duty
    ESP_LOGI(tag, "Playing C5 - 523.25 Hz.");
    vTaskDelay((200) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 587.33)); // 50% duty
    ESP_LOGI(tag, "Playing D5 - 587.33 Hz.");
    vTaskDelay((250) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 659.26)); // 50% duty
    ESP_LOGI(tag, "Playing E5 - 659.26 Hz.");
    vTaskDelay((200) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 698.46)); // 50% duty
    ESP_LOGI(tag, "Playing F5 - 698.46 Hz.");
    vTaskDelay((275) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 783.99)); // 50% duty
    ESP_LOGI(tag, "Playing G5 - 783.99 Hz.");
    vTaskDelay((225) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 880.00)); // 50% duty
    ESP_LOGI(tag, "Playing A5 - 880.00 Hz.");
    vTaskDelay((150) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 987.77)); // 50% duty
    ESP_LOGI(tag, "Playing B5 - 987.77 Hz.");
    vTaskDelay((800) / portTICK_PERIOD_MS);

    ESP_LOGI(tag, "Now we come to the fast part.");
    // Second wave of music coming right here, but now faster.
    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 523.25)); // 50% duty
    ESP_LOGI(tag, "Playing C5 - 523.25 Hz.");
    vTaskDelay((75) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 587.33)); // 50% duty
    ESP_LOGI(tag, "Playing D5 - 587.33 Hz.");
    vTaskDelay((100) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 659.26)); // 50% duty
    ESP_LOGI(tag, "Playing E5 - 659.26 Hz.");
    vTaskDelay((125) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 698.46)); // 50% duty
    ESP_LOGI(tag, "Playing F5 - 698.46 Hz.");
    vTaskDelay((100) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 783.99)); // 50% duty
    ESP_LOGI(tag, "Playing G5 - 783.99 Hz.");
    vTaskDelay((125) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 880.00)); // 50% duty
    ESP_LOGI(tag, "Playing A5 - 880.00 Hz.");
    vTaskDelay((100) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 987.77)); // 50% duty
    ESP_LOGI(tag, "Playing B5 - 987.77 Hz.");
    vTaskDelay((125) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 880.00)); // 50% duty
    ESP_LOGI(tag, "Playing A5 - 880.00 Hz.");
    vTaskDelay((100) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 783.99)); // 50% duty
    ESP_LOGI(tag, "Playing G5 - 783.99 Hz.");
    vTaskDelay((75) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 698.46)); // 50% duty
    ESP_LOGI(tag, "Playing F5 - 698.46 Hz.");
    vTaskDelay((50) / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 659.26)); // 50% duty
    ESP_LOGI(tag, "Playing E5 - 659.26 Hz.");
    vTaskDelay((25) / portTICK_PERIOD_MS);

    // Turning buzzer off here
    vTaskDelay((400 / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 0)); // 0% duty
    ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));
    ESP_LOGI(tag, "Buzzer is now off.");
}

void initDisplay(SSD1306_t *dev)
{
    i2c_master_shared_i2c_init(dev);

    ssd1306_init(dev, 128, 64);

    ssd1306_clear_screen(dev, false);

    ssd1306_contrast(dev, 0xff);
}

void receive_data()
{
    soil_sensor(&moisture_result, &temperature_result);
    air_sensor(&temp, &hum);
    light_sensor(&light_result);
}

void evaluate_conditions()
{
    air_t_bad = (temp < 10) ? 1 : (temp > 35) ? 2
                                              : 0;
    soil_m_bad = (moisture_result < 50) ? 1 : (moisture_result > 350) ? 2
                                                                      : 0;
    soil_t_bad = (temperature_result < 12) ? 1 : (temperature_result > 30) ? 2
                                                                           : 0;
    air_h_bad = (hum < 10) ? 1 : (hum > 35) ? 2
                                            : 0;
    air_t_bad = (temp < 10) ? 1 : (temp > 35) ? 2
                                              : 0;

    strcpy(light_quality, (light_result < 100) ? "Dark" : (light_result < 250) ? "Dim"
                                                      : (light_result < 600)   ? "Light"
                                                      : (light_result < 900)   ? "Bright"
                                                                               : "Very bright");

    strcpy(soil_moisture_quality, (soil_m_bad == 1) ? "Dry" : (soil_m_bad == 2) ? "Wet"
                                                                                : "Good");

    strcpy(soil_temperature_quality, (soil_t_bad == 1) ? "Cold" : (soil_t_bad == 2) ? "Hot"
                                                                                    : "Good");

    strcpy(air_humidity_quality, (air_h_bad == 1) ? "Dry" : (air_h_bad == 2) ? "Wet"
                                                                             : "Good");

    strcpy(air_temperature_quality, (air_t_bad == 1) ? "Cold" : (air_t_bad == 2) ? "Hot"
                                                                                 : "Good");
}

void display_values(SSD1306_t *dev)
{
    receive_data();
    char soil_m_result[32];
    char soil_t_result[32];
    sprintf(soil_m_result, "Gnd Mst: %d", moisture_result);
    sprintf(soil_t_result, "Gnd Tmp: %.1fC", temperature_result);

    char air_m_result[32];
    char air_t_result[32];
    sprintf(air_m_result, "Air Hum: %.1f%%  ", hum);
    sprintf(air_t_result, "Air Tmp: %.1fC", temp);

    char light_display[32];
    sprintf(light_display, "LGT lvl: %d", light_result);

    ssd1306_display_text(dev, 2, soil_m_result, strlen(soil_m_result), false);
    ssd1306_display_text(dev, 3, soil_t_result, strlen(soil_t_result), false);
    ssd1306_display_text(dev, 4, air_m_result, strlen(air_t_result), false);
    ssd1306_display_text(dev, 5, air_t_result, strlen(air_m_result), false);
    ssd1306_display_text(dev, 6, light_display, strlen(light_display), false);
    if (soil_m_bad || soil_t_bad || air_h_bad || air_t_bad)
    {
        printf("\nLight: %s, Soil M : %s, Soil T: %s, Air T: %s, Air H: %s\n", light_quality, soil_moisture_quality, soil_temperature_quality, air_temperature_quality, air_humidity_quality);
        gpio_set_level(RED_LED_GPIO, 1);
        rgb(soil_m_bad, soil_t_bad, air_h_bad, air_t_bad, 0);
        buzzer();
    }
    else
    {
        printf("\nLight: %s, Soil M : %s, Soil T: %s, Air T: %s, Air H: %s\n", light_quality, soil_moisture_quality, soil_temperature_quality, air_temperature_quality, air_humidity_quality);
        gpio_set_level(RED_LED_GPIO, 0);
        rgb(0, 0, 0, 0, 1);
    }
}

char *pad_string(char *str, int line_length) {
    int len = strlen(str);
    if (len < line_length) {
        for (int i = len; i < line_length; i++) {
            str[i] = ' '; // Fill the rest of the string with spaces
        }
        str[line_length] = '\0'; // Null-terminate the string
    }
    return str;
}

char *display_condition(SSD1306_t *dev)
{
    receive_data();
    evaluate_conditions();

    char soil_m_result[64];
    char soil_t_result[64];
    sprintf(soil_m_result, "Gnd Mst: %s", soil_moisture_quality);
    sprintf(soil_t_result, "Gnd Tmp: %s", soil_temperature_quality);
    // Ensure each string fills the entire line
    pad_string(soil_m_result, 63);
    pad_string(soil_t_result, 63);

    char air_m_result[64];
    char air_t_result[64];
    sprintf(air_m_result, "Air Hum: %s", air_humidity_quality);
    sprintf(air_t_result, "Air Tmp: %s", air_temperature_quality);
    pad_string(air_m_result, 63);
    pad_string(air_t_result, 63);

    char light_display[64];
    sprintf(light_display, "LGT lvl: %s", light_quality);
    pad_string(light_display, 63);

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

void initfileread()
{
    ESP_LOGI(TAG, "Initializing SPIFFS");
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
}

void write_to_file(char *str)
{
    ESP_LOGI(TAG, "Writing to myfile.txt");

    FILE *f = fopen("/storage/myfile.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open myfile.txt for writing");
        return;
    }
    fprintf(f, "%s", str); // Use fprintf to write the string to the file
    fclose(f);             // Close the file pointer f, not stdout
}

int read_to_file()
{
    FILE *fp = fopen("/storage/myfile.txt", "r");
    if (fp == NULL)
    {
        ESP_LOGE(TAG, "Failed to open myfile.txt for reading");
        return -1; // Indicate failure to open file
    }

    int c;
    while ((c = fgetc(fp)) != EOF) // Read character and check for EOF in one step
    {
        printf("%c", c);
    }

    fclose(fp);
    return 0; // Indicate success
}

char *sensor_data()
{
    static char data_str[128];

    soil_sensor(&moisture_result, &temperature_result);

    air_sensor(&temp, &hum);

    light_sensor(&light_result);

    snprintf(data_str, sizeof(data_str), "%d, %.1f, %.1f, %.1f, %d\n",
             moisture_result, temperature_result, hum, temp, light_result);
    return data_str;
}

char *data_write(int cycles)
{
    int totalLength = 0;
    char *all_data = malloc(1); // Start with an allocated empty string
    if (!all_data)
        return NULL; // Check for allocation failure

    all_data[0] = '\0'; // Initialize the string to be empty

    if (cycles == 0)
    {
        return NULL;
    }
    else
    {
        for (int i = 0; i < cycles; i++)
        {
            char *tempStr = sensor_data();
            if (tempStr != NULL)
            {
                totalLength += strlen(tempStr);
                char *new_all_data = realloc(all_data, totalLength + 1); // +1 for null-terminator
                if (new_all_data == NULL)
                {
                    ESP_LOGE(TAG, "Memory reallocation failed for all_data");
                    free(all_data);
                    return NULL;
                }
                all_data = new_all_data;
                strcat(all_data, tempStr); // Concatenate tempStr to all_data
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 10 seconds
        }
    }

    return all_data;
}

void button_switch(SSD1306_t *dev)
{
    int switchState = 0;
    int lastState = 2;
    const char *programRunning[] = {"Display values", "Display condi.", "Start Logging", "Display data", "Data write?"}; // Array of strings
    const char currentProgram[32];

    sprintf(currentProgram, "%d. %s", switchState + 1, programRunning[switchState]);
    display_menu(dev, currentProgram);
    printf("Program : %s \t|", currentProgram);

    while (true)
    {
        if (btn1)
        {
            btn1 = 0;
            switchState = (switchState + 1) % 5;

            sprintf(currentProgram, "%d. %s", switchState + 1, programRunning[switchState]);

            display_menu(dev, currentProgram);
            printf("Program : %s \t|", currentProgram);
        }
        else if (btn2)
        {
            btn2 = 0;
            switch (switchState)
            {
            case 0:
                lastState = 0;
                display_values(dev);
                break;
            case 1:
                lastState = 1;
                display_condition(dev);
                break;
            case 2:
                // Start data logging for a specified duration
                lastState = 2;
                char *sensorDataString = data_write(10);
                if (sensorDataString != NULL)
                {
                    write_to_file(sensorDataString);
                    free(sensorDataString); // Free the allocated memory
                }

                break;
            case 3:
                lastState = 3;
                read_to_file(); // Implement this function to read and display the logged data
                break;
            case 4:
                lastState = 4;
                data_write(0);
                // stemma_soil_demo();
                break;
            }
        }
        else
        {
            // receive_data();
            if (lastState == 0)
            {
                display_values(dev);
            }
            else if (lastState == 1)
            {
                display_condition(dev);
            }
            else
            {
                printf("waiting\n");
            }
            vTaskDelay(100);
        }
    }
}

void gpio_interrupt_handler_1(void *args)
{
    btn1 = 1;
}
void gpio_interrupt_handler_2(void *args)
{
    btn2 = 1;
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
}

void app_main(void)
{
    // startup
    SSD1306_t dev;
    init_i2c();
    initfileread();
    initDisplay(&dev);

    // Configure RED LED GPIO (make a function maybe)
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL << RED_LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Buttons
    gpio_install_isr_service(0);
    button(BUTTON_1_GPIO_PIN);
    gpio_isr_handler_add(BUTTON_1_GPIO_PIN, gpio_interrupt_handler_1, (void *)BUTTON_1_GPIO_PIN);
    button(BUTTON_2_GPIO_PIN);
    gpio_isr_handler_add(BUTTON_2_GPIO_PIN, gpio_interrupt_handler_2, (void *)BUTTON_2_GPIO_PIN);

    // Go to main loop
    button_switch(&dev);
}
