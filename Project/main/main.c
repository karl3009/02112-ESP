#include "main.h"
#include "bitmaps.h"
#include "buzzer.h"
// #include <displayhappines.c>

void display_menu(SSD1306_t *dev, const char *message)
{
    ESP_LOGI(tag, "Displaying menu on OLED.");
    ssd1306_clear_screen(dev, false);
    ssd1306_contrast(dev, 0xff);
    ssd1306_display_text(dev, 0, message, strlen(message), false);
    if (logging == 1)
    {
        ssd1306_display_text(dev, 1, "Logging...", strlen("Logging..."), false);
    }
    else
    {
        ssd1306_display_text(dev, 1, "Not Logging...", strlen("Not Logging..."), false);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void init_red_led()
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL << RED_LED_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

void air_sensor()
{
    i2c_dev_t dev = {0};

    // Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    esp_err_t res = am2320_get_rht(&dev, &air_temperature_value, &air_humidity_value);
    // 500 ms delay
    vTaskDelay((500) / portTICK_PERIOD_MS);
}

void soil_sensor(int *soil_moisture_value, float *soil_temperature_value)
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
    *soil_moisture_value = (moisture_value - 650) * 100 / 450;
    *soil_temperature_value = temperature_value;
}

void light_sensor(int *light_value)
{
    // Configuring the ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11); // ADC1_CHANNEL_0 is on GPIO0 (GPIOzero)
    int val = adc1_get_raw(ADC1_CHANNEL_1);
    ESP_LOGI(tag, "Light sensor ADC value: %d", val);
    // 500 ms delay
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1 secon
    *light_value = val / 41;
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

void rgb(int light)
{
    int red_duty = 0;
    int green_duty = 0;
    int blue_duty = 0;
    int scale = 8100 / 255;
gpio_set_level(LEDC_OUTPUT_IO_BLUE, 0);
gpio_set_level(LEDC_OUTPUT_IO_GREEN, 0);
gpio_set_level(LEDC_OUTPUT_IO_RED, 0);
    if (light == 1)
    {

    }
    else if (soil_m_bad == 1 || soil_m_bad == 2)
    {
        blue_duty = scale * 255;
    }
    else if (air_h_bad == 1 || air_h_bad == 2)
    {
        blue_duty = scale * 125;
        red_duty = scale * 130;
    }
    else if (air_t_bad == 1 || air_t_bad == 2)
    {
        blue_duty = scale * 255;
        red_duty = scale * 255;
        green_duty = scale * 255;
    }
    else if (soil_t_bad == 1 || soil_t_bad == 2)
    {
        red_duty = scale * 255;
    }
    else
    {
        green_duty = scale * 255 / 8;
        vTaskDelay(150);
        green_duty = scale * 255 / 4;
        vTaskDelay(150);
        green_duty = scale * 255 / 2;
        vTaskDelay(150);
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
        .duty = green_duty, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));

    ledc_channel_config_t ledc_channel_blue = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_BLUE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO_BLUE,
        .duty = blue_duty, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue));

    // Now the initialization is done
    ESP_LOGI(tag, "start color.");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, red_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, green_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, blue_duty));

    // 1000 ms delay
}



void buzzer_single_sound()
{
    buzzer_init();
    buzzer_play_tone(523.25, 100);
    buzzer_stop();
}

void buzzer()
{
    buzzer_init();
    buzzer_play_tone(659, 200);
    buzzer_play_tone(831, 200);
    buzzer_play_tone(988, 200);
    buzzer_play_tone(1319, 400);
    buzzer_play_tone(988, 200);
    buzzer_play_tone(1319, 400);
    buzzer_stop();
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
    soil_sensor(&soil_moisture_value, &soil_temperature_value);
    air_sensor(&air_temperature_value, &air_humidity_value);
    light_sensor(&light_value);
}

void evaluate_conditions()
{
    soil_m_bad = soil_moisture_value < 15 ? 1 : soil_moisture_value > 80 ? 2
                                                                         : 0;
    soil_t_bad = soil_temperature_value < 12 ? 1 : soil_temperature_value > 30 ? 2
                                                                               : 0;
    air_h_bad = air_humidity_value < 10 ? 1 : air_humidity_value > 35 ? 2
                                                                      : 0;
    air_t_bad = air_temperature_value < 10 ? 1 : air_temperature_value > 35 ? 2
                                                                            : 0;
    soil_temperature_happiness = soil_temperature_value > 16 && soil_temperature_value < 22 ? 1 : 0;
    soil_moisture_happiness = soil_moisture_value > 25 && soil_moisture_value < 70 ? 1 : 0;
    air_temperature_happiness = air_temperature_value > 16 && air_temperature_value < 22 ? 1 : 0;
    air_humidity_happiness = air_humidity_value > 11 && air_humidity_value < 25 ? 1 : 0;
    light_happiness = light_value > 400 / 41 && light_value < 700 / 41 ? 1 : 0;

    strcpy(light_quality, light_value < 100 / 41 ? "Dark" : light_value < 250 / 41 ? "Dim"
                                                        : light_value < 600 / 41   ? "Light"
                                                        : light_value < 900 / 41   ? "Bright"
                                                                                   : "Very bright");
    strcpy(soil_moisture_quality, soil_m_bad == 1 ? "Dry" : soil_m_bad == 2 ? "Wet"
                                                                            : "Good");
    strcpy(soil_temperature_quality, soil_t_bad == 1 ? "Cold" : soil_t_bad == 2 ? "Hot"
                                                                                : "Good");
    strcpy(air_humidity_quality, air_h_bad == 1 ? "Dry" : air_h_bad == 2 ? "Wet"
                                                                         : "Good");
    strcpy(air_temperature_quality, air_t_bad == 1 ? "Cold" : air_t_bad == 2 ? "Hot"
                                                                             : "Good");
    if (soil_m_bad || soil_t_bad || air_h_bad || air_t_bad)
    {
        general_happiness = 0;
    }
    else if (soil_moisture_happiness && soil_temperature_happiness && air_temperature_happiness && air_humidity_happiness && light_happiness)
    {
        general_happiness = 2;
    }
    else
    {
        general_happiness = 1;
    }
}

void thresholder()
{
    if (light_value < 4)
    {
        soil_m_bad = 0;
        soil_t_bad = 0;
        air_h_bad = 0;
        air_t_bad = 0;
        rgb(1);
        gpio_set_level(RED_LED_GPIO, 0);
    }
    else if (soil_m_bad || soil_t_bad || air_h_bad || air_t_bad)
    {
        printf("\nLight: %s, Soil M : %s, Soil T: %s, Air T: %s, Air H: %s\n", light_quality, soil_moisture_quality, soil_temperature_quality, air_temperature_quality, air_humidity_quality);
        gpio_set_level(RED_LED_GPIO, 1);
        rgb(0);
        buzzer();
    }
    else
    {
        printf("\nLight: %s, Soil M : %s, Soil T: %s, Air T: %s, Air H: %s\n", light_quality, soil_moisture_quality, soil_temperature_quality, air_temperature_quality, air_humidity_quality);
        gpio_set_level(RED_LED_GPIO, 0);
        rgb(0);
    }
}

char *pad_string(char *str, int line_length)
{
    int len = strlen(str);
    if (len < line_length)
    {
        for (int i = len; i < line_length; i++)
        {
            str[i] = ' '; // Fill the rest of the string with spaces
        }
        str[line_length] = '\0'; // Null-terminate the string
    }
    return str;
}

void display_values(SSD1306_t *dev)
{
    char soil_moisture_string_value[32];
    char soil_temperature_string_value[32];
    sprintf(soil_moisture_string_value, "Gnd Mst: %d%%", soil_moisture_value);
    sprintf(soil_temperature_string_value, "Gnd Tmp: %.1fC", soil_temperature_value);
    pad_string(soil_moisture_string_value, 31);
    pad_string(soil_temperature_string_value, 31);

    char air_humidity_string_value[32];
    char air_temperature_string_value[32];
    sprintf(air_humidity_string_value, "Air Hum: %.1f%%  ", air_humidity_value);
    sprintf(air_temperature_string_value, "Air Tmp: %.1fC", air_temperature_value);
    pad_string(air_humidity_string_value, 31);
    pad_string(air_temperature_string_value, 31);

    char light_string_value[32];
    sprintf(light_string_value, "LGT lvl: %d%%", light_value);
    pad_string(light_string_value, 31);

    ssd1306_display_text(dev, 2, soil_moisture_string_value, strlen(soil_moisture_string_value), false);
    ssd1306_display_text(dev, 3, soil_temperature_string_value, strlen(soil_temperature_string_value), false);
    ssd1306_display_text(dev, 4, air_humidity_string_value, strlen(air_temperature_string_value), false);
    ssd1306_display_text(dev, 5, air_temperature_string_value, strlen(air_humidity_string_value), false);
    ssd1306_display_text(dev, 6, light_string_value, strlen(light_string_value), false);
    evaluate_conditions();
    thresholder();
}

void display_condition(SSD1306_t *dev)
{
    evaluate_conditions();

    char soil_moisture_condition[64];
    char soil_tempereature_condition[64];
    sprintf(soil_moisture_condition, "Gnd Mst: %s", soil_moisture_quality);
    sprintf(soil_tempereature_condition, "Gnd Tmp: %s", soil_temperature_quality);
    // Ensure each string fills the entire line
    pad_string(soil_moisture_condition, 63);
    pad_string(soil_tempereature_condition, 63);

    char air_humidity__condition[64];
    char air_temperature_condition[64];
    sprintf(air_humidity__condition, "Air Hum: %s", air_humidity_quality);
    sprintf(air_temperature_condition, "Air Tmp: %s", air_temperature_quality);
    pad_string(air_humidity__condition, 63);
    pad_string(air_temperature_condition, 63);

    char light_condition[64];
    sprintf(light_condition, "LGT lvl: %s", light_quality);
    pad_string(light_condition, 63);

    ssd1306_display_text(dev, 2, soil_moisture_condition, strlen(soil_moisture_condition), false);
    ssd1306_display_text(dev, 3, soil_tempereature_condition, strlen(soil_tempereature_condition), false);
    ssd1306_display_text(dev, 4, air_humidity__condition, strlen(air_temperature_condition), false);
    ssd1306_display_text(dev, 5, air_temperature_condition, strlen(air_humidity__condition), false);
    ssd1306_display_text(dev, 6, light_condition, strlen(light_condition), false);

    thresholder();
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

void write_to_file()
{
    logging = 1;
    ESP_LOGI(TAG, "Writing to myfile.txt");

    // ssd1306_display_text(dev, 2, soil_m_result, strlen(soil_m_result), false);

    FILE *f = fopen("/storage/myfile.txt", "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open myfile.txt for writing");
        return;
    }
    fprintf(f, "\nTime, Soil moisture, Soil temperature, Air humidity, Air temperature, Light intensity\n"); // Use fprintf to write the string to the file
    fclose(f);                                                                                               // Close the file pointer f, not stdout
}

void append_to_file(char *str)
{
    ESP_LOGI(TAG, "Appending to myfile.txt");
    FILE *f = fopen("/storage/myfile.txt", "a"); // Redirect stdout to the file
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open myfile.txt for writing");
        return;
    }
    fprintf(f, "%s", str); // Use fprintf to write the string to the file
    fclose(f);             // Close the file pointer f, not stdout
    return;
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
    snprintf(data_str, sizeof(data_str), "%lld, %d, %.1f, %.1f, %.1f, %d\n",
             currentTime / 100000, soil_moisture_value, soil_temperature_value, air_humidity_value, air_temperature_value, light_value);
    return data_str;
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

void display_happines(SSD1306_t *dev)
{
    evaluate_conditions();
    char Happiness[64];
    sprintf(Happiness, "Happiness: %1.f%%", general_happiness);
    pad_string(Happiness, 63);
    ssd1306_display_text(dev, 2, Happiness, strlen(Happiness), false);
}

void app_main(void)
{
    // startup
    SSD1306_t dev;
    init_i2c();
    initfileread();
    initDisplay(&dev);
    ssd1306_bitmaps(&dev, 0, 0, happyFace(), 128, 64, false);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    init_red_led();


    // Buttons
    gpio_install_isr_service(0);
    button(BUTTON_1_GPIO_PIN);
    gpio_isr_handler_add(BUTTON_1_GPIO_PIN, gpio_interrupt_handler_1, (void *)BUTTON_1_GPIO_PIN);
    button(BUTTON_2_GPIO_PIN);
    gpio_isr_handler_add(BUTTON_2_GPIO_PIN, gpio_interrupt_handler_2, (void *)BUTTON_2_GPIO_PIN);

    // time
    esp_err_t esp_timer_init();
    baseTime = esp_timer_get_time();

    // Go to main loop
    // button_switch(&dev);

    int switchState = 0;
    const char *programRunning[] = {"Display values", "Display condi.", "Start Logging", "Stop Logging", "Data output", "Happiness"}; // Array of strings
    const char currentProgram[32];

    sprintf(currentProgram, "%d. %s", switchState + 1, programRunning[switchState]);
    display_menu(&dev, currentProgram);
    printf("Program : %s \t|", currentProgram);

    while (true)
    {
        if (btn1)
        {
            buzzer_single_sound();
            btn1 = 0;
            switchState = (switchState + 1) % 6;

            sprintf(currentProgram, "%d. %s", switchState + 1, programRunning[switchState]);

            display_menu(&dev, currentProgram);
            printf("Program : %s \t|", currentProgram);
        }
        else if (btn2)
        {
            buzzer_single_sound();
            btn2 = 0;

            switch (switchState)
            {
            case 0:
                display_values(&dev);
                break;
            case 1:
                display_condition(&dev);
                break;
            case 2:
                // Start data logging for a specified duration
                printf("Start data\n");
                write_to_file(); // Implement this function to write the header to the file
                display_menu(&dev, currentProgram);
                break;
            case 3:
                logging = 0; // Stops writing to file
                printf("Stop Write\n");
                display_menu(&dev, currentProgram);
                break;
            case 4:
                read_to_file(); // Implement this function to read and display the logged data
                break;
            case 5:
                display_happines(&dev);
                break;
            }
        }
        else
        {
            receive_data();
            currentTime = esp_timer_get_time();
            if (currentTime > baseTime + 5000000 && logging == 1)
            {
                baseTime += currentTime - baseTime;
                printf("Appending\n");
                append_to_file(sensor_data());
            }
            vTaskDelay(50);
            switchState == 0 ? display_values(&dev) : (switchState == 1 ? display_condition(&dev) : printf("waiting\n"));
        }
    }
}
c:\Users\dingv\OneDrive\Uni\1.Semester\02112\DEMO\sample_project\main\bitmaps.h c:\Users\dingv\OneDrive\Uni\1.Semester\02112\DEMO\sample_project\main\bitmaps.c