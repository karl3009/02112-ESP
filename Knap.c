#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include <string.h>
#include "esp_log.h"

// driver
#include "driver/gpio.h"

#include "freertos/queue.h"

#define INPUT_PIN 19
#define LED_PIN 9

int state = 0;
QueueHandle_t interputQueue;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

void LED_Control_Task(void *params)
{
    int state = 0;
    int pinNumber, count = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
        {
            count++;
            if (count % 2 == 0)
            {
                if (state == 0)
                {
                    state = 1;
                    gpio_set_level(LED_PIN, state);
                }
                else
                {
                    state = 0;
                    gpio_set_level(LED_PIN, state);
                }
                printf("GPIO %d was pressed %d times. The state is %d\n", pinNumber, count / 2, state);
            }
        }
    }
}

void app_main()
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE);

    interputQueue = xQueueCreate(1, sizeof(int));
    if (interputQueue == NULL)
    {
        // Handle error: Queue creation failed
        ESP_LOGE("Queue Create", "Failed to create queue");
        return;
    }

    BaseType_t taskCreated = xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 1, NULL);
    if (taskCreated != pdPASS)
    {
        // Handle error: Task creation failed
        ESP_LOGE("Task Create", "Failed to create task");
        return;
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
}