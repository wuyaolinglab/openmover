#include "led.h"

// LED 数组
CRGB leds[LED_NUM];

// 上一次灯光颜色
uint8_t led_current_color = 0;

// LED 消息队列
QueueHandle_t led_queue;

void led_init()
{
    // 初始化队列
    led_queue = xQueueCreate(1, sizeof(uint8_t));

    // 初始化灯带
    FastLED.addLeds<LED_TYPE, LED_DATA_PIN, LED_COLOR_ORDER>(leds, LED_NUM);

    // 灯带默认白光
    led_set_color(led_current_color, 1);

    // 创建任务，绑定到核心1
    xTaskCreatePinnedToCore(
        led_handler,
        "led_handler",
        4096,
        NULL,
        1,
        NULL,
        1);
}

void led_set_color(uint8_t color, uint8_t force_set)
{
    if (led_current_color != color || force_set)
    {
        for (int i = 0; i < LED_NUM; i += 1)
        {
            switch (color)
            {
            // 白色
            case 0:
                leds[i] = CRGB(255, 255, 255);
                break;
                // 红色
            case 1:
                leds[i] = CRGB(255, 0, 0);
                break;
            // 绿色
            case 2:
                leds[i] = CRGB(0, 255, 0);
                break;
            // 蓝色
            case 3:
                leds[i] = CRGB(0, 0, 255);
                break;
            }
        }

        FastLED.show();

        led_current_color = color;
    }
}

void led_handler(void *pt)
{
    while (1)
    {
        uint8_t color_received = 0;

        // 等待消息队列有数据
        if (xQueueReceive(led_queue, &color_received, portMAX_DELAY))
        {
            led_set_color(color_received, 0);
        }

        // 50Hz 运行
        vTaskDelay(LED_CONTROL_PERIOD / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}