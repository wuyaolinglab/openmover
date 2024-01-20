#include "button.h"

// 按钮消息队列
QueueHandle_t button_queue;

void button_init()
{
    // 初始化队列
    button_queue = xQueueCreate(10, sizeof(uint8_t));

    // 初始化IO
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // 创建任务，绑定到核心1
    xTaskCreatePinnedToCore(
        button_handler,
        "button_handler",
        4096,
        NULL,
        1,
        NULL,
        1);
}

void button_handler(void *pt)
{
    while (1)
    {
        // 读取按钮
        uint8_t button_state = !digitalRead(BUTTON_PIN);

        // 发送按钮状态
        xQueueSend(button_queue, &button_state, 0);

        // 50Hz 运行
        vTaskDelay(BUTTON_CONTROL_PERIOD / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}