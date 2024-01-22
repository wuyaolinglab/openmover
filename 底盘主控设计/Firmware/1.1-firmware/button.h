#ifndef BUTTON_H
#define BUTTON_H

#include "Arduino.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// 按钮消息队列
extern QueueHandle_t button_queue;

void button_init();
void button_handler(void *pt);

#endif