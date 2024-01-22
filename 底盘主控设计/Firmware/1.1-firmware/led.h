#ifndef LED_H
#define LED_H

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <FastLED.h>
#include "config.h"

// LED 消息队列
extern QueueHandle_t led_queue;

void led_init();
void led_set_color(uint8_t color, uint8_t force_set);
void led_handler(void *pt);

#endif