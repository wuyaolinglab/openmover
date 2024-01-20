#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utils.h"
#include "config.h"

// 电机指令类型
struct motor_cmd
{
    int16_t cmd_left_rpm;
    int16_t cmd_right_rpm;
};

// 电机反馈类型
struct motor_response
{
    int16_t speed_left_rpm;
    int16_t speed_right_rpm;
    int64_t counter_left_pulses;
    int64_t counter_right_pulses;
    uint8_t error_code_left;
    uint8_t error_code_right;
};

// 电机控制量队列
extern QueueHandle_t motor_cmd_queue;
// 电机反馈量队列
extern QueueHandle_t motor_response_queue;

void motor_init();
void motor_handler(void *pt);

#endif