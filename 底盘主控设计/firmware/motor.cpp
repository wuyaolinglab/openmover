#include "motor.h"

// 电机控制量队列
QueueHandle_t motor_cmd_queue;
// 电机反馈量队列
QueueHandle_t motor_response_queue;

// 电机控制量
motor_cmd motor_cmd_data;

// 电机反馈量
motor_response motor_response_data;

// 电机控制量buffer
uint8_t motor_cmd_buffer[MOTOR_CMD_BUFFER_SIZE] = {0x00};
uint8_t motor_recv_buffer[MOTOR_RECV_BUFFER_SIZE] = {0x00};

// 上一次收到数据的时候与这一次收到数据的时间
unsigned long last_recv_time_left_ms = 0;
unsigned long last_recv_time_right_ms = 0;
unsigned long recv_time_left_ms = 0;
unsigned long recv_time_right_ms = 0;

void motor_init()
{
    // 初始化队列
    motor_cmd_queue = xQueueCreate(1, sizeof(motor_cmd));
    motor_response_queue = xQueueCreate(10, sizeof(motor_response));

    // 初始化串口
    MOTOR_SERIAL.begin(MOTOR_BAUDRATE, SERIAL_8N1, 7, 6);

    // 创建任务，绑定到核心0
    xTaskCreatePinnedToCore(
        motor_handler,
        "motor_handler",
        8192,
        NULL,
        3,
        NULL,
        0);
}

void motor_build_cmd_frame(uint8_t *buffer, uint8_t motor_id, int16_t speed_rpm)
{
    // 防止速度出错
    speed_rpm = abs(speed_rpm) > MOTOR_MAX_RPM ? copysign(MOTOR_MAX_RPM, speed_rpm) : speed_rpm;

    // 数据帧
    buffer[0] = motor_id;
    buffer[1] = 0x64;
    buffer[2] = (uint8_t)(speed_rpm >> 8);
    buffer[3] = (uint8_t)(speed_rpm & 0xFF);
    buffer[4] = 0x00;
    buffer[5] = 0x00;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    buffer[8] = 0x00;

    // CRC校验
    buffer[MOTOR_RECV_BUFFER_SIZE - 1] = crc8_maxim(buffer, MOTOR_RECV_BUFFER_SIZE - 1);
}

void motor_parse_response_frame(uint8_t *buffer, uint8_t motor_id, motor_response &data)
{
    if (motor_id == buffer[0] &&
        crc8_maxim(buffer, MOTOR_RECV_BUFFER_SIZE - 1) == buffer[MOTOR_RECV_BUFFER_SIZE - 1])
    {
        // 计算速度
        int16_t speed = ((int16_t)buffer[4] << 8) | buffer[5];

        if (motor_id == MOTOR_LEFT_ID)
        {
            recv_time_left_ms = millis();

            if (last_recv_time_left_ms != 0)
            {
                data.speed_left_rpm = speed;
                data.counter_left_pulses += (int64_t)(((float)speed) *
                                                      ((float)(recv_time_left_ms - last_recv_time_left_ms)) * ((float)MOTOR_PULSE_PER_ROUND) / 60000.0f);
                data.error_code_left = buffer[8];
            }

            last_recv_time_left_ms = recv_time_left_ms;
        }
        else if (motor_id == MOTOR_RIGHT_ID)
        {
            recv_time_right_ms = millis();

            if (last_recv_time_right_ms != 0)
            {
                data.speed_right_rpm = speed;
                data.counter_right_pulses += (int64_t)(((float)speed) *
                                                       ((float)(recv_time_right_ms - last_recv_time_right_ms)) * ((float)MOTOR_PULSE_PER_ROUND) / 60000.0f);
                data.error_code_right = buffer[8];
            }

            last_recv_time_right_ms = recv_time_right_ms;
        }
    }
}

void motor_handler(void *pt)
{
    while (1)
    {
        // 等待电机指令
        if (xQueueReceive(motor_cmd_queue, &motor_cmd_data, 0))
        {
            // 发送指令到左电机
            motor_build_cmd_frame(motor_cmd_buffer, MOTOR_LEFT_ID, motor_cmd_data.cmd_left_rpm);
            MOTOR_SERIAL.setTimeout(MOTOR_RECV_TIMEOUT);
            MOTOR_SERIAL.write(motor_cmd_buffer, MOTOR_CMD_BUFFER_SIZE);

            // 等待并解析左电机反馈
            if (MOTOR_SERIAL.readBytes(motor_recv_buffer, MOTOR_RECV_BUFFER_SIZE) ==
                MOTOR_RECV_BUFFER_SIZE)
            {
                motor_parse_response_frame(motor_recv_buffer, MOTOR_LEFT_ID, motor_response_data);
            }

            // 发送指令到右电机
            motor_build_cmd_frame(motor_cmd_buffer, MOTOR_RIGHT_ID, motor_cmd_data.cmd_right_rpm);
            MOTOR_SERIAL.setTimeout(MOTOR_RECV_TIMEOUT);
            MOTOR_SERIAL.write(motor_cmd_buffer, MOTOR_CMD_BUFFER_SIZE);

            // 等待并解析右电机反馈
            if (MOTOR_SERIAL.readBytes(motor_recv_buffer, MOTOR_RECV_BUFFER_SIZE) ==
                MOTOR_RECV_BUFFER_SIZE)
            {
                motor_parse_response_frame(motor_recv_buffer, MOTOR_RIGHT_ID, motor_response_data);
            }

            // 发送反馈数据到队列
            xQueueSend(motor_response_queue, &motor_response_data, 0);
        }

        // 200Hz 运行
        vTaskDelay(MOTOR_CONTROL_PERIOD / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}