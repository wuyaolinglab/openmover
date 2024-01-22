#include <Arduino.h>
#include "led.h"
#include "button.h"
#include "motor.h"
#include "config.h"
#include "esp_task_wdt.h"

// 通讯相关变量
String comm_recv_buffer = "";
String comm_send_buffer = "";
uint8_t comm_state = COMM_STATE_WAITFORHEAD;

// 外设相关变量
motor_cmd comm_motor_cmd_data;
motor_response comm_motor_response_data;
uint8_t comm_button_data = 0;
uint8_t comm_led_color_data = 0;

void comm_send_handler(void *pt)
{
	while (1)
	{
		// 获取实时数据
		xQueueReceive(motor_response_queue, &comm_motor_response_data, 0);
		xQueueReceive(button_queue, &comm_button_data, 0);

		// 发送数据
		comm_send_buffer = COMM_HEAD;
		comm_send_buffer += comm_motor_response_data.speed_left_rpm;
		comm_send_buffer += COMM_FILTER;
		comm_send_buffer += comm_motor_response_data.speed_right_rpm;
		comm_send_buffer += COMM_FILTER;
		comm_send_buffer += comm_motor_response_data.counter_left_pulses;
		comm_send_buffer += COMM_FILTER;
		comm_send_buffer += comm_motor_response_data.counter_right_pulses;
		comm_send_buffer += COMM_FILTER;
		comm_send_buffer += comm_motor_response_data.error_code_left;
		comm_send_buffer += COMM_FILTER;
		comm_send_buffer += comm_motor_response_data.error_code_right;
		comm_send_buffer += COMM_FILTER;
		comm_send_buffer += comm_button_data;
		comm_send_buffer += COMM_TAIL;
		COMM_SERIAL.println(comm_send_buffer);

		// 100Hz 运行
		vTaskDelay(COMM_CONTROL_PERIOD / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void comm_recv_handler(void *pt)
{
	while (1)
	{
		// 接收数据
		while (COMM_SERIAL.available())
		{
			char current_char = COMM_SERIAL.read();

			if (current_char == COMM_HEAD)
			{
				comm_recv_buffer = "";
				comm_state = COMM_STATE_WAITFORTAIL;
			}
			else if (comm_state == COMM_STATE_WAITFORTAIL && current_char == COMM_TAIL)
			{
				int data_index = 0;
				int last_filter_pos = 0;
				int buffer_length = comm_recv_buffer.length();

				for (int i = 0; i < buffer_length; i++)
				{
					if (comm_recv_buffer.charAt(i) == COMM_FILTER || i == buffer_length - 1)
					{
						if (data_index == 0)
						{
							comm_motor_cmd_data.cmd_left_rpm = comm_recv_buffer.substring(last_filter_pos, i).toInt();
						}
						else if (data_index == 1)
						{
							comm_motor_cmd_data.cmd_right_rpm = comm_recv_buffer.substring(last_filter_pos + 1, i).toInt();
						}
						else if (data_index == 2)
						{
							comm_led_color_data = comm_recv_buffer.substring(last_filter_pos + 1, buffer_length).toInt();
						}

						last_filter_pos = i;
						data_index++;
					}
				}

				comm_recv_buffer = "";
				comm_state = COMM_STATE_WAITFORHEAD;
			}
			else if (comm_state == COMM_STATE_WAITFORTAIL)
			{
				comm_recv_buffer += current_char;

				if (comm_recv_buffer.length() >= COMM_MAX_BUFFER_SIZE)
				{
					comm_recv_buffer = "";
					comm_state = COMM_STATE_WAITFORHEAD;
				}
			}
		}

		// 指令发送到队列
		xQueueOverwrite(motor_cmd_queue, &comm_motor_cmd_data);
		xQueueOverwrite(led_queue, &comm_led_color_data);

		// 100Hz 运行
		vTaskDelay(COMM_CONTROL_PERIOD / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void setup()
{
	// 初始化串口
	COMM_SERIAL.begin(COMM_BAUDRATE, SERIAL_8N1, 2, 1);

	// 初始化外设任务
	led_init();
	button_init();
	motor_init();

	// 初始化通讯任务
	// 创建发送任务，绑定到核心1
	xTaskCreatePinnedToCore(
		comm_send_handler,
		"comm_send_handler",
		4096,
		NULL,
		1,
		NULL,
		1);

	// 创建接收任务，绑定到核心1
	xTaskCreatePinnedToCore(
		comm_recv_handler,
		"comm_recv_handler",
		4096,
		NULL,
		1,
		NULL,
		1);

	// 启动看门狗
	esp_task_wdt_init(WDT_TIMEOUT, true);
	esp_task_wdt_add(NULL);
}

void loop()
{
	// 喂狗
	esp_task_wdt_reset();

	// 延时
	vTaskDelay(WDT_FEED_PERIOD / portTICK_PERIOD_MS);
}